"""Check the `path:line` citations in a Markdown document against the repository.

docs/security/THREAT_MODEL.md backs every claim with a citation such as
`hud/server.py:26-39`, and follows a path with shorthand for more lines in the
same file (`:83-86`). Code moves, and a citation that points at the wrong lines
is worse than none, because it looks like evidence. This reports every citation
whose file does not exist or whose lines run past the end of the file.

    python3 scripts/check_citations.py docs/security/THREAT_MODEL.md

The threat model cites a fixed revision. --rev checks every citation against
that revision instead of the working tree, so uncommitted edits (or files that
were never committed) cannot make a citation look valid:

    python3 scripts/check_citations.py docs/security/THREAT_MODEL.md --rev 63c05ef

A file added alongside the document and absent from REV can be named with
--added PATH (repeatable; a directory covers everything under it). Such a file
may be cited by name only, and is then checked in the working tree; a line
citation of it is reported, because its lines are not part of REV.

It cannot tell whether the cited lines still say what the text claims. For
that, --changed-since lists the citations whose lines were edited since a
revision, so a person can re-read exactly those:

    python3 scripts/check_citations.py docs/security/THREAT_MODEL.md --changed-since REV

and --remap-from rewrites the line numbers of citations whose lines are
unchanged since REV but have moved, leaving the edited ones for that person:

    python3 scripts/check_citations.py docs/security/THREAT_MODEL.md --remap-from REV

What counts as a citation: a backticked span whose first path segment is a
file or directory at the repository root. That keeps `/api` and `ros:jazzy`
out. Spans that name a git revision (`origin/main:...`, `git show abc1234:...`)
point outside the tree and are skipped, and a shorthand `:N` after one of them
is not attributed to the previous path. Other spans (a function or topic name
between a path and its shorthand) do not end the path's scope. A shorthand with
no path earlier on the same line is an orphan: which file it continues is a
guess, so it is reported rather than checked. So is a span that starts with a
root segment and contains a colon but does not parse as a citation (an en dash
in a range, a comma list), so neither can slip through unchecked.

Exit status is 1 if any citation is broken or unparsed, so CI can run it.
Standard library only.
"""

from __future__ import annotations

import argparse
import difflib
import fnmatch
import glob
import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

SPAN = re.compile(r"`([^`\n]+)`")
# path, then optionally :start or :start-end
CITATION = re.compile(r"^(?P<path>[\w./*-]+?)(?::(?P<start>\d+)(?:-(?P<end>\d+))?)?$")
SHORTHAND = re.compile(r"^:(?P<start>\d+)(?:-(?P<end>\d+))?$")
REVISION = re.compile(r"^(git |origin/|[0-9a-f]{7,40}:)")
# A citation pinned to a revision: `63c05ef:agent/mission.py:1262-1263`. Checked
# against that revision (the file existed there, the lines were in range), and
# left alone by --remap-from and --changed-since.
PINNED = re.compile(r"^(?P<rev>[0-9a-f]{7,40}):(?P<rest>.+)$")


@dataclass
class Citation:
    line_no: int  # line in the Markdown document
    path: str
    start: int | None
    end: int | None
    col: int  # offset of the span in the document line, for rewriting
    text: str  # the span as written, without backticks
    rev: str | None = None  # set for a citation pinned to a revision

    def label(self) -> str:
        if self.start is None:
            return self.path
        rng = f"{self.start}" if self.end is None else f"{self.start}-{self.end}"
        return f"{self.path}:{rng}"


def git(*args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["git", "-C", str(ROOT), *args],
        capture_output=True,
        text=True,
        check=False,
    )


class Tree:
    """File existence and line counts, from the working tree or from a revision."""

    def __init__(self, rev: str | None) -> None:
        self.rev = rev
        self.files: set[str] = set()
        self.dirs: set[str] = set()
        self._lines: dict[str, int] = {}
        if rev is None:
            return
        r = git("ls-tree", "-r", "--name-only", rev)
        if r.returncode != 0:
            raise SystemExit(f"cannot read revision {rev}: {r.stderr.strip()}")
        for name in r.stdout.splitlines():
            self.files.add(name)
            parts = name.split("/")
            for i in range(1, len(parts)):
                self.dirs.add("/".join(parts[:i]))

    def roots(self) -> set[str]:
        names = {p.name for p in ROOT.iterdir()}
        if self.rev is not None:
            names |= {f.split("/")[0] for f in self.files}
        return names

    def exists(self, path: str) -> bool:
        if self.rev is None:
            return (ROOT / path).exists()
        return path in self.files or path in self.dirs

    def is_dir(self, path: str) -> bool:
        if self.rev is None:
            return (ROOT / path).is_dir()
        return path in self.dirs

    def matches(self, pattern: str) -> bool:
        if self.rev is None:
            return bool(glob.glob(str(ROOT / pattern)))
        return any(fnmatch.fnmatchcase(f, pattern) for f in self.files)

    def line_count(self, path: str) -> int:
        if path not in self._lines:
            if self.rev is None:
                text = (ROOT / path).read_text(errors="replace")
            else:
                r = subprocess.run(
                    ["git", "-C", str(ROOT), "cat-file", "-p", f"{self.rev}:{path}"],
                    capture_output=True,
                    check=False,
                )
                text = r.stdout.decode("utf-8", errors="replace")
            self._lines[path] = len(text.splitlines())
        return self._lines[path]


def parse(doc: Path, roots: set[str]) -> tuple[list[Citation], list[tuple[int, str]]]:
    found: list[Citation] = []
    unparsed: list[tuple[int, str]] = []
    for n, line in enumerate(doc.read_text().splitlines(), start=1):
        last: str | None = None  # the path a `:N` shorthand refers to
        last_rev: str | None = None  # its revision, when the path was pinned
        for m in SPAN.finditer(line):
            text = m.group(1).strip()
            pinned = PINNED.match(text)
            if pinned:
                c = CITATION.match(pinned.group("rest"))
                if c and c.group("path").split("/")[0] in roots:
                    start, end = c.group("start"), c.group("end")
                    found.append(
                        Citation(
                            n,
                            c.group("path"),
                            int(start) if start else None,
                            int(end) if end else None,
                            m.start(1),
                            text,
                            pinned.group("rev"),
                        )
                    )
                    last, last_rev = c.group("path"), pinned.group("rev")
                    continue
            if REVISION.match(text):
                last, last_rev = None, None
                continue
            short = SHORTHAND.match(text)
            if short:
                if last is None:
                    unparsed.append((n, text))  # orphan: no file before it on the line
                    continue
                start, end = short.group("start"), short.group("end")
                found.append(
                    Citation(
                        n,
                        last,
                        int(start),
                        int(end) if end else None,
                        m.start(1),
                        text,
                        last_rev,
                    )
                )
                continue
            c = CITATION.match(text)
            if not c or c.group("path").split("/")[0] not in roots:
                # A function name or topic between a path and its shorthand
                # (`a.py:1` (`fn`), `:9`) does not end the path's scope.
                head = re.split(r"[/:\s]", text, maxsplit=1)[0]
                if ":" in text and head in roots:
                    unparsed.append((n, text))
                continue
            path, start, end = c.group("path"), c.group("start"), c.group("end")
            found.append(
                Citation(
                    n,
                    path,
                    int(start) if start else None,
                    int(end) if end else None,
                    m.start(1),
                    text,
                )
            )
            last, last_rev = path, None
    return found, unparsed


def is_added(path: str, added: list[str]) -> bool:
    path = path.rstrip("/")
    return any(path == a or path.startswith(a + "/") for a in added)


def problems(c: Citation, tree: Tree, added: list[str]) -> str | None:
    path = c.path.rstrip("/")
    if tree.rev is not None and is_added(path, added) and not tree.exists(path):
        if c.start is not None:
            return f"line citation of a file that is not in {tree.rev}"
        local = Tree(None)
        if "*" in path:
            return None if local.matches(path) else "no file matches (working tree)"
        return None if local.exists(path) else "file does not exist (working tree)"
    where = f" in {tree.rev}" if tree.rev is not None else ""
    if "*" in path:
        return None if tree.matches(path) else f"no file matches{where}"
    if not tree.exists(path):
        if tree.rev is not None and (ROOT / path).exists():
            return f"not in {tree.rev} (exists only in the working tree)"
        return f"file does not exist{where}"
    if c.start is None:
        return None
    if tree.is_dir(path):
        return "line numbers on a directory"
    end = c.end if c.end is not None else c.start
    if c.start < 1 or end < c.start:
        return "bad line range"
    total = tree.line_count(path)
    if end > total:
        return f"past end of file ({total} lines{where})"
    return None


def git_lines(rev: str, path: str) -> list[str] | None:
    r = git("show", f"{rev}:{path}")
    return r.stdout.splitlines() if r.returncode == 0 else None


def line_map(rev: str, path: str) -> dict[int, int] | None:
    """Old line number -> new line number, for lines unchanged since rev."""
    old = git_lines(rev, path)
    new_path = ROOT / path
    if old is None or not new_path.is_file():
        return None
    new = new_path.read_text(errors="replace").splitlines()
    mapping: dict[int, int] = {}
    matcher = difflib.SequenceMatcher(a=old, b=new, autojunk=False)
    for tag, i1, i2, j1, _ in matcher.get_opcodes():
        if tag == "equal":
            for k in range(i2 - i1):
                mapping[i1 + k + 1] = j1 + k + 1
    return mapping


def moved(c: Citation, mapping: dict[int, int]) -> tuple[int, int | None] | None:
    """The citation's new range, or None if any cited line was edited."""
    assert c.start is not None
    end = c.end if c.end is not None else c.start
    cited = range(c.start, end + 1)
    if not all(k in mapping for k in cited):
        return None
    new = [mapping[k] for k in cited]
    if new != list(range(new[0], new[0] + len(new))):
        return None  # lines inserted inside the range
    return new[0], (new[-1] if c.end is not None else None)


def main() -> int:
    ap = argparse.ArgumentParser(description=(__doc__ or "").split("\n\n")[0])
    ap.add_argument("docs", nargs="+", type=Path)
    ap.add_argument(
        "--rev",
        metavar="REV",
        help="check citations against this git revision, not the working tree",
    )
    ap.add_argument(
        "--allow-missing-revisions",
        action="store_true",
        help="count, instead of failing on, citations pinned to a revision this clone "
        "does not have (a fresh export or a shallow clone has none of the history)",
    )
    ap.add_argument(
        "--added",
        metavar="PATH",
        action="append",
        default=[],
        help="a path added alongside the document and absent from --rev",
    )
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--changed-since", metavar="REV")
    mode.add_argument("--remap-from", metavar="REV")
    args = ap.parse_args()

    tree = Tree(args.rev)
    roots = tree.roots()
    added = [a.rstrip("/") for a in args.added]

    status = 0
    for doc in args.docs:
        cites, unparsed = parse(doc, roots)
        rev = args.changed_since or args.remap_from
        if rev:
            maps: dict[str, dict[int, int] | None] = {}
            edits: dict[int, list[tuple[Citation, str]]] = {}
            for c in cites:
                if c.start is None or c.rev is not None:
                    continue
                if c.path not in maps:
                    maps[c.path] = line_map(rev, c.path)
                mapping = maps[c.path]
                if mapping is None:
                    print(
                        f"{doc}:{c.line_no}: {c.label()}: not in {rev} or gone; check by hand"
                    )
                    continue
                new = moved(c, mapping)
                if new is None:
                    print(
                        f"{doc}:{c.line_no}: {c.label()}: cited lines edited since {rev}"
                    )
                elif args.remap_from and (new[0], new[1]) != (c.start, c.end):
                    rng = f"{new[0]}" if new[1] is None else f"{new[0]}-{new[1]}"
                    prefix = "" if c.text.startswith(":") else c.path
                    edits.setdefault(c.line_no, []).append((c, f"{prefix}:{rng}"))
            if args.remap_from and edits:
                lines = doc.read_text().splitlines(keepends=True)
                for n, changes in edits.items():
                    line = lines[n - 1]
                    for c, replacement in sorted(changes, key=lambda e: -e[0].col):
                        line = line[: c.col] + replacement + line[c.col + len(c.text) :]
                    lines[n - 1] = line
                doc.write_text("".join(lines))
                print(f"{doc}: moved {sum(len(v) for v in edits.values())} citations")

        pinned_trees: dict[str, Tree | None] = {}

        def tree_for(c: Citation) -> Tree | None:
            if c.rev is None:
                return tree
            if c.rev not in pinned_trees:
                if git("cat-file", "-e", f"{c.rev}^{{commit}}").returncode == 0:
                    pinned_trees[c.rev] = Tree(c.rev)
                elif args.allow_missing_revisions:
                    pinned_trees[c.rev] = None
                else:
                    raise SystemExit(
                        f"cannot read revision {c.rev}, which pinned citations name "
                        "(use --allow-missing-revisions in a clone without that history)"
                    )
            return pinned_trees[c.rev]

        unverifiable = [c for c in cites if c.rev is not None and tree_for(c) is None]
        broken = [(c, p) for c in cites
                  if (t := tree_for(c)) is not None
                  and (p := problems(c, t, added if c.rev is None else []))]
        for c, p in broken:
            print(f"{doc}:{c.line_no}: {c.label()}: {p}")
        for n, text in unparsed:
            kind = "orphan shorthand" if SHORTHAND.match(text) else "unparsed citation-like span"
            print(f"{doc}:{n}: `{text}`: {kind}")
        if broken or unparsed:
            status = 1
        with_lines = sum(1 for c in cites if c.start is not None)
        print(
            f"{doc}: {len(cites)} citations ({with_lines} with lines), "
            f"{len(broken)} broken, {len(unparsed)} unparsed"
            + (f", checked against {args.rev}" if args.rev else "")
            + (
                f", {len(unverifiable)} pinned to revisions not in this clone "
                f"({', '.join(sorted({c.rev for c in unverifiable}))}), not checked"
                if unverifiable
                else ""
            )
        )
    return status


if __name__ == "__main__":
    sys.exit(main())
