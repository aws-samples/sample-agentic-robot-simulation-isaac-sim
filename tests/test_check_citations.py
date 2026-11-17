"""Tests for scripts/check_citations.py, the check that every path:line citation in
the threat model (docs/security/THREAT_MODEL.md) resolves.

Run from the repo root:

    python3 -m unittest tests.test_check_citations -v
"""

import os
import shutil
import subprocess
import sys
import tempfile
import unittest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CHECKER = os.path.join(REPO, "scripts", "check_citations.py")


@unittest.skipUnless(shutil.which("git"), "needs git")
class TestCheckCitations(unittest.TestCase):
    """The checker reports citations that name a missing file or a line past its end,
    and checks a citation pinned to a revision (`<sha>:path:line`) against that
    revision rather than the working tree."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.tmp)
        os.makedirs(os.path.join(self.tmp, "scripts"))
        shutil.copy(CHECKER, os.path.join(self.tmp, "scripts", "check_citations.py"))
        os.makedirs(os.path.join(self.tmp, "src"))
        self._write("src/a.py", "".join("line %d\n" % i for i in range(1, 11)))
        self._git("init", "-q")
        self._git("add", "-A")
        self._git("commit", "-q", "-m", "one")
        self.first = self._git("rev-parse", "--short=7", "HEAD").strip()
        self._write("src/a.py", "".join("line %d\n" % i for i in range(1, 4)))
        self._write("src/b.py", "new\n")
        self._git("add", "-A")
        self._git("commit", "-q", "-m", "two")

    def _write(self, rel, text):
        with open(os.path.join(self.tmp, rel), "w", encoding="utf-8") as handle:
            handle.write(text)

    def _git(self, *args):
        return subprocess.run(["git", "-c", "user.email=t@example.com", "-c", "user.name=t", *args],
                              cwd=self.tmp, check=True, capture_output=True, text=True).stdout

    def _run_checker(self, *args):
        """The temporary repository's copy of the checker, run by the python3 next to
        this Python (its directory first on PATH)."""
        path = os.pathsep.join(p for p in (os.path.dirname(sys.executable), os.environ.get("PATH")) if p)
        return subprocess.run(["python3", "scripts/check_citations.py", *args], cwd=self.tmp,
                              env=dict(os.environ, PATH=path), capture_output=True, text=True)

    def _check(self, body):
        self._write("doc.md", body)
        return self._run_checker(os.path.join(self.tmp, "doc.md"))

    def test_good_citations_pass(self):
        result = self._check("see `src/a.py:2-3` and `src/b.py:1`\n")
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn("0 broken", result.stdout)

    def test_missing_file_and_line_past_the_end_fail(self):
        result = self._check("see `src/a.py:9` and `src/gone.py`\n")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("src/a.py:9: past end of file", result.stdout)
        self.assertIn("src/gone.py: file does not exist", result.stdout)

    def test_a_pinned_citation_is_checked_at_its_revision(self):
        ok = self._check("was `%s:src/a.py:9-10`, now `src/a.py:3`\n" % self.first)
        self.assertEqual(ok.returncode, 0, ok.stdout)
        bad = self._check("was `%s:src/a.py:11` and `%s:src/b.py`\n" % (self.first, self.first))
        self.assertNotEqual(bad.returncode, 0)
        self.assertIn("src/a.py:11: past end of file (10 lines in %s)" % self.first, bad.stdout)
        self.assertIn("src/b.py: not in %s" % self.first, bad.stdout)

    def test_a_shorthand_after_a_pinned_path_uses_its_revision(self):
        ok = self._check("`%s:src/a.py:1`, `:10`\n" % self.first)
        self.assertEqual(ok.returncode, 0, ok.stdout)

    def test_a_revision_missing_from_the_clone_fails_unless_allowed(self):
        body = "was `deadbee:src/a.py:1`, now `src/a.py:1`\n"
        strict = self._check(body)
        self.assertNotEqual(strict.returncode, 0)
        self.assertIn("cannot read revision deadbee", strict.stdout + strict.stderr)
        self._write("doc.md", body)
        allowed = self._run_checker("--allow-missing-revisions", os.path.join(self.tmp, "doc.md"))
        self.assertEqual(allowed.returncode, 0, allowed.stdout + allowed.stderr)
        self.assertIn("1 pinned to revisions not in this clone (deadbee), not checked", allowed.stdout)

    def test_a_present_revision_is_still_checked_when_missing_ones_are_allowed(self):
        self._write("doc.md", "`%s:src/a.py:11`\n" % self.first)
        result = self._run_checker("--allow-missing-revisions", os.path.join(self.tmp, "doc.md"))
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("past end of file", result.stdout)


if __name__ == "__main__":
    unittest.main()
