"""Every process the repository starts is named in its source.

Content scanner (Semgrep dangerous-subprocess-use-audit, the rule behind its
High findings): a ``subprocess`` call is reported unless its program is a
string literal, a ``sh``/``bash -c`` call's inline code is a literal, and a
``python`` call's script is a literal. This test applies that rule to every
tracked Python file, tests included, and adds what the rule leaves out: no
shell, no string command line, no ``**kwargs`` that could carry
``shell=True``, and no ``os.system``, ``os.popen``, ``os.exec*`` or
``os.spawn*``. Values a call computes reach its program as arguments only.

Run from the repo root:

    python3 -m unittest tests.test_process_calls -v
"""

import ast
import os
import re
import subprocess
import unittest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

SUBPROCESS = {"run", "Popen", "call", "check_call", "check_output", "getoutput", "getstatusoutput"}
FORBIDDEN = {("os", "system"), ("os", "popen"), ("os", "posix_spawn"), ("os", "posix_spawnp"),
             ("asyncio", "create_subprocess_shell"), ("asyncio", "create_subprocess_exec"),
             ("pty", "spawn")}
SHELLS = re.compile(r"(sh|bash|ksh|csh|tcsh|zsh)")


def _is_str(node):
    return isinstance(node, ast.Constant) and isinstance(node.value, str)


def _module_strings(tree):
    """Names bound once, at module level, to a string literal (the scanner
    propagates these, so ``[CHROME, ...]`` names its program in the source)."""
    bound = {}
    for node in ast.walk(tree):
        targets = node.targets if isinstance(node, ast.Assign) else (
            [node.target] if isinstance(node, (ast.AugAssign, ast.AnnAssign)) else [])
        for target in targets:
            if isinstance(target, ast.Name):
                bound.setdefault(target.id, []).append(node)
    return {name: nodes[0].value.value for name, nodes in bound.items()
            if len(nodes) == 1 and nodes[0] in tree.body and isinstance(nodes[0], ast.Assign)
            and _is_str(nodes[0].value)}


def problems(source, filename="<source>"):
    """``(line, reason)`` for each process start in ``source`` that breaks the rule."""
    tree = ast.parse(source, filename)
    strings = _module_strings(tree)
    found = []
    for node in ast.walk(tree):
        if not (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and isinstance(node.func.value, ast.Name)):
            continue
        module, name = node.func.value.id, node.func.attr
        if (module, name) in FORBIDDEN or (module == "os" and name.startswith(("exec", "spawn"))):
            found.append((node.lineno, "%s.%s starts a process outside subprocess" % (module, name)))
            continue
        if module != "subprocess" or name not in SUBPROCESS:
            continue
        if any(kw.arg is None for kw in node.keywords):
            found.append((node.lineno, "**kwargs could carry shell=True"))
        for kw in node.keywords:
            if kw.arg == "shell" and not (isinstance(kw.value, ast.Constant) and kw.value.value is False):
                found.append((node.lineno, "shell is not False"))
        argv = node.args[0] if node.args else None
        if not isinstance(argv, (ast.List, ast.Tuple)) or not argv.elts:
            found.append((node.lineno, "the command is not a list of arguments"))
            continue
        elts = argv.elts

        def literal(i):
            if i >= len(elts):
                return None
            if _is_str(elts[i]):
                return elts[i].value
            if isinstance(elts[i], ast.Name):
                return strings.get(elts[i].id)
            return None

        program = literal(0)
        if program is None:
            found.append((node.lineno, "the program is not named in the source"))
            continue
        if SHELLS.search(program) and literal(1) == "-c" and literal(2) is None:
            found.append((node.lineno, "the shell's inline code is not a literal"))
        if "python" in program and literal(1) is None:
            found.append((node.lineno, "the script Python runs is not a literal"))
    return sorted(found)


def _tracked_python():
    out = subprocess.run(["git", "ls-files", "-z", "--", "*.py"], cwd=REPO, check=True,
                         capture_output=True).stdout
    return sorted(path.decode() for path in out.split(b"\0") if path)


class TestEveryProcessIsNamedInTheSource(unittest.TestCase):
    """Content scanner (Semgrep dangerous-subprocess-use-audit): no tracked
    Python file starts a program chosen at run time, runs computed shell or
    Python code, or uses a shell."""

    def test_the_repository_follows_the_rule(self):
        files = _tracked_python()
        self.assertGreater(len(files), 40)
        report = []
        for rel in files:
            with open(os.path.join(REPO, rel), encoding="utf-8") as handle:
                report += ["%s:%d: %s" % (rel, line, why) for line, why in problems(handle.read(), rel)]
        self.assertEqual(report, [])

    def test_the_check_reports_what_the_scanner_reports(self):
        """Each case is the scanner's own pattern; the check must flag it."""
        flagged = (
            "subprocess.run(argv)",
            "subprocess.run([sys.executable, '-V'])",
            "subprocess.run([BASH, script, *args])",
            "subprocess.run(['python3', script, *args])",
            "subprocess.run(['bash', '-c', code])",
            "subprocess.run(['sh', '-c', 'echo %s' % name])",
            "subprocess.run('echo hi')",
            "subprocess.run(['echo', 'hi'], shell=True)",
            "subprocess.run(['echo', 'hi'], shell=flag)",
            "subprocess.run(['echo', 'hi'], **options)",
            "subprocess.Popen([*argv])",
            "os.system('echo hi')",
            "os.popen('echo hi')",
            "os.execv('/bin/echo', ['echo'])",
            "os.spawnlp(os.P_WAIT, 'echo', 'echo')",
            "asyncio.create_subprocess_exec('echo')",
        )
        for call in flagged:
            with self.subTest(call=call):
                self.assertEqual(len(problems("import os, subprocess, sys\n" + call + "\n")), 1)

    def test_the_check_passes_what_the_scanner_passes(self):
        """A named program, with computed values as its arguments only."""
        passed = (
            "subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=path)",
            "subprocess.run(['bash', os.path.join(REPO, 'host', 'run_sim.sh'), *args])",
            "subprocess.run(['python3', 'scripts/check_citations.py', *args], cwd=tmp)",
            "subprocess.run(['python3', '-c', 'pass'])",
            "subprocess.run(['bash', '-c', 'source lib.sh; \"$@\"', 'bash', *call])",
            "subprocess.run(['echo', 'hi'], shell=False)",
            "subprocess.CalledProcessError(1, ['x'])",
            "CHROME = '/Applications/Chrome'\nsubprocess.Popen([CHROME, '--headless=new', url])",
        )
        for call in passed:
            with self.subTest(call=call):
                self.assertEqual(problems("import os, subprocess\n" + call + "\n"), [])

    def test_a_reassigned_name_is_not_a_literal(self):
        source = "import subprocess\nPROG = 'echo'\nPROG = input()\nsubprocess.run([PROG])\n"
        self.assertEqual(problems(source), [(4, "the program is not named in the source")])


if __name__ == "__main__":
    unittest.main()
