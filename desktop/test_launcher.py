"""Offline tests for the desktop launcher (desktop/launcher.py) - no arm, no network, no real build.

Run with:  python3 desktop/test_launcher.py        (or: pytest desktop/test_launcher.py)

Each test makes its own temp world: a bare "origin" repo, the installed copy cloned from it, and
fake cargo / uv / notify-send on PATH. Nothing outside the temp folder is touched (DUMMY_HOME,
XDG_DATA_HOME and XDG_CACHE_HOME all point into it).
"""
import json, os, shutil, subprocess, sys, tempfile, traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import launcher as L  # noqa: E402


# --------------------------------------------------------------------------- helpers

def sh(cwd, *args):
    r = subprocess.run(["git", "-c", "user.name=t", "-c", "user.email=t@t", "-c", "init.defaultBranch=master",
                        *args], cwd=cwd, capture_output=True, text=True)
    assert r.returncode == 0, (args, r.stderr)
    return r.stdout.strip()


class World:
    def __init__(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="dummy-launcher-test-"))
        self.old_env = dict(os.environ)
        os.environ.update(DUMMY_HOME=str(self.tmp / "home"), XDG_DATA_HOME=str(self.tmp / "xdg-data"),
                          XDG_CACHE_HOME=str(self.tmp / "xdg-cache"))
        self.bin = self.tmp / "bin"; self.bin.mkdir()
        os.environ["PATH"] = f"{self.bin}{os.pathsep}{os.environ['PATH']}"
        self.calls = self.tmp / "calls.jsonl"
        self.origin = self.tmp / "origin.git"
        self.work = self.tmp / "work"  # where "master" commits are made (the dev side)
        sh(self.tmp, "init", "-q", "--bare", str(self.origin))
        sh(self.tmp, "clone", "-q", str(self.origin), str(self.work))
        self.write_work("config.json", {"links": {"l1": 100, "l2": 120}, "speed": 1.0})
        self.write_work("vision.json", {"pick": {"drop_z": 180.0}, "vocab": ["cup"]})
        (self.work / "server.py").write_text("print('v1')\n")
        (self.work / "requirements.txt").write_text("aiohttp\n")
        (self.work / "desktop" / "src-tauri" / "icons").mkdir(parents=True)
        (self.work / "desktop" / "src-tauri" / "Cargo.toml").write_text("[package]\n")
        (self.work / "desktop" / "src-tauri" / "icons" / "128x128@2x.png").write_bytes(b"png")
        (self.work / "desktop" / "launcher.py").write_text("")
        self.commit("one")
        self.paths = L.Paths()
        sh(self.tmp, "clone", "-q", "--branch", "master", str(self.origin), self.paths.live)
        self.log = L.Log(self.paths.log)
        self.fake("notify-send", "")
        self.fake("cargo", f"""
exe = os.path.join(os.environ['CARGO_TARGET_DIR'], 'release', 'dummy-desktop')
if os.path.exists({str(self.tmp / 'FAIL')!r}): sys.exit(1)
os.makedirs(os.path.dirname(exe), exist_ok=True); open(exe, 'w').write('#!/bin/sh\\n'); os.chmod(exe, 0o755)
""")
        self.fake("uv", "")

    def fake(self, name, body):
        script = self.bin / f"{name}.py"
        script.write_text("import json, os, sys\n"
                          f"open({str(self.calls)!r}, 'a').write(json.dumps({{'who': {name!r}, 'argv': sys.argv[1:], "
                          "'cwd': os.getcwd()}) + '\\n')\n" + body)
        wrapper = self.bin / name
        wrapper.write_text(f'#!/bin/sh\nexec "{sys.executable}" "{script}" "$@"\n'); wrapper.chmod(0o755)

    def called(self, who):
        if not self.calls.exists():
            return []
        return [c for c in map(json.loads, self.calls.read_text().splitlines()) if c["who"] == who]

    def write_work(self, rel, data):
        (self.work / rel).write_text(json.dumps(data, indent=2))

    def commit(self, msg):
        sh(self.work, "add", "-A"); sh(self.work, "commit", "-q", "-m", msg); sh(self.work, "push", "-q", "origin", "master")

    def live(self, rel):
        return Path(self.paths.live) / rel

    def live_json(self, rel):
        return json.loads(self.live(rel).read_text())

    def head(self):
        return sh(self.paths.live, "rev-parse", "HEAD")

    def backups(self):
        b = Path(self.paths.backups)
        return sorted(p.relative_to(b).as_posix() for p in b.rglob("*") if p.is_file()) if b.exists() else []

    def close(self):
        os.environ.clear(); os.environ.update(self.old_env)
        shutil.rmtree(self.tmp, ignore_errors=True)


def world_test(fn):
    def run():
        w = World()
        try:
            fn(w)
        finally:
            w.close()
    run.__name__ = fn.__name__
    return run


# --------------------------------------------------------------------------- the JSON merge

def test_merge3_takes_each_sides_own_changes():
    base = {"a": 1, "b": {"x": 1, "y": 2}, "gone": 0}
    ours = {"a": 1, "b": {"x": 5, "y": 2}, "gone": 0, "mine": True}   # local calibration
    theirs = {"a": 2, "b": {"x": 1, "y": 2, "z": 3}}                   # master: new key, a removed key
    merged, conflicts = L.merge3(base, ours, theirs)
    assert merged == {"a": 2, "b": {"x": 5, "y": 2, "z": 3}, "mine": True}, merged
    assert conflicts == []


def test_merge3_master_wins_a_real_conflict():
    merged, conflicts = L.merge3({"z": 1, "k": [1]}, {"z": 2, "k": [1]}, {"z": 3, "k": [1]})
    assert merged == {"z": 3, "k": [1]} and conflicts == ["z"]
    merged, conflicts = L.merge3({"p": {"q": 1}}, {"p": {"q": 2}}, {"p": {"q": 4}})
    assert merged == {"p": {"q": 4}} and conflicts == ["p.q"]
    merged, conflicts = L.merge3([1], [2], [3])
    assert merged == [3] and conflicts == ["(the whole file)"]


def test_merge3_deletes_and_adds():
    # deleted here, untouched on master: stays deleted; deleted on master, edited here: master wins
    assert L.merge3({"a": 1}, {}, {"a": 1}) == ({}, [])
    assert L.merge3({"a": 1}, {"a": 5}, {}) == ({}, ["a"])
    # added on both sides alike, or as two objects
    assert L.merge3({}, {"n": {"x": 1}}, {"n": {"y": 2}}) == ({"n": {"x": 1, "y": 2}}, [])


def test_dump_like_keeps_the_trailing_newline_choice():
    assert L.dump_like({"a": 1}, b'{\n  "a": 0\n}') == b'{\n  "a": 1\n}'
    assert L.dump_like({"a": 1}, b'{"a": 0}\n') == b'{\n  "a": 1\n}\n'


# --------------------------------------------------------------------------- pull

@world_test
def test_pull_fast_forwards_a_clean_copy(w):
    (w.work / "server.py").write_text("print('v2')\n"); w.commit("two")
    old = w.head()
    assert L.pull(w.paths, w.log) == (old, sh(w.work, "rev-parse", "HEAD"))
    assert w.live("server.py").read_text() == "print('v2')\n"
    assert sh(w.paths.live, "status", "--porcelain") == ""
    assert L.pull(w.paths, w.log) is None  # nothing new


@world_test
def test_local_calibration_survives_and_merges_with_master(w):
    cfg = w.live_json("config.json"); cfg["links"]["l1"] = 101.5            # calibrated in the app
    w.live("config.json").write_text(json.dumps(cfg, indent=2))
    vis = w.live_json("vision.json"); vis["follow"] = {"hover": 110}       # a file master won't touch
    w.live("vision.json").write_text(json.dumps(vis, indent=2))
    w.write_work("config.json", {"links": {"l1": 100, "l2": 120}, "speed": 1.5, "tool": {"len": 60}})
    w.commit("master changes config.json elsewhere")
    assert L.pull(w.paths, w.log) is not None
    assert w.live_json("config.json") == {"links": {"l1": 101.5, "l2": 120}, "speed": 1.5, "tool": {"len": 60}}
    assert w.live_json("vision.json")["follow"] == {"hover": 110}          # untouched by git
    assert any(b.endswith("config.json") for b in w.backups())             # the local copy, kept
    assert w.called("notify-send") == []                                    # nothing lost: no fuss


@world_test
def test_a_key_both_sides_changed_takes_masters_and_says_so(w):
    cfg = w.live_json("config.json"); cfg["links"]["l1"] = 99
    w.live("config.json").write_text(json.dumps(cfg, indent=2))
    w.write_work("config.json", {"links": {"l1": 98, "l2": 120}, "speed": 1.0})
    w.commit("calib(l1): measured")
    assert L.pull(w.paths, w.log) is not None
    assert w.live_json("config.json")["links"]["l1"] == 98
    log = Path(w.paths.log).read_text()
    assert "master's value won for config.json: links.l1" in log
    backup = next(Path(w.paths.backups).rglob("config.json"))
    assert json.loads(backup.read_text())["links"]["l1"] == 99              # yours, in the backup
    assert len(w.called("notify-send")) == 1


@world_test
def test_other_files_both_changed_take_masters(w):
    w.live("server.py").write_text("print('local hack')\n")
    (w.work / "server.py").write_text("print('v2')\n"); w.commit("two")
    assert L.pull(w.paths, w.log) is not None
    assert w.live("server.py").read_text() == "print('v2')\n"
    assert "master's server.py replaced yours" in Path(w.paths.log).read_text()
    assert any(b.endswith("server.py") for b in w.backups())


@world_test
def test_an_untracked_file_master_adds_is_moved_aside(w):
    w.live("servo_cal.json").write_text('{"mine": 1}')
    w.write_work("servo_cal.json", {"master": 1}); w.commit("track servo_cal")
    assert L.pull(w.paths, w.log) is not None
    assert w.live_json("servo_cal.json") == {"master": 1}
    assert any(b.endswith("servo_cal.json") for b in w.backups())


@world_test
def test_a_locally_deleted_file_master_changed_comes_back(w):
    w.live("vision.json").unlink()
    w.write_work("vision.json", {"pick": {"drop_z": 170.0}, "vocab": ["cup"]}); w.commit("vision")
    assert L.pull(w.paths, w.log) is not None
    assert w.live_json("vision.json")["pick"]["drop_z"] == 170.0


@world_test
def test_a_copy_with_its_own_commits_is_left_alone(w):
    w.live("x.txt").write_text("x"); sh(w.paths.live, "add", "x.txt"); sh(w.paths.live, "commit", "-q", "-m", "local")
    (w.work / "server.py").write_text("print('v2')\n"); w.commit("two")
    before = w.head()
    assert L.pull(w.paths, w.log) is None and w.head() == before
    assert "commits master doesn't" in Path(w.paths.log).read_text()


@world_test
def test_a_copy_off_master_is_left_alone(w):
    sh(w.paths.live, "checkout", "-q", "-b", "other")
    (w.work / "server.py").write_text("print('v2')\n"); w.commit("two")
    assert L.pull(w.paths, w.log) is None
    assert "isn't on master" in Path(w.paths.log).read_text()


@world_test
def test_offline_starts_what_is_installed(w):
    sh(w.paths.live, "remote", "set-url", "origin", str(w.tmp / "nowhere.git"))
    assert L.pull(w.paths, w.log) is None
    assert "fetch failed" in Path(w.paths.log).read_text()


@world_test
def test_an_untracked_file_where_master_adds_a_folder_is_moved_aside(w):
    w.live("calib").write_text("scratch notes")
    (w.work / "calib").mkdir(); w.write_work("calib/x.json", {"a": 1}); w.commit("calib folder")
    assert L.pull(w.paths, w.log) is not None
    assert w.live_json("calib/x.json") == {"a": 1}
    assert any(b.endswith("/calib") for b in w.backups())


@world_test
def test_a_staged_new_file_master_also_adds(w):
    w.live("new.json").write_text('{"mine": 1}'); sh(w.paths.live, "add", "new.json")
    w.write_work("new.json", {"master": 1}); w.commit("new")
    assert L.pull(w.paths, w.log) is not None
    assert w.live_json("new.json") == {"master": 1} and any(b.endswith("/new.json") for b in w.backups())


@world_test
def test_a_failed_update_puts_everything_back_and_says_so(w):
    cfg = w.live_json("config.json"); cfg["speed"] = 2.0
    w.live("config.json").write_text(json.dumps(cfg, indent=2))
    w.write_work("config.json", {"links": {"l1": 100, "l2": 120}, "speed": 1.0, "x": 1}); w.commit("x")
    real = L.Git.ok
    L.Git.ok = lambda self, *a, **k: False if a[:1] == ("merge",) else real(self, *a, **k)
    try:
        assert L.pull(w.paths, w.log) is None
    finally:
        L.Git.ok = real
    assert w.live_json("config.json")["speed"] == 2.0          # the local edit is back
    assert "didn't update" in " ".join(w.called("notify-send")[-1]["argv"])
    assert L.pull(w.paths, w.log) is not None                   # and the next start updates
    assert w.live_json("config.json") == {"links": {"l1": 100, "l2": 120}, "speed": 2.0, "x": 1}


@world_test
def test_no_update_while_something_runs_in_the_installed_copy(w):
    os.makedirs(os.path.dirname(w.paths.python)); Path(w.paths.python).write_text(""); os.chmod(w.paths.python, 0o755)
    (w.work / "server.py").write_text("print('v2')\n"); w.commit("two")
    sleeper = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"], cwd=w.paths.live)
    try:
        assert L.busy_in(w.paths.live)
        assert L.launch(w.paths, exec_fn=lambda exe, argv: None) == 0
        assert w.live("server.py").read_text() == "print('v1')\n"
        assert "not updating now" in Path(w.paths.log).read_text()
    finally:
        sleeper.kill(); sleeper.wait()
    assert not L.busy_in(w.paths.live)


@world_test
def test_a_waiting_launcher_hands_over_once_the_app_runs(w):
    import fcntl
    os.makedirs(w.paths.home, exist_ok=True)
    holder = os.open(os.path.join(w.paths.home, "launcher.lock"), os.O_RDWR | os.O_CREAT)
    fcntl.flock(holder, fcntl.LOCK_EX)  # another launcher, busy building
    real, L.shell_running = L.shell_running, lambda *a, **k: True
    try:
        assert L.take_lock(w.paths, w.log) is None
    finally:
        L.shell_running = real
        os.close(holder)
    fd = L.take_lock(w.paths, w.log)
    assert fd is not None and not os.get_inheritable(fd)
    os.close(fd)


# --------------------------------------------------------------------------- venv, build, launch

@world_test
def test_venv_syncs_only_when_requirements_change(w):
    os.makedirs(os.path.dirname(w.paths.python)); Path(w.paths.python).write_text(""); os.chmod(w.paths.python, 0o755)
    assert L.sync_venv(w.paths, w.log) and L.sync_venv(w.paths, w.log)
    calls = w.called("uv")
    assert len(calls) == 1 and calls[0]["argv"][:3] == ["pip", "install", "--python"]
    (w.work / "requirements.txt").write_text("aiohttp\nnumpy\n"); w.commit("numpy")
    L.pull(w.paths, w.log)
    assert L.sync_venv(w.paths, w.log) and len(w.called("uv")) == 2


@world_test
def test_build_once_per_commit_and_a_failure_keeps_the_old_binary(w):
    assert L.build(w.paths, w.log) and L.build(w.paths, w.log)
    builds = w.called("cargo")
    assert len(builds) == 1 and builds[0]["argv"] == ["build", "--release", "--locked"]
    assert builds[0]["cwd"] == w.paths.src
    (w.work / "server.py").write_text("print('v2')\n"); w.commit("two"); L.pull(w.paths, w.log)
    (w.tmp / "FAIL").write_text("")
    assert not L.build(w.paths, w.log) and os.path.isfile(w.paths.exe)
    assert "couldn't update itself" in " ".join(w.called("notify-send")[-1]["argv"])
    (w.tmp / "FAIL").unlink()
    assert L.build(w.paths, w.log) and len(w.called("cargo")) == 3


@world_test
def test_launch_updates_then_execs_the_shell(w):
    os.makedirs(os.path.dirname(w.paths.python)); Path(w.paths.python).write_text(""); os.chmod(w.paths.python, 0o755)
    (w.work / "server.py").write_text("print('v2')\n"); w.commit("two")
    seen = []
    assert L.launch(w.paths, exec_fn=lambda exe, argv: seen.append(argv)) == 0
    assert w.live("server.py").read_text() == "print('v2')\n"
    assert seen == [[w.paths.exe, "--app", w.paths.live, "--python", w.paths.python, "--log", w.paths.server_log]]


@world_test
def test_launch_hands_over_when_the_app_runs(w):
    L.build(w.paths, w.log)
    (w.work / "server.py").write_text("print('v2')\n"); w.commit("two")
    real, L.shell_running = L.shell_running, lambda *a, **k: True
    try:
        seen = []
        assert L.launch(w.paths, exec_fn=lambda exe, argv: seen.append(exe)) == 0
    finally:
        L.shell_running = real
    assert seen == [w.paths.exe]
    assert w.live("server.py").read_text() == "print('v1')\n"   # never updated under a running app


@world_test
def test_install_writes_the_entry_and_icon(w):
    shutil.rmtree(w.paths.live)
    seed = w.tmp / "seedvenv" / "bin"; seed.mkdir(parents=True); (seed / "python").write_text(""); (seed / "python").chmod(0o755)
    assets = w.tmp / "dev"; (assets / "weights").mkdir(parents=True); (assets / "weights" / "hand.task").write_text("m")
    (assets / "yolo.pt").write_text("pt"); (assets / "notes.txt").write_text("no")
    sh(w.tmp, "clone", "-q", str(w.origin), str(w.tmp / "src"))
    report = L.install(w.paths, source=str(w.tmp / "src"), seed_venv=str(w.tmp / "seedvenv"), assets_from=str(assets),
                       log=w.log)
    assert report["cloned"] == str(w.origin) and report["assets"] == ["weights", "yolo.pt"]
    assert w.live("weights/hand.task").read_text() == "m" and not w.live("notes.txt").exists()
    assert os.path.isfile(w.paths.python) and os.path.isfile(w.paths.exe)
    text = Path(w.paths.desktop_file).read_text()
    assert f'Exec="/usr/bin/python3" -I -S "{w.paths.live}/desktop/launcher.py"' in text
    assert "Icon=dev.dummy.desktop" in text and "StartupWMClass=dev.dummy.desktop" in text
    assert Path(w.paths.icon).read_bytes() == b"png"
    # again: nothing to clone or copy, still fine
    assert "cloned" not in L.install(w.paths, source=str(w.tmp / "src"), log=w.log)
    assert L.uninstall(w.paths)["removed"] == [w.paths.desktop_file, w.paths.icon]


def test_exec_arg_quotes_like_the_spec():
    assert L.exec_arg("/a b/c") == '"/a b/c"'
    assert L.exec_arg('/x$"%') == '"/x\\\\$\\\\"%%"'


def test_shell_running_reads_comm():
    tmp = Path(tempfile.mkdtemp())
    try:
        (tmp / "12").mkdir(); (tmp / "12" / "comm").write_text("dummy-desktop\n")
        assert L.shell_running(str(tmp)) and not L.shell_running(str(tmp), uid=os.getuid() + 1)
    finally:
        shutil.rmtree(tmp)


def test_stdlib_only():
    import ast
    tree = ast.parse(Path(L.__file__).read_text())
    names = {a.name.split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
    names |= {n.module.split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.ImportFrom) and n.module}
    assert all(n in sys.stdlib_module_names for n in names), names


# --------------------------------------------------------------------------- runner

def main():
    tests = [(n, f) for n, f in globals().items() if n.startswith("test_") and callable(f)]
    fails = []
    for name, fn in tests:
        try:
            fn()
            print(f"  PASS  {name}")
        except AssertionError as e:
            fails.append(name)
            print(f"  FAIL  {name}: {e}")
            traceback.print_exc()
        except Exception as e:
            fails.append(name)
            print(f"  ERROR {name}: {type(e).__name__}: {e}")
            traceback.print_exc()
    print(f"\n{len(tests) - len(fails)}/{len(tests)} passed")
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(main())
