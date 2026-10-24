"""DUMMY's launcher (Linux): keep the installed copy on master, then open the desktop app.

    python3 -I -S desktop/launcher.py install [--seed-venv DIR] [--assets-from DIR]
    python3 -I -S <data>/app/desktop/launcher.py            (what the KDE launcher runs)
    python3 -I -S desktop/launcher.py uninstall

Everything lives in the data folder, `$DUMMY_HOME`, else `${XDG_DATA_HOME:-~/.local/share}/dummy`:
`app/` (a clone of this repo on master: the installed copy, never edited by hand), `venv/` (its
Python), `server.log`, `launcher.log`, `backups/`. The shell is built into
`${XDG_CACHE_HOME:-~/.cache}/dummy-target`.

**install** (run once, from any checkout): clone `origin` into `app/` (branch master), copy the
git-ignored model files (`weights/`, `*.pt`) from `--assets-from` if given, make the venv
(`--seed-venv` copies an existing one, reflinked on btrfs, so a 6 GB torch install costs nothing;
otherwise `python3 -m venv` plus requirements.txt), build the shell, and write the launcher entry
`dev.dummy.desktop.desktop` and its icon. Running it again repairs whatever is missing.

**A launch** (no arguments). When a `dummy-desktop` already runs, it just hands over (the app
comes to the front). Otherwise, one launcher at a time (`launcher.lock`):
1. **Pull:** `git fetch origin master`, then fast-forward `app/` to it. The installed copy is also
   where the arm's calibration is saved (`config.json`, `vision.json`, … are tracked files the
   server writes), so local changes are kept:
   - a file master didn't change keeps its local edits (git keeps them through the fast-forward);
   - a JSON file both sides changed is merged key by key: a key only one side changed takes that
     side's value; a key both changed takes master's (a committed calibration is deliberate),
     and the launcher says which keys those were;
   - any other file both sides changed takes master's version;
   - an untracked or ignored file master now adds is moved aside.
   Every local version that is replaced or merged is copied to `backups/<time>/` first. A copy
   that has commits of its own, or isn't on master, is left alone (and the launcher says so).
2. **Python:** when requirements.txt differs from what the venv was last synced with, install it
   (uv when present, else pip).
3. **Build:** when HEAD isn't the commit the shell was last built from, `cargo build --release
   --locked` in `app/desktop/src-tauri` (cargo redoes only what changed).
4. **Start:** exec the shell with `--app`, `--python` and `--log`.
A failed step is logged and shown as a desktop notification, and never stops the app from
opening with what it has.

Stdlib only, so it runs with the system python3 before the venv exists.
"""
import argparse
import fcntl
import hashlib
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime

SHELL = "dummy-desktop"
APP_ID = "dev.dummy.desktop"
NET_LIMIT_S = 30
SYNC_LIMIT_S = 30 * 60
BUILD_LIMIT_S = 30 * 60
LOG_MAX = 1 << 20
ASSET_DIRS = ("weights",)
ASSET_GLOBS = (".pt",)
_GIT_REPO_VARS = ("GIT_DIR", "GIT_WORK_TREE", "GIT_INDEX_FILE", "GIT_OBJECT_DIRECTORY", "GIT_COMMON_DIR")
MISSING = object()


# -- folders -------------------------------------------------------------------------------------

def _xdg(name, default):
    value = os.environ.get(name)
    return value if value and os.path.isabs(value) else os.path.join(os.path.expanduser("~"), default)


class Paths:
    def __init__(self, home=None, target=None, data_home=None):
        self.home = home or os.environ.get("DUMMY_HOME") or os.path.join(_xdg("XDG_DATA_HOME", ".local/share"), "dummy")
        self.live = os.path.join(self.home, "app")
        self.venv = os.path.join(self.home, "venv")
        self.python = os.path.join(self.venv, "bin", "python")
        self.backups = os.path.join(self.home, "backups")
        self.log = os.path.join(self.home, "launcher.log")
        self.server_log = os.path.join(self.home, "server.log")
        self.stamp = os.path.join(self.home, "launcher.json")
        self.target = target or os.path.join(_xdg("XDG_CACHE_HOME", ".cache"), "dummy-target")
        self.exe = os.path.join(self.target, "release", SHELL)
        data_home = data_home or _xdg("XDG_DATA_HOME", ".local/share")
        self.desktop_file = os.path.join(data_home, "applications", APP_ID + ".desktop")
        self.icon = os.path.join(data_home, "icons", "hicolor", "256x256", "apps", APP_ID + ".png")

    @property
    def src(self):
        return os.path.join(self.live, "desktop", "src-tauri")


# -- small helpers -------------------------------------------------------------------------------

class Log:
    def __init__(self, path):
        self.path = path

    def __call__(self, msg):
        line = f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} {msg}\n"
        try:
            os.makedirs(os.path.dirname(self.path), exist_ok=True)
            with open(self.path, "a", encoding="utf-8") as f:
                f.write(line)
        except OSError:
            sys.stderr.write(line)

    def trim(self):
        try:
            if os.path.getsize(self.path) > LOG_MAX:
                with open(self.path, "rb") as f:
                    f.seek(-(LOG_MAX // 2), os.SEEK_END)
                    tail = f.read()
                with open(self.path, "wb") as f:
                    f.write(tail[tail.find(b"\n") + 1:])
        except OSError:
            pass


def notify(summary, body, log, urgent=False):
    exe = shutil.which("notify-send")
    if exe is None:
        return
    try:
        subprocess.run([exe, "--app-name=DUMMY", f"--icon={APP_ID}", f"--urgency={'critical' if urgent else 'normal'}",
                        summary, body], stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                       timeout=5)
    except (OSError, subprocess.SubprocessError) as e:
        log(f"notify-send failed: {e}")


def read_json(path):
    try:
        with open(path, "rb") as f:
            value = json.loads(f.read().decode("utf-8"))
        return value if isinstance(value, dict) else None
    except (OSError, ValueError):
        return None


def write_json(path, data):
    tmp = f"{path}.{os.getpid()}.tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(json.dumps(data, indent=2) + "\n")
    os.replace(tmp, path)


def child_env(extra=None):
    env = {k: v for k, v in os.environ.items() if k not in _GIT_REPO_VARS}
    env.update(GIT_TERMINAL_PROMPT="0", GCM_INTERACTIVE="never", SSH_ASKPASS_REQUIRE="never")
    env.update(extra or {})
    return env


def run(cmd, *, cwd=None, limit=60, log=None, what=None, env=None, capture=True):
    """Run `cmd` in a session of its own. With `capture`, (code, stdout bytes, stderr text); else its
    output goes to the log and (code, b"", ""). A timeout kills its group and gives code None."""
    out = None
    if not capture and log is not None:
        try:
            out = open(log.path, "ab")
        except OSError:
            out = None
    if log is not None and what:
        log(f"{what}: {' '.join(cmd)}" + (f" (in {cwd})" if cwd else ""))
    try:
        proc = subprocess.Popen(cmd, cwd=cwd, env=env or child_env(), stdin=subprocess.DEVNULL,
                                stdout=subprocess.PIPE if capture else (out or subprocess.DEVNULL),
                                stderr=subprocess.PIPE if capture else subprocess.STDOUT, start_new_session=True)
    except OSError as e:
        if log is not None:
            log(f"{what or cmd[0]}: could not start: {e}")
        return None, b"", str(e)
    finally:
        if out is not None:
            out.close()
    try:
        stdout, stderr = proc.communicate(timeout=limit)
    except subprocess.TimeoutExpired:
        for sig in (signal.SIGTERM, signal.SIGKILL):
            try:
                os.killpg(proc.pid, sig)
            except OSError:
                pass
            try:
                proc.communicate(timeout=5)
                break
            except subprocess.TimeoutExpired:
                continue
        if log is not None:
            log(f"{what or cmd[0]}: stopped after {limit} s")
        return None, b"", "timed out"
    if log is not None and what:
        log(f"{what}: exit {proc.returncode}")
    return proc.returncode, stdout or b"", (stderr or b"").decode("utf-8", "replace")


class Git:
    def __init__(self, path, log):
        self.path, self.log = path, log

    def __call__(self, *args, limit=60, what=None):
        return run(["git", "-c", f"safe.directory={self.path}", "-c", "gc.auto=0", "-C", self.path, *args],
                   limit=limit, log=self.log if what else None, what=what)

    def ok(self, *args, **kw):
        return self(*args, **kw)[0] == 0

    def out(self, *args, **kw):
        code, stdout, _ = self(*args, **kw)
        return stdout.decode("utf-8", "surrogateescape").strip() if code == 0 else None

    def blob(self, rev, path):
        code, stdout, _ = self("show", f"{rev}:{path}")
        return stdout if code == 0 else None

    def paths(self, *args):
        """NUL-separated path output of a git command, as a list."""
        code, stdout, _ = self(*args)
        if code != 0:
            return None
        return [p.decode("utf-8", "surrogateescape") for p in stdout.split(b"\0") if p]


def shell_running(proc="/proc", uid=None):
    uid = os.getuid() if uid is None else uid
    try:
        pids = [p for p in os.listdir(proc) if p.isdigit() and int(p) != os.getpid()]
    except OSError:
        return False
    for pid in pids:
        try:
            with open(os.path.join(proc, pid, "comm"), "rb") as f:
                if f.read().strip() != SHELL.encode():
                    continue
            if os.stat(os.path.join(proc, pid)).st_uid == uid:
                return True
        except OSError:
            continue
    return False


def file_hash(path):
    try:
        with open(path, "rb") as f:
            return hashlib.sha256(f.read()).hexdigest()
    except OSError:
        return None


# -- the JSON merge ------------------------------------------------------------------------------

def merge3(base, ours, theirs, where=""):
    """Three-way merge of JSON values (MISSING = no such key). Returns (merged, conflicts): a key
    changed on one side only takes that side; changed on both, differently, takes `theirs`
    (master) and is listed in conflicts. Objects merge key by key; anything else is one value."""
    if ours == theirs:
        return ours, []
    if ours == base:
        return theirs, []
    if theirs == base:
        return ours, []
    if all(isinstance(v, dict) for v in (base, ours, theirs)) or (
            base is MISSING and isinstance(ours, dict) and isinstance(theirs, dict)):
        base = base if isinstance(base, dict) else {}
        out, conflicts = {}, []
        for key in list(theirs) + [k for k in ours if k not in theirs]:
            value, more = merge3(base.get(key, MISSING), ours.get(key, MISSING), theirs.get(key, MISSING),
                                 f"{where}.{key}" if where else key)
            conflicts += more
            if value is not MISSING:
                out[key] = value
        return out, conflicts
    return theirs, [where or "(the whole file)"]


def dump_like(value, sample):
    """`value` as JSON text in the server's style (indent 2), ending in a newline only when
    `sample` (the file it replaces) did."""
    text = json.dumps(value, indent=2)
    return (text + "\n" if sample.endswith(b"\n") else text).encode("utf-8")


# -- the steps -----------------------------------------------------------------------------------

def pull(paths, log):
    """Fast-forward the installed copy to origin/master, keeping local edits (module doc). Returns
    (old head, new head) when it moved, else None."""
    git = Git(paths.live, log)
    if not git.ok("fetch", "--quiet", "origin", "master", limit=NET_LIMIT_S, what="fetch"):
        log("pull: fetch failed (offline?); starting what is installed")
        return None
    head, remote = git.out("rev-parse", "HEAD"), git.out("rev-parse", "refs/remotes/origin/master")
    if not head or not remote:
        remote = git.out("rev-parse", "FETCH_HEAD")
    if not head or not remote or head == remote:
        return None
    if git.out("symbolic-ref", "--short", "HEAD") != "master":
        log("pull: the installed copy isn't on master; leaving it")
        return None
    if not git.ok("merge-base", "--is-ancestor", head, remote):
        log("pull: the installed copy has commits master doesn't; leaving it")
        notify("DUMMY didn't update", f"{paths.live} has commits of its own, so it isn't updated. Details: {paths.log}", log)
        return None
    changed = git.paths("diff", "-z", "--name-only", "--no-renames", head, remote) or []
    edited = set(git.paths("diff", "-z", "--name-only", "--no-renames", "HEAD") or [])
    at_head = set(git.paths("ls-tree", "-r", "-z", "--name-only", head) or [])
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    backup = os.path.join(paths.backups, stamp)
    saved, moved, gone = {}, [], []

    def move_aside(rel):
        os.makedirs(os.path.dirname(os.path.join(backup, rel)), exist_ok=True)
        shutil.move(os.path.join(paths.live, rel), os.path.join(backup, rel))
        moved.append(rel)

    try:
        for rel in changed:
            full = os.path.join(paths.live, rel)
            # An untracked file where master needs a folder: move it aside, or the fast-forward can
            # never pass. (A tracked one is git's own file-to-folder change.)
            parts = rel.split("/")
            for i in range(1, len(parts)):
                parent = "/".join(parts[:i])
                pfull = os.path.join(paths.live, parent)
                if parent not in at_head and parent not in moved and os.path.lexists(pfull) and (
                        os.path.islink(pfull) or not os.path.isdir(pfull)):
                    move_aside(parent)
                    break
            if rel in edited and rel not in at_head:
                # new here (staged), and master adds it too: unstage it and move it aside
                git("rm", "--cached", "--quiet", "--", rel)
                if os.path.lexists(full):
                    move_aside(rel)
            elif rel in edited and not os.path.lexists(full):
                gone.append(rel)  # deleted here, changed on master: master's comes back
            elif rel in edited and os.path.isdir(full) and not os.path.islink(full):
                move_aside(rel)   # a folder where a tracked file was
                gone.append(rel)
            elif rel in edited:
                os.makedirs(os.path.dirname(os.path.join(backup, rel)), exist_ok=True)
                shutil.copy2(full, os.path.join(backup, rel), follow_symlinks=False)
                with open(full, "rb") as f:
                    saved[rel] = f.read()
            elif rel not in at_head and os.path.lexists(full):  # untracked or ignored: master adds it
                move_aside(rel)
        if (saved or gone) and not git.ok("checkout", head, "--", *saved, *gone):
            raise OSError("resetting the edited files failed")
        if not git.ok("merge", "--ff-only", "--quiet", remote, what="fast-forward"):
            raise OSError("the fast-forward failed")
    except OSError as e:
        log(f"pull: {e}; putting everything back")
        for rel in gone:
            try:
                os.unlink(os.path.join(paths.live, rel))
            except OSError:
                pass
        for rel, data in saved.items():
            with open(os.path.join(paths.live, rel), "wb") as f:
                f.write(data)
        for rel in reversed(moved):
            try:
                shutil.move(os.path.join(backup, rel), os.path.join(paths.live, rel))
            except OSError as e2:
                log(f"pull: couldn't put {rel} back ({e2}); it is in {backup}")
        notify("DUMMY didn't update", f"Updating to master failed, so it opens the version you had. Details: {paths.log}",
               log, urgent=True)
        return None
    merged_keys, replaced, kept = [], [], []
    for rel, ours in saved.items():
        full = os.path.join(paths.live, rel)
        try:
            with open(full, "rb") as f:
                theirs_bytes = f.read()
        except OSError:
            replaced.append(rel)  # master deleted it: the local version is in the backup
            continue
        base_bytes = git.blob(head, rel)
        try:
            base, mine, theirs = (json.loads(b.decode("utf-8")) for b in (base_bytes or b"", ours, theirs_bytes))
        except (ValueError, UnicodeDecodeError):
            replaced.append(rel)
            continue
        merged, conflicts = merge3(base, mine, theirs)
        if merged != theirs:
            with open(full, "wb") as f:
                f.write(dump_like(merged, theirs_bytes))
        kept.append(rel)
        merged_keys += [f"{rel}: {c}" for c in conflicts]
    back = [g for g in gone if os.path.lexists(os.path.join(paths.live, g))]
    log(f"pull: {head[:8]} -> {remote[:8]} ({len(changed)} files)")
    for rel in kept:
        log(f"pull: merged your edits to {rel} with master's (backup in {backup})")
    if merged_keys or replaced or moved or back:
        lines = [f"master's value won for {k}" for k in merged_keys] + \
                [f"master's {r} replaced yours" for r in replaced] + [f"moved aside: {m}" for m in moved] + \
                [f"{g} was deleted here; master's version is back" for g in back]
        for line in lines:
            log(f"pull: {line}")
        notify("DUMMY updated", f"Some local changes were replaced by master's (copies in {backup}). "
               f"Details: {paths.log}", log)
    return head, remote


def sync_venv(paths, log, force=False):
    """Install requirements.txt into the venv when it changed since the last sync."""
    req = os.path.join(paths.live, "requirements.txt")
    want = file_hash(req)
    stamp = read_json(paths.stamp) or {}
    if want is None or (stamp.get("requirements") == want and os.path.exists(paths.python) and not force):
        return True
    notify("Updating DUMMY", "Installing the arm's Python packages. DUMMY opens when it's done.", log)
    uv = shutil.which("uv") or next((p for p in (os.path.expanduser("~/.local/bin/uv"),) if os.access(p, os.X_OK)), None)
    cmd = ([uv, "pip", "install", "--python", paths.python, "-r", req] if uv else
           [paths.python, "-m", "pip", "install", "--disable-pip-version-check", "-r", req])
    code, _, _ = run(cmd, cwd=paths.live, limit=SYNC_LIMIT_S, log=log, what="python packages", capture=False,
                     env=child_env({"VIRTUAL_ENV": paths.venv}))
    if code != 0:
        notify("DUMMY couldn't update its Python packages", f"It opens with what it has. Details: {paths.log}", log, urgent=True)
        return False
    stamp["requirements"] = want
    write_json(paths.stamp, stamp)
    return True


def find_cargo():
    exe = shutil.which("cargo")
    local = os.path.join(os.path.expanduser("~"), ".cargo", "bin", "cargo")
    if exe is None and os.access(local, os.X_OK):
        exe = local
    return exe


def build(paths, log):
    head = Git(paths.live, log).out("rev-parse", "HEAD")
    stamp = read_json(paths.stamp) or {}
    if head and stamp.get("built") == head and os.path.isfile(paths.exe):
        return True
    cargo = find_cargo()
    if not head or cargo is None or not os.path.isfile(os.path.join(paths.src, "Cargo.toml")):
        log(f"build: not building (head {head}, cargo {cargo})")
        return False
    first = not os.path.isfile(paths.exe)
    notify("Updating DUMMY", "Building the latest version from master. DUMMY opens by itself when it's done"
           + (" (the first build takes a few minutes)." if first else " (about a minute)."), log)
    code, _, _ = run([cargo, "build", "--release", "--locked"], cwd=paths.src, limit=BUILD_LIMIT_S, log=log,
                     what="build", capture=False, env=child_env({"CARGO_TARGET_DIR": paths.target}))
    if code != 0:
        notify("DUMMY couldn't update itself", ("It opens the version you had." if os.path.isfile(paths.exe) else
               "There is no version to open yet.") + f" Details: {paths.log}", log, urgent=True)
        return False
    stamp = read_json(paths.stamp) or {}
    stamp.update(built=head, at=datetime.now().isoformat(timespec="seconds"))
    write_json(paths.stamp, stamp)
    return True


def take_lock(paths, log):
    """The launcher lock, or None when a shell started while we waited (then just hand over). The
    fd is close-on-exec and is never closed by hand: the exec that starts the shell releases it, so
    a second launcher can't slip in between the release and the shell showing up."""
    os.makedirs(paths.home, exist_ok=True)
    fd = os.open(os.path.join(paths.home, "launcher.lock"), os.O_RDWR | os.O_CREAT | os.O_CLOEXEC, 0o600)
    deadline = time.monotonic() + BUILD_LIMIT_S + SYNC_LIMIT_S
    while True:
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            return fd
        except OSError:
            if shell_running():
                os.close(fd)
                return None
            if time.monotonic() > deadline:
                log("lock: another launcher kept it too long; going on")
                return fd
            time.sleep(0.5)


def busy_in(folder, proc="/proc", uid=None):
    """A process of this user whose working folder is inside `folder` (a server.py left running
    from the installed copy writes its calibration there: never update under it)."""
    uid = os.getuid() if uid is None else uid
    root = os.path.realpath(folder)
    try:
        pids = [p for p in os.listdir(proc) if p.isdigit() and int(p) != os.getpid()]
    except OSError:
        return False
    for pid in pids:
        try:
            if os.stat(os.path.join(proc, pid)).st_uid != uid:
                continue
            cwd = os.readlink(os.path.join(proc, pid, "cwd"))
        except OSError:
            continue
        if cwd == root or cwd.startswith(root + os.sep):
            return True
    return False


def shell_args(paths):
    return [paths.exe, "--app", paths.live, "--python", paths.python, "--log", paths.server_log]


def launch(paths, exec_fn=os.execv):
    log = Log(paths.log)
    try:
        if not shell_running() and os.path.isdir(os.path.join(paths.live, ".git")):
            log.trim()
            if take_lock(paths, log) is not None and not shell_running():
                if busy_in(paths.live):
                    log("pull: a program runs in the installed copy (a server.py?); not updating now")
                else:
                    pull(paths, log)
                sync_venv(paths, log)
                build(paths, log)
    except Exception as e:  # never stop the app from opening
        log(f"launcher: {type(e).__name__}: {e}")
    if not os.access(paths.exe, os.X_OK):
        notify("DUMMY can't start", f"It isn't built yet: run `python3 desktop/launcher.py install`. Details: {paths.log}",
               log, urgent=True)
        return 1
    exec_fn(paths.exe, shell_args(paths))
    return 0


# -- install / uninstall -------------------------------------------------------------------------

def exec_arg(path):
    """One quoted `Exec` argument (Desktop Entry spec): `"`, `` ` ``, `$`, `\\` escaped, `%` doubled,
    then every backslash doubled again (the value is itself a string)."""
    if any(ord(c) < 0x20 or ord(c) == 0x7f for c in path):
        raise ValueError("the path has a control character")
    quoted = "".join("\\" + c if c in '"`$\\' else c for c in path)
    return ('"' + quoted.replace("%", "%%") + '"').replace("\\", "\\\\")


def desktop_entry(python, launcher):
    return "\n".join([
        "[Desktop Entry]",
        "Type=Application",
        "Version=1.5",
        "Name=DUMMY",
        "GenericName=Robot arm",
        "Comment=The 6-DOF arm: move it, watch the camera, calibrate",
        f"Exec={exec_arg(python)} -I -S {exec_arg(launcher)}",
        f"Icon={APP_ID}",
        f"StartupWMClass={APP_ID}",
        "Categories=Development;Electronics;",
        "Keywords=arm;robot;servo;6-DOF;",
        "StartupNotify=false",
        "Terminal=false",
        "",
    ])


def copy_tree_reflink(src, dst, log):
    code, _, err = run(["cp", "-a", "--reflink=auto", src, dst], limit=SYNC_LIMIT_S, log=log, what="copy")
    if code != 0:
        raise OSError(f"copying {src} failed: {err.strip()}")


def install(paths, *, source, seed_venv=None, assets_from=None, log=None, build_fn=None):
    log = log or Log(paths.log)
    os.makedirs(paths.home, exist_ok=True)
    report = {"home": paths.home}
    if not os.path.isdir(os.path.join(paths.live, ".git")):
        url = Git(source, log).out("remote", "get-url", "origin") or source
        code, _, err = run(["git", "clone", "--quiet", "--branch", "master", url, paths.live], limit=SYNC_LIMIT_S,
                           log=log, what="clone")
        if code != 0:
            raise OSError(f"git clone {url} failed: {err.strip()}")
        report["cloned"] = url
    if assets_from:
        copied = []
        for name in sorted(os.listdir(assets_from)):
            src, dst = os.path.join(assets_from, name), os.path.join(paths.live, name)
            wanted = (name in ASSET_DIRS and os.path.isdir(src)) or (name.endswith(ASSET_GLOBS) and os.path.isfile(src))
            if wanted and not os.path.exists(dst):
                copy_tree_reflink(src, dst, log)
                copied.append(name)
        report["assets"] = copied
    if not os.path.exists(paths.python):
        if seed_venv:
            copy_tree_reflink(os.path.abspath(seed_venv), paths.venv, log)
            report["venv"] = f"copied from {seed_venv}"
        else:
            code, _, err = run([sys.executable, "-m", "venv", paths.venv], limit=600, log=log, what="venv")
            if code != 0:
                raise OSError(f"python3 -m venv failed: {err.strip()}")
            report["venv"] = "new"
    if not sync_venv(paths, log):
        raise OSError("installing requirements.txt failed (see the log)")
    if not (build_fn or build)(paths, log):
        raise OSError("building the shell failed (see the log)")
    # The system python: the launcher is stdlib only, and must run before (and without) the venv.
    python = "/usr/bin/python3" if os.access("/usr/bin/python3", os.X_OK) else shutil.which("python3")
    entry = desktop_entry(python, os.path.join(paths.live, "desktop", "launcher.py"))
    os.makedirs(os.path.dirname(paths.desktop_file), exist_ok=True)
    with open(paths.desktop_file, "w", encoding="utf-8", newline="\n") as f:
        f.write(entry)
    os.makedirs(os.path.dirname(paths.icon), exist_ok=True)
    shutil.copyfile(os.path.join(paths.src, "icons", "128x128@2x.png"), paths.icon)
    report.update(desktop_file=paths.desktop_file, icon=paths.icon, exe=paths.exe)
    return report


def uninstall(paths):
    removed = []
    for p in (paths.desktop_file, paths.icon):
        try:
            os.unlink(p)
            removed.append(p)
        except FileNotFoundError:
            pass
    return {"removed": removed, "kept": paths.home}


def main(argv):
    ap = argparse.ArgumentParser(prog="launcher.py")
    sub = ap.add_subparsers(dest="cmd")
    i = sub.add_parser("install", help="set up the installed copy, its venv, the shell and the launcher entry")
    i.add_argument("--seed-venv", help="copy this venv instead of making a new one")
    i.add_argument("--assets-from", help="copy the git-ignored model files (weights/, *.pt) from this checkout")
    sub.add_parser("uninstall", help="remove the launcher entry and icon (keeps the data folder)")
    args = ap.parse_args(argv)
    if sys.platform != "linux":
        print("launcher: Linux only for now", file=sys.stderr)
        return 2
    paths = Paths()
    if args.cmd == "install":
        source = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        try:
            print(json.dumps(install(paths, source=source, seed_venv=args.seed_venv, assets_from=args.assets_from), indent=2))
        except OSError as e:
            print(f"install: {e}", file=sys.stderr)
            return 1
        return 0
    if args.cmd == "uninstall":
        print(json.dumps(uninstall(paths), indent=2))
        return 0
    return launch(paths)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
