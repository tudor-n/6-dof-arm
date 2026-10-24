# DUMMY: the desktop app (Linux)

A window around `server.py`'s own UI, so the arm starts from the KDE launcher instead of a
terminal, and an installed copy that keeps itself on master.

## Install (once)
From any checkout (a dev clone is fine), with the system python:
```
python3 -I -S desktop/launcher.py install --seed-venv .venv --assets-from .
```
- clones this repo's `origin` into `~/.local/share/dummy/app` on branch master (the **installed
  copy**: the app runs this, never edit it by hand);
- `--assets-from .` copies the git-ignored model files (`weights/`, `*.pt`) so Detect and Follow
  work offline at once;
- `--seed-venv .venv` copies your venv to `~/.local/share/dummy/venv` (reflinked on btrfs: free,
  instant). Without it a new venv gets `requirements.txt`, a big download;
- builds the shell into `~/.cache/dummy-target` (Rust and the Tauri libraries: see
  `app/desktop/README.md` in ATLAS; the first build takes a few minutes);
- writes `~/.local/share/applications/dev.dummy.desktop.desktop` and its icon. **DUMMY** is then
  in the launcher (run `kbuildsycoca6` if it doesn't show up).

`python3 -I -S desktop/launcher.py uninstall` removes the entry and icon (the data folder stays).

## What a start does
Each start from the launcher runs `~/.local/share/dummy/app/desktop/launcher.py`:
1. If DUMMY already runs, its window comes to the front. Nothing else.
2. **Update:** `git fetch origin master`, then a fast-forward of the installed copy. The installed
   copy is also where the arm's calibration is saved (the server writes `config.json`,
   `vision.json` and friends), so local changes survive:
   - a file master didn't change keeps its local edits;
   - a JSON file both sides changed is merged key by key; when both changed the *same* key,
     master's value wins (a committed calibration is deliberate) and a notification says so;
   - any other file both sides changed takes master's version;
   - every local version that gets replaced or merged is copied to
     `~/.local/share/dummy/backups/<time>/` first.
   A copy with commits of its own, or off master, isn't touched. Offline, it starts what it has.
3. **Python:** when `requirements.txt` changed, it is installed into the venv (uv, else pip).
4. **Build:** when the commit changed, `cargo build --release --locked` (only what changed is
   rebuilt), with an "Updating DUMMY" notification.
5. **Start** the window (`dummy-desktop --app … --python … --log …`).

So: merge a PR or push to master, close DUMMY, start it again, and it runs the new master.

## The window
- It starts `server.py` (auto-detecting the ESP32) and shows the arm UI once it answers. A
  server already running on port 8080 (one started in a terminal) is used as is, and left
  running when the window closes.
- No arm plugged in, port busy, no permission: it says which, shows the server's last lines,
  and offers **TRY AGAIN** and **OPEN WITHOUT THE ARM** (`--sim`).
- **Closing the window stops the server it started** (SIGTERM, SIGKILL after 5 s). The tablet's
  arm UI goes with it, so keep the window open (minimised is fine) while using the tablet.
- Serial permission: when you are in `dialout` but the desktop session started before you were
  added, the server is started through `sg dialout` (no password). Logging out and back in once
  fixes it for good.
- The window remembers its size and position. KDE groups it with the launcher entry
  (Wayland app id `dev.dummy.desktop`).

## Files
| Path | What |
|---|---|
| `~/.local/share/dummy/app/` | the installed copy (master) |
| `~/.local/share/dummy/venv/` | its Python |
| `~/.local/share/dummy/server.log` | everything `server.py` printed, plus the window's own lines |
| `~/.local/share/dummy/launcher.log` | updates, merges, builds |
| `~/.local/share/dummy/backups/` | local versions replaced or merged by an update |
| `~/.cache/dummy-target/release/dummy-desktop` | the shell |

## Develop
- `python3 desktop/test_launcher.py` (offline, temp folders only).
- In `desktop/src-tauri`: `CARGO_TARGET_DIR=~/.cache/dummy-target-dev cargo test` and
  `cargo clippy --all-targets -- -D warnings`. Never build into `src-tauri/target`.
- Try a build by hand: `dummy-desktop --app <checkout> --python <venv>/bin/python --log /tmp/x.log --port 8182`.
