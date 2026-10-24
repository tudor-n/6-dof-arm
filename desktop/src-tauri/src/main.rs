//! DUMMY: the arm's desktop app. A window around `server.py`'s own UI, so the arm starts from the
//! launcher instead of a terminal.
//!
//!     dummy-desktop --app <repo> --python <venv python> --log <server.log> [--port 8080]
//!
//! `desktop/launcher.py` passes those (after it has updated the repo); run by hand it works the
//! same way. The window opens on the local start page (`../dist/index.html`, "STARTING THE ARM
//! SERVER…"). Then:
//! - a server already answering on the port (one started in a terminal) is used as it is, and left
//!   running at the end;
//! - otherwise `<python> -u server.py [--sim]` starts in `<repo>`, its output appended to the log,
//!   and once it answers the window goes to `http://127.0.0.1:<port>/`. A server that exits (no
//!   arm plugged in, the port busy) takes the window back to the start page, which says why and
//!   offers TRY AGAIN and OPEN WITHOUT THE ARM (`--sim`): the one command, `start`, which only the
//!   start page may call (capabilities/start.json).
//! - Closing the window (or SIGTERM, or Ctrl+C) stops the server it started: SIGTERM to its process
//!   group, SIGKILL 5 s later.
//!
//! Serial access: a desktop session started before the user joined `dialout` can't open the ESP32's
//! port. The server is then started through `sg dialout` (no password: the user is a member).
//! Linux is the target; the code builds elsewhere but only Linux is tried.

#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use std::fs::{File, OpenOptions};
use std::io::{Read, Seek, SeekFrom, Write};
use std::net::{SocketAddr, TcpStream};
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use tauri::webview::PageLoadEvent;
use tauri::{AppHandle, Manager, RunEvent, Url, WebviewUrl, WebviewWindow, WebviewWindowBuilder};

const MAIN: &str = "main";
const APP_ID: &str = "dev.dummy.desktop";
const STOP_GRACE: Duration = Duration::from_secs(5);
const LOG_MAX: u64 = 4 << 20;

#[derive(Debug, Clone)]
struct Cfg {
    app: PathBuf,
    python: PathBuf,
    log: PathBuf,
    port: u16,
}

fn parse_args(args: &[String]) -> Result<Cfg, String> {
    let (mut app, mut python, mut log, mut port) = (None, None, None, 8080u16);
    let mut it = args.iter();
    while let Some(a) = it.next() {
        let mut val = || it.next().cloned().ok_or(format!("{a} needs a value"));
        match a.as_str() {
            "--app" => app = Some(PathBuf::from(val()?)),
            "--python" => python = Some(PathBuf::from(val()?)),
            "--log" => log = Some(PathBuf::from(val()?)),
            "--port" => port = val()?.parse().map_err(|_| "--port needs a number".to_string())?,
            _ => {} // single-instance hands a second launch's args over; anything else is ignored
        }
    }
    let app = app.ok_or("--app <repo> is required")?;
    let python = python.ok_or("--python <venv python> is required")?;
    let log = log.unwrap_or_else(|| app.join("dummy-server.log"));
    Ok(Cfg { app, python, log, port })
}

/// Why a server exited, from what it printed (the start page words it).
fn failure_kind(output: &str) -> &'static str {
    if output.contains("No ESP32 found") {
        "noarm"
    } else if output.contains("Permission denied") {
        "perm"
    } else if output.contains("Cannot open") {
        "busy"
    } else if output.contains("address already in use") || output.contains("Address already in use") {
        "port"
    } else {
        "exit"
    }
}

/// The last `n` lines of `text`, at most `max` bytes (cut at a char boundary).
fn tail(text: &str, n: usize, max: usize) -> String {
    let lines: Vec<&str> = text.lines().filter(|l| !l.trim().is_empty()).collect();
    let mut out = lines[lines.len().saturating_sub(n)..].join("\n");
    if out.len() > max {
        let mut cut = out.len() - max;
        while !out.is_char_boundary(cut) {
            cut += 1;
        }
        out = out[cut..].to_string();
    }
    out
}

/// Is `gid` among this process's groups (`/proc/self/status`)?
fn have_group(status: &str, gid: u32) -> bool {
    status
        .lines()
        .find_map(|l| l.strip_prefix("Groups:"))
        .is_some_and(|g| g.split_whitespace().any(|v| v.parse() == Ok(gid)))
}

/// `dialout`'s gid when `user` is a member in `/etc/group` (the text given).
fn member_gid(group_file: &str, group: &str, user: &str) -> Option<u32> {
    group_file.lines().find_map(|l| {
        let f: Vec<&str> = l.split(':').collect();
        (f.len() == 4 && f[0] == group && f[3].split(',').any(|m| m == user)).then(|| f[2].parse().ok()).flatten()
    })
}

/// True when the server must go through `sg dialout`: the user is in `dialout` but this session
/// started before they joined it.
fn needs_sg() -> bool {
    let user = std::env::var("USER").or_else(|_| std::env::var("LOGNAME")).unwrap_or_default();
    let groups = std::fs::read_to_string("/etc/group").unwrap_or_default();
    let status = std::fs::read_to_string("/proc/self/status").unwrap_or_default();
    match member_gid(&groups, "dialout", &user) {
        Some(gid) => !have_group(&status, gid) && Path::new("/usr/bin/sg").exists(),
        None => false,
    }
}

/// One shell word for `sg -c`.
fn sh_quote(s: &str) -> String {
    format!("'{}'", s.replace('\'', "'\\''"))
}

fn server_command(cfg: &Cfg, sim: bool, sg: bool) -> Command {
    let mut args: Vec<String> = vec!["-u".into(), "server.py".into(), "--http".into(), cfg.port.to_string()];
    if sim {
        args.push("--sim".into());
    }
    let mut cmd = if sg {
        let line = std::iter::once(cfg.python.to_string_lossy().into_owned())
            .chain(args)
            .map(|a| sh_quote(&a))
            .collect::<Vec<_>>()
            .join(" ");
        let mut c = Command::new("/usr/bin/sg");
        c.args(["dialout", "-c", &format!("exec {line}")]);
        c
    } else {
        let mut c = Command::new(&cfg.python);
        c.args(args);
        c
    };
    cmd.current_dir(&cfg.app).env("PYTHONUNBUFFERED", "1").stdin(Stdio::null());
    #[cfg(unix)]
    {
        use std::os::unix::process::CommandExt;
        cmd.process_group(0);
    }
    cmd
}

/// Does the arm's server answer on the port? (`/vocab.json` is its own, small and always there.)
fn arm_up(port: u16) -> bool {
    let addr = SocketAddr::from(([127, 0, 0, 1], port));
    let Ok(mut s) = TcpStream::connect_timeout(&addr, Duration::from_millis(400)) else { return false };
    let _ = s.set_read_timeout(Some(Duration::from_millis(1500)));
    let req = format!("GET /vocab.json HTTP/1.0\r\nHost: 127.0.0.1:{port}\r\n\r\n");
    if s.write_all(req.as_bytes()).is_err() {
        return false;
    }
    let mut buf = Vec::new();
    let _ = s.take(64 * 1024).read_to_end(&mut buf);
    let text = String::from_utf8_lossy(&buf);
    text.starts_with("HTTP/1.") && text.contains(" 200 ") && text.contains("\"vocab\"")
}

/// Whatever the server started in its process group, once the server itself has exited.
fn kill_leftovers(pid: u32) {
    #[cfg(unix)]
    unsafe {
        libc::killpg(pid as libc::pid_t, libc::SIGTERM);
    }
    #[cfg(not(unix))]
    let _ = pid;
}

/// The server child the app started (never one it found running).
#[derive(Default)]
struct Server {
    child: Option<Child>,
}

impl Server {
    fn stop(&mut self) {
        let Some(mut child) = self.child.take() else { return };
        if let Ok(Some(_)) = child.try_wait() {
            kill_leftovers(child.id());
            return;
        }
        #[cfg(unix)]
        unsafe {
            libc::killpg(child.id() as libc::pid_t, libc::SIGTERM);
        }
        #[cfg(not(unix))]
        let _ = child.kill();
        let deadline = Instant::now() + STOP_GRACE;
        while Instant::now() < deadline {
            if let Ok(Some(_)) = child.try_wait() {
                return;
            }
            std::thread::sleep(Duration::from_millis(100));
        }
        #[cfg(unix)]
        unsafe {
            libc::killpg(child.id() as libc::pid_t, libc::SIGKILL);
        }
        let _ = child.kill();
        let _ = child.wait();
    }
}

struct State {
    cfg: Cfg,
    server: Mutex<Server>,
    /// Each start bumps it; a watcher whose number is no longer current stops.
    generation: AtomicU64,
    /// The start page's own URL (`tauri://localhost/` on Linux), taken from the first page load.
    start_url: Mutex<Option<Url>>,
}

fn log_line(cfg: &Cfg, msg: &str) {
    if let Ok(mut f) = OpenOptions::new().create(true).append(true).open(&cfg.log) {
        let _ = writeln!(f, "[dummy-desktop] {msg}");
    }
}

fn read_from(path: &Path, offset: u64) -> String {
    let mut text = String::new();
    if let Ok(mut f) = File::open(path) {
        if f.seek(SeekFrom::Start(offset)).is_ok() {
            let mut bytes = Vec::new();
            let _ = f.read_to_end(&mut bytes);
            text = String::from_utf8_lossy(&bytes).into_owned();
        }
    }
    text
}

/// Back to the start page, in the state given (`starting`, or `failed` with a kind and the
/// server's last lines). The page reads it from its query, so there is no race with its load.
fn show_start(app: &AppHandle, query: &[(&str, &str)]) {
    let state = app.state::<Arc<State>>();
    let Some(w) = app.get_webview_window(MAIN) else { return };
    let base = state.start_url.lock().unwrap().clone().or_else(|| w.url().ok());
    let Some(mut url) = base else { return };
    url.set_fragment(None);
    url.query_pairs_mut().clear().extend_pairs(query);
    let _ = w.navigate(url);
}

fn show_arm(app: &AppHandle, port: u16) {
    if let (Some(w), Ok(url)) = (app.get_webview_window(MAIN), Url::parse(&format!("http://127.0.0.1:{port}/"))) {
        let _ = w.navigate(url);
    }
}

/// Start (or restart) the server and follow it, off the main thread.
fn begin(app: AppHandle, sim: bool) {
    let state = app.state::<Arc<State>>().inner().clone();
    let me = state.generation.fetch_add(1, Ordering::SeqCst) + 1;
    std::thread::spawn(move || {
        state.server.lock().unwrap().stop();
        let cfg = &state.cfg;
        show_start(&app, &[("state", "starting"), ("sim", if sim { "1" } else { "0" })]);
        if arm_up(cfg.port) {
            log_line(cfg, &format!("a server already answers on port {}: using it", cfg.port));
            show_arm(&app, cfg.port);
            return;
        }
        let offset = std::fs::metadata(&cfg.log).map(|m| m.len()).unwrap_or(0);
        let sg = !sim && needs_sg();
        log_line(cfg, &format!("starting {} server.py{}{} in {}", cfg.python.display(), if sim { " --sim" } else { "" },
                               if sg { " (through sg dialout)" } else { "" }, cfg.app.display()));
        // Spawned and stored under one hold of the lock, and only while this start is still the
        // current one: a newer start or the app's exit (which bumps the number first) can never
        // leave a server behind that nothing tracks.
        let spawned = {
            let mut srv = state.server.lock().unwrap();
            if state.generation.load(Ordering::SeqCst) != me {
                return;
            }
            srv.stop();
            let out = OpenOptions::new().create(true).append(true).open(&cfg.log);
            out.and_then(|out| {
                let err = out.try_clone()?;
                server_command(cfg, sim, sg).stdout(out).stderr(err).spawn()
            })
            .map(|child| srv.child = Some(child))
        };
        match spawned {
            Ok(()) => {}
            Err(e) => {
                log_line(cfg, &format!("could not start the server: {e}"));
                let msg = format!("Couldn't start {}: {e}", cfg.python.display());
                show_start(&app, &[("state", "failed"), ("kind", "exit"), ("log", &msg)]);
                return;
            }
        }
        let mut shown = false;
        while state.generation.load(Ordering::SeqCst) == me {
            let exited = {
                let mut srv = state.server.lock().unwrap();
                match srv.child.as_mut().map(|c| c.try_wait()) {
                    Some(Ok(Some(status))) => {
                        if let Some(c) = srv.child.take() {
                            kill_leftovers(c.id());
                        }
                        Some(status.to_string())
                    }
                    Some(Ok(None)) => None,
                    _ => return, // stopped by someone else (the exit path, or a newer start)
                }
            };
            if let Some(status) = exited {
                log_line(cfg, &format!("server.py ended: {status}"));
                let out = read_from(&cfg.log, offset);
                let kind = if shown { "stopped" } else { failure_kind(&out) };
                show_start(&app, &[("state", "failed"), ("kind", kind), ("log", &tail(&out, 14, 1800))]);
                return;
            }
            if !shown && arm_up(cfg.port) {
                shown = true;
                show_arm(&app, cfg.port);
            }
            std::thread::sleep(Duration::from_millis(if shown { 1000 } else { 250 }));
        }
    });
}

#[tauri::command]
fn start(app: AppHandle, sim: bool) {
    begin(app, sim);
}

/// SIGTERM (logout) and SIGINT (Ctrl+C) quit the way closing the window does, so the server is
/// stopped too. The handler only writes a byte to a pipe; a thread reads it and asks the app to exit.
#[cfg(unix)]
fn quit_on_signals(app: AppHandle) {
    use std::os::fd::FromRawFd;
    use std::sync::atomic::AtomicI32;
    static WAKE_FD: AtomicI32 = AtomicI32::new(-1);
    extern "C" fn on_signal(_: libc::c_int) {
        let _ = unsafe { libc::write(WAKE_FD.load(Ordering::Relaxed), b"q".as_ptr().cast(), 1) };
    }
    let mut fds: [libc::c_int; 2] = [-1; 2];
    if unsafe { libc::pipe2(fds.as_mut_ptr(), libc::O_CLOEXEC) } != 0
        || unsafe { libc::fcntl(fds[1], libc::F_SETFL, libc::O_NONBLOCK) } == -1
    {
        return;
    }
    WAKE_FD.store(fds[1], Ordering::Relaxed);
    let mut wake = unsafe { File::from_raw_fd(fds[0]) };
    std::thread::spawn(move || {
        let mut byte = [0u8; 1];
        loop {
            match wake.read(&mut byte) {
                Ok(1) => break,
                Err(e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
                _ => return,
            }
        }
        app.exit(0);
    });
    for sig in [libc::SIGTERM, libc::SIGINT] {
        unsafe {
            libc::signal(sig, on_signal as extern "C" fn(libc::c_int) as libc::sighandler_t);
        }
    }
}

fn trim_log(path: &Path) {
    if std::fs::metadata(path).map(|m| m.len() > LOG_MAX).unwrap_or(false) {
        let text = read_from(path, std::fs::metadata(path).map(|m| m.len() - LOG_MAX / 2).unwrap_or(0));
        let _ = std::fs::write(path, text.split_once('\n').map(|(_, rest)| rest).unwrap_or(""));
    }
}

fn main_window(app: &AppHandle) -> tauri::Result<WebviewWindow> {
    WebviewWindowBuilder::new(app, MAIN, WebviewUrl::App("index.html".into()))
        .title("DUMMY")
        // 1024x600 at 1.25: the arm UI scales its frame to fit, so this shows it edge to edge.
        .inner_size(1280.0, 750.0)
        .min_inner_size(640.0, 375.0)
        .center()
        .theme(Some(tauri::Theme::Dark))
        .background_color(tauri::window::Color(0, 0, 0, 255))
        .on_page_load(|w, p| {
            if p.event() == PageLoadEvent::Finished && p.url().scheme() != "http" {
                let state = w.app_handle().state::<Arc<State>>();
                let mut start = state.start_url.lock().unwrap();
                if start.is_none() {
                    let mut url = p.url().clone();
                    url.set_query(None);
                    *start = Some(url);
                }
            }
        })
        .build()
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let cfg = match parse_args(&args) {
        Ok(cfg) => cfg,
        Err(e) => {
            eprintln!("dummy-desktop: {e}\nusage: dummy-desktop --app <repo> --python <venv python> [--log <file>] [--port 8080]");
            std::process::exit(2);
        }
    };
    // The Wayland app id (and the X11 class): KDE matches it to dev.dummy.desktop.desktop, so the
    // window gets the launcher's icon and groups with it. Before GTK starts.
    #[cfg(target_os = "linux")]
    gtk::glib::set_prgname(Some(APP_ID));
    if let Some(dir) = cfg.log.parent() {
        let _ = std::fs::create_dir_all(dir);
    }
    trim_log(&cfg.log);
    let state = Arc::new(State { cfg, server: Mutex::new(Server::default()), generation: AtomicU64::new(0),
                                 start_url: Mutex::new(None) });
    let for_exit = state.clone();
    let app = tauri::Builder::default()
        .plugin(tauri_plugin_single_instance::init(|app, _args, _cwd| {
            if let Some(w) = app.get_webview_window(MAIN) {
                let _ = w.unminimize();
                let _ = w.show();
                let _ = w.set_focus();
            }
        }))
        .plugin(tauri_plugin_window_state::Builder::default().build())
        .manage(state)
        .invoke_handler(tauri::generate_handler![start])
        .setup(|app| {
            main_window(app.handle())?;
            #[cfg(unix)]
            quit_on_signals(app.handle().clone());
            begin(app.handle().clone(), false);
            Ok(())
        })
        .build(tauri::generate_context!())
        .expect("dummy-desktop: could not start");
    app.run(move |_app, event| {
        if let RunEvent::Exit = event {
            for_exit.generation.fetch_add(1, Ordering::SeqCst);
            if for_exit.server.lock().unwrap().child.is_some() {
                log_line(&for_exit.cfg, "the window closed: stopping server.py");
            }
            for_exit.server.lock().unwrap().stop();
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    fn s(v: &[&str]) -> Vec<String> {
        v.iter().map(|x| x.to_string()).collect()
    }

    #[test]
    fn args() {
        let c = parse_args(&s(&["--app", "/r", "--python", "/v/bin/python", "--log", "/l/server.log"])).unwrap();
        assert_eq!((c.app, c.python, c.log, c.port), ("/r".into(), "/v/bin/python".into(), "/l/server.log".into(), 8080));
        let c = parse_args(&s(&["--app", "/r", "--python", "/p", "--port", "9000", "--open"])).unwrap();
        assert_eq!((c.port, c.log), (9000, PathBuf::from("/r/dummy-server.log")));
        assert!(parse_args(&s(&["--python", "/p"])).is_err());
        assert!(parse_args(&s(&["--app", "/r"])).is_err());
        assert!(parse_args(&s(&["--app", "/r", "--python", "/p", "--port", "x"])).is_err());
        assert!(parse_args(&s(&["--app"])).is_err());
    }

    #[test]
    fn kinds() {
        assert_eq!(failure_kind("Serial ports: []\nNo ESP32 found. Use --port COMx or --sim"), "noarm");
        assert_eq!(failure_kind("Cannot open /dev/ttyUSB0: [Errno 13] Permission denied: '/dev/ttyUSB0'"), "perm");
        assert_eq!(failure_kind("Cannot open /dev/ttyUSB0: [Errno 16] Device or resource busy"), "busy");
        assert_eq!(failure_kind("OSError: [Errno 98] error while attempting to bind: address already in use"), "port");
        assert_eq!(failure_kind("Traceback ..."), "exit");
    }

    #[test]
    fn tails() {
        assert_eq!(tail("a\n\nb\nc\nd\n", 2, 100), "c\nd");
        assert_eq!(tail("ééééé", 3, 4), "éé");
        assert_eq!(tail("", 3, 10), "");
    }

    #[test]
    fn groups() {
        let etc = "root:x:0:\ndialout:x:20:tudor,other\nplugdev:x:46:tudor\n";
        assert_eq!(member_gid(etc, "dialout", "tudor"), Some(20));
        assert_eq!(member_gid(etc, "dialout", "tud"), None);
        assert_eq!(member_gid("dialout:x:20:\n", "dialout", "tudor"), None);
        let status = "Name:\tx\nGroups:\t4 24 27 1000 \nNSpid:\t1\n";
        assert!(!have_group(status, 20) && have_group(status, 27) && have_group(status, 1000));
    }

    #[test]
    fn sg_command_line_is_quoted() {
        assert_eq!(sh_quote("/a b/it's"), "'/a b/it'\\''s'");
        let cfg = Cfg { app: "/r".into(), python: "/v x/python".into(), log: "/l".into(), port: 8080 };
        let cmd = server_command(&cfg, false, true);
        let args: Vec<_> = cmd.get_args().map(|a| a.to_string_lossy().into_owned()).collect();
        assert_eq!(cmd.get_program(), "/usr/bin/sg");
        assert_eq!(args, ["dialout", "-c", "exec '/v x/python' '-u' 'server.py' '--http' '8080'"]);
        let cmd = server_command(&cfg, true, false);
        let args: Vec<_> = cmd.get_args().map(|a| a.to_string_lossy().into_owned()).collect();
        assert_eq!((cmd.get_program().to_string_lossy().into_owned(), args),
                   ("/v x/python".to_string(), s(&["-u", "server.py", "--http", "8080", "--sim"])));
    }

    #[test]
    fn arm_up_is_false_with_nothing_listening() {
        let free = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let port = free.local_addr().unwrap().port();
        drop(free);
        assert!(!arm_up(port));
    }

    #[test]
    fn arm_up_needs_the_arm_servers_answer() {
        use std::net::TcpListener;
        for (body, want) in [("{\"vocab\": [\"cup\"]}", true), ("<html>other</html>", false)] {
            let l = TcpListener::bind("127.0.0.1:0").unwrap();
            let port = l.local_addr().unwrap().port();
            let body = body.to_string();
            let t = std::thread::spawn(move || {
                let (mut c, _) = l.accept().unwrap();
                let mut buf = [0u8; 1024];
                let _ = c.read(&mut buf);
                let _ = c.write_all(format!("HTTP/1.1 200 OK\r\nContent-Length: {}\r\n\r\n{body}", body.len()).as_bytes());
            });
            assert_eq!(arm_up(port), want);
            t.join().unwrap();
        }
    }
}
