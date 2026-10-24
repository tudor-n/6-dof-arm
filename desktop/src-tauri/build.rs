fn main() {
    // The start page's one command, `start` (try again / open without the arm). Only the local
    // start page gets it (capabilities/start.json); the arm UI, served by server.py, gets no IPC.
    let attributes = tauri_build::Attributes::new().app_manifest(tauri_build::AppManifest::new().commands(&["start"]));
    println!("cargo:rerun-if-changed=icons");
    if let Err(e) = tauri_build::try_build(attributes) {
        panic!("tauri build step failed: {e:#}");
    }
}
