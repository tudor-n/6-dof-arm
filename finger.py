"""finger.py - finger follow: is the hand pointing, where at, and where should the arm be.

Knows nothing about the arm. Everything here is in table coordinates (mm, arm frame), and
server.py turns a target into joint angles. The pieces:
  classify_pointing  pure test on MediaPipe world landmarks: index straight, the rest curled
  OneEuro            the standard One Euro low-pass, per axis
  FollowController   pure state machine: debounce, filtering, deadband, jump rejection, hold
  HandTracker        thread running MediaPipe's hand landmarker on vision.frame
Needs: pip install mediapipe   (only for HandTracker; the rest is plain Python)
"""
import importlib.util, math, os, threading, time, urllib.request
from collections import deque
from dataclasses import dataclass
from pathlib import Path
import cv2

# MediaPipe hand landmark indices
WRIST = 0
INDEX = (5, 6, 7, 8)                                  # MCP, PIP, DIP, TIP
CURLERS = ((9, 10, 11, 12), (13, 14, 15, 16), (17, 18, 19, 20))   # middle, ring, pinky

STRAIGHT_DEG = 150.0     # index PIP and DIP must both be straighter than this
EXTEND_RATIO = 1.15      # |wrist->tip| over |wrist->PIP| for an extended index
CURL_DEG = 120.0         # a PIP bent past this counts as curled

MODEL_URL = "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task"
DETECT_W = 640           # the landmarker gains nothing from full res, and 1080p would cost the frame rate
TRACK_HZ = 30.0
ERR_EVERY_S = 5.0        # at most one error log line per this many seconds


@dataclass
class Observation:
    t: float                    # time.time() of the frame
    hand: bool = False          # any hand detected
    pointing: bool = False      # classify_pointing said yes for the chosen hand
    score: float = 0.0
    x: float = None             # fingertip on the table plane, mm
    y: float = None
    dx: float = None            # unit pointing direction on the table (PIP -> TIP)
    dy: float = None
    tip_px: tuple = None        # fingertip, full-res undistorted pixels
    lm_px: list = None          # all 21 landmarks in pixels, for the overlay
    lag: float = 0.0            # seconds from t (frame picked up) until the observation was ready


def _angle(a, b, c):
    """Angle at b, degrees, between b->a and b->c. 180 = straight."""
    u = [a[i] - b[i] for i in range(3)]; w = [c[i] - b[i] for i in range(3)]
    nu, nw = math.sqrt(sum(x * x for x in u)), math.sqrt(sum(x * x for x in w))
    if nu < 1e-9 or nw < 1e-9: return 180.0
    return math.degrees(math.acos(max(-1.0, min(1.0, sum(u[i] * w[i] for i in range(3)) / (nu * nw)))))


def _clip(x): return max(0.0, min(1.0, x))


def classify_pointing(world_lm):
    """(pointing, score) from 21 world landmarks in metres.

    World landmarks are 3D and centred on the hand, so this works from any camera angle,
    including the dorsal view from overhead. The thumb is ignored: people point with it
    tucked in or sticking out. Each condition gets a margin that is 0.5 right at its
    threshold, and the score is the weakest one, so a score above 0.5 means every
    condition holds with room to spare and the score still ranks non-pointing hands.
    """
    if world_lm is None or len(world_lm) < 21: return False, 0.0
    p = [tuple(float(c) for c in q[:3]) for q in world_lm]
    w = p[WRIST]
    dist = lambda i: math.dist(w, p[i])
    m, pip, dip, tip = INDEX
    a_pip, a_dip = _angle(p[m], p[pip], p[dip]), _angle(p[pip], p[dip], p[tip])
    ratio = dist(tip) / max(dist(pip), 1e-9)
    straight = min(a_pip, a_dip) > STRAIGHT_DEG
    extended = ratio > EXTEND_RATIO
    margins = [_clip(0.5 + (min(a_pip, a_dip) - STRAIGHT_DEG) / 60.0),
               _clip(0.5 + (ratio - EXTEND_RATIO) / 0.3)]
    curled = True
    for fm, fp, fd, ft in CURLERS:
        dp, dt = dist(fp), dist(ft)
        a = _angle(p[fm], p[fp], p[fd])
        curled = curled and (dt < dp or a < CURL_DEG)
        margins.append(max(_clip(0.5 + (dp - dt) / max(dp, 1e-9)), _clip(0.5 + (CURL_DEG - a) / 120.0)))
    return straight and extended and curled, min(margins)


class OneEuro:
    """One Euro filter (Casiez et al. 2012): a low-pass whose cutoff rises with speed.

    Slow movement (tremor) gets the low min_cutoff and is smoothed hard; fast movement
    raises the cutoff by beta * |speed| so the output doesn't lag behind a real move.
    beta is in 1 / (units per second), so its scale depends on the units fed in.
    """
    def __init__(self, min_cutoff=1.0, beta=0.02, d_cutoff=1.0):
        self.min_cutoff, self.beta, self.d_cutoff = min_cutoff, beta, d_cutoff
        self.reset()

    def reset(self):
        self.x = self.dx = self.t = None

    @staticmethod
    def _alpha(cutoff, dt):
        tau = 1.0 / (2 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def __call__(self, x, t):
        if self.t is None:
            self.x, self.dx, self.t = float(x), 0.0, t
            return self.x
        dt = t - self.t
        if dt <= 0: return self.x          # duplicate or out-of-order sample: nothing to learn
        a = self._alpha(self.d_cutoff, dt)
        self.dx += a * ((x - self.x) / dt - self.dx)
        a = self._alpha(self.min_cutoff + self.beta * abs(self.dx), dt)
        self.x += a * (x - self.x); self.t = t
        return self.x


@dataclass
class FollowParams:
    engage_s: float = 0.15      # pointing must last this long before the arm moves
    lost_s: float = 0.12        # no pointing frame for this long = let go
    rewind_s: float = 0.15      # on letting go, hold where the target was this long before the last good frame
    jump_mm: float = 80.0       # a frame this far from the filtered position is suspect...
    agree_mm: float = 25.0      # ...until a second frame lands within this of it
    deadband_mm: float = 3.0    # output only moves when the filtered target moved this much
    lead_mm: float = 30.0       # aim this far ahead of the fingertip, along the finger
    min_cutoff: float = 1.0
    beta: float = 0.02
    d_cutoff: float = 1.0


@dataclass
class FollowOut:
    target: tuple               # (x, y) the arm should be over, or None
    changed: bool               # differs from the previous step's target
    state: str                  # idle | arming | tracking | holding
    status: str                 # short text for the UI


class FollowController:
    """Turns a stream of Observations into a steady target for the arm.

    feed() runs on the tracker thread at camera rate, step() on the server's 50 Hz loop,
    so both take the lock. Timeouts are measured against step()'s clock, which is what
    makes the arm hold even when frames stop arriving altogether.
    """
    HIST_S = 1.0                # how much filtered history to keep for the release rewind

    def __init__(self, params=None):
        self.p = params or FollowParams()
        self.lock = threading.Lock()
        self.reset()

    def reset(self, now=None):
        """Back to idle. With now, frames picked up before it are dropped: the tracker may be
        mid-detection on a frame from before the reset (a quick off/on, a pose that just went
        bad) and that frame must not seed the fresh session."""
        with self.lock:
            self.since = now
            self.lag = 0.0              # smoothed detection latency, see step()
            self.state = "idle"
            self.target = None          # what the arm is told; None until first engaged
            self.prev_out = None        # target as of the previous step, for FollowOut.changed
            self.last_t = None          # newest observation time seen (drops duplicates)
            self.last_hand_t = None     # last frame with any hand in it, for the status text
            self._reseed()

    def _reseed(self):
        # Fresh filters for every engagement: a hand that comes back somewhere else must not
        # be dragged in from where it left, and the old speed estimate means nothing now.
        p = self.p
        self.fx, self.fy, self.fdx, self.fdy = (OneEuro(p.min_cutoff, p.beta, p.d_cutoff) for _ in range(4))
        self.pos = None                 # filtered fingertip (x, y)
        self.dir = None                 # filtered unit direction, None if never seen
        self.hist = deque()             # (t, x, y, dir) per accepted frame
        self.pending = None             # a suspect far-away frame waiting for a second opinion
        self.arm_t0 = None              # first frame of the current pointing streak
        self.good_t = None              # last accepted pointing frame

    def _rest(self):
        return "holding" if self.target is not None else "idle"

    def feed(self, obs):
        with self.lock:
            if obs is None or (self.last_t is not None and obs.t <= self.last_t): return
            if self.since is not None and obs.t < self.since: return
            self.last_t = obs.t
            self.lag += 0.3 * (min(max(float(obs.lag or 0.0), 0.0), 0.5) - self.lag)
            if obs.hand: self.last_hand_t = obs.t
            good = obs.pointing and obs.x is not None and obs.y is not None
            if not good:
                # Arming needs an unbroken streak; while tracking, a bad frame does nothing
                # by itself and step() lets go once lost_s passes without a good one.
                if self.state == "arming": self.state = self._rest()
                return
            if self.state in ("idle", "holding"):
                self._reseed()
                self.state, self.arm_t0 = "arming", obs.t
            if not self._accept(obs): return
            self.good_t = obs.t
            if self.state == "arming" and obs.t - self.arm_t0 >= self.p.engage_s:
                self.state = "tracking"

    def _accept(self, obs):
        """Jump rejection, then the filters. False if the frame was held back."""
        x, y = float(obs.x), float(obs.y)
        if self.pos is not None and math.hypot(x - self.pos[0], y - self.pos[1]) > self.p.jump_mm:
            pd = self.pending
            if pd is None or math.hypot(x - pd[0], y - pd[1]) > self.p.agree_mm:
                self.pending = (x, y)       # one wild detection must not fling the arm
                return False
        self.pending = None
        self.pos = (self.fx(x, obs.t), self.fy(y, obs.t))
        if obs.dx is not None and obs.dy is not None:
            dx, dy = self.fdx(obs.dx, obs.t), self.fdy(obs.dy, obs.t)
            n = math.hypot(dx, dy)
            if n > 1e-6: self.dir = (dx / n, dy / n)
        self.hist.append((obs.t, self.pos[0], self.pos[1], self.dir))
        while self.hist and self.hist[0][0] < obs.t - self.HIST_S: self.hist.popleft()
        return True

    def _aim(self, x, y, d):
        # Aim ahead of the fingertip so the arm doesn't park on top of it and hide it from the camera.
        if d is None: return (x, y)
        return (x + self.p.lead_mm * d[0], y + self.p.lead_mm * d[1])

    def step(self, now):
        with self.lock:
            p = self.p
            # t is when the frame was picked up, so by the time it arrives it is already `lag`
            # old: a slow detector (70 ms a frame) would otherwise look like a lost finger
            # between every pair of frames and never get past arming.
            lost = p.lost_s + self.lag
            if self.state in ("arming", "tracking") and (self.good_t is None or now - self.good_t > lost):
                if self.state == "tracking" and self.hist:
                    # Curling the finger moves the fingertip before the classifier notices, so
                    # the last good frames are already dragged. Go back to before that started.
                    cut = self.good_t - p.rewind_s
                    h = next((e for e in reversed(self.hist) if e[0] <= cut), self.hist[0])
                    self.target = self._aim(*h[1:])
                self.state = self._rest()
            if self.state == "tracking" and self.hist:
                _, x, y, d = self.hist[-1]
                aim = self._aim(x, y, d)
                if self.target is None or math.hypot(aim[0] - self.target[0], aim[1] - self.target[1]) > p.deadband_mm:
                    self.target = aim
            changed = self.target != self.prev_out
            self.prev_out = self.target
            if self.state == "holding":
                seen = self.last_hand_t is not None and now - self.last_hand_t <= lost
                status = "holding: not pointing" if seen else "holding: finger lost"
            else:
                status = {"idle": "show a pointing finger", "arming": "arming", "tracking": "tracking"}[self.state]
            return FollowOut(self.target, changed, self.state, status)


class HandTracker:
    """MediaPipe hand landmarker on the newest camera frame, in a daemon thread.

    Only works while running (follow is on); stop() just parks the thread so the model
    stays loaded for next time. MediaPipe and the model load lazily inside the thread,
    because the import alone takes a while and the model may need downloading.
    """
    def __init__(self, vision, log, here):
        self.v, self.log, self.here = vision, log, Path(here)
        self.model = self.here / "weights" / "hand_landmarker.task"
        self.latest = None
        self.running = False
        self.error = ""             # '' when fine, else why the landmarker could not load
        self.on_obs = None          # called with every Observation (the server points it at controller.feed)
        self._thread = None; self._lm = None; self._ts = 0
        self._err_t = 0.0; self._errs = 0

    def available(self):
        # find_spec, not import: this runs on the server's event loop and mediapipe takes ~0.5 s to import
        if importlib.util.find_spec("mediapipe") is None:
            return False, "mediapipe is not installed (pip install mediapipe)"
        return True, ""

    def start(self):
        self.error = ""             # a failed load gets another try
        self.running = True
        if self._thread is None or not self._thread.is_alive():
            self._thread = threading.Thread(target=self._loop, daemon=True); self._thread.start()

    def stop(self):
        self.running = False; self.latest = None

    def _fail(self, msg):
        self._errs += 1
        if time.time() - self._err_t > ERR_EVERY_S:
            self.log(msg + (f" ({self._errs}x)" if self._errs > 1 else ""), "err")
            self._err_t = time.time(); self._errs = 0

    def _download(self):
        self.model.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.model.with_suffix(".part")
        self.log(f"Downloading the hand model to {self.model} ...")
        try:
            with urllib.request.urlopen(MODEL_URL, timeout=30) as r, open(tmp, "wb") as f:
                while True:
                    b = r.read(1 << 16)
                    if not b: break
                    f.write(b)
            os.replace(tmp, self.model)   # never leave a half-written model where the loader looks
        finally:
            tmp.unlink(missing_ok=True)
        self.log(f"Hand model downloaded ({self.model.stat().st_size // 1024} KB).", "ok")

    def _load(self):
        from mediapipe.tasks.python import BaseOptions, vision as mv
        if not self.model.exists(): self._download()
        opts = mv.HandLandmarkerOptions(base_options=BaseOptions(model_asset_path=str(self.model)),
                                        running_mode=mv.RunningMode.VIDEO, num_hands=2)
        self._lm = mv.HandLandmarker.create_from_options(opts)
        self.log("Hand tracker ready.", "ok")

    def _loop(self):
        last = None
        while True:
            if not self.running or self.error:
                if self.latest is not None: self.latest = None
                time.sleep(0.1); continue
            if self._lm is None:
                try:
                    self.log("Loading the hand tracker (MediaPipe)...")
                    self._load()
                except Exception as e:
                    self.error = f"hand tracker unavailable: {type(e).__name__}: {e}"
                    self.log(self.error, "err")
                continue
            frame = self.v.frame
            if frame is None or frame is last:      # the capture thread replaces it with a new array per frame
                time.sleep(0.005); continue
            last, t0 = frame, time.time()
            try:
                obs = self.process(frame, t0)
            except Exception as e:
                self._fail(f"hand tracker error: {type(e).__name__}: {e}")
                obs = Observation(t0)
            if not self.running: continue           # stopped mid-frame: publish nothing
            obs.lag = time.time() - t0
            self.latest = obs
            if self.on_obs is not None:
                try: self.on_obs(obs)
                except Exception as e: self._fail(f"follow error: {type(e).__name__}: {e}")
            time.sleep(max(0.0, 1.0 / TRACK_HZ - (time.time() - t0)))

    def _detect(self, rgb):
        import mediapipe as mp
        # VIDEO mode tracks across frames and insists on strictly increasing timestamps
        self._ts = max(self._ts + 1, int(time.monotonic() * 1000))
        return self._lm.detect_for_video(mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), self._ts)

    def process(self, frame, t):
        """One frame -> Observation. Landmarks come back normalised, so they map straight to full res."""
        h, w = frame.shape[:2]
        small = cv2.resize(frame, (DETECT_W, round(h * DETECT_W / w)), interpolation=cv2.INTER_AREA) if w > DETECT_W else frame
        res = self._detect(cv2.cvtColor(small, cv2.COLOR_BGR2RGB))
        if not res.hand_landmarks: return Observation(t)
        best = None
        for lm, wl in zip(res.hand_landmarks, res.hand_world_landmarks):
            ok, score = classify_pointing([(q.x, q.y, q.z) for q in wl])
            key = (ok, score)                   # any pointing hand beats any non-pointing one
            if best is None or key > best[0]: best = (key, lm)
        (ok, score), lm = best
        px = [(q.x * w, q.y * h) for q in lm]
        o = Observation(t, hand=True, pointing=ok, score=score, tip_px=px[8], lm_px=px)
        fz = float(self.v.s.get("follow_finger_z", 15.0))
        tip = self.v.pixel_to_table(*px[8], z_off=fz)          # None without a camera pose
        if tip is None: return o
        o.x, o.y = tip
        pip = self.v.pixel_to_table(*px[6], z_off=fz)
        if pip is not None:
            dx, dy = tip[0] - pip[0], tip[1] - pip[1]; n = math.hypot(dx, dy)
            if n > 1e-6: o.dx, o.dy = dx / n, dy / n
        return o
