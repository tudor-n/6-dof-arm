"""Offline finger-follow tests - no camera, no MediaPipe, no arm required.

Run with:  python test_finger.py        (or: pytest test_finger.py)

The classifier is fed synthetic hands posed from joint angles, so a case reads as "index
straight, the rest curled" rather than 63 numbers. The controller is driven by a small
simulator that interleaves 30 Hz camera frames with 50 Hz control steps on a fake clock.
"""
import math, random, threading
import numpy as np

import finger as F

LEAD = F.FollowParams().lead_mm


# --------------------------------------------------------------------------- helpers

STRAIGHT = (0, 0, 0)          # flexion at MCP, PIP, DIP in degrees; 0 = straight
CURLED = (85, 100, 60)
# (knuckle x, knuckle y, phalanx lengths) per finger, metres, an adult right hand palm-down
LAYOUT = {"index": (0.025, 0.085, (0.040, 0.025, 0.020)), "middle": (0.005, 0.088, (0.045, 0.028, 0.021)),
          "ring": (-0.015, 0.082, (0.042, 0.027, 0.020)), "pinky": (-0.032, 0.072, (0.033, 0.020, 0.018))}


def chain(base, flex, segs, spread=0.0):
    """A finger as 4 points: the base, then each joint and the tip, bending toward the palm (-z)."""
    pts = [np.array(base, float)]; phi = 0.0
    fwd = np.array([math.sin(math.radians(spread)), math.cos(math.radians(spread))])
    for f, L in zip(flex, segs):
        phi += math.radians(f)
        pts.append(pts[-1] + L * np.array([fwd[0] * math.cos(phi), fwd[1] * math.cos(phi), -math.sin(phi)]))
    return pts


def hand(index=STRAIGHT, middle=CURLED, ring=CURLED, pinky=CURLED, thumb=(20, 20, 10), R=None, scale=1.0):
    """21 MediaPipe-ordered world landmarks (metres) for a hand posed by joint flexion.

    Defaults to a pointing hand. R rotates the whole hand and scale resizes it, which is
    what a different camera angle or a different person looks like in world landmarks.
    """
    lm = [np.zeros(3)] + chain((0.020, 0.025, 0.0), thumb, (0.040, 0.032, 0.025), spread=50)
    for name, flex in (("index", index), ("middle", middle), ("ring", ring), ("pinky", pinky)):
        x, y, segs = LAYOUT[name]
        lm += chain((x, y, 0.0), flex, segs)
    R = np.eye(3) if R is None else R
    return [tuple(scale * (R @ p)) for p in lm]


def random_rotation(rng):
    q = np.array([rng.gauss(0, 1) for _ in range(4)]); q /= np.linalg.norm(q)
    a, b, c, d = q
    return np.array([[a*a + b*b - c*c - d*d, 2*(b*c - a*d), 2*(b*d + a*c)],
                     [2*(b*c + a*d), a*a - b*b + c*c - d*d, 2*(c*d - a*b)],
                     [2*(b*d - a*c), 2*(c*d + a*b), a*a - b*b - c*c + d*d]])


def ob(t, x=100.0, y=0.0, pointing=True, hand=True, d=(1.0, 0.0)):
    return F.Observation(t=t, hand=hand, pointing=pointing and hand, score=0.9 if pointing else 0.2,
                         x=x if hand else None, y=y if hand else None,
                         dx=d[0] if (hand and d) else None, dy=d[1] if (hand and d) else None)


def sim(c, t0, t1, frame=lambda t: ob(t), fps=30.0, hz=50.0):
    """Feed frames at fps and step at hz between t0 and t1. frame(t) -> Observation or None
    (None = no frame at all this tick). Returns [(t, FollowOut)] for every step."""
    ev = [(t0 + i / fps, 0) for i in range(int((t1 - t0) * fps) + 1)]
    ev += [(t0 + i / hz, 1) for i in range(int((t1 - t0) * hz) + 1)]
    outs = []
    for t, kind in sorted(ev):
        if t > t1: continue
        if kind == 0:
            o = frame(t)
            if o is not None: c.feed(o)
        else:
            outs.append((t, c.step(t)))
    return outs


def tracking_at(x=100.0, y=0.0, d=(1.0, 0.0), until=1.0, **kw):
    c = F.FollowController(F.FollowParams(**kw))
    outs = sim(c, 0.0, until, lambda t: ob(t, x, y, d=d))
    assert outs[-1][1].state == "tracking", outs[-1][1]
    return c, outs


def near(a, b, tol):
    return a is not None and math.hypot(a[0] - b[0], a[1] - b[1]) <= tol


# ------------------------------------------------------------------ classify_pointing

def test_pointing_hand_is_pointing():
    ok, score = F.classify_pointing(hand())
    assert ok and score > 0.5, (ok, score)


def test_pointing_survives_a_relaxed_index_and_loose_curl():
    ok, _ = F.classify_pointing(hand(index=(10, 15, 10), middle=(40, 80, 50), ring=(50, 90, 40), pinky=(60, 70, 40)))
    assert ok


def test_thumb_is_ignored():
    for th in ((0, 0, 0), (60, 60, 40), (-20, 0, 0)):
        assert F.classify_pointing(hand(thumb=th))[0], th


def test_pointing_does_not_depend_on_orientation_or_hand_size():
    rng = random.Random(3)
    for _ in range(50):
        ok, _ = F.classify_pointing(hand(R=random_rotation(rng), scale=rng.uniform(0.75, 1.3)))
        assert ok


def test_open_palm_is_not_pointing():
    ok, score = F.classify_pointing(hand(middle=STRAIGHT, ring=STRAIGHT, pinky=STRAIGHT))
    assert not ok and score < 0.5, score


def test_fist_is_not_pointing():
    ok, score = F.classify_pointing(hand(index=CURLED))
    assert not ok and score < 0.5, score


def test_v_sign_is_not_pointing():
    assert not F.classify_pointing(hand(middle=STRAIGHT))[0]


def test_bent_index_is_not_pointing():
    assert not F.classify_pointing(hand(index=(0, 50, 30)))[0]
    assert not F.classify_pointing(hand(index=(0, 0, 45)))[0]      # hooked at the DIP only


def test_pointing_scores_above_near_misses():
    s = F.classify_pointing(hand())[1]
    for other in (hand(middle=STRAIGHT), hand(index=(0, 50, 30)), hand(middle=STRAIGHT, ring=STRAIGHT, pinky=STRAIGHT)):
        assert F.classify_pointing(other)[1] < s


def test_classifier_rejects_garbage():
    assert F.classify_pointing(None) == (False, 0.0)
    assert F.classify_pointing(hand()[:20]) == (False, 0.0)


# ---------------------------------------------------------------------------- OneEuro

def test_one_euro_converges_on_a_step_without_overshoot():
    f = F.OneEuro(1.0, 0.02, 1.0); out = []
    for i in range(120):
        t = i / 30.0
        out.append(f(0.0 if t < 1.0 else 100.0, t))
    assert abs(out[-1] - 100.0) < 1.0, out[-1]
    assert max(out) <= 100.0 + 1e-9


def test_one_euro_attenuates_jitter():
    rng = random.Random(7); f = F.OneEuro(1.0, 0.02, 1.0)
    raw, out = [], []
    for i in range(300):
        x = 50.0 + rng.gauss(0, 2.0)
        y = f(x, i / 30.0)
        if i > 60: raw.append(x); out.append(y)
    assert np.std(out) < 0.4 * np.std(raw), (np.std(out), np.std(raw))
    assert abs(np.mean(out) - 50.0) < 0.5


def test_one_euro_follows_fast_motion_better_than_slow():
    # the speed term should raise the cutoff: the lag on a fast ramp is a smaller fraction of the move
    def lag(speed):
        f = F.OneEuro(1.0, 0.02, 1.0); y = 0.0
        for i in range(60): y = f(speed * i / 30.0, i / 30.0)
        return (speed * 59 / 30.0 - y) / speed
    assert lag(500.0) < 0.5 * lag(10.0)


def test_one_euro_reset_and_duplicate_samples():
    f = F.OneEuro()
    f(0.0, 0.0); f(10.0, 0.1)
    held = f.x
    assert f(500.0, 0.1) == held             # dt = 0: ignored
    f.reset()
    assert f(42.0, 5.0) == 42.0


# ------------------------------------------------------------------- FollowController

def test_idle_until_a_finger_is_shown():
    c = F.FollowController()
    o = c.step(0.0)
    assert o.state == "idle" and o.target is None and not o.changed and "pointing" in o.status


def test_a_single_frame_never_engages():
    c = F.FollowController()
    c.feed(ob(0.0))
    assert c.step(0.0).state == "arming"
    for t in (0.05, 0.1, 0.15, 0.2, 0.5):
        o = c.step(t)
        assert o.state != "tracking" and o.target is None, (t, o)
    assert o.state == "idle"                  # the lone frame timed out


def test_engages_after_engage_s_of_pointing():
    c = F.FollowController()
    outs = sim(c, 0.0, 0.5)
    first = next(t for t, o in outs if o.state == "tracking")
    assert 0.15 <= first < 0.2, first
    assert all(o.target is None for t, o in outs if t < first)
    o = next(o for t, o in outs if t == first)
    assert o.changed and near(o.target, (100 + LEAD, 0.0), 0.5), o


def test_a_broken_streak_does_not_engage():
    c = F.FollowController()
    # every fourth frame says "not pointing": no run of pointing frames spans 0.15 s
    outs = sim(c, 0.0, 1.0, lambda t: ob(t, pointing=round(t * 30) % 4 != 3))
    assert all(o.state != "tracking" for _, o in outs)


def test_holds_when_frames_stop_altogether():
    c, _ = tracking_at()
    held = c.step(1.0).target
    outs = sim(c, 1.0, 2.0, lambda t: None)
    late = [o for t, o in outs if t > 1.0 + 0.12 + 0.03]
    assert late and all(o.state == "holding" and o.target == held for o in late)
    assert late[0].status == "holding: finger lost"
    assert not any(o.changed for o in late[1:])


def test_holds_when_the_hand_stops_pointing():
    c, _ = tracking_at()
    outs = sim(c, 1.0, 1.5, lambda t: ob(t, pointing=False))
    o = outs[-1][1]
    assert o.state == "holding" and o.status == "holding: not pointing"


def test_release_rewinds_past_the_curl():
    # steady at x=100, then the curl drags the fingertip 40 mm in 0.1 s before the
    # classifier gives up: the arm should go back to where the finger was before that
    def frame(t):
        if t < 1.0: return ob(t, 100.0)
        if t <= 1.1: return ob(t, 100.0 + 400.0 * (t - 1.0))
        return ob(t, 140.0, pointing=False)
    c = F.FollowController()
    outs = sim(c, 0.0, 1.6, frame)
    before = [o.target for t, o in outs if o.state == "tracking"][-1]
    after = outs[-1][1]
    assert before[0] > 100 + LEAD + 10, before          # tracking really followed the drag
    assert after.state == "holding" and near(after.target, (100 + LEAD, 0.0), 2.0), after.target


def test_one_wild_frame_is_ignored():
    c, _ = tracking_at()
    outs = sim(c, 1.0, 2.0, lambda t: ob(t, 300.0) if abs(t - 1.5) < 0.01 else ob(t, 100.0))
    assert all(near(o.target, (100 + LEAD, 0.0), 1.0) for _, o in outs)


def test_a_real_jump_is_accepted_once_confirmed():
    c, _ = tracking_at()
    outs = sim(c, 1.0, 3.0, lambda t: ob(t, 100.0) if t < 1.5 else ob(t, 250.0))
    # within the deadband of the new spot: the filter's approach stops the output short by up to that
    assert near(outs[-1][1].target, (250 + LEAD, 0.0), c.p.deadband_mm), outs[-1][1].target


def test_disagreeing_wild_frames_never_get_through():
    c, _ = tracking_at()
    k = [0]
    def frame(t):        # every other frame is garbage, and no two pieces of garbage agree
        k[0] += 1
        return ob(t, 100.0) if k[0] % 2 else ob(t, 300.0 + 40.0 * (k[0] % 7), -200.0)
    outs = sim(c, 1.0, 2.0, frame)
    assert all(near(o.target, (100 + LEAD, 0.0), 1.0) for _, o in outs)


def test_deadband_stops_tremor_reaching_the_arm():
    rng = random.Random(1)
    c, _ = tracking_at()
    outs = sim(c, 1.0, 3.0, lambda t: ob(t, 100.0 + rng.uniform(-1.5, 1.5), rng.uniform(-1.5, 1.5)))
    assert not any(o.changed for _, o in outs)
    outs = sim(c, 3.0, 4.0, lambda t: ob(t, 110.0))
    assert any(o.changed for _, o in outs) and near(outs[-1][1].target, (110 + LEAD, 0.0), 3.0)


def test_deadband_moves_in_steps_of_at_least_deadband():
    c, _ = tracking_at(deadband_mm=5.0)
    outs = sim(c, 1.0, 3.0, lambda t: ob(t, 100.0 + 20.0 * (t - 1.0)))      # slow 20 mm/s drift
    tg = [o.target for _, o in outs if o.changed]
    assert len(tg) >= 4
    assert all(math.hypot(b[0] - a[0], b[1] - a[1]) > 5.0 for a, b in zip(tg, tg[1:]))


def test_lead_is_along_the_pointing_direction():
    for d in ((0.0, 1.0), (0.6, 0.8), (-1.0, 0.0)):
        c, outs = tracking_at(100.0, 50.0, d=d)
        assert near(outs[-1][1].target, (100 + LEAD * d[0], 50 + LEAD * d[1]), 0.5), (d, outs[-1][1].target)


def test_no_lead_without_a_direction():
    c, outs = tracking_at(100.0, 50.0, d=None)
    assert near(outs[-1][1].target, (100.0, 50.0), 0.5)


def test_lead_follows_the_live_setting():
    c, _ = tracking_at(d=(0.0, 1.0))
    c.p.lead_mm = 60.0
    outs = sim(c, 1.0, 1.5, lambda t: ob(t, d=(0.0, 1.0)))
    assert near(outs[-1][1].target, (100.0, 60.0), 0.5)


def test_re_engages_after_a_hold_without_dragging_from_the_old_spot():
    c, _ = tracking_at()
    sim(c, 1.0, 1.5, lambda t: None)
    held = c.step(1.5).target
    assert c.step(1.5).state == "holding"
    outs = sim(c, 1.5, 2.5, lambda t: ob(t, 220.0, 120.0, d=(0.0, 1.0)))
    arming = [o for _, o in outs if o.state == "arming"]
    assert arming and all(o.target == held for o in arming)        # holds while it re-arms
    first = next(o for _, o in outs if o.state == "tracking")
    # the filters re-seeded: straight to the new spot, not a slow slide from the old one
    assert near(first.target, (220.0, 120.0 + LEAD), 1.0), first.target


def test_reset_forgets_everything():
    c, _ = tracking_at()
    c.reset()
    o = c.step(1.0)
    assert o.state == "idle" and o.target is None and not o.changed


def test_duplicate_and_stale_frames_are_ignored():
    c = F.FollowController()
    c.feed(ob(0.0)); c.feed(ob(0.0)); c.feed(ob(-1.0))
    assert len(c.hist) == 1


def test_feed_and_step_from_two_threads():
    c = F.FollowController(); errs = []; n = 3000
    def feeder():
        try:
            for i in range(n):
                c.feed(ob(i * 0.001, 100.0 + (i % 50), (i % 13) - 6.0, pointing=i % 97 != 0))
        except Exception as e:
            errs.append(e)
    th = threading.Thread(target=feeder); th.start()
    k = 0
    while th.is_alive() or k < 10:
        o = c.step(c.last_t or 0.0)
        assert o.state in ("idle", "arming", "tracking", "holding")
        k += 1
    th.join()
    assert not errs, errs
    assert c.last_t == (n - 1) * 0.001


# ------------------------------------------------------------------------------- runner

def main():
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_") and callable(f)]
    fails = []
    for name, fn in tests:
        try:
            fn()
            print(f"  PASS  {name}")
        except AssertionError as e:
            fails.append(name)
            print(f"  FAIL  {name}: {e}")
        except Exception as e:
            fails.append(name)
            print(f"  ERROR {name}: {type(e).__name__}: {e}")
    print(f"\n{len(tests) - len(fails)}/{len(tests)} passed")
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(main())
