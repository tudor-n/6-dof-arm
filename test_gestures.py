"""Offline personality tests - no camera, no microphone, no arm required.

Run with:  python test_gestures.py        (or: pytest test_gestures.py)

The first half checks gestures.plan() on its own: what a nod, a shake and the dance turn into from a
given pose, and that none of them can leave the joint limits, come near the table or leave the 360
gripper driving. The second half runs server.py's gesture code against the real 50 Hz control loop,
with a fake serial port and page (and, for follow, a fake camera and hand tracker driving the real
follow_tick), and samples the arm every tick.
"""
import asyncio, contextlib, copy, io, time, types

import gestures as G
import server as sv

S = sv.S
CFG = S.cfg
J = CFG["joints"]


def tip_z(s): return sv.fk(list(s[:5]) + [90.0], CFG)[0][-1][2]


def rest_pose(): return sv.rest_servos()[0] + [90.0, 90.0]


START = list(sv.START_POSE)
FLOOR = 5.0 + G.FLOOR_MM


def plan(name, home=START, **kw): return G.plan(name, home, J, tip_z, FLOOR, **kw)


def in_limits(s):
    return all(min(J[i]["min"], J[i]["max"]) - 1e-6 <= s[i] <= max(J[i]["min"], J[i]["max"]) + 1e-6 for i in range(5))


# --------------------------------------------------------------------------- the planner

def test_every_gesture_fits_at_the_start_pose():
    for name in G.GESTURES:
        steps, scale, why = plan(name)
        assert steps and scale == 1.0, (name, why)


def test_nod_tips_the_tool_down_first():
    steps, _, _ = plan("nod")
    assert tip_z(steps[0][1]) < tip_z(START) - 20, (tip_z(steps[0][1]), tip_z(START))


def test_down_comes_from_the_kinematics_not_an_assumed_sign():
    flipped = copy.deepcopy(J); flipped[3]["sign"] *= -1; flipped[3]["zero"] = 180 - flipped[3]["zero"]
    cfg = dict(CFG, joints=flipped)
    home = list(START); home[3] = 180 - home[3]      # the same physical pose under the mirrored wrist convention
    tz = lambda s: sv.fk(list(s[:5]) + [90.0], cfg)[0][-1][2]
    steps, _, why = G.plan("nod", home, flipped, tz, FLOOR)
    assert steps and tz(steps[0][1]) < tz(home) - 20, why


def test_keyframes_stay_in_limits_and_above_the_table():
    for name in G.GESTURES:
        steps, _, _ = plan(name)
        for _, s in steps:
            assert in_limits(s), (name, s)
            assert tip_z(s) >= FLOOR and tip_z(s) >= tip_z(START) - G.DROP_MM - 1e-6, (name, tip_z(s))


def test_every_gesture_ends_exactly_where_it_started():
    for name in G.GESTURES:
        steps, _, _ = plan(name)
        assert steps[-1][1][:5] == START[:5], (name, steps[-1][1])


def test_shake_moves_only_the_base_and_nothing_moves_the_shoulder_or_elbow():
    steps, _, _ = plan("shake")
    assert all(s[1:5] == START[1:5] for _, s in steps)
    for name in G.GESTURES:
        assert all(s[1] == START[1] and s[2] == START[2] for _, s in plan(name)[0]), name


def test_the_gripper_is_braked_between_claps_and_at_the_end():
    steps, _, _ = plan("dance")
    grips = [s[5] for _, s in steps]
    assert all(g in (60.0, 90.0, 120.0) for g in grips) and grips[-1] == 90.0, grips
    assert 120.0 in grips and 60.0 in grips
    for a, b in zip(grips, grips[1:]):       # never open-then-hold: an open pulse is followed by a close or the brake
        assert not (a != 90.0 and b == a), grips
    assert all(s[5] is None for _, s in plan("nod")[0])              # a nod never touches it


def test_dance_without_the_clap_never_touches_the_gripper():
    steps, _, _ = plan("dance", clap=False)
    assert all(s[5] is None for _, s in steps)


def test_scales_down_near_a_limit():
    home = list(START); j = J[3]
    lo, hi = sorted((j["min"], j["max"]))
    home[3] = lo + 12.0                          # a full nod would cross the wrist's limit, a smaller one fits
    steps, scale, why = plan("nod", home)
    assert steps and scale < 1.0 and "limit" in why or steps and scale < 1.0, (scale, why)
    assert all(in_limits(s) for _, s in steps)


def test_refuses_near_the_table():
    servo, _, _ = sv.ik(230.0, 0.0, 40.0, -90.0, CFG)      # tool pointing down, low over the table
    low = list(servo) + [90.0, 90.0]
    assert tip_z(low) < FLOOR, tip_z(low)
    for name in G.GESTURES:                               # a shake too: it would sweep the tool along the table
        steps, _, why = plan(name, low)
        assert steps is None and why == "the table", (name, why)


def test_nothing_fits_at_the_rest_pose():
    # the rest pose parks the tool on the desk, below where any gesture may start
    for name in G.GESTURES:
        steps, _, why = plan(name, rest_pose())
        assert steps is None and why == "the table", (name, why)


def test_size_scales_the_motion():
    big, small = plan("shake", size=1.5)[0], plan("shake", size=0.5)[0]
    assert abs(big[0][1][0] - START[0]) > 2.5 * abs(small[0][1][0] - START[0])


# --------------------------------------------------------------------------- server: against the control loop

class _Ser:
    def __init__(self): self.sent = []
    def write(self, b): self.sent.append(b.decode().strip())


class _Page:
    async def send_json(self, m): pass


class _Hear:
    def __init__(self, **kw):
        self.s = {"dry_run": False, "mode": "wake", "sounds": False, "wake_word": "dummy", "personality": True,
                  "gesture_size": 1.0, "gesture_speed": 120.0, "dance_clap": True}
        self.s.update(kw); self.gen = 0
    def sound(self, n): pass
    def ptt(self, on): pass
    def cancel_pending(self): self.gen += 1
    def status(self): return {"type": "voice"}


def _run(scenario, **hear):
    """Run scenario() with the control loop going, sampling the tool height every tick."""
    keep = ("ser", "live", "speed", "clients", "follow", "cur", "tgt", "limp", "pick_task", "voice_task", "gesture", "gesture_home")
    saved = {k: copy.copy(getattr(S, k)) for k in keep}; saved_g = (sv.HEAR, sv.V, sv.FC, sv.HT, sv.follow_tick)
    low = [1e9]

    async def main():
        sv.HEAR = _Hear(**hear)
        S.ser, S.live, S.speed, S.clients, S.follow, S.limp = _Ser(), True, 60.0, {_Page()}, False, 0
        S.cur, S.tgt, S.pick_task, S.voice_task, S.gesture = list(START), list(START), None, None, None
        loop = asyncio.create_task(sv.control_loop())

        async def watch():
            while True: low[0] = min(low[0], tip_z(S.cur)); await asyncio.sleep(0.02)
        w = asyncio.create_task(watch())
        try: await scenario()
        finally: loop.cancel(); w.cancel()

    try:
        with contextlib.redirect_stdout(io.StringIO()): asyncio.run(main())
        return low[0]
    finally:
        for k, v in saved.items(): setattr(S, k, v)
        sv.HEAR, sv.V, sv.FC, sv.HT, sv.follow_tick = saved_g


async def _done():
    try: await asyncio.wait_for(asyncio.shield(S.voice_task), 15)
    except BaseException: pass
    await asyncio.sleep(0.1)


def test_nod_and_shake_come_back_to_the_start_pose():
    for name in ("nod", "shake"):
        async def go(name=name):
            r, ok = await sv.voice_gesture(name)
            assert ok, r
            await _done()
            assert S.tgt[:5] == START[:5] and max(abs(S.cur[i] - START[i]) for i in range(5)) < 0.2, (name, S.cur)
            assert S.gesture is None
        _run(go)


def test_dance_ends_home_with_the_gripper_braked_and_never_near_the_table():
    async def go():
        r, ok = await sv.voice_gesture("dance")
        assert ok, r
        await _done()
        assert S.tgt[:5] == START[:5] and S.tgt[5] == 90.0 and S.cur[5] == 90.0, (S.tgt, S.cur)
    low = _run(go)
    assert low >= FLOOR, low


def test_gestures_move_at_their_own_speed_and_leave_the_slider_alone():
    async def go():
        t0 = time.time(); await sv.voice_gesture("shake"); await _done()
        assert time.time() - t0 < 3.0 and S.speed == 20.0
    async def slow():
        S.speed = 20.0; await go()
    _run(slow)


def test_refused_without_live_while_limp_during_a_pick_or_with_personality_off():
    async def go():
        S.live = False
        assert (await sv.voice_gesture("nod")) == ("didn't nod: Live is off", False)
        S.live = True; S.limp = 0x3F
        r, ok = await sv.voice_gesture("dance"); assert not ok and "relaxed" in r, r
        S.limp = 0; S.pick_task = asyncio.create_task(asyncio.sleep(5))
        r, ok = await sv.voice_gesture("shake"); assert not ok and "pick" in r, r
        S.pick_task.cancel(); S.pick_task = None
        sv.HEAR.s["personality"] = False
        r, ok = await sv.voice_gesture("nod"); assert not ok and "personality" in r, r
        assert S.tgt == START and S.voice_task is None
    _run(go)


def test_a_relaxed_roll_or_gripper_is_woken_but_a_relaxed_arm_joint_refuses():
    async def go():
        S.limp = 0x30                                  # after relax + start pose: only roll and gripper still limp
        r, ok = await sv.voice_gesture("nod"); assert ok, r   # uses neither: runs, wakes nothing
        await _done(); assert S.limp == 0x30
        r, ok = await sv.voice_gesture("dance"); assert ok, r # rolls and claps: wakes both
        assert S.limp == 0 and S.ser.sent[-1] == "R 0", (S.limp, S.ser.sent[-3:])
        await _done()
        S.limp = 0x08                                  # the wrist itself is limp
        r, ok = await sv.voice_gesture("nod"); assert not ok, r
    _run(go)


def test_spoken_relax_leaves_the_arm_unable_to_gesture():
    async def go():
        await sv.handle(_Page(), {"type": "relax", "mask": 63})
        r, ok = await sv.voice_gesture("dance"); assert not ok and "relaxed" in r, r
    _run(go)


def test_stop_mid_dance_holds_brakes_and_cancels():
    async def go():
        await sv.voice_gesture("dance"); await asyncio.sleep(1.0)
        await sv.handle(_Page(), {"type": "stop"})
        await asyncio.sleep(0.1)
        held = list(S.cur)
        assert S.voice_task.cancelled() and S.gesture is None
        assert S.cur[5] == 90.0 and S.tgt[5] == 90.0
        await asyncio.sleep(1.0)
        assert max(abs(S.cur[i] - held[i]) for i in range(5)) < 0.5, (held, S.cur)    # holds, doesn't go home
    _run(go)


def test_a_dance_cut_off_mid_clap_leaves_the_gripper_braked():
    async def go():
        await sv.voice_gesture("dance")
        t0 = time.time()
        # wait for the clap (the start pose's own gripper value is 100, so look for a clap's 60 / 120)
        while S.tgt[5] not in (60.0, 120.0) and time.time() - t0 < 10: await asyncio.sleep(0.02)
        assert S.tgt[5] in (60.0, 120.0), "the dance never clapped"
        clapping = S.voice_task
        await sv.voice_act("yes", "a yes question")          # interrupted by another spoken reaction, not STOP
        await asyncio.sleep(0.1)
        assert clapping.cancelled()
        assert S.tgt[5] == 90.0 and S.cur[5] == 90.0, (S.tgt[5], S.cur[5])     # the nod never touches it
    _run(go)


def test_a_gesture_interrupted_by_another_still_ends_where_the_first_began():
    async def go():
        await sv.voice_gesture("dance"); await asyncio.sleep(0.6)
        r, ok = await sv.voice_gesture("shake"); assert ok, r
        await _done()
        assert S.tgt[:5] == START[:5], S.tgt
    _run(go)


def test_a_bare_wake_word_nods_but_never_over_a_running_task():
    async def go():
        ev = {"kind": "wake", "source": "vad", "gen": 0}
        await sv.voice_event(ev); assert ev["ok"] and ev["result"] == "nodded", ev
        await asyncio.sleep(0.2)
        ev2 = {"kind": "wake", "source": "vad", "gen": 0}
        await sv.voice_event(ev2); assert "result" not in ev2, ev2      # the first nod is still running
        await _done()
        sv.HEAR.s["dry_run"] = True
        ev3 = {"kind": "wake", "source": "vad", "gen": 0}
        await sv.voice_event(ev3); assert ev3["result"] == "dry run: would nod" and S.voice_task.done()
    _run(go)


def test_spoken_reactions_pick_their_gesture():
    for cmd, verb in (("yes", "nodded"), ("no", "shook his head"), ("praise", "danced")):
        async def go(cmd=cmd, verb=verb):
            r, ok = await sv.voice_act(cmd, "x"); assert ok and r.startswith(verb), (cmd, r)
            await _done()
        _run(go)


def test_losing_the_serial_link_ends_a_gesture_for_good():
    class Dead:
        def write(self, b): raise OSError("unplugged")
        def close(self): pass
    async def go():
        await sv.voice_gesture("dance"); await asyncio.sleep(0.5)
        task = S.voice_task; S.ser = Dead()                   # the USB cable comes out
        await asyncio.sleep(0.2)
        assert task.done() and not S.live and S.gesture is None
        S.ser = _Ser(); S.tgt = list(S.cur); S.live = True     # what reconnect_loop does, then Live back on
        before = list(S.tgt); await asyncio.sleep(1.0)
        assert S.tgt[:5] == before[:5], (before, S.tgt)       # nothing of the dance comes back
    _run(go)


def test_a_gesture_never_wakes_the_gripper_it_doesnt_use_and_brakes_it_before_waking():
    async def go():
        S.limp = 0x20; S.cur[5] = S.tgt[5] = 100.0             # gripper relaxed while it was driving
        r, ok = await sv.voice_gesture("nod"); assert ok, r
        await _done()
        assert S.limp == 0x20 and not any(w.startswith("R") for w in S.ser.sent), (S.limp, S.ser.sent)
        r, ok = await sv.voice_gesture("dance"); assert ok, r   # the clap does wake it: braked first
        assert S.limp == 0 and "R 0" in S.ser.sent and S.tgt[5] == 90.0 and S.cur[5] == 90.0, (S.limp, S.tgt[5])
    _run(go)


def test_a_reaction_that_cannot_run_leaves_the_running_gesture_to_finish():
    async def go():
        lo = min(J[0]["min"], J[0]["max"])
        S.cur[0] = S.tgt[0] = lo + 1.0                          # the base right by its limit: no room to shake
        home = list(S.tgt)
        r, ok = await sv.voice_gesture("nod", spoken=True); assert ok, r
        await asyncio.sleep(0.15)
        r, ok = await sv.voice_gesture("shake", spoken=True); assert not ok and "limit" in r, r
        await _done()
        assert S.tgt[:5] == home[:5], (home, S.tgt)              # the nod still came home
    _run(go)


def test_spoken_reactions_never_interrupt_a_relax_fold():
    for personality in (True, False):
        async def go(personality=personality):
            sv.HEAR.s["personality"] = personality
            await sv.voice_act("relax", "relax"); fold = S.voice_task
            # the moment it matters: the arm has reached rest and the fold is settling before it goes limp,
            # so nothing is moving any more and a shake would fit
            t0 = time.time()
            while max(abs(S.tgt[i] - S.cur[i]) for i in range(5)) > 0.5 and time.time() - t0 < 15: await asyncio.sleep(0.02)
            assert not fold.done()
            r, ok = await sv.voice_act("no", "a no question"); assert not ok, r
            assert S.voice_task is fold and not fold.done(), r
            await _done()
            assert "R 63" in S.ser.sent, S.ser.sent[-3:]          # the relax went through
        _run(go)


def test_no_gesture_while_the_arm_is_still_moving():
    async def go():
        S.tgt[1] += 30.0                                         # say, right after "start pose"
        r, ok = await sv.voice_gesture("nod"); assert not ok and "moving" in r, r
    _run(go)


def test_a_gesture_during_follow_pauses_its_writes_and_follow_carries_on():
    target = [(230.0, 0.0)]

    class FC:
        p = types.SimpleNamespace(lead_mm=30.0, deadband_mm=3.0)
        def step(self, now): return types.SimpleNamespace(changed=True, target=target[0], state="tracking", status="tracking")
        def reset(self, now=None): pass
    V = types.SimpleNamespace(s={}, roll={"zero": 90.0, "sign": 1}, follow_overlay=None, tip_cur=None, tip_tgt=None, jaw_deg=None,
                              pose_health=lambda: (True, ""), table_z=lambda x, y: 5.0, status=lambda: {"type": "vision"})
    HT = types.SimpleNamespace(error="", latest=None)

    async def go():
        sv.V, sv.FC, sv.HT = V, FC(), HT
        S.follow, S.follow_pending, S.follow_last_write, S.follow_fail, S.follow_pitch = True, True, 0.0, None, -40.0
        await asyncio.sleep(1.5)                              # follow writes and the arm gets there
        at = list(S.tgt[:4]); assert at != START[:4]
        r, ok = await sv.voice_gesture("nod"); assert ok, r
        target[0] = (260.0, 40.0)                             # the finger moves during the nod
        await asyncio.sleep(0.3)
        assert S.gesture is not None and S.tgt[0] == at[0]    # follow didn't write over the gesture
        await _done(); await asyncio.sleep(0.3)
        assert S.follow and S.tgt[:4] != at                    # afterwards follow wrote the new target, still on
    _run(go)


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
