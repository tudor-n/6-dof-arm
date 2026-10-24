"""Offline voice tests - no microphone, no speech model, no arm required.

Run with:  python test_voice.py        (or: pytest test_voice.py)

The matcher is fed transcripts, the segmenter a made-up speech probability per 32 ms frame (each
frame's samples hold its own index, so a test can see exactly which frames an utterance kept), and
the trainer the takes Whisper might produce. The server half runs server.py's voice code against a
fake serial port, a fake page and a fake Hearing, with the real 50 Hz control loop moving the arm:
a spoken relax must only ever go limp at the rest pose, and a spoken stop must always work.

With DUMMY_VOICE_CLIPS=<folder> it also runs real audio through the whole chain (Silero VAD,
segmenter, faster-whisper, matcher). The folder holds 16 kHz mono WAVs and a clips.json list of
{"file", "wake", "ptt"}: the command each should give in wake-word mode and in push-to-talk mode
("follow", "home", "relax", "stop", "wake" for a bare wake word, or null for "must not act").
A wrong action fails the test outright; misses (nothing happened, say it again) may be up to 5%.
"""
import asyncio, contextlib, io, json, os, threading, time, wave
import numpy as np

import hearing as H


# --------------------------------------------------------------------------- matcher

M = H.Matcher()


def kind(text, need_wake=True, m=M):
    r = m.match(text, need_wake)
    return r["cmd"] if r["kind"] == "command" else r["kind"]


def test_wake_word_commands():
    for text, cmd in [("Dummy, come here.", "follow"), ("Dummy come here", "follow"), ("Hey Dummy, come here!", "follow"),
                      ("Okay dummy, please come here", "follow"), ("Dummy, follow me", "follow"),
                      ("Dummy, start pose.", "home"), ("Dummy Start Pose", "home"), ("Dummy, go to the start position.", "home"),
                      ("Dummy, relax.", "relax"), ("Dummie, relax", "relax"), ("Dummy, stop!", "stop"), ("Dummy, freeze", "stop")]:
        assert kind(text) == cmd, f"{text!r} -> {kind(text)}, wanted {cmd}"


def test_wake_mode_ignores_speech_without_the_wake_word():
    for text in ("Come here.", "Start pose", "Relax.", "Stop talking and come over here for a second.", "I really need to relax this weekend"):
        assert kind(text) == "nowake", f"{text!r} -> {kind(text)}"


def test_chatter_with_the_wake_word_is_not_a_command():
    for text in ("The dummy variable in this function is wrong.", "What a dummy, he forgot his keys.", "Dummy, relax this weekend",
                 "Dummy, can you start the pose estimation again?"):
        assert kind(text) == "unknown", f"{text!r} -> {kind(text)}"


def test_lookalike_words_are_not_the_wake_word():
    # a fuzzy match would make these triggers for everyone; calibration learns them only for a voice that needs it
    for text in ("Mummy, come here", "My tummy hurts after lunch", "Tommy, stop", "Dumb, relax"):
        assert kind(text) == "nowake", f"{text!r} -> {kind(text)}"


def test_wake_word_must_come_first():
    assert kind("I told my friend dummy come here") == "nowake"
    assert kind("Hey so dummy, come here") == "follow"            # within the first three words


def test_bare_wake_word_opens_a_window():
    for text in ("Dummy.", "Hey Dummy", "dummy, uh"):
        assert kind(text) == "wake", f"{text!r} -> {kind(text)}"
    assert kind("") == "empty" and kind("...") == "empty"


def test_push_to_talk_needs_no_wake_word():
    for text, cmd in [("Come here!", "follow"), ("Start pose", "home"), ("Relax", "relax"), ("Dummy, stop.", "stop"),
                      ("Can you start the pose estimation again?", "unknown"), ("Sarpose", "home")]:
        assert kind(text, need_wake=False) == cmd, f"{text!r} -> {kind(text, need_wake=False)}, wanted {cmd}"


def test_close_commands_are_not_confused():
    assert kind("Dummy, stop following") == "stop"
    assert kind("Dummy, follow") == "follow"
    assert kind("Dummy, go home") == "home"
    assert kind("Dummy, go to sleep") == "relax"


def test_learned_aliases_count():
    m = H.Matcher(wake_aliases=["tummy", "dumb me"], phrase_aliases={"relax": ["rilex"]})
    assert kind("Tummy, come here", m=m) == "follow"
    assert kind("Dumb me, rilex", m=m) == "relax"
    assert kind("Mummy, come here", m=m) == "nowake"             # only what was learned


def test_renamed_wake_word():
    m = H.Matcher("robo")
    assert kind("Robo, relax", m=m) == "relax" and kind("Dummy, relax", m=m) == "nowake"


# --------------------------------------------------------------------------- segmenter

def run(seg, probs, ptt=None):
    """Feed one frame per probability (frame i is filled with i). ptt: {frame index: True/False}."""
    out = []
    for i, p in enumerate(probs):
        if ptt and i in ptt: seg.ptt(ptt[i])
        out += seg.feed(np.full(H.FRAME, float(i), np.float32), p)
    return out


def frames_of(u): return [int(v) for v in u.audio[::H.FRAME]]


def test_segmenter_cuts_one_utterance_with_pre_roll():
    seg = H.Segmenter(thr=0.5, end_silence=0.5, pre_roll=0.4)
    us = run(seg, [0.0] * 20 + [0.9] * 30 + [0.0] * 30)
    assert len(us) == 1, us
    u = us[0]; f = frames_of(u)
    assert f[0] == 8 and f[11] == 19 and f[12] == 20, f[:14]       # 12 frames (0.4 s) before speech, in order
    assert abs(u.speech_s - 30 * H.FRAME_S) < 1e-6 and u.source == "vad"
    assert f[-1] == 49 + 16, f[-3:]                                 # ends 16 quiet frames (0.5 s) after the speech


def test_segmenter_drops_short_blips():
    seg = H.Segmenter(min_speech=0.25)
    assert run(seg, [0.0] * 10 + [0.9] * 5 + [0.0] * 30) == [] and seg.dropped == 1


def test_segmenter_hysteresis_keeps_a_dip():
    seg = H.Segmenter(thr=0.5, end_silence=0.3)
    us = run(seg, [0.0] * 5 + [0.9] * 10 + [0.4] * 20 + [0.9] * 10 + [0.0] * 20)   # 0.4 is above 0.5 - 0.15
    assert len(us) == 1 and abs(us[0].speech_s - 20 * H.FRAME_S) < 1e-6


def test_segmenter_max_length_cuts():
    seg = H.Segmenter(max_len=2.0)
    us = run(seg, [0.9] * 200)
    assert len(us) == 3 and all(len(u.audio) <= int(2.0 / H.FRAME_S + 1) * H.FRAME for u in us)


def test_vad_off_makes_nothing_without_push_to_talk():
    assert run(H.Segmenter(vad=False), [0.9] * 100) == []


def test_push_to_talk_takes_everything_and_a_tail():
    seg = H.Segmenter(vad=False, pre_roll=0.4, tail=0.3)
    us = run(seg, [0.0] * 30 + [0.9] * 20 + [0.0] * 30, ptt={30: True, 50: False})
    assert len(us) == 1 and us[0].source == "ptt"
    f = frames_of(us[0])
    assert f[0] == 16 and f[-1] == 50 + 9 - 1, (f[0], f[-1])      # 14 frames of pre-roll, then 9 frames (0.3 s) of tail
    assert abs(us[0].speech_s - 20 * H.FRAME_S) < 1e-6


def test_push_to_talk_keeps_silence_too():
    us = run(H.Segmenter(vad=False), [0.0] * 40, ptt={5: True, 20: False})
    assert len(us) == 1 and us[0].speech_s == 0.0                   # Hearing reports it as "nothing heard"


def test_push_to_talk_during_speech_continues_it():
    seg = H.Segmenter()
    us = run(seg, [0.0] * 10 + [0.9] * 10 + [0.0] * 40, ptt={15: True, 40: False})
    assert len(us) == 1 and us[0].source == "ptt" and frames_of(us[0])[0] < 10


# --------------------------------------------------------------------------- calibration

def test_learn_accepts_a_correct_take():
    assert H.learn("Dummy, come here.", "follow", M) == (True, None, None, "")


def test_learn_learns_a_misheard_wake_word():
    ok, wake, phrase, why = H.learn("Tommy, come here.", "follow", M)
    assert ok and wake == "tommy" and phrase is None, (ok, wake, phrase, why)


def test_learn_learns_a_misheard_phrase():
    ok, wake, phrase, why = H.learn("Dummy, rilex.", "relax", M)
    assert ok and wake is None and phrase == "rilex", (ok, wake, phrase, why)


def test_learn_refuses_what_looks_like_another_command():
    ok, wake, phrase, why = H.learn("Dummy, rest pose", "relax", M)
    assert not ok and wake is None and phrase is None and "start pose" in why, why


def test_learn_needs_something_like_the_wake_word():
    ok, wake, phrase, why = H.learn("come here", "follow", M)
    assert not ok and wake is None and "at the start" in why, why
    assert not H.learn("", "follow", M)[0]


def test_trainer_walks_every_command_twice():
    t = H.Trainer()
    assert t.say() == "Dummy, come here"
    t.take("", M)                                                   # heard nothing: the step stays
    assert t.i == 0
    takes = {"follow": "Dummy come here", "home": "Dummy, start pose", "relax": "Dummy relax", "stop": "Tommy, stop"}
    while not t.done: t.take(takes[t.expect()], M)
    s = t.summary()
    assert s["ok"] == 8 and s["n"] == 8 and ["wake", "tommy"] in s["learned"], s


def test_quiet_threshold():
    assert H.quiet_threshold([0.05] * 100)[0] == 0.5
    assert H.quiet_threshold([0.05] * 90 + [0.5] * 10)[0] == 0.65
    assert H.quiet_threshold([0.95] * 100)[0] == 0.85


# --------------------------------------------------------------------------- stale speech never acts

def _hearing(asr_delay=0.3, text="Dummy, come here"):
    h = H.Hearing({"mode": "wake"}, lambda *a: None, lambda ev: events.append(ev), lambda s: None)
    h.ok, h.why = False, "test: no threads"          # apply() then never starts the mic threads
    h._asr = lambda audio, hot: (time.sleep(asr_delay), (text, 0.01, asr_delay * 1000))[1]
    events = []; h.events = events
    return h


def _utt(h): return H.Utterance(np.zeros(H.SR, np.float32), 1.0, "vad", -20.0, time.time(), h.gen)


def test_speech_being_transcribed_when_stop_comes_never_acts():
    h = _hearing(); u = _utt(h)
    t = threading.Thread(target=h._handle, args=(u,)); t.start()
    time.sleep(0.1); h.cancel_pending(); t.join()    # what STOP does
    assert h.events == [], h.events


def test_speech_from_before_a_mode_change_never_acts():
    h = _hearing(asr_delay=0.0); u = _utt(h)
    h.set("mode", "ptt")
    h._handle(u)
    assert h.events == [], h.events
    h._handle(_utt(h))                               # said after it: counts
    assert [e.get("cmd") for e in h.events] == ["follow"], h.events


def test_a_calibration_take_cut_short_is_not_a_command():
    h = _hearing(); h.train_start(); u = _utt(h)
    t = threading.Thread(target=h._handle, args=(u,)); t.start()
    time.sleep(0.1); h.train_cancel(); t.join()      # Stop on the calibration while the take is transcribing
    assert h.events == [], h.events


# --------------------------------------------------------------------------- server: what a spoken command does

class _Ser:
    def __init__(self, S): self.S, self.sent = S, []
    def write(self, b): self.sent.append((b.decode().strip(), list(self.S.cur[:4])))


class _Page:
    async def send_json(self, m): pass


class _Hear:
    def __init__(self): self.s = {"dry_run": False, "mode": "wake", "sounds": False, "wake_word": "dummy"}; self.gen = 0
    def set(self, k, v): self.s[k] = v; return ""
    def sound(self, n): pass
    def ptt(self, on): pass
    def cancel_pending(self): self.gen += 1
    def status(self): return {"type": "voice"}


def _server(scenario):
    """Run scenario(sv, ser, page) with the control loop going; returns the serial lines sent."""
    import server as sv
    S = sv.S
    saved = (S.ser, S.live, S.speed, set(S.clients), S.follow, list(S.cur), list(S.tgt), sv.HEAR)
    async def main():
        ser, page = _Ser(S), _Page()
        sv.HEAR, S.ser, S.live, S.speed, S.clients, S.follow = _Hear(), ser, True, 180.0, {page}, False
        S.cur, S.tgt = list(sv.START_POSE), list(sv.START_POSE)
        loop = asyncio.create_task(sv.control_loop())
        try: await scenario(sv, ser, page)
        finally: loop.cancel()
        return [w for w, _ in ser.sent if w.startswith("R ")], ser
    try:
        with contextlib.redirect_stdout(io.StringIO()): return asyncio.run(main())   # the server's log lines
    finally: S.ser, S.live, S.speed, S.clients, S.follow, S.cur, S.tgt, sv.HEAR = saved


def _rest_servos(sv):
    R, J = sv.S.cfg["rest"], sv.S.cfg["joints"]
    return [round(sv.clamp(J[i]["zero"] + J[i]["sign"] * a, J[i]["min"], J[i]["max"]), 1) for i, a in enumerate([R["base"], R["shoulder"], R["elbow"], R["wrist"]])]


async def _finish(sv):
    try: await asyncio.wait_for(asyncio.shield(sv.S.voice_task), 20)
    except BaseException: pass


def test_spoken_relax_goes_limp_only_at_the_rest_pose():
    async def go(sv, ser, page):
        assert (await sv.voice_act("relax", "relax"))[1]
        await _finish(sv)
    relax, ser = _server(go)
    import server as sv
    assert relax == ["R 63"], relax
    at = [cur for w, cur in ser.sent if w == "R 63"][0]
    assert max(abs(a - b) for a, b in zip(at, _rest_servos(sv))) <= 1.0, (at, _rest_servos(sv))


def test_spoken_relax_never_goes_limp_when_follow_takes_over():
    async def go(sv, ser, page):
        await sv.voice_act("relax", "relax"); await asyncio.sleep(0.3)
        tick = sv.follow_tick; sv.follow_tick = lambda now: None      # no camera here: hold follow's target still
        try:
            sv.S.follow = True; sv.S.tgt[:4] = [90.0, 60.0, 60.0, 60.0]   # follow driving the arm, however it got switched on
            await _finish(sv)
        finally: sv.follow_tick = tick; sv.S.follow = False
    relax, _ = _server(go)
    assert relax == [], relax


def test_spoken_relax_never_goes_limp_after_a_reconnect():
    async def go(sv, ser, page):
        await sv.voice_act("relax", "relax"); await asyncio.sleep(0.3)
        S = sv.S; S.tgt = list(S.cur); S.live = False                 # what reconnect_loop does
        await _finish(sv)
    relax, _ = _server(go)
    assert relax == [], relax


def test_spoken_relax_is_cancelled_by_stop_follow_and_voice_off():
    for how in ("stop", "follow", "off"):
        async def go(sv, ser, page, how=how):
            await sv.voice_act("relax", "relax"); await asyncio.sleep(0.3)
            if how == "stop": await sv.handle(page, {"type": "stop"})
            elif how == "follow": sv.follow_on()                          # refused here (no camera), still cancels
            else: await sv.voice_msg("voice_set", {"key": "mode", "value": "off"})
            await _finish(sv)
            assert sv.S.voice_task.cancelled(), how
        relax, _ = _server(go)
        assert relax == [], (how, relax)


def test_spoken_motion_is_refused_with_live_off_or_no_page_or_dry_run():
    async def go(sv, ser, page):
        S = sv.S
        S.live = False
        assert (await sv.voice_act("home", "start pose")) == ("refused: Live is off", False)
        S.live = True; S.clients = set()
        assert not (await sv.voice_act("home", "start pose"))[1]
        S.clients = {page}; sv.HEAR.s["dry_run"] = True
        assert (await sv.voice_act("relax", "relax")) == ("dry run, not acted on", True)
        assert S.tgt == list(sv.START_POSE), S.tgt
        assert S.voice_task is None or S.voice_task.done()
    _server(go)


def test_spoken_stop_works_with_no_page_and_in_dry_run():
    async def go(sv, ser, page):
        S = sv.S; S.clients = set(); sv.HEAR.s["dry_run"] = True
        S.tgt[1] += 30.0                                              # the arm is on its way somewhere
        assert (await sv.voice_act("stop", "stop")) == ("stopped", True)
        assert S.tgt[:5] == S.cur[:5], (S.tgt, S.cur)
    _server(go)


def test_a_command_overtaken_by_stop_is_dropped():
    async def go(sv, ser, page):
        ev = {"kind": "command", "cmd": "home", "say": "start pose", "gen": sv.HEAR.gen}
        sv.S.tgt[1] += 30.0
        await sv.handle(page, {"type": "stop"})                       # STOP while it was being transcribed
        tgt = list(sv.S.tgt)
        await sv.voice_event(ev)
        assert not ev["ok"] and "dropped" in ev["result"] and sv.S.tgt == tgt, ev
    _server(go)


# --------------------------------------------------------------------------- real audio (optional)

def _wav(p):
    with wave.open(p) as w:
        assert w.getframerate() == H.SR and w.getnchannels() == 1, f"{p}: needs 16 kHz mono"
        return np.frombuffer(w.readframes(w.getnframes()), np.int16).astype(np.float32) / 32768.0


def test_recognition_on_recorded_clips():
    d = os.environ.get("DUMMY_VOICE_CLIPS")
    if not d: print("        (skipped: set DUMMY_VOICE_CLIPS to a folder of clips)"); return
    clips = json.load(open(os.path.join(d, "clips.json")))
    vad, asr = H.SileroVAD(), H.Recognizer("small.en", print)
    print(f"        model on {asr.load()}")
    wrong, missed = [], []
    for c in clips:
        a = _wav(os.path.join(d, c["file"])); pad = np.zeros(H.SR, np.float32)
        a = np.concatenate([pad, a, pad]); a = a[:len(a) // H.FRAME * H.FRAME]
        for mode in ("wake", "ptt"):
            seg = H.Segmenter(vad=mode == "wake"); vad.reset(); us = []
            n = len(a) // H.FRAME; press, release = n // 8, n - n // 3       # held over the speech, the tail fits after
            for k, i in enumerate(range(0, len(a), H.FRAME)):
                if mode == "ptt" and k in (press, release): seg.ptt(k == press)
                us += seg.feed(a[i:i + H.FRAME], vad(a[i:i + H.FRAME]))
            got = []
            for u in us:
                text, nsp, _ = asr(u.audio, "Dummy")
                r = M.match("" if nsp > H.NO_SPEECH else text, need_wake=mode == "wake")
                got.append(r["cmd"] if r["kind"] == "command" else "wake" if r["kind"] == "wake" else None)
            acted, want = [g for g in got if g], c[mode]
            line = f"{c['file']} [{mode}] wanted {want}, got {got} from {[asr(u.audio, 'Dummy')[0] for u in us]}"
            if any(g != want for g in acted): wrong.append(line)
            elif want and not acted: missed.append(line)
    n = 2 * len(clips)
    print(f"        {n - len(wrong) - len(missed)}/{n} right, {len(missed)} missed, {len(wrong)} wrong")
    for line in missed: print("        missed: " + line)
    assert not wrong, f"{len(wrong)} wrong actions:\n          " + "\n          ".join(wrong)
    assert len(missed) <= 0.05 * n, f"{len(missed)} misses of {n}"


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
