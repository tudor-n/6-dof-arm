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


def test_personality_reactions():
    for text, cmd in [("Dummy, are you dumb?", "no"), ("Dummy, you stupid", "no"), ("Dummy, are you an idiot", "no"),
                      ("Dummy, good boy!", "praise"), ("Dummy, who's a good boy", "praise"), ("Dummy, well done", "praise"),
                      ("Dummy, are you awake?", "yes"), ("Dummy, can you hear me", "yes")]:
        assert kind(text) == cmd, f"{text!r} -> {kind(text)}, wanted {cmd}"


def test_a_negation_is_never_a_reaction_or_a_command():
    for text in ("Dummy, you're not dumb", "Dummy, aren't you stupid", "Dummy, don't relax", "Dummy, never stop",
                 "Dummy, you are not a good boy"):
        assert kind(text) == "unknown", f"{text!r} -> {kind(text)}"


def test_praise_in_chatter_is_not_for_the_arm():
    assert kind("He was such a good boy at the vet today.") == "nowake"
    assert kind("He was such a good boy at the vet today.", need_wake=False) == "unknown"


def test_your_own_reaction_phrases_from_voice_json():
    h = H.Hearing({"reactions": {"praise": ["attaboy"], "no": ["are you a toaster"]}}, lambda *a: None, lambda ev: None, lambda s: None)
    assert kind("Dummy, attaboy", m=h.matcher) == "praise" and kind("Dummy, are you a toaster", m=h.matcher) == "no"
    assert kind("Dummy, good boy", m=h.matcher) == "praise"          # the built-in ones still count


def test_renamed_wake_word():
    m = H.Matcher("robo")
    assert kind("Robo, relax", m=m) == "relax" and kind("Dummy, relax", m=m) == "nowake"


# --------------------------------------------------------------------------- the whole catalogue

# Phrases of different commands that read within the fuzzy threshold of each other. Each is fine said
# as it is (an exact phrase wins), and they sound different; a new pair here needs the same thought.
KNOWN_CLOSE = {frozenset(p) for p in (("lower", "slower"), ("close", "closer"), ("drop it", "do it"),
               ("are you a bad robot", "bad robot"), ("start position", "rest position"), ("tilt down", "set it down"),
               ("tilt down", "put it down"), ("open the gripper", "close the gripper"), ("open the gripper", "close gripper"),
               ("open gripper", "close the gripper"), ("open gripper", "close gripper"), ("no thanks", "thanks"),
               ("mute", "unmute"), ("good boy", "goodbye"))}


def test_every_phrase_is_its_own_command():
    for c, ps in H.COMMANDS.items():
        for p in ps:
            assert kind("Dummy, " + p) == c, f"{p!r} -> {kind('Dummy, ' + p)}, wanted {c}"
            assert kind(p, need_wake=False) == c, f"push to talk {p!r} -> {kind(p, need_wake=False)}"


def test_no_two_commands_sound_alike():
    import itertools
    # what the fuzzy match compares: "let go" counts only as said, so its filler-free "let" isn't there
    items = [(c, p, " ".join(H._core(H.normalize(p)))) for c, ps in H.COMMANDS.items() for p in ps]
    items = [i for i in items if i[2] in M.phrases[i[0]]] + [("?", p, p) for p in H.AMBIGUOUS]
    for (c1, p1, k1), (c2, p2, k2) in itertools.combinations(items, 2):
        if c1 == c2: continue
        assert k1 != k2, f"{p1!r} ({c1}) and {p2!r} ({c2}) are the same words"
        if H.sim(k1, k2) >= H.CMD_SIM:
            assert frozenset((p1, p2)) in KNOWN_CLOSE, f"{p1!r} ({c1}) and {p2!r} ({c2}) are {H.sim(k1, k2):.2f} alike"


def test_every_command_has_a_help_line_and_a_group():
    grouped = {c for _, ids in H.GROUPS for c in ids}
    assert set(H.COMMANDS) <= grouped and grouped <= set(H.COMMANDS) | {"huh"}, set(H.COMMANDS) ^ grouped
    assert all(c in H.HELP for c in grouped) and all(c in H.SAY for c in H.COMMANDS)
    cat = H.Hearing.catalog()
    assert [g for g, _ in cat] == [g for g, _ in H.GROUPS] and all(len(r) == 4 for _, rows in cat for r in rows)


def arg(text, m=M):
    r = m.match(text); return r.get("cmd"), r.get("arg")


def test_amounts():
    assert arg("Dummy, up") == ("up", None)
    assert arg("Dummy, up a bit") == ("up", {"n": None, "unit": None, "size": "small"})
    assert arg("Dummy, move up a little bit") == ("up", {"n": None, "unit": None, "size": "small"})
    assert arg("Dummy, left a lot") == ("left", {"n": None, "unit": None, "size": "big"})
    assert arg("Dummy, left 3 cm") == ("left", {"n": 3.0, "unit": "cm", "size": None})
    assert arg("Dummy, left 3cm.") == ("left", {"n": 3.0, "unit": "cm", "size": None})
    assert arg("Dummy, up thirty") == ("up", {"n": 30.0, "unit": None, "size": None})
    assert arg("Dummy, down twenty five millimetres") == ("down", {"n": 25.0, "unit": "mm", "size": None})
    assert arg("Dummy, roll right 45 degrees") == ("roll_right", {"n": 45.0, "unit": "deg", "size": None})
    assert arg("Dummy, open a little") == ("open", {"n": None, "unit": None, "size": "small"})
    assert arg("Dummy, one more time") == ("again", None)                 # an exact phrase, not "one"
    assert kind("Dummy, five") == "unknown" and kind("Dummy, relax a bit") == "unknown"   # no amount for relax


def test_objects():
    assert arg("Dummy, pick up the red pen") == ("pick", {"obj": "red pen"})
    assert arg("Dummy, pick the pen up") == ("pick", {"obj": "pen"})
    assert arg("Dummy, grab it") == ("pick", {"obj": None})
    assert arg("Dummy, pick it up") == ("pick", {"obj": None})
    assert arg("Dummy, bring me the scissors") == ("bring", {"obj": "scissors"})
    assert arg("Dummy, can you get me the cup") == ("bring", {"obj": "cup"})
    assert arg("Dummy, can you find my scissors") == ("find", {"obj": "scissors"})
    assert arg("Dummy, where is the tape") == ("find", {"obj": "tape"})
    assert arg("Dummy, do you see the pen") == ("find", {"obj": "pen"})
    assert arg("Dummy, bring it to me") == ("handover", None)             # what it holds, not a new pick
    assert kind("Dummy, can you see the table") == "pose_ok" and kind("Dummy, get up") == "wake"
    assert kind("Dummy, see you later") == "hello"
    assert kind("Dummy, don't pick up the pen") == "unknown" and kind("Dummy, pick up the big heavy red pen") == "unknown"


def test_near_misses_say_why_and_never_act():
    for text, hint in (("Dummy, slower", "slow down"), ("Dummy, closer", "toward me")):
        r = M.match(text)
        assert r["kind"] == "unknown" and hint in r["hint"], r
    assert kind("Dummy, lower") == "down" and kind("Dummy, close") == "close" and kind("Dummy, slow down") == "slower"
    assert kind("Dummy, let go") == "open" and kind("Dummy, let's go") == "unknown"


def test_chatter_with_the_name_is_still_not_a_command():
    for text in ("Dummy, I'm going to lunch", "Dummy is a funny name", "Dummy, what's up", "Dummy, hello there",
                 "Dummy, that's right", "Dummy, no way", "Dummy, are you ok", "Dummy, sorry", "Dummy, the cup is over there"):
        assert kind(text) == "unknown", f"{text!r} -> {kind(text)}"


def test_the_window_after_a_step_takes_only_steps():
    h = _hearing(asr_delay=0.0, text="left")
    h.keep_awake(5.0, {"left", "again", "stop"})
    h._handle(_utt(h)); assert [e.get("cmd") for e in h.events] == ["left"], h.events
    h.keep_awake(5.0, {"left", "again", "stop"}); h.events.clear()
    h._asr = lambda audio, hot: ("relax", 0.01, 0.0)
    h._handle(_utt(h)); assert [e["kind"] for e in h.events] == ["nowake"], h.events     # needs the name
    h._asr = lambda audio, hot: ("Dummy, relax", 0.01, 0.0); h.events.clear()
    h._handle(_utt(h)); assert [e.get("cmd") for e in h.events] == ["relax"], h.events   # with it, anything


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
    ok, wake, phrase, why = H.learn("Dummy, rest pose", "relax", M)       # rest pose is a command of its own
    assert not ok and wake is None and phrase is None and "rest pose" in why, why
    ok, _, phrase, why = H.learn("Dummy, rest position please", "home", M)
    assert not ok and phrase is None, why


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
    def __init__(self):
        self.s = {"dry_run": False, "mode": "wake", "sounds": False, "wake_word": "dummy", "personality": False,
                  "pick_confirm": True, "awake_window": 5.0, "gesture_size": 1.0, "gesture_speed": 120.0, "dance_clap": True}
        self.gen = 0; self.awake = []; self.sounds = []
    def set(self, k, v): self.s[k] = v; return ""
    def sound(self, n): self.sounds.append(n)
    def ptt(self, on): pass
    def cancel_pending(self): self.gen += 1
    def keep_awake(self, seconds, only=None): self.awake.append((seconds, set(only) if only else None))
    def status(self): return {"type": "voice"}


def _server(scenario):
    """Run scenario(sv, ser, page) with the control loop going; returns the serial lines sent."""
    import server as sv
    S = sv.S
    saved = (S.ser, S.live, S.speed, set(S.clients), S.follow, list(S.cur), list(S.tgt), sv.HEAR, S.limp, S.held, sv.V)
    async def main():
        ser, page = _Ser(S), _Page()
        sv.HEAR, S.ser, S.live, S.speed, S.clients, S.follow = _Hear(), ser, True, 180.0, {page}, False
        S.cur, S.tgt = list(sv.START_POSE), list(sv.START_POSE)
        S.limp, S.held, S.voice_again, S.voice_ask = 0, None, None, None; S.voice_undo.clear()
        loop = asyncio.create_task(sv.control_loop())
        try: await scenario(sv, ser, page)
        finally: loop.cancel()
        return [w for w, _ in ser.sent if w.startswith("R ")], ser
    try:
        with contextlib.redirect_stdout(io.StringIO()): return asyncio.run(main())   # the server's log lines
    finally: S.ser, S.live, S.speed, S.clients, S.follow, S.cur, S.tgt, sv.HEAR, S.limp, S.held, sv.V = saved


def _rest_servos(sv): return sv.rest_servos()[0]


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


def _tip(sv, s=None): return sv.fk(sv.S.tgt if s is None else s, sv.S.cfg)[0][-1]


async def _arrive(sv, secs=10.0):
    t0 = time.time()
    while time.time() - t0 < secs and max(abs(a - b) for a, b in zip(sv.S.tgt[:5], sv.S.cur[:5])) > 0.2: await asyncio.sleep(0.05)


def test_spoken_steps_move_exactly_that_far_in_your_frame():
    async def go(sv, ser, page):
        for cmd, a, d in (("left", None, (0, -20, 0)), ("toward", None, (20, 0, 0)), ("up", {"size": "small"}, (0, 0, 10)),
                          ("right", {"n": 3.0, "unit": "cm"}, (0, 30, 0)), ("down", {"n": 15.0}, (0, 0, -15))):
            before = _tip(sv)
            r = await sv.voice_act(cmd, cmd, a)
            assert r[1], (cmd, r)
            got = [b - c for b, c in zip(_tip(sv), before)]
            assert all(abs(g - w) < 0.5 for g, w in zip(got, d)), (cmd, got, d)
            await _arrive(sv)
    _server(go)
    import server as sv
    assert sv.voice_amount(sv.STEP_MM, {"n": 200.0}) == (50.0, " (asked 200: 50 is the most per step)")
    assert sv.voice_amount(sv.STEP_MM, {"n": 3.0, "unit": "cm"}) == (30.0, "")
    assert sv.voice_amount(sv.GRIP_S["open"], {"n": 9.0}, numbers=False) == (1.0, "")      # a gripper takes no number


def test_a_step_out_of_reach_is_refused_without_moving():
    async def go(sv, ser, page):
        for _ in range(12):
            before = list(sv.S.tgt)
            r = await sv.voice_act("up", "up", {"size": "big"})
            if not r[1]:
                assert sv.S.tgt == before and "can't" in r[0], r; return
        raise AssertionError("never reached the top")
    _server(go)


def test_spoken_steps_are_refused_limp_live_off_dry_run_and_with_no_page():
    async def go(sv, ser, page):
        S = sv.S; before = list(S.tgt)
        S.limp = 0x0F; assert "relaxed" in (await sv.voice_act("up", "up"))[0]; S.limp = 0
        S.live = False; assert (await sv.voice_act("left", "left")) == ("refused: Live is off", False); S.live = True
        sv.HEAR.s["dry_run"] = True; assert (await sv.voice_act("tilt_down", "tilt down")) == ("dry run, not acted on", True)
        assert (await sv.voice_act("where", "where are you?"))[1], "a question still answers in dry run"
        sv.HEAR.s["dry_run"] = False
        S.clients = set(); assert not (await sv.voice_act("roll_left", "roll left"))[1]
        assert S.tgt == before, S.tgt
    _server(go)


def test_turn_tilt_point_down_and_roll():
    async def go(sv, ser, page):
        S, cfg = sv.S, sv.S.cfg
        y0 = sv.joint_angles(S.tgt, cfg)[0]
        assert (await sv.voice_act("turn_left", "turn left"))[1] and abs(sv.joint_angles(S.tgt, cfg)[0] - (y0 - 15)) < 0.01
        await _arrive(sv)
        tip, p0 = _tip(sv), sv.fk(S.tgt, cfg)[1]
        # the start pose has the shoulder at its limit: no tilt up there, and only a little down
        assert not (await sv.voice_act("tilt_up", "tilt up"))[1] and sv.fk(S.tgt, cfg)[1] == p0
        assert (await sv.voice_act("tilt_down", "tilt down", {"size": "small"}))[1]
        assert abs(sv.fk(S.tgt, cfg)[1] - (p0 - 5)) < 0.5 and sv.math.dist(_tip(sv), tip) < 2.0, "the tip stays put"
        await _arrive(sv)
        r = await sv.voice_act("roll_left", "roll left")
        assert r[1] and abs(sv.roll_angle(S.tgt, cfg) - sv.roll_angle(list(sv.START_POSE), cfg) + 30) < 0.01, r
        assert (await sv.voice_act("roll_center", "straighten"))[1] and S.tgt[4] == cfg["joints"][4]["zero"]
    _server(go)


def test_the_gripper_pulse_always_ends_braked():
    async def go(sv, ser, page):
        S = sv.S
        assert (await sv.voice_act("close", "close"))[1]
        await asyncio.sleep(0.3); assert S.tgt[5] == 50.0, S.tgt[5]
        await sv.handle(page, {"type": "stop"}); await asyncio.sleep(0.05)
        assert S.tgt[5] == 90.0 and S.voice_task.done(), "STOP mid-pulse brakes it"
        assert (await sv.voice_act("open", "open", {"size": "small"}))[1]
        await _finish(sv); assert S.tgt[5] == S.cur[5] == 90.0
        S.follow = True
        try:
            assert (await sv.voice_act("close", "close"))[1] and S.follow, "the gripper doesn't end follow"
        finally: S.follow = False
        await _finish(sv)
    _server(go)


def _camera(**kw):
    """Enough of a Vision for the control loop, which feeds it the tip and asks its status every few ticks."""
    import types
    return types.SimpleNamespace(roll={"zero": 90.0, "sign": 1}, status=lambda: {"type": "vision"}, follow_overlay=None, **kw)


def test_up_and_down_while_following_change_the_hover_height():
    async def go(sv, ser, page):
        s = {"follow_hover": 110.0}
        sv.V = _camera(s=s, set_setting=lambda k, v: s.update({k: v}))
        tick = sv.follow_tick; sv.follow_tick = lambda now: None
        try:
            sv.S.follow = True; before = list(sv.S.tgt)
            assert (await sv.voice_act("up", "up"))[0] == "follow height 130 mm"
            assert (await sv.voice_act("down", "down", {"n": 50.0}))[0] == "follow height 80 mm"
            assert sv.S.follow and sv.S.tgt == before, "follow keeps going, and only the hover height changed"
        finally: sv.follow_tick = tick; sv.S.follow = False
    _server(go)


def test_speed_words_undo_and_again():
    async def go(sv, ser, page):
        S = sv.S; S.speed = 60.0
        for cmd, want in (("faster", 120), ("faster", 180), ("faster", 180), ("slower", 120), ("normal_speed", 60),
                          ("slower", 20), ("slower", 20), ("full_speed", 180)):
            await sv.voice_act(cmd, cmd); assert S.speed == want, (cmd, S.speed)
        start = list(S.tgt[:5]); y0 = _tip(sv)[1]
        await sv.voice_act("left", "left"); await _arrive(sv)
        assert (await sv.voice_act("again", "again"))[1] and abs(_tip(sv)[1] - (y0 - 40)) < 0.5, _tip(sv)
        await _arrive(sv)
        assert (await sv.voice_act("undo", "undo"))[1] and abs(_tip(sv)[1] - (y0 - 20)) < 0.5
        assert (await sv.voice_act("undo", "undo"))[1] and S.tgt[:5] == start
        assert (await sv.voice_act("undo", "undo")) == ("nothing to undo", False)
    _server(go)


def test_a_step_opens_the_no_name_window():
    async def go(sv, ser, page):
        ev = {"kind": "command", "cmd": "left", "say": "left", "gen": sv.HEAR.gen}
        await sv.voice_event(ev)
        assert ev["ok"] and sv.HEAR.awake[-1] == (5.0, set(sv.AFTER_STEP)), sv.HEAR.awake
        assert "relax" not in sv.AFTER_STEP and "stop" in sv.AFTER_STEP
    _server(go)


def test_stop_listening_and_mute():
    async def go(sv, ser, page):
        assert (await sv.voice_act("mute", "be quiet"))[1] and sv.HEAR.s["sounds"] is False
        assert (await sv.voice_act("unmute", "sounds on"))[1] and sv.HEAR.s["sounds"] is True
        assert (await sv.voice_act("listen_off", "stop listening"))[1] and sv.HEAR.s["mode"] == "off"
    _server(go)


def _fake_v(sv, seen=True):
    """A camera that sees a pen at (200, 0), and a start_pick that only records what it was asked."""
    import copy
    import vision
    s = copy.deepcopy(vision.DEFAULT_SETTINGS)
    det = {"x": 200.0, "y": 0.0, "closing": 30.0, "name": "pen", "conf": 0.8}
    v = _camera(s=s, detect_on=False, prompts=[], dets=[det] if seen else [], track=[(200.0, 0.0)] * 3,
                table_z=lambda x, y: 5.0, pose_health=lambda: (True, "ok"))
    v.best_target = lambda: dict(det) if seen and v.detect_on else None
    picks = []
    def start_pick(x, y, closing, name="", bring=False): picks.append((x, y, name, bring)); return True
    return v, picks, start_pick


def _pick_scenario(answer=None, confirm=True, seen=True, cmd="pick", obj="pen"):
    """Say "pick up the pen", then `answer` (a coroutine fn of sv and page) while it waits. Returns
    (picks asked for, the heard entry, whether it hovered over the pen first)."""
    out = {}
    async def go(sv, ser, page):
        v, picks, fake = _fake_v(sv, seen)
        sv.V, start, sv.start_pick = v, sv.start_pick, fake
        sv.HEAR.s["pick_confirm"] = confirm
        ask_s, sv.ASK_S = sv.ASK_S, 1.0
        try:
            ev = {"id": 1}
            r = await sv.voice_act(cmd, "pick", {"obj": obj}, ev)
            assert r[1] and v.detect_on and v.prompts == [obj], (r, v.prompts)
            t0 = time.time()
            while confirm and seen and sv.S.voice_ask is None and time.time() - t0 < 10: await asyncio.sleep(0.05)
            out["hovered"] = sv.math.dist(_tip(sv)[:2], (200.0, 0.0)) < 2.0
            if answer: await answer(sv, page)
            await _finish(sv)
            out["picks"], out["ev"] = picks, ev
        finally: sv.start_pick, sv.ASK_S = start, ask_s
    _server(go)
    return out["picks"], out["ev"], out["hovered"]


def test_a_spoken_pick_hovers_and_waits_for_yes():
    async def yes(sv, page): assert (await sv.voice_act("confirm", "yes")) == ("picking it up", True)
    picks, ev, hovered = _pick_scenario(yes)
    assert hovered and picks == [(200.0, 0.0, "pen", False)], (hovered, picks)
    assert ev["ok"] and "picking up pen" in ev["result"], ev


def test_a_spoken_pick_never_picks_on_no_silence_stop_or_another_command():
    async def no(sv, page): await sv.voice_act("cancel", "no")
    async def stop(sv, page): await sv.handle(page, {"type": "stop"})
    async def other(sv, page): await sv.voice_act("where", "where are you?")
    for answer in (no, None, stop, other):
        picks, ev, hovered = _pick_scenario(answer)
        assert hovered and picks == [], (answer, picks)


def test_a_spoken_pick_with_confirm_off_picks_at_once_and_bring_hands_over():
    picks, ev, hovered = _pick_scenario(confirm=False, cmd="bring")
    assert not hovered and picks == [(200.0, 0.0, "pen", True)], (hovered, picks)


def test_nothing_in_sight_is_no_pick():
    picks, ev, _ = _pick_scenario(confirm=True, seen=False)
    assert picks == [] and not ev["ok"] and "no pen in sight" in ev["result"], ev


def test_yes_with_nothing_waiting_does_nothing():
    async def go(sv, ser, page):
        assert (await sv.voice_act("confirm", "yes")) == ("nothing to confirm", False)
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
