"""hearing.py - the Voice tab: hear "Dummy, come here" and say which command it was.

Knows nothing about the arm. It reports what it heard and a command id, and server.py decides what
that command does. The pieces:
  normalize, Matcher   pure: transcript -> command id, with the wake word and learned aliases
  Segmenter            pure: per-frame speech probability -> utterances, and push-to-talk
  learn, Trainer       pure: the guided "say each phrase" calibration; learns how Whisper hears you
  quiet_threshold      pure: the speech threshold a quiet-room measurement calls for
  Hearing              mic -> Silero VAD -> Segmenter -> faster-whisper -> Matcher, in two threads
The wake word isn't a separate model. Every utterance the VAD finds is transcribed, and it counts
only if it starts with "Dummy" and the rest is a known command. That needs no training data and no
account, the word can be changed, and the Voice tab can show exactly what was heard.
Needs: pip install faster-whisper (plus nvidia-cublas-cu12 for the GPU). The pure parts are plain Python.
"""
import difflib, glob, importlib.util, io, json, os, queue, re, shutil, subprocess, sys, tempfile, threading, time, wave
from collections import OrderedDict, deque
from dataclasses import dataclass
import numpy as np

SR = 16000
FRAME = 512                  # Silero VAD's frame at 16 kHz: 32 ms
FRAME_S = FRAME / SR

# Command id -> what may follow the wake word. Matching ignores FILLER words; an exact phrase wins
# outright, anything else is fuzzy. test_voice.py checks that no two commands' phrases come close.
COMMANDS = {
    "follow": ("come here", "come over here", "come", "follow me", "follow my finger", "follow finger", "follow"),
    "home": ("start pose", "start position", "starting pose", "starting position", "go to start", "start", "go home", "home"),
    "relax": ("relax", "relax all", "go limp", "go to sleep", "sleep", "good night", "night night", "nap time"),
    "stop": ("stop", "stop it", "halt", "freeze", "stop following", "stay", "hold it"),
    # moves: one bounded step each, an amount optional ("up a bit", "left 3 cm", "roll right 45")
    "up": ("up", "go up", "move up", "higher", "raise", "lift"),
    "down": ("down", "go down", "move down", "lower"),
    "left": ("left", "go left", "move left", "to the left"),
    "right": ("right", "go right", "move right", "to the right"),
    "toward": ("toward me", "towards me", "come closer", "this way"),
    "away": ("away", "away from me", "further away", "go away"),
    "turn_left": ("turn left", "rotate left", "swing left"),
    "turn_right": ("turn right", "rotate right", "swing right"),
    "tilt_up": ("tilt up", "look up", "head up"),
    "tilt_down": ("tilt down", "look down", "head down"),
    "point_down": ("point down", "straight down", "face down"),
    "roll_left": ("roll left", "twist left", "wrist left"),
    "roll_right": ("roll right", "twist right", "wrist right"),
    "roll_center": ("straighten", "straighten the wrist", "roll centre", "roll center"),
    "open": ("open", "open up", "open the gripper", "open gripper", "open your hand", "let go", "let it go", "release", "drop it"),
    "close": ("close", "close the gripper", "close gripper", "close your hand", "grip", "squeeze", "clamp"),
    "rest": ("rest pose", "rest position", "fold up", "tuck in"),
    "wake": ("wake up", "get up", "stand up"),
    "undo": ("undo", "go back", "back", "undo that"),
    "again": ("again", "repeat", "once more", "one more time", "more", "do it again"),
    # pick and hand-over: "pick up the pen" / "bring me the pen" / "find the pen" are SLOTS, below
    "pick": ("pick it up", "pick that up", "pick up", "grab it"),
    "bring": ("bring me that", "bring me this"),
    "find": ("find it",),
    "handover": ("bring it to me", "bring it here", "bring it over", "hand it over", "give it to me", "give it here"),
    "put_down": ("put it down", "put down", "set it down", "place it"),
    "put_back": ("put it back", "put back"),
    # speed, follow, camera, questions
    "slower": ("slow down", "slow it down", "careful", "be careful", "gently"),
    "faster": ("faster", "speed up", "go faster", "quicker", "hurry up"),
    "full_speed": ("full speed", "top speed", "max speed"),
    "normal_speed": ("normal speed", "regular speed"),
    "hover_low": ("hover low",),
    "hover_high": ("hover high",),
    "hover_normal": ("hover normal", "normal height"),
    "detect_off": ("stop looking", "detect off", "stop searching"),
    "pose_ok": ("is the pose ok", "is the pose good", "are the markers ok", "check the camera", "can you see the table"),
    "what_see": ("what do you see", "what can you see", "what's there"),
    "where": ("where are you", "status", "report"),
    # voice itself
    "confirm": ("yes", "yeah", "yep", "yes please", "do it", "go ahead", "confirm"),
    "cancel": ("no", "nope", "cancel", "never mind", "no thanks"),
    "help": ("what can i say", "help", "commands", "what can you do"),
    "listen_off": ("stop listening", "voice off", "ignore me"),
    "mute": ("be quiet", "quiet", "shush", "shut up", "mute"),
    "unmute": ("sounds on", "unmute", "you can beep"),
    # personality (gestures.py): a question he answers yes with a nod, one he answers no to, praise
    "yes": ("are you awake", "are you there", "are you listening", "are you ready", "can you hear me",
            "are you happy", "do you like me", "are we friends", "how are you"),
    "no": ("are you dumb", "are you stupid", "are you an idiot", "are you broken", "are you useless",
           "are you a bad robot", "you're dumb", "you are dumb", "you're stupid", "you are stupid",
           "you dumb", "you stupid", "you idiot"),
    "praise": ("good boy", "who's a good boy", "good job", "well done", "good robot"),
    "dance": ("dance", "let's dance", "party", "do a dance"),
    "hello": ("hello", "hi", "say hi", "say hello", "good morning", "wave", "bye", "bye bye", "goodbye",
              "see you", "see you later", "see ya"),
    "thanks": ("thank you", "thanks", "thanks a lot", "take a bow", "bow"),
    "scold": ("bad boy", "bad robot", "naughty", "shame on you"),
    "trick": ("do a trick", "show off", "surprise me"),
}
# What the Voice tab shows for each (and what the Commands page lists it as)
SAY = {"follow": "come here", "home": "start pose", "relax": "relax", "stop": "stop",
       "up": "up", "down": "down", "left": "left", "right": "right", "toward": "toward me", "away": "away",
       "turn_left": "turn left", "turn_right": "turn right", "tilt_up": "tilt up", "tilt_down": "tilt down",
       "point_down": "point down", "roll_left": "roll left", "roll_right": "roll right", "roll_center": "straighten",
       "open": "open", "close": "close", "rest": "rest pose", "wake": "wake up", "undo": "undo", "again": "again",
       "pick": "pick up the pen", "bring": "bring me the pen", "find": "find the pen", "handover": "bring it to me",
       "put_down": "put it down", "put_back": "put it back",
       "slower": "slow down", "faster": "faster", "full_speed": "full speed", "normal_speed": "normal speed",
       "hover_low": "hover low", "hover_high": "hover high", "hover_normal": "normal height",
       "detect_off": "stop looking", "pose_ok": "is the pose OK?", "what_see": "what do you see?", "where": "where are you?",
       "confirm": "yes", "cancel": "no", "help": "what can I say?", "listen_off": "stop listening",
       "mute": "be quiet", "unmute": "sounds on",
       "yes": "a yes question", "no": "a no question", "praise": "good boy", "dance": "dance", "hello": "hello",
       "thanks": "thank you", "scold": "bad boy", "trick": "do a trick"}
# One line of what each does, for the Commands page. "huh" is no command: what an unknown phrase gets.
HELP = {"follow": "finger follow on", "home": "the start pose", "relax": "park on the desk, then go limp",
        "stop": "STOP: holds where it is, gripper braked",
        "up": "20 mm up (a bit 10, a lot 50). While following: the hover height", "down": "20 mm down, the same",
        "left": "20 mm to your left", "right": "20 mm to your right", "toward": "20 mm toward you", "away": "20 mm away from you",
        "turn_left": "the base 15° to your left", "turn_right": "the base 15° to your right",
        "tilt_up": "the tool 15° up, tip in place", "tilt_down": "the tool 15° down, tip in place",
        "point_down": "the tool straight down", "roll_left": "the wrist 30°, top to your left",
        "roll_right": "the wrist 30°, top to your right", "roll_center": "the wrist back to roll zero",
        "open": "the gripper opens for 1 s (a bit 0.4, a lot 2.5), then brakes",
        "close": "the gripper closes for 1.5 s, then brakes", "rest": "park on the desk, staying engaged",
        "wake": "wake every servo, then the start pose", "undo": "back to before the last spoken move",
        "again": "the last step once more",
        "pick": "find it and pick it up; with Confirm picks on, it hovers over it and waits for yes",
        "bring": "pick it up, then bring it over to you", "find": "look for it: a nod if it's there, a shake if not",
        "handover": "carry what it holds over to you, then wait for let go",
        "put_down": "put it down on the table right here", "put_back": "put it back where it was picked",
        "slower": "one speed step slower", "faster": "one speed step faster", "full_speed": "speed 180",
        "normal_speed": "speed 60", "hover_low": "follow height Low", "hover_high": "follow height High",
        "hover_normal": "follow height Hover", "detect_off": "Detect off",
        "pose_ok": "a nod if the camera pose is good, a shake (and why) if not",
        "what_see": "what Detect sees, with a nod or a shake", "where": "where the tool is, Live, speed and follow",
        "confirm": "answers a pick waiting for yes", "cancel": "says no to it", "help": "opens this page",
        "listen_off": "voice off", "mute": "no more beeps", "unmute": "beeps again",
        "yes": "a nod (are you awake, ready, happy, there...)", "no": "a head shake (are you dumb, stupid, broken...)",
        "praise": "the happy dance (good job, well done, good robot)", "dance": "the dance, on request",
        "hello": "a wave (hi, bye, good morning, see you)", "thanks": "a bow", "scold": "a sad droop",
        "trick": "a random one", "huh": "a puzzled head tilt"}
GROUPS = (("Basics", ("follow", "home", "relax", "stop")),
          ("Moves", ("up", "down", "left", "right", "toward", "away", "turn_left", "turn_right", "tilt_up", "tilt_down",
                     "point_down", "roll_left", "roll_right", "roll_center", "open", "close", "rest", "wake", "undo", "again")),
          ("Pick and hand-over", ("pick", "bring", "handover", "put_down", "put_back")),
          ("Speed, follow, camera", ("slower", "faster", "full_speed", "normal_speed", "hover_low", "hover_high",
                                     "hover_normal", "find", "detect_off", "pose_ok", "what_see", "where")),
          ("Voice", ("confirm", "cancel", "help", "listen_off", "mute", "unmute")),
          ("Personality", ("yes", "no", "praise", "dance", "hello", "thanks", "scold", "trick", "huh")))
REACTIONS = ("yes", "no", "praise", "dance", "hello", "thanks", "scold", "trick")   # body language, not tasks
# Phrases that read like one command but usually mean another: they never act, the Heard list says why
AMBIGUOUS = {"slower": 'too close to "lower": say "slow down"', "closer": 'too close to "close": say "toward me"'}
# The object slots: verb, then up to 3 words of object ("pick up the red pen", "do you see my scissors")
SLOTS = (("pick", ("pick up", "pick", "grab", "fetch", "get")),
         ("bring", ("bring me", "give me", "hand me", "get me", "bring")),
         ("find", ("find", "look for", "search for", "where is", "where's", "do see", "see")))
THIS = frozenset(("it", "that", "this", "one", "that one", "this one", "them"))   # = what Detect sees now
# Commands that take an amount, and the words for one
AMOUNT_CMDS = frozenset(("up", "down", "left", "right", "toward", "away", "turn_left", "turn_right", "tilt_up",
                         "tilt_down", "roll_left", "roll_right", "open", "close"))
SMALL, BIG = frozenset("bit little slightly tiny touch smidge".split()), frozenset("lot lots much".split())
UNITS = {"mm": "mm", "millimeter": "mm", "millimeters": "mm", "millimetre": "mm", "millimetres": "mm",
         "cm": "cm", "centimeter": "cm", "centimeters": "cm", "centimetre": "cm", "centimetres": "cm",
         "degree": "deg", "degrees": "deg", "deg": "deg"}
ONES = {w: i for i, w in enumerate("zero one two three four five six seven eight nine ten eleven twelve thirteen "
                                   "fourteen fifteen sixteen seventeen eighteen nineteen".split())}
TENS = {w: 10 * (i + 2) for i, w in enumerate("twenty thirty forty fifty sixty seventy eighty ninety".split())}
NEGATIONS = frozenset(("not", "never"))
FILLER = frozenset("please now uh um er hey ok okay so the to go and a an your you can could would just".split())
MODELS = ("small.en", "medium.en", "large-v3-turbo")

WAKE_WITHIN = 3      # the wake word must be one of the first words: "hey dummy", "ok so dummy"
CMD_SIM = 0.8        # how close what follows must be to a command phrase
CMD_MARGIN = 0.1     # ...and how much closer than to any other command
NO_SPEECH = 0.5      # Whisper's own "this was not speech" above this: the text is ignored

DEFAULTS = {
    "mode": "off",           # off | ptt (push to talk only) | wake (wake word, and push to talk too)
    "mic": "",               # '' = the system default input
    "model": "small.en",
    "wake_word": "dummy",
    "vad_threshold": 0.5,    # Silero speech probability that starts an utterance
    "end_silence": 0.5,      # s of quiet that ends one ("pause before acting")
    "awake_window": 5.0,     # s it keeps listening after a bare "Dummy"
    "dry_run": False,        # hear and show, never act
    "sounds": True,          # beeps on the laptop: heard the name / done / refused
    "wake_aliases": [],      # learned: other ways Whisper writes the wake word for this voice
    "phrase_aliases": {},    # learned: command id -> other ways it writes that command
    "noise_db": None,        # from the last quiet-room check
    "personality": True,     # nod / shake / dance (gestures.py)
    "gesture_size": 1.0,     # gesture amplitude, x the choreography
    "gesture_speed": 120.0,  # deg/s while gesturing, whatever the speed slider says
    "dance_clap": True,      # the gripper claps in the dance
    "reactions": {},         # your own extra phrases: {"yes" / "no" / "praise" / "hello" / ...: [...]}
    "pick_confirm": True,    # a spoken pick hovers over what it found and waits for "yes"
}


def normalize(text):
    """Lower-case words, punctuation dropped: "Dummy, come here!" -> ['dummy', 'come', 'here']."""
    return re.findall(r"[a-z0-9']+", str(text).lower().replace("’", "'"))


def _core(words): return [w for w in words if w not in FILLER]


def _negated(words): return any(w in NEGATIONS or w.endswith("n't") for w in words)


def sim(a, b): return difflib.SequenceMatcher(None, a, b).ratio()


def spellings(wake):
    """The wake word as Whisper may spell it: dummy -> dummy, dummie, dummey, dummi, dummy's. Only these
    and the learned aliases count, never a fuzzy match: "mummy" or "tummy" would then be a trigger for
    everyone, while calibration learns them only for a voice that really sounds like that."""
    w = " ".join(normalize(wake)); out = [w, w + "'s"]
    if w.endswith("y"): out += [w[:-1] + "ie", w[:-1] + "ey", w[:-1] + "i"]
    return out


def _number(words, i):
    """(value, words used) for a number starting at words[i]: digits ("30", "3cm") or words
    ("thirty five", "a hundred" once the filler is gone), else (None, 0)."""
    w = words[i]
    m = re.fullmatch(r"(\d+)(mm|cm)?", w)
    if m: return float(m.group(1)), 1
    if w == "hundred": return 100.0, 1
    if w in TENS:
        if i + 1 < len(words) and words[i + 1] in ONES and ONES[words[i + 1]] < 10: return float(TENS[w] + ONES[words[i + 1]]), 2
        return float(TENS[w]), 1
    if w in ONES: return float(ONES[w]), 1
    return None, 0


def parse_amount(words):
    """Filler-free words -> (the words without their amount, amount or None). The amount is
    {"n": number or None, "unit": "mm" / "cm" / "deg" or None, "size": "small" / "big" or None}:
    "up 3 cm" -> (['up'], n 3 cm); "up a little bit" -> (['up'], small)."""
    out, n, unit, size, i = [], None, None, None, 0
    while i < len(words):
        w = words[i]
        v, used = (None, 0) if n is not None else _number(words, i)
        if used:
            n = v; i += used
            g = re.fullmatch(r"\d+(mm|cm)", w)
            if g: unit = g.group(1)
            elif i < len(words) and words[i] in UNITS: unit = UNITS[words[i]]; i += 1
            continue
        if w in SMALL: size = "small"
        elif w in BIG: size = "big"
        else: out.append(w)
        i += 1
    return (out, {"n": n, "unit": unit, "size": size}) if n is not None or size else (words, None)


class Matcher:
    """Transcript -> command. Pure, so the whole policy is tested offline from text."""

    def __init__(self, wake="dummy", wake_aliases=(), phrase_aliases=None):
        self.wake = " ".join(normalize(wake)) or "dummy"
        self.wakes = [normalize(w) for w in spellings(self.wake) + list(wake_aliases) if normalize(w)]
        extra = {c: [p for p in ps if _core(normalize(p))] for c, ps in (phrase_aliases or {}).items() if c in COMMANDS}
        table = {c: list(ps) + extra.get(c, []) for c, ps in COMMANDS.items()}
        table.update({"?" + p: [p] for p in AMBIGUOUS})          # near-misses compete, and never act
        core = {c: [" ".join(_core(normalize(p))) for p in ps] for c, ps in table.items()}
        # A phrase whose filler-free form reads like another command counts only as said: "let go"
        # without its filler is "let", too close to "left" to fuzzy-match or to match bare
        def as_said(c, p):
            k = " ".join(_core(normalize(p)))
            return k != " ".join(normalize(p)) and any(sim(k, o) >= CMD_SIM for d, ks in core.items() if d != c for o in ks)
        self.phrases = {c: [" ".join(_core(normalize(p))) for p in ps if not as_said(c, p)] for c, ps in table.items()}
        # exact: the words as said, or without the filler; an exact phrase is never a near miss
        self.exact = {}
        for c, ps in table.items():
            for p in ps:
                self.exact.setdefault(" ".join(normalize(p)), c)
                if not as_said(c, p): self.exact.setdefault(" ".join(_core(normalize(p))), c)
        self.slots = sorted(((c, v.split()) for c, vs in SLOTS for v in vs), key=lambda cv: -len(cv[1]))

    def find_wake(self, words):
        """Index just past the wake word if it is among the first WAKE_WITHIN words, else None."""
        for i in range(min(WAKE_WITHIN, len(words))):
            for w in self.wakes:
                if words[i:i + len(w)] == w: return i + len(w)
        return None

    def scores(self, words):
        """Best similarity of the filler-free words to each command's phrases. A not / never / n't the
        phrase itself lacks rules it out: "you're not dumb" is no insult, "don't relax" no command."""
        cw = _core(words); core = " ".join(cw); neg = _negated(cw)
        def score(p): return 0.0 if neg and not _negated(p.split()) else 1.0 if core == p else sim(core, p)
        return {c: max(score(p) for p in ps) for c, ps in self.phrases.items() if ps} if core else {}

    def _fuzzy(self, words):
        sc = sorted(self.scores(words).items(), key=lambda kv: -kv[1])
        if not sc: return None, 0.0, 0.0
        best, margin = sc[0][1], sc[0][1] - (sc[1][1] if len(sc) > 1 else 0.0)
        return (sc[0][0] if best >= CMD_SIM and margin >= CMD_MARGIN else None), best, margin

    def _slot(self, cw):
        """(command, object or None) for "pick up the pen" / "bring me it" / "do you see my scissors", else None."""
        for c, v in self.slots:
            if cw[:len(v)] != v: continue
            obj = cw[len(v):]
            if obj and obj[-1] == "up": obj = obj[:-1]                      # "pick the pen up"
            obj = [w for w in obj if w not in ("me", "my", "some")]
            if not obj or len(obj) > 3: return None
            return c, None if " ".join(obj) in THIS else " ".join(obj)
        return None

    def parse(self, words):
        """The words after the wake word -> (command or None, score, margin, arg, hint). In order: an
        exact phrase; an object slot; a command with an amount; the fuzzy match. arg is an amount
        (parse_amount) or {"obj": name or None}; hint says why a near miss didn't act."""
        cw = _core(words)
        if not cw: return None, 0.0, 0.0, None, ""
        hit = self.exact.get(" ".join(words)) or self.exact.get(" ".join(cw))
        if hit is None and not _negated(cw):
            s = self._slot(cw)
            if s: return s[0], 1.0, 1.0, {"obj": s[1]}, ""
            rest, amount = parse_amount(cw)
            if amount and rest:
                c = self.exact.get(" ".join(rest))
                c, score, margin = (c, 1.0, 1.0) if c else self._fuzzy(rest)
                if c in AMOUNT_CMDS: return c, score, margin, amount, ""
        cmd, score, margin = (hit, 1.0, 1.0) if hit else self._fuzzy(words)
        if cmd and cmd.startswith("?"): return None, score, margin, None, AMBIGUOUS[cmd[1:]]
        return cmd, score, margin, ({"obj": None} if cmd in ("pick", "bring", "find") else None), ""

    def match_command(self, words):
        """(command or None, best score, margin over the runner-up command)."""
        return self.parse(words)[:3]

    def match(self, text, need_wake=True):
        """What a transcript means. kind is one of:
        empty    nothing (or only noise) was heard
        nowake   speech without the wake word where one was needed: never acts
        wake     only the wake word: the caller opens its listening window
        unknown  the wake word (or push to talk) and then something that is not a command
        command  cmd says which"""
        words = normalize(text)
        if not words: return {"kind": "empty"}
        i = self.find_wake(words)
        if i is None and need_wake: return {"kind": "nowake"}
        rest = words[i:] if i is not None else words
        if i is not None and not _core(rest): return {"kind": "wake", "wake": True}
        cmd, score, margin, arg, hint = self.parse(rest)
        r = {"kind": "command" if cmd else "unknown", "cmd": cmd, "score": round(score, 2),
             "margin": round(margin, 2), "wake": i is not None, "words": len(_core(rest))}
        if arg: r["arg"] = arg
        if hint: r["hint"] = hint
        return r


@dataclass
class Utterance:
    audio: np.ndarray        # float32 mono 16 kHz, including the pre-roll
    speech_s: float          # seconds the VAD called speech
    source: str              # 'vad' or 'ptt'
    peak_db: float = -90.0
    t_end: float = 0.0       # time.time() when it was cut (stamped by Hearing)
    gen: int = 0             # Hearing.gen when it started: from before a STOP, Off or calibration change it never acts


class Segmenter:
    """Per-frame speech probability -> utterances. Pure: feed() frames in, get finished ones back.

    Speech starts after START_FRAMES frames at or above the threshold and ends after end_silence
    seconds below threshold - 0.15 (Silero's own hysteresis). Each utterance starts pre_roll seconds
    early, so the first syllable isn't lost to the detector's reaction time, and blips with less than
    min_speech seconds of speech are dropped. Push-to-talk overrides all of it: everything from
    ptt(True) to ptt(False) plus the pre-roll and a short tail is one utterance, speech or not."""
    START_FRAMES = 2

    def __init__(self, thr=0.5, end_silence=0.5, pre_roll=0.4, max_len=8.0, min_speech=0.15, tail=0.3, vad=True):
        self.thr, self.end_silence, self.max_len, self.min_speech, self.tail = thr, end_silence, max_len, min_speech, tail
        self.vad = vad              # False: only push-to-talk makes utterances
        self.pre = deque(maxlen=max(1, int(round(pre_roll / FRAME_S))) + self.START_FRAMES)   # + the frames that confirm the start
        self.cur = None             # frames of the utterance being collected
        self.run = self.n_speech = self.n_quiet = self.tail_left = 0
        self.ptt_on = False
        self.dropped = 0            # blips too short to be speech

    @property
    def active(self): return self.cur is not None

    def ptt(self, on):
        if on and not self.ptt_on:
            self.ptt_on, self.tail_left = True, 0
            if self.cur is None: self.cur = list(self.pre); self.pre.clear(); self.n_speech = 0
        elif not on and self.ptt_on:
            self.ptt_on = False; self.tail_left = max(1, int(round(self.tail / FRAME_S)))

    def reset(self):
        self.cur = None; self.pre.clear(); self.run = self.n_speech = self.n_quiet = self.tail_left = 0; self.ptt_on = False

    def _finish(self, source):
        frames, self.cur = self.cur, None
        speech_s, self.run, self.n_quiet = self.n_speech * FRAME_S, 0, 0
        if source == "vad" and speech_s < self.min_speech:
            self.dropped += 1; return None
        a = np.concatenate(frames).astype(np.float32)
        return Utterance(a, speech_s, source, max(-90.0, float(20 * np.log10(np.abs(a).max() + 1e-9))))

    def feed(self, frame, p):
        """One FRAME of audio and its speech probability. Returns the utterances it finished (0 or 1)."""
        if self.ptt_on or self.tail_left:                      # push to talk: take everything
            self.cur.append(frame)
            if p >= self.thr: self.n_speech += 1
            if self.tail_left:
                self.tail_left -= 1
                if not self.tail_left: u = self._finish("ptt"); return [u] if u else []
            elif len(self.cur) * FRAME_S >= 30.0:              # a stuck button
                self.ptt_on = False; u = self._finish("ptt"); return [u] if u else []
            return []
        if self.cur is None:
            self.pre.append(frame)
            self.run = self.run + 1 if (self.vad and p >= self.thr) else 0
            if self.run >= self.START_FRAMES:
                self.cur = list(self.pre); self.pre.clear(); self.n_speech, self.n_quiet = self.run, 0
            return []
        self.cur.append(frame)
        if p >= self.thr: self.n_speech += 1; self.n_quiet = 0
        elif p < self.thr - 0.15: self.n_quiet += 1
        if self.n_quiet * FRAME_S >= self.end_silence or len(self.cur) * FRAME_S >= self.max_len:
            u = self._finish("vad"); return [u] if u else []
        return []


def said(r):
    """What a matched command was, for the Heard list and the log: the object of a pick as said."""
    forms = {"pick": ("pick up the {}", "pick it up"), "bring": ("bring me the {}", "bring it"), "find": ("find the {}", "find it")}.get(r["cmd"])
    if not forms: return SAY[r["cmd"]]
    obj = (r.get("arg") or {}).get("obj")
    return forms[0].format(obj) if obj else forms[1]


def quiet_threshold(probs):
    """(threshold, p99) for a room whose non-speech scored these probabilities: just above the
    loudest 1% of them, never below Silero's default 0.5, never so high that speech gets missed."""
    p99 = float(np.percentile(probs, 99)) if len(probs) else 0.0
    return round(min(0.85, max(0.5, p99 + 0.15)), 2), round(p99, 2)


def learn(text, cmd, m):
    """What one training take teaches. Returns (ok, wake_alias, phrase_alias, why).
    ok: the take counts as understood (possibly thanks to what it learned). Learning is cautious: a
    wake alias must look at least a bit like the wake word, and a phrase alias must be short and
    must not look like another command, so chatter doesn't become a trigger."""
    words = normalize(text)
    if not words: return False, None, None, "heard nothing"
    r = m.match(text, need_wake=True)
    if r["kind"] == "command":
        return (True, None, None, "") if r["cmd"] == cmd else (False, None, None, f'sounded like "{SAY[r["cmd"]]}"')
    wake_alias, i = None, m.find_wake(words)
    if i is None:
        cands = [" ".join(words[:k]) for k in (1, 2) if len(words) > k]
        best = max(cands, key=lambda c: sim(c, m.wake), default=None)
        if best is None or sim(best, m.wake) < 0.5: return False, None, None, f'didn\'t hear "{m.wake.title()}" at the start'
        wake_alias, i = best, len(best.split())
    rest = words[i:]
    if m.match_command(rest)[0] == cmd: return True, wake_alias, None, ""
    core = " ".join(_core(rest))
    if not core: return False, None, None, "heard only the wake word"
    if len(core.split()) > 4: return False, None, None, "too many words to learn"
    near = [c for c, v in sorted(m.scores(rest).items(), key=lambda kv: -kv[1]) if c != cmd and v >= 0.7]
    if near: return False, None, None, f'"{core}" is too close to "{SAY.get(near[0], near[0].lstrip("?"))}"'
    return True, wake_alias, core, ""


class Trainer:
    """The guided "Your voice" calibration: each command TAKES times, in order."""
    TAKES = 2
    ORDER = ("follow", "home", "relax", "stop")

    def __init__(self, wake="dummy"):
        self.wake = wake.title()
        self.steps = [c for c in self.ORDER for _ in range(self.TAKES)]
        self.i, self.results, self.learned = 0, [], []

    @property
    def done(self): return self.i >= len(self.steps)

    def expect(self): return None if self.done else self.steps[self.i]

    def say(self):
        c = self.expect(); return f"{self.wake}, {SAY[c]}" if c else ""

    def take(self, text, m):
        """Score one take (text from Whisper) against the expected command. Returns the result dict;
        a take that heard nothing at all doesn't use up the step."""
        cmd = self.expect()
        ok, wa, pa, why = learn(text, cmd, m)
        r = {"cmd": cmd, "say": self.say(), "text": text, "ok": ok, "why": why, "learned": []}
        if wa: r["learned"].append(["wake", wa])
        if pa: r["learned"].append([cmd, pa])
        self.learned += r["learned"]
        if text.strip(): self.i += 1; self.results.append(r)
        return r

    def summary(self):
        return {"ok": sum(1 for r in self.results if r["ok"]), "n": len(self.results), "learned": list(self.learned)}


# ------------------------------------------------------------------ runtime (mic, VAD, Whisper)

def _run(cmd, timeout=3.0):
    try: return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout).stdout
    except (OSError, subprocess.SubprocessError): return ""


def _pulse():
    return sys.platform.startswith("linux") and shutil.which("parec") and shutil.which("pactl")


def list_mics():
    """[{id, name}] of microphones, the system default first (id '')."""
    out = [{"id": "", "name": "System default"}]
    if _pulse():
        try: srcs = json.loads(_run(["pactl", "-f", "json", "list", "sources"]) or "[]")
        except ValueError: srcs = []
        dflt = _run(["pactl", "get-default-source"]).strip()
        for s in srcs:
            name = s.get("name", "")
            if name.endswith(".monitor") or s.get("monitor_of_sink") not in (None, "", "n/a"): continue   # speaker loopbacks
            label = s.get("description") or name
            if name == dflt: out[0]["name"] = f"System default ({label})"
            out.append({"id": name, "name": label})
        return out
    try:
        import sounddevice as sd
        for i, d in enumerate(sd.query_devices()):
            if d.get("max_input_channels", 0) > 0: out.append({"id": str(i), "name": d["name"]})
    except Exception: pass
    return out


class ParecMic:
    """The mic through PipeWire / PulseAudio's parec: no PortAudio needed, and the sound server
    resamples to 16 kHz mono for us."""
    def __init__(self, device):
        cmd = ["parec", "--raw", f"--rate={SR}", "--channels=1", "--format=s16le", "--latency-msec=30"]
        if device: cmd.append(f"--device={device}")
        self.p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)

    def read(self):
        need, buf = FRAME * 2, b""
        while len(buf) < need:
            b = self.p.stdout.read(need - len(buf))
            if not b: raise EOFError(self.p.stderr.read().decode(errors="replace").strip() or "the microphone stream ended")
            buf += b
        return np.frombuffer(buf, np.int16).astype(np.float32) / 32768.0

    def close(self):
        try: self.p.kill(); self.p.wait(1)
        except Exception: pass


class SdMic:
    """The mic through PortAudio (sounddevice): Windows and macOS, or Linux without parec."""
    def __init__(self, device):
        import sounddevice as sd
        dev = int(device) if str(device).isdigit() else (device or None)
        self.s = sd.RawInputStream(samplerate=SR, channels=1, dtype="int16", blocksize=FRAME, device=dev)
        self.s.start()

    def read(self):
        data, _ = self.s.read(FRAME)
        return np.frombuffer(bytes(data), np.int16).astype(np.float32) / 32768.0

    def close(self):
        try: self.s.stop(); self.s.close()
        except Exception: pass


class WavMic:
    """For testing without talking: DUMMY_VOICE_WAV=<a .wav, or a folder of them> replaces the mic.
    Plays them in real time, 2 s of faint noise apart, then goes quiet."""
    def __init__(self, path):
        files = sorted(glob.glob(os.path.join(path, "*.wav"))) if os.path.isdir(path) else [path]
        rng, parts = np.random.default_rng(0), []
        for f in files:
            with wave.open(f) as w: a = np.frombuffer(w.readframes(w.getnframes()), np.int16).astype(np.float32) / 32768.0
            parts += [(rng.standard_normal(2 * SR) * 0.002).astype(np.float32), a]
        self.a = np.concatenate(parts + [np.zeros(FRAME, np.float32)]); self.i = 0; self.t0 = None

    def read(self):
        if self.t0 is None: self.t0 = time.time()
        wait = self.t0 + self.i / SR - time.time()
        if wait > 0: time.sleep(wait)
        f = self.a[self.i:self.i + FRAME]; self.i += FRAME
        return f if len(f) == FRAME else np.zeros(FRAME, np.float32)

    def close(self): pass


def open_mic(device):
    if os.environ.get("DUMMY_VOICE_WAV"): return WavMic(os.environ["DUMMY_VOICE_WAV"])
    return ParecMic(device) if _pulse() else SdMic(device)


def _pkg_dir(name):
    spec = importlib.util.find_spec(name)
    return list(spec.submodule_search_locations)[0] if spec and spec.submodule_search_locations else None


class SileroVAD:
    """Silero VAD v6 (the ONNX file faster-whisper ships), streamed one 32 ms frame at a time."""
    def __init__(self):
        import onnxruntime
        path = os.path.join(_pkg_dir("faster_whisper"), "assets", "silero_vad_v6.onnx")
        o = onnxruntime.SessionOptions(); o.inter_op_num_threads = o.intra_op_num_threads = 1; o.log_severity_level = 4
        self.s = onnxruntime.InferenceSession(path, providers=["CPUExecutionProvider"], sess_options=o)
        self.reset()

    def reset(self):
        self.h = np.zeros((1, 1, 128), np.float32); self.c = np.zeros((1, 1, 128), np.float32); self.ctx = np.zeros(64, np.float32)

    def __call__(self, frame):
        x = np.concatenate([self.ctx, frame]).astype(np.float32)[None, :]
        p, self.h, self.c = self.s.run(None, {"input": x, "h": self.h, "c": self.c})
        self.ctx = frame[-64:].astype(np.float32)
        return float(np.asarray(p).reshape(-1)[0])


def _load_cublas12():
    """CTranslate2 (inside faster-whisper) is built against CUDA 12, while torch here brings CUDA 13,
    so its cuBLAS 12 comes from the nvidia-cublas-cu12 wheel. That isn't on the loader's search path:
    load it by full path first, and CTranslate2's own dlopen then finds it already loaded."""
    for p in sys.path:
        if sys.platform == "win32":
            d = os.path.join(p, "nvidia", "cublas", "bin")
            if glob.glob(os.path.join(d, "cublas64_12.dll")): os.add_dll_directory(d); return True
        else:
            libs = [os.path.join(p, "nvidia", "cublas", "lib", n) for n in ("libcublasLt.so.12", "libcublas.so.12")]
            if all(os.path.exists(x) for x in libs):
                import ctypes
                for x in libs: ctypes.CDLL(x, mode=ctypes.RTLD_GLOBAL)
                return True
    return False


class Recognizer:
    """faster-whisper on the GPU when it works, else int8 on the CPU (~10x slower, still usable)."""
    def __init__(self, name, status):
        self.name, self.status, self.m, self.device = name, status, None, ""

    def load(self):
        from faster_whisper import WhisperModel
        import ctranslate2
        gpu = ctranslate2.get_cuda_device_count() > 0 and _load_cublas12()
        for dev, ct in ([("cuda", "int8_float16")] if gpu else []) + [("cpu", "int8")]:
            try:
                try: m = WhisperModel(self.name, device=dev, compute_type=ct, cpu_threads=8, local_files_only=True)
                except Exception:
                    self.status(f"downloading {self.name} (first time only)")
                    m = WhisperModel(self.name, device=dev, compute_type=ct, cpu_threads=8)
                list(m.transcribe(np.zeros(SR // 2, np.float32), language="en", beam_size=1)[0])   # fails here if cuBLAS is wrong
                self.m, self.device = m, dev
                return dev
            except Exception as e:
                if dev == "cpu": raise
                self.status(f"GPU unavailable ({type(e).__name__}: {e}), using the CPU")
        return self.device

    def __call__(self, audio, hotwords):
        t = time.time()
        segs, _ = self.m.transcribe(audio, language="en", beam_size=3, temperature=0.0, without_timestamps=True,
                                    condition_on_previous_text=False, vad_filter=False, hotwords=hotwords)
        segs = list(segs)
        text = " ".join(s.text.strip() for s in segs).strip()
        nsp = max((s.no_speech_prob for s in segs), default=1.0)
        return text, nsp, (time.time() - t) * 1000.0


def _tone(parts, vol=0.22):
    out = []
    for f, ms in parts:
        n = int(SR * ms / 1000); t = np.arange(n) / SR
        a = np.sin(2 * np.pi * f * t) if f else np.zeros(n)
        ramp = min(n // 4, int(SR * 0.008))
        if ramp: env = np.ones(n); env[:ramp] = np.linspace(0, 1, ramp); env[-ramp:] = np.linspace(1, 0, ramp); a = a * env
        out.append(a)
    a = (np.concatenate(out) * vol * 32767).astype(np.int16)
    b = io.BytesIO()
    with wave.open(b, "wb") as w: w.setnchannels(1); w.setsampwidth(2); w.setframerate(SR); w.writeframes(a.tobytes())
    return b.getvalue(), len(a) / SR


SOUNDS = {"wake": [(660, 70), (0, 25), (990, 90)],    # heard the name: say the command
          "ok": [(880, 60), (0, 20), (1320, 80)],     # done
          "no": [(300, 110), (0, 40), (220, 160)],    # heard you, but refused or not understood
          "nope": [(392, 150), (0, 70), (330, 240)],  # "uh-uh", with the head shake
          "happy": [(523, 90), (0, 20), (659, 90), (0, 20), (784, 90), (0, 20), (1047, 110), (0, 40), (784, 80), (0, 20), (1047, 220)],
          "hello": [(587, 90), (0, 30), (880, 160)],                       # "hi-i", with the wave
          "bow": [(784, 120), (0, 30), (659, 120), (0, 30), (523, 260)],    # a little curtsey down the scale
          "sad": [(440, 220), (0, 40), (392, 220), (0, 40), (311, 420)],    # the droop
          "huh": [(330, 90), (0, 20), (494, 180)],                         # a puzzled "hm?", with the head tilt
          "ask": [(523, 80), (0, 30), (659, 80), (0, 30), (880, 140)]}      # say yes or no


class Sounds:
    """Earcons on the laptop's speaker, played without waiting."""
    def __init__(self):
        self.files, self.player = {}, None
        if sys.platform.startswith("linux"): self.player = shutil.which("pw-play") or shutil.which("paplay")
        d = os.path.join(tempfile.gettempdir(), f"dummy-voice-{os.getuid() if hasattr(os, 'getuid') else 0}")
        os.makedirs(d, exist_ok=True)
        for k, parts in SOUNDS.items():
            data, dur = _tone(parts); p = os.path.join(d, k + ".wav")
            with open(p, "wb") as f: f.write(data)
            self.files[k] = (p, dur)

    def play(self, name):
        """Starts the sound and returns its length in seconds (0 if it could not play)."""
        p, dur = self.files[name]
        try:
            if self.player: subprocess.Popen([self.player, p], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            elif sys.platform == "win32":
                import winsound; winsound.PlaySound(p, winsound.SND_FILENAME | winsound.SND_ASYNC)
            else: return 0.0
        except Exception: return 0.0
        return dur


class Hearing:
    """The runtime. Two daemon threads while the mode isn't off (or a calibration runs):
      capture    mic -> level meter -> Silero VAD -> Segmenter -> queue
      recognise  queue -> Whisper -> Matcher (or the Trainer) -> on_event(dict)
    on_event is called from the recognise thread; the server marshals it onto its own loop.
    Settings are a plain dict that save(dict) persists (voice.json)."""
    CLIPS = 20

    def __init__(self, settings, log, on_event, save):
        self.s = dict(DEFAULTS); self.s.update({k: v for k, v in (settings or {}).items() if k in DEFAULTS})
        self.log, self.on_event, self.save = log, on_event, save
        self.matcher = self._matcher(); self.sounds = None
        self.seg = Segmenter(thr=self.s["vad_threshold"], end_silence=self.s["end_silence"])
        self.q = queue.Queue(maxsize=4)
        self.level, self.speech, self.noise_db, self.speech_db = -90.0, False, self.s.get("noise_db"), None
        self.state_note, self.error, self.device, self.asr_ms = "", "", "", None
        self.thinking = False; self.awake_until = self.mute_until = 0.0; self.ptt_on = False
        self.awake_only = None       # the commands the no-name window takes (None: any, after a bare name)
        self.mics, self.clips, self.n = list_mics(), OrderedDict(), 0
        self.ok, self.why = self.available()
        self.train = None; self.train_last = None; self.train_summary = None
        self.quiet = None; self.quiet_result = None
        self._run = False; self._cap = self._rec = None; self._mic_dirty = self._model_dirty = False
        # the Segmenter belongs to the capture thread: other threads only ask, and it applies the ask per frame
        self._ptt_want = False; self._seg_reset = False
        self.gen = self._utt_gen = 0
        self._asr = None; self._lv = deque(maxlen=4); self._quiet_db = deque(maxlen=300)

    # ---------- availability, settings ----------
    @staticmethod
    def available():
        for mod, pip in (("faster_whisper", "faster-whisper"), ("onnxruntime", "onnxruntime")):
            if importlib.util.find_spec(mod) is None: return False, f"{pip} is not installed (pip install -r requirements.txt)"
        if not _pulse() and not os.environ.get("DUMMY_VOICE_WAV") and importlib.util.find_spec("sounddevice") is None:
            return False, "no way to read the microphone (needs parec, or pip install sounddevice)"
        return True, ""

    def _matcher(self):
        extra = {c: list(self.s["phrase_aliases"].get(c, [])) + list(self.s["reactions"].get(c, []))
                 for c in set(self.s["phrase_aliases"]) | set(self.s["reactions"])}
        return Matcher(self.s["wake_word"], self.s["wake_aliases"], extra)

    def set(self, key, value):
        """Change one setting from the Voice tab. Returns an error string, or '' when applied."""
        s = self.s
        if key == "mode":
            if value not in ("off", "ptt", "wake"): return f"unknown mode {value!r}"
            if value != s["mode"]: self.cancel_pending()
            s["mode"] = value
        elif key == "mic": s["mic"] = str(value or ""); self._mic_dirty = True
        elif key == "model":
            if value not in MODELS: return f"unknown model {value!r}"
            if value != s["model"]: s["model"] = value; self._model_dirty = True
        elif key in ("vad_threshold", "end_silence", "awake_window"):
            lo, hi = {"vad_threshold": (0.2, 0.95), "end_silence": (0.2, 1.5), "awake_window": (1.0, 15.0)}[key]
            s[key] = round(max(lo, min(hi, float(value))), 2)
        elif key in ("gesture_size", "gesture_speed"):
            lo, hi = {"gesture_size": (0.5, 1.5), "gesture_speed": (60.0, 180.0)}[key]
            s[key] = round(max(lo, min(hi, float(value))), 2)
        elif key in ("dry_run", "sounds", "personality", "dance_clap", "pick_confirm"): s[key] = bool(value)
        elif key == "forget":       # a learned alias: ["wake", text] or [command, text]
            kind, text = value
            if kind == "wake": s["wake_aliases"] = [a for a in s["wake_aliases"] if a != text]
            else: s["phrase_aliases"][kind] = [a for a in s["phrase_aliases"].get(kind, []) if a != text]
            self.matcher = self._matcher()
        elif key == "forget_all": s["wake_aliases"], s["phrase_aliases"] = [], {}; self.matcher = self._matcher()
        else: return f"unknown voice setting {key!r}"
        self.seg.thr, self.seg.end_silence = s["vad_threshold"], s["end_silence"]
        self.save(s); self.apply()
        return ""

    def refresh_mics(self): self.mics = list_mics()

    # ---------- threads ----------
    def apply(self):
        """Start or stop the threads to match the mode (a calibration also needs them)."""
        want = self.s["mode"] != "off" or self.train is not None or self.quiet is not None
        if want and not self._run:
            if not self.ok: self.error = self.why; return
            self.error = ""; self._run = True
            if self._cap is None or not self._cap.is_alive():
                self._cap = threading.Thread(target=self._capture, daemon=True, name="voice-capture"); self._cap.start()
            if self._rec is None or not self._rec.is_alive():
                self._rec = threading.Thread(target=self._recognise, daemon=True, name="voice-recognise"); self._rec.start()
        elif not want and self._run:
            self._run = False; self.ptt_on = self._ptt_want = False; self.cancel_pending()
        self.seg.vad = self.s["mode"] == "wake" or self.train is not None or self.quiet is not None

    def shutdown(self): self._run = False

    def cancel_pending(self):
        """Nothing heard so far may act: STOP, a mode change, or a calibration starting or ending. Speech
        being collected is dropped, and anything queued or being transcribed is ignored when it comes out."""
        self.gen += 1; self._seg_reset = True; self.awake_until = 0.0; self.awake_only = None

    def keep_awake(self, seconds, only=None):
        """Open the no-name window for `seconds` (0 closes it): after a step, so "up ... up ... left"
        needs the name once, or while a pick waits for yes. `only` is the set of commands it takes."""
        self.awake_only = frozenset(only) if only else None; self.awake_until = time.time() + seconds if seconds > 0 else 0.0

    def ptt(self, on):
        """Push to talk, held (True) or released (False). Works in both modes; never needs the name."""
        if on and not self._run: return
        self.ptt_on = self._ptt_want = bool(on)

    def sound(self, name):
        if self.s["sounds"] and self.sounds is not None and not self.ptt_on:
            self.mute_until = max(self.mute_until, time.time() + self.sounds.play(name) + 0.15)

    def _capture(self):
        vad, mic, err_t, crash_t = None, None, 0.0, 0.0
        while True:
            if not self._run:
                if mic: mic.close(); mic = None
                time.sleep(0.1); continue
            try:
                if vad is None: vad = SileroVAD()
                if mic is None or self._mic_dirty:
                    if mic: mic.close()
                    self._mic_dirty = False; mic = open_mic(self.s["mic"]); vad.reset(); self.seg.reset(); self.error = ""
                f = mic.read()
            except Exception as e:
                if mic: mic.close(); mic = None
                self.error = f"microphone: {type(e).__name__}: {e}"
                if time.time() - err_t > 30: self.log(f"Voice: {self.error} - retrying", "err"); err_t = time.time()
                time.sleep(2.0); continue
            try: self._frame(f, vad)
            except Exception as e:                   # one bad frame must not end listening
                if time.time() - crash_t > 30: self.log(f"Voice: audio processing failed: {type(e).__name__}: {e}", "err"); crash_t = time.time()

    def _frame(self, f, vad):
        if self._seg_reset: self._seg_reset = False; self.seg.reset()
        if not self.seg.active: self._utt_gen = self.gen     # an utterance belongs to the generation it started in
        if self._ptt_want != self.seg.ptt_on: self.seg.ptt(self._ptt_want)
        db = max(-90.0, float(20 * np.log10(np.sqrt(np.mean(f * f)) + 1e-9)))     # digital silence would read -180
        if time.time() < self.mute_until and not self.ptt_on: f = np.zeros_like(f)   # our own beep
        p = vad(f)
        self._lv.append(db); self.level = max(self._lv); self.speech = p >= self.seg.thr
        if p < 0.2: self._quiet_db.append(db)
        if len(self._quiet_db) >= 60 and self.quiet is None: self.noise_db = round(float(np.percentile(self._quiet_db, 50)), 1)
        if self.quiet is not None: self._quiet_frame(p, db)
        for u in self.seg.feed(f, p):
            u.t_end, u.gen = time.time(), self._utt_gen
            if self.quiet is not None: self.quiet["starts"] += 1; continue     # a false start during the quiet check
            try: self.q.put_nowait(u)
            except queue.Full: self.log("Voice: still busy with earlier speech, dropped one", "warn")

    def _recognise(self):
        while True:
            if not self._run: time.sleep(0.1); continue
            if self._asr is None or self._model_dirty:
                self._model_dirty = False
                self.state_note = f"loading {self.s['model']}"
                try:
                    r = Recognizer(self.s["model"], self._note); dev = r.load(); self._asr = r; self.device = dev
                    if self.sounds is None: self.sounds = Sounds()
                    self.log(f"Voice: {self.s['model']} ready on the {'GPU' if dev == 'cuda' else 'CPU'}", "ok")
                except Exception as e:
                    self.error = f"speech model: {type(e).__name__}: {e}"; self.log(f"Voice: {self.error}", "err")
                    self._asr = None; self.state_note = ""; time.sleep(10); continue
                self.state_note = ""
            try: u = self.q.get(timeout=0.2)
            except queue.Empty: continue
            try: self._handle(u)
            except Exception as e: self.log(f"Voice: recognition crashed: {type(e).__name__}: {e}", "err")

    def _note(self, msg): self.state_note = msg; self.log(f"Voice: {msg}", "info")

    def _handle(self, u):
        if u.gen != self.gen: return                 # said before a STOP, a mode change or a calibration change
        self.n += 1; ev = {"id": self.n, "t": time.strftime("%H:%M:%S"), "source": u.source, "speech_s": round(u.speech_s, 2),
                           "peak_db": round(u.peak_db, 1), "gen": u.gen}
        if u.speech_s >= 0.1: self.speech_db = round(u.peak_db, 1)          # for the mic check: speech only
        self.clips[self.n] = (u.audio * 32767).clip(-32768, 32767).astype(np.int16).tobytes()
        while len(self.clips) > self.CLIPS: self.clips.popitem(last=False)
        if u.source == "ptt" and u.speech_s < 0.1:
            ev.update(kind="empty", text=""); self.on_event(ev); return
        self.thinking = True
        try: text, nsp, ms = self._asr(u.audio, self.s["wake_word"].title())
        finally: self.thinking = False
        if u.gen != self.gen: return                 # ...or one that came while it was being transcribed
        self.asr_ms = round(ms)
        ev.update(text=text, ms=round(ms), latency=round((time.time() - u.t_end) * 1000), no_speech=round(nsp, 2))
        if nsp > NO_SPEECH: text = ""                                      # Whisper thinks it was noise
        tr = self.train                              # read once: Stop on the tab may clear it meanwhile
        if tr is not None:
            r = tr.take(text, self.matcher)
            for kind, alias in r["learned"]:
                have = self.s["wake_aliases"] if kind == "wake" else self.s["phrase_aliases"].setdefault(kind, [])
                if alias not in have: have.append(alias)
            if r["learned"]: self.matcher = self._matcher(); self.save(self.s)
            self.train_last = r
            ev.update(kind="train", train=r)
            if tr.done:
                self.train_summary = tr.summary()
                if self.train is tr: self.train = None; self.cancel_pending()    # a take said late is not a command
                self.apply()
                self.log(f"Voice calibration: {self.train_summary['ok']}/{self.train_summary['n']} understood"
                         + (f", learned {len(self.train_summary['learned'])}" if self.train_summary["learned"] else ""), "ok")
            self.sound("ok" if r["ok"] else "no")
            self.on_event(ev); return
        awake = u.source == "vad" and u.t_end - len(u.audio) / SR < self.awake_until
        r = self.matcher.match(text, need_wake=u.source == "vad" and not awake)
        if nsp > NO_SPEECH and r["kind"] == "empty": r = {"kind": "noise"}
        only = self.awake_only        # a window opened after a step takes only steps without the name
        if awake and only is not None and u.source == "vad" and not r.get("wake") and r["kind"] in ("command", "unknown") \
                and r.get("cmd") not in only:
            r = {"kind": "nowake"}
        ev.update(r)
        if r.get("cmd"): ev["say"] = said(r)
        if r["kind"] == "wake" and u.source == "vad":
            self.keep_awake(self.s["awake_window"]); self.sound("wake")
        elif r["kind"] == "command": self.keep_awake(0)        # the server opens it again after a step
        elif r["kind"] == "unknown" and (r.get("wake") or u.source == "ptt") and r["words"] <= 4:
            self.sound("huh" if self.s["personality"] else "no")    # a short miss after the name; a long sentence is just talk
        self.on_event(ev)

    # ---------- calibration ----------
    def train_start(self):
        self.train, self.train_last, self.train_summary = Trainer(self.s["wake_word"]), None, None
        self.cancel_pending(); self.apply()

    def train_cancel(self):
        self.train = None; self.cancel_pending(); self.apply()

    def quiet_start(self, seconds=8.0):
        self.quiet = {"until": time.time() + seconds, "secs": seconds, "probs": [], "dbs": [], "starts": 0}
        self.quiet_result = None; self.apply()

    def _quiet_frame(self, p, db):
        q = self.quiet; q["probs"].append(p); q["dbs"].append(db)
        if time.time() < q["until"]: return
        thr, p99 = quiet_threshold(q["probs"])
        old = self.s["vad_threshold"]; self.s["vad_threshold"] = thr; self.seg.thr = thr
        self.noise_db = self.s["noise_db"] = round(float(np.percentile(q["dbs"], 50)), 1)
        self.quiet_result = {"noise_db": self.noise_db, "p99": p99, "starts": q["starts"], "thr": thr, "old": old}
        self.quiet = None; self.save(self.s); self.apply()
        self.log(f"Voice quiet check: noise {self.noise_db} dB, loudest non-speech {p99}, "
                 f"{q['starts']} false start(s); speech threshold {old} -> {thr}", "ok")

    # ---------- for the Voice tab ----------
    def state(self):
        if self.error and not self._run: return "error"
        if not self._run: return "off"
        if self.error: return "error"
        if self.state_note or self._asr is None: return "loading"
        if self.quiet is not None: return "quiet"
        if self.ptt_on or self.seg.active: return "hearing"
        if self.thinking or not self.q.empty(): return "thinking"
        if self.train is not None: return "training"
        if time.time() < self.awake_until: return "awake"
        return "listening" if self.s["mode"] == "wake" else "ready"

    def status(self):
        s, q, t = self.s, self.quiet, self.train      # each read once: the threads may change them meanwhile
        tr = None
        if t is not None or self.train_last or self.train_summary:
            tr = {"on": t is not None, "i": t.i if t else None, "n": len(t.steps) if t else None, "say": t.say() if t else "",
                  "last": self.train_last, "summary": self.train_summary}
        return {"type": "voice", "ok": self.ok, "why": self.error or self.why, "state": self.state(), "note": self.state_note,
                "mode": s["mode"], "level": round(self.level, 1), "speech": self.speech, "noise_db": self.noise_db,
                "speech_db": self.speech_db, "ptt": self.ptt_on, "device": self.device, "asr_ms": self.asr_ms,
                "awake": max(0.0, round(self.awake_until - time.time(), 1)), "mics": self.mics, "train": tr,
                "quiet": {"on": True, "left": round(max(0.0, q["until"] - time.time()), 1), "secs": q["secs"]} if q else None,
                "quiet_result": self.quiet_result,
                "settings": {k: s[k] for k in ("mic", "model", "wake_word", "vad_threshold", "end_silence", "awake_window",
                                                "dry_run", "sounds", "wake_aliases", "phrase_aliases", "personality",
                                                "gesture_size", "gesture_speed", "dance_clap", "pick_confirm")},
                "commands": [[c, SAY[c]] for c in Trainer.ORDER + ("praise",)], "models": list(MODELS)}

    @staticmethod
    def catalog():
        """The Commands page: [[group, [[id, say, does, [other phrases]], ...]], ...]."""
        return [[g, [[c, SAY.get(c, c), HELP[c], [p for p in COMMANDS.get(c, ()) if p != SAY.get(c)]] for c in ids]]
                for g, ids in GROUPS]

    def clip_wav(self, i):
        a = self.clips.get(i)
        if a is None: return None
        b = io.BytesIO()
        with wave.open(b, "wb") as w: w.setnchannels(1); w.setsampwidth(2); w.setframerate(SR); w.writeframes(a)
        return b.getvalue()
