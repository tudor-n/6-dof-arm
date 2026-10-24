# Plan: hearing

**Goal.** Say a command and the arm does it, without making the arm any easier to move by
accident than it is today. Voice is a convenience on top of the tablet, never a replacement for
STOP / Esc.

Status: the first part is built on branch `feat/voice` (2026-10-24); see **Built** just below.
Everything after it is the original option survey, written against `master` at `a9452ae` before
anything was built. Where the build went another way, a note says so. English commands only.

## Built (`feat/voice`)

What was asked for: a **Voice** tab to configure it, see what it hears and calibrate it, with wake
word and push to talk side by side. Wake word is meant for everyday use, and push to talk for
testing. Starter commands, and only these:

| Say | Does |
|---|---|
| "Dummy, come here" | Finger follow on (refused like the button when Live is off or the pose is bad) |
| "Dummy, start pose" | The Start pose button |
| "Dummy, relax" | **Fold to the rest pose, then go limp** (the owner's choice; it never drops from mid-air) |
| "Dummy, stop" | STOP (added so that follow can be ended by voice) |

Vision prompts ("pick up the pen") and everything else wait until these are reliable.

Decisions that differ from the survey below:
- **The wake word is Whisper**, not a spotter like openWakeWord or Porcupine. Every utterance the VAD
  finds is transcribed, and only "Dummy" + a command counts. Nothing is trained and no account is
  needed. The GPU makes it cheap (60-80 ms per phrase with `small.en`, measured), and a dedicated
  spotter is weakest on exactly this kind of two-syllable word. The wake word is matched exactly or
  by learned alias, never fuzzily ("Mummy, come here" must not count).
- **Wake word and push to talk ship together** (the survey had push to talk first, wake word later).
  Push to talk works in both modes and skips the name.
- **Relax by voice is allowed** (§7 had it as never), because folding to rest first removes the
  reason it was banned.
- **A presence check**: commands are refused when no page is open; motion is refused while Live is off.
  "Stop" is exempt from both, and from dry run.
- **Relax re-checks before going limp** (Live on, follow off, at exactly the rest pose), and nothing heard
  before a STOP or a voice change acts after it. An independent review found both gaps; `test_voice.py`
  now covers them against the real control loop.
- **Calibration replaces hand tuning**: mic check, an 8 s quiet-room measurement that sets the speech
  threshold, and two takes of each command that learn how Whisper spells this voice.
- **No PortAudio needed on Linux**: the mic is read through `parec`.
- **Silero VAD is the copy faster-whisper ships**, run with onnxruntime, so no torch import in the
  audio thread and no extra package.

Measured with synthetic speech (5 Piper voices × 2 speeds, 17 phrases including chatter, both
modes): 337/340 right, **0 wrong actions**, 3 misses ("Start pose" without the name read as
"Suppose"; a 0.18 s "Dummy" read as "Duddy"). Not yet tried with a person talking to the real arm.
See ARCHITECTURE.md (Voice) for how it works and TEST_INSTRUCTIONS.md §6 for the hardware test.

## 0. What already exists to build on

| Existing piece | Why it matters for hearing |
|---|---|
| `server.py` `handle(ws, m)`: every action is a JSON message | Voice can be **one more client** that sends the same messages. Live, pose health, pick refusal and `follow_breaks()` then apply with no extra work. Voice gets no new path to the servos |
| `FOLLOW_SAFE` allowlist + `follow_breaks()` | The pattern for a `VOICE_ALLOWED` allowlist, and it already decides which commands end follow |
| `finger.py` shape: optional dependency, lazy import, a thread that knows nothing about the arm, a pure state machine tested offline in `test_finger.py` | Hearing copies it: `hearing.py` + `test_hearing.py` |
| The hold-and-resend dead-man (creep, `follow_z`: resend every 100 ms, server stops 0.4 s after the last) | Reused as is for a tablet push-to-talk button |
| YOLO-World with editable prompts (`detect` message, `V.prompts`, `vocab` in `vision.json`) + `pick_detected` / `V.best_target()` | "Pick up the pen" is mostly plumbing. Open-vocabulary detection pairs well with an open-vocabulary recogniser |
| `log()` → the log panel on every tab; `state` broadcast every 2 ticks | Where "heard: …" goes, and where a `voice` status field goes |
| `do_stop()` holds position and brakes the gripper | A false voice STOP is harmless, so a STOP word can be tuned to fire easily |

**Hardware and environment facts that narrow the options** (checked on this machine):

- Laptop: i7-14700HX, RTX 5050 8 GB. The repo's `.venv` has torch 2.14 + CUDA 13 working, so YOLO
  can share the GPU with a speech model. `sounddevice` is already installed (it came with mediapipe).
- Microphones already plugged in: **the overhead USB webcam has one** (`card 2: USB 2.0 Camera`),
  and the laptop has its own (ALC257).
- The tablet (Galaxy Tab 3 Lite, Android 4.2 WebView, no WebSocket) **can't record audio from the
  page**. It can only show status and host buttons.
- The noise sources are six MG996R servos (whine under load, clicks), the laptop fans under GPU
  load, and the PSU.
- ESP32 serial runs at 115200 baud. 16 kHz 16-bit audio needs 256 kbit/s, more than twice what that
  link carries.
- It should also run on Windows (the same box dual-boots), so audio goes through PortAudio
  (`sounddevice`), which works on both.

## 1. The pipeline

```
mic ─► GATE ─────────────► RECOGNISE ─────────► PARSE ─────────────► SAFETY ──────────► handle(msg)
       when to listen      sound → text         text → command       allowlist,          the same messages
       (PTT/VAD/wake…)     (Whisper/Vosk…)      (grammar/LLM…)       confirm             the tablet sends
                                                                                         │
                                                         FEEDBACK ◄──────────────────────┘
                                                         beep, log line, UI badge, speech
```

Each stage's options are independent of the others. §2–§6 cover them, §7 is the command
catalogue, §8 the safety rules, and §9 the recommendation.

## 2. Where the microphone lives

| Option | Pros | Cons |
|---|---|---|
| **A. Webcam's built-in mic** | Already plugged in, free, you can test it today | Above the table, right next to the servos and far from your mouth. Cheap webcam mics are noisy and their automatic gain "pumps" |
| **B. Laptop's internal mic** | Free | Fan noise when the GPU is busy, keyboard clicks, and it depends where the laptop sits |
| **C. USB desk / gooseneck mic (cardioid)** | Cheap. Aim it at you, away from the arm | Another USB device. Still 30–60 cm from your mouth |
| **D. Headset or wireless lavalier (USB receiver)** | By far the best signal-to-noise. The gate and the recogniser both work better, and servo noise nearly disappears | You have to wear it, and it has a battery |
| **E. Phone (or another browser) streams its mic over the WebSocket** | Wireless mic you already own | The browser only gives mic access over HTTPS or `localhost`, so you'd need a self-signed certificate. Wi-Fi lag and dropouts. The tablet can't do it |
| **F. Mic on the ESP32 (I2S, e.g. INMP441)** | Lives on the arm | Can't stream over 115200 serial. Needs Wi-Fi streaming or an on-chip wake word (ESP32-S3 + microWakeWord). That's a big firmware detour, against the "deliberately dumb executor" rule |

**Recommended:** measure A first because it's free. If recognition suffers with the servos
running, which is likely, move to D (or C).

## 3. When it listens: activation modes

This is the biggest choice. Every mode trades two kinds of mistake:

- **False accept:** it acts when you didn't mean it to. Dangerous for an arm.
- **Miss:** you said it and nothing happened. Annoying, but safe.

### 3.1 Push-to-talk (PTT)

Hold something while you speak. The audio between press and release is the command.

| Trigger | Notes |
|---|---|
| Tablet hold button (Camera tab) | Reuses the 100 ms resend / 0.4 s deadline pattern. Long-poll adds lag at the press, so keep a pre-roll buffer |
| Laptop key (hold Space, say; F is taken by follow and Esc by STOP) | Only works while the laptop's browser has focus and the laptop is within reach |
| **USB foot pedal** (shows up as a keyboard key) | Keeps both hands free, which matters because finger follow needs your hand |
| Wireless presenter clicker | Fits in a pocket, and it's also just a keyboard device |
| Button on an ESP32 GPIO | Physical, but needs a firmware and serial-protocol change |

- **Pros**
  - Practically zero false accepts: nothing counts unless you press.
  - Clean start and end points give the best recognition accuracy, and there's no wake word to train.
  - Simplest to build and test. No always-on CPU/GPU load, and nothing listens otherwise.
  - The listening state is obvious.
- **Cons**
  - Needs a hand, which clashes with pointing during finger follow. A pedal fixes this.
  - Releasing the button clips the last word. Keep ~300 ms of pre-roll and ~300 ms of tail.
  - Not hands-free, so it feels less magical.
  - Saying STOP through PTT is slower than just pressing STOP.

### 3.2 Toggle-to-talk (listening window)

Tap once to open a window (until ~10 s pass or you go quiet). Tap again to close it early.

- **Pros:** one tap per burst of commands, hands free while you talk, no wake word.
- **Cons:** anything said inside the window counts. It's easy to forget it's open, so it needs a
  loud indicator and a timeout.

### 3.3 VAD only (always listening)

A voice activity detector cuts the audio into speech segments. Every segment gets recognised and
parsed.

| VAD engine | Pros | Cons |
|---|---|---|
| Energy threshold | Trivial | Useless here: servo whine and fans trip it |
| WebRTC VAD | Tiny, fast, pip-installable | Old statistical model. Noise and music trigger it |
| **Silero VAD** | Neural and much better at rejecting noise. Under 1 ms per 30 ms chunk on one CPU core. MIT licence | Needs onnxruntime or torch (torch is already installed) |

- **Pros**
  - Fully hands-free, with no extra words. The least friction of any mode.
  - Quick for rapid sequences ("up", "up", "left").
- **Cons**
  - Everything said in the room becomes a candidate command: talking to someone, a phone call, a
    video. **The highest false-accept risk of any mode.**
  - Runs the recogniser on every bit of speech, on the GPU that YOLO uses too.
  - Whisper is known to invent text from noise or silence ("Thank you."), so it needs no-speech
    filtering.
  - Servo noise can set off the VAD exactly while the arm is moving.
- **Verdict:** acceptable only with a strict closed grammar plus confirmation for movement, and
  realistically only when you're alone in a quiet room.

### 3.4 Wake word + VAD

The arm always listens for one name ("hey gripper", say). When it fires, VAD records until you
stop talking, and that segment is the command.

| Wake-word engine | Pros | Cons |
|---|---|---|
| **openWakeWord** | Free and open source, a few % of one CPU core. Custom words are trained from synthetic speech (Colab notebook, about an hour) | Pretrained models are CC BY-NC-SA, which is fine for a personal project. Custom-model quality varies and needs threshold tuning |
| **Porcupine** (Picovoice) | Best accuracy and fewest false alarms. A custom word trains in seconds in their web console. Tiny CPU load | Needs a free Picovoice account and AccessKey (non-commercial terms, online activation). Closed model, and the terms have changed before |
| Whisper as the wake word (transcribe every VAD segment, check it starts with the name) | No extra engine, any name you like | Has all the costs of VAD-only (always transcribing). Slower, and more false accepts than a real spotter |
| microWakeWord on an ESP32-S3 | Runs on a microcontroller (the repo already has an S3 sdkconfig) | A separate hardware, firmware and Wi-Fi project |

- **Pros**
  - Hands-free, and background chatter is ignored because only speech after the name counts.
  - Cheap when idle: one tiny model, and VAD/ASR run only after the name.
  - Natural to use: "Hey gripper, go home."
- **Cons**
  - Adds 2–3 syllables to every command, which gets tiring for jog sequences. Fix: a **follow-up
    window**, where it keeps listening for ~5 s after a command without needing the name.
  - Two thresholds to tune (misses and false triggers) for your voice, your room, and the servos
    running.
  - The word matters: 3+ syllables with unusual sounds. "Arm" alone is too short and too common.
    The arm's name gives an obvious candidate, **"Hey Dummy"**. (Built: plain "Dummy", with "hey" optional.) The "hey" matters, because
    "dummy" on its own turns up in normal talk.
  - Detection lands ~0.2–0.5 s after the word ends. A pre-roll buffer is needed so a command said
    without a pause doesn't lose its first word.

### 3.5 Always-on closed grammar (no wake word)

The recogniser only knows ~30–60 exact phrases (Vosk in grammar mode, or Picovoice Rhino
speech-to-intent). Anything else should come out as "unknown".

- **Pros:** hands-free with no wake word. The fastest option, because it streams and decides as
  you finish. Light on CPU, deterministic, and easy to test.
- **Cons:** rigid phrasing ("up twenty", not "go up a bit"). Grammar mode tends to force random
  speech onto the nearest allowed phrase, so it needs an "unknown" entry and a confidence
  threshold. Object names must be in the grammar ahead of time (they can be generated from
  `vocab`). Rhino needs a Picovoice account.

### 3.6 Dedicated STOP spotter (runs alongside any other mode)

A separate always-on detector for **only** "stop" (plus "halt", "freeze"). It skips the wake word,
PTT and confirmation, and sends `stop` directly.

- **Pros:** a hands-free emergency stop whatever the main mode is, on the fastest path there is. A
  false STOP is harmless: the arm holds and the gripper brakes.
- **Cons:** "stop" is a common word, so expect occasional stops while you're chatting (annoying,
  not dangerous). It's still slower and less certain than Esc or the STOP button. **It supplements
  them and never replaces them.**
- **Engine:** a custom openWakeWord or Porcupine keyword, or Vosk grammar mode with only the stop
  words in it.

### 3.7 Vision-gated listening (multimodal)

Listen only while the camera sees your hand. Or combine speech with pointing: "go there" or "pick
that" while pointing uses the finger-follow target.

- **Pros:** "this / that / there" commands feel very natural, and it reuses `finger.py`, which is
  already running. No hand in view means no movement commands, which cuts false accepts.
- **Cons:** only works while hand tracking is on. It couples two fragile systems and adds states to
  test.

### 3.8 Non-speech sounds (clap, whistle, snap)

- **Pros:** trivial models.
- **Cons:** servo clicks, desk knocks and dropped tools sound like claps, and you only get one or
  two distinguishable commands. A curiosity, not recommended.

### Mode comparison

| Mode | False accepts | Misses | Hands-free | Build effort | Idle cost | Added latency |
|---|---|---|---|---|---|---|
| Push-to-talk | ~none | low | no (yes with a pedal) | low | none | low |
| Toggle window | medium | low | mostly | low | ASR only inside the window | low |
| VAD only | **high** | low | yes | medium | VAD + ASR on all speech | low–medium |
| Wake word + VAD | low | medium (tunable) | yes | medium–high | tiny | +0.2–0.5 s |
| Closed grammar | low–medium | medium | yes | medium | small | lowest |
| STOP spotter (add-on) | harmless | must be low | yes | medium | tiny | lowest |
| Vision-gated | low | medium | yes | high | already running | low |

## 4. Sound → text (recognisers)

| Engine | Pros | Cons |
|---|---|---|
| **faster-whisper** on the GPU (`small.en` / `medium.en` / `large-v3-turbo`, int8) | Best open-source accuracy. Takes any phrasing and any noun, so whatever you name can become a YOLO-World prompt. An initial prompt or hotwords can bias it toward your command words and `vocab`. Offline | Not truly streaming: it decodes after the segment ends (a few hundred ms for a short command, to be measured). Shares 8 GB of VRAM with YOLO-World. Hallucinates on noise and silence. Its CTranslate2 backend must support the new Blackwell GPU; if it doesn't, the fallback is openai-whisper on the torch that's already working |
| Whisper on the CPU (`base.en` / `tiny.en`, int8) | Leaves the GPU alone | Slower and less accurate, though adequate for short commands |
| Moonshine / Parakeet / distil-whisper | Faster than Whisper on short clips (Moonshine is built for live commands). Parakeet is very accurate in English | Less mature tooling. Parakeet's NeMo stack is heavy to install, especially on Windows |
| **Vosk** (Kaldi) | Streams, with partial results while you speak. Small English model (~50 MB), CPU only, grammar mode for closed sets, offline, Apache 2.0 | Weaker at open dictation than Whisper. Grammar mode is rigid |
| **Picovoice Rhino** (speech-to-intent) | Audio goes straight to an intent like `jog{direction, amount}`. Very accurate in a known domain, and tiny | Account + AccessKey. Fixed vocabulary, so new object names mean retraining the context |
| Cloud ASR (Google, Deepgram, OpenAI) or Chrome's Web Speech API in the laptop browser | Top accuracy, no GPU use | Needs internet, so the arm's controls get worse when Wi-Fi drops. Variable latency, privacy, cost. Web Speech is Chrome-only, and the tablet can't run it |

## 5. Text → command (parsing)

| Approach | Pros | Cons |
|---|---|---|
| **Grammar / patterns with synonyms and number words** ("twenty", "2 cm") | Deterministic, testable offline from text like `test_finger.py`, and easy to say what's allowed | Brittle to phrasing. Every synonym has to be added by hand |
| **Fuzzy matching** on top (e.g. rapidfuzz) | Absorbs recognition slips ("jock up" → "jog up") | Can pick the wrong near-match ("home" vs "hold"), so it needs a margin between the best and second-best match |
| Local LLM via Ollama, forced to JSON (you already run Ollama) | Free phrasing, compound commands ("grab the pliers and drop them"), clarifying questions | Adds ~0.5–2 s. Fights Whisper and YOLO for 8 GB of VRAM. Not deterministic, so it's hard to test exhaustively. It may only output an intent from the same allowlist, never joints |
| Cloud LLM (e.g. Claude API) | Best understanding | Internet, cost, latency, privacy: the same objections as cloud ASR, but worse |
| Speech-to-intent (Rhino) | Recogniser and parser in one | See §4 |

## 6. Feedback: how you know what it heard

| Channel | Pros | Cons |
|---|---|---|
| **Earcons** (short beeps for listening / accepted / rejected) | Fastest, unambiguous, no screen needed | Needs a speaker near you (the laptop's is fine) |
| **Log line + banner** ("heard: 'up 20' → jog dz=+20") **and a mic badge in `state.voice`** | Works on the tablet with no new transport. Showing the transcript makes misrecognitions obvious | You have to look at the screen |
| Spoken replies (offline TTS: Piper, or espeak-ng) | Confirmations and answers ("pose is healthy", "I see a pen and pliers"), hands-free | The mic hears the arm's own voice, so input must be muted while it speaks (half-duplex). Adds latency |
| The tablet speaks (Fully Kiosk TTS) | The speaker is where you're looking | Unverified on Android 4.2. One more moving part |

## 7. Command catalogue, by risk tier

Every command maps onto an **existing** `handle()` message. Directions are in **your** frame,
facing the robot (HARDWARE.md): "left" is your left, which is the robot's right, which is −Y.
"Forward / away" is +X, and "toward me" is −X.

| Tier | You say | Becomes | Confirm? | Follow |
|---|---|---|---|---|
| **0 Safety** | "stop", "halt", "freeze" | `stop` | never, instant | ends it (as today) |
| **1 Questions** | "status", "where are you", "what do you see", "is the pose OK" | Read-only: `tool_state()`, `V.status()`, the detections. Spoken or logged | no | unaffected |
| **2 Modes / settings** | "follow on / off", "height low / hover / high", "detect on / off", "look for pliers", "speed slow / normal / fast", "lock pose" | `follow`, `follow_z` preset, `detect` (+ prompt), `speed`, `pose_lock` | no | kept (all in `FOLLOW_SAFE`) |
| **3 Small moves** | "up / down / left / right / forward / back [N mm or cm]", "roll left 30", "jaws across", "open / close gripper" | `jog` (default 20 mm, capped at ~50), `roll` / `jaws`, gripper as a **timed pulse then brake** | no (the step is bounded) | `jog` ends it; grip and roll keep it |
| **4 Big moves** | "home", "rest", "go there" (after a click on the video) | `home`, `rest`, `goto_click` | **yes** | ends it |
| **5 Tasks** | "pick up the pen", "pick that" (while pointing), "drop it" | `detect` with prompt "pen" → wait for a detection → `pick_detected`. "Pick that" needs a new message that uses the follow target | **yes**, with what and where read back | ends it |
| **6 Never by voice** | live on, set zero, flip, limits, links, markers, roll zero / flip, unlock, raw, creep (relax was here; built as fold-to-rest-then-limp) | Not in the allowlist | — | — |

**Meta commands:** "yes / confirm", "no / cancel", "repeat", "what can I say", "stop listening".

Notes:

- **Gripper.** It's a 360° servo with no position (HARDWARE.md, the dead-man's switch). "Close"
  can't mean "keep closing until I say stop". It has to be a fixed pulse (~1.5 s, like
  `pick_seq`) that ends in the 90 brake.
- **No continuous voice motion** ("keep going left… stop"). Voice STOP takes 0.3–1 s to land, so
  the arm overshoots. Use bounded steps instead.
- **Live on stays physical.** Live on engages the servos. (Relax was here too; it's built as fold to rest, then limp.)
- **Numbers:** accept digits and words, in mm or cm. Clamp to the maximum step, and log or say the
  value that was actually used.
- **"Pick up the pen":** read it back before acting ("pen, 0.78, at 230 / 40, say yes"). The pick
  still refuses on an unhealthy pose, exactly as today.

## 8. Safety rules (whatever modes are picked)

1. **Voice is just a client.** It sends ordinary `handle()` messages, so every existing interlock
   applies unchanged.
2. **A `VOICE_ALLOWED` allowlist** of message types, like `FOLLOW_SAFE`: an allowlist, because a
   denylist goes stale silently. Tier 6 isn't in it.
3. **Off at startup.** Turning it on is a visible toggle. Its state lives in `state.voice`, so two
   tablets can't disagree. Movement tiers need Live on, as they already do.
4. **Confirm tiers 4–5** within ~5 s, by "yes" or a tap. Anything else cancels.
5. **Voice STOP supplements Esc and the STOP button, and never replaces them.** Say so in the docs.
6. **Ignore the mic while the arm talks** (TTS). Optionally raise the VAD threshold while the
   servos move.
7. **Failure turns hearing off.** A crash or a lost mic switches it off and logs why. It never
   touches the arm, just like `follow_off()` on a crash.
8. **Optional dependency, lazy import,** like mediapipe and ultralytics. The server runs without it.
9. **Log every transcript and its action,** so any wrong move can be explained afterwards.
10. **Open decision:** should hearing also switch off when the last browser disconnects, as follow
    does? Recommended **yes**: "someone is at the console" doubles as the presence check.

## 9. Recommendation, in phases

**Phase 1: push-to-talk with a small command set.** Lowest risk and quickest to be useful.

- **Mic:** the webcam mic to start. Measure it with the servos moving, and switch to a wireless
  lav or a USB cardioid if it isn't good enough.
- **Trigger:** a tablet hold button, a laptop key, and optionally a USB foot pedal (hands free for
  pointing).
- **Recogniser:** faster-whisper on the GPU (`small.en`, or `large-v3-turbo` if latency allows),
  with an initial prompt built from the command words and `vocab`.
- **Parser:** grammar + fuzzy match, written as a pure function and tested offline on text.
- **Commands:** tiers 0–3 (stop, questions, modes, bounded jogs, the timed gripper).
- **Feedback:** earcons, "heard: … → …" in the log, and a mic badge.
- **Why first:** almost no false accepts and clean audio segments. It proves the mic and the
  recogniser before any always-on listening gets built.

**Phase 2: hands-free.**

- A wake word (openWakeWord, free and offline; or Porcupine if a Picovoice account is fine), with
  Silero VAD to find the end of the command and a 5 s follow-up window.
- An always-on STOP spotter.
- PTT stays as an override.
- Tune on recordings of your own room with the servos running.

**Phase 3: tasks and confirmation.**

- "Pick up the <object>", "go there", and "pick that" while pointing.
- Spoken confirmation, and Piper TTS replies with the mic muted while it speaks.

**Phase 4 (optional): free phrasing.**

- A local Ollama model turns utterances the grammar rejects into a JSON intent from the same
  allowlist, always confirmed.
- Only if Phases 1–3 feel too rigid, and only after checking the VRAM budget.

**Not recommended:**

- VAD-only as the default (at most a "quiet room" setting).
- Cloud ASR or LLMs for anything that moves the arm.
- A mic on the ESP32.
- Clap triggers.

## 10. How it would fit the code (later, no code now)

- **`hearing.py`**, shaped like `finger.py` and knowing nothing about the arm. It has an audio
  thread (`sounddevice`, already installed) with a ring buffer for pre-roll, the gate
  (PTT / VAD / wake), and the recogniser. It publishes `Heard(t, text, confidence)`.
- **A pure parser**, `text → Intent or None`, with **`test_hearing.py`** testing it offline from
  text, plus a few recorded WAVs for the gate.
- **`server.py`:** lazy import, a `hearing` toggle message, `VOICE_ALLOWED`, intent → existing
  message → `handle()`, the confirmation state, and `voice` in the state broadcast.
- **`cam_panel.js`:** a mic toggle, a PTT hold button and the "heard" banner. ES5 only, because of
  Android 4.2.
- **`requirements.txt`:** a new optional block (faster-whisper, silero-vad, openwakeword). The
  desktop launcher (`desktop/launcher.py`) installs `requirements.txt` changes into the installed
  copy's venv on its next start, so they reach the DUMMY app with no extra step. That's also why
  the block should stay small: the first start after a change waits for the download.
- **Settings:** a `hearing` section (mic device, mode, model, thresholds, step sizes), in
  `vision.json` or `config.json`.
- **Docs:** README, ARCHITECTURE (a "Hearing" section) and TEST_INSTRUCTIONS, once it's built.

## 11. Measure before building (cheap, and it settles the open choices)

1. **Mic:** record 30 s from each mic while the arm jogs and YOLO runs, then run Whisper on each
   recording.
2. **Model:** time faster-whisper `small.en` against `large-v3-turbo` on the RTX 5050 with YOLO and
   MediaPipe running.
3. **Thresholds:** count false wake and STOP triggers over 30 min of normal room noise with a
   video playing.
