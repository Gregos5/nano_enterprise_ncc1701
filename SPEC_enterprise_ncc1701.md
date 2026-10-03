# SPEC — enterprise_ncc1701

Living spec for this project. Add new feature requests under **Planned Features**;
implemented work moves to **Implemented Features** with the date it landed. This file
is the source of truth for what the firmware does and how new work should be shaped —
update it before or alongside code changes, not after the fact.

## 1. Overview

Arduino Nano (ATmega328P) sketch built with PlatformIO. Runs a minimal cooperative
scheduler in `loop()`; all behavior is implemented as non-blocking modules ("tasks")
registered with that scheduler. No task may call `delay()` or otherwise block — doing
so stalls every other task's timing.

## 2. Hardware Target

| | |
|---|---|
| Board | Arduino Nano |
| MCU | ATmega328P, 16 MHz |
| RAM | 2 KB |
| Flash | 30 KB |
| Framework | Arduino (via PlatformIO, `platform = atmelavr`) |

### Pin map

All pin numbers live in `include/Calibration.h` — change them there, never in a module.

| Pin | Direction | Used by | Purpose |
|-----|-----------|---------|---------|
| D5  | Output    | LedFader      | Wing LEDs, PWM (Timer0) |
| D6  | Output    | LedFader      | Red alert LEDs, PWM (Timer0) |
| D9  | Output    | LedFader      | Saucer LEDs, PWM (Timer1) |
| D10 | Output    | LedFader      | Bridge LEDs, PWM (Timer1) |
| D11 | Output    | ChirpNotifier | Piezo buzzer (`tone()`/`noTone()`, Timer2) |
| D12 | Input     | PowerToggle   | Touch button, polled both edges (no interrupt on D12) |

Timer notes: `tone()` owns Timer2, so D3's PWM is unavailable — nothing uses it.
D5/D6 share Timer0 with `millis()`, which is fine for `analogWrite()` but means
their duty cycle is slightly coarse at the very bottom of the range.

## 3. Architecture

```
include/Calibration.h  Every tunable number in one place: pin assignments,
                       touch debounce and hold threshold, chirp note tables,
                       per-channel LED fade parameters, the per-scene channel
                       masks and the fade rate-gain curve. Includes no
                       project headers (so no module ends up depending on
                       another through it) and therefore also defines the
                       table types themselves. Modules read it at init only.

src/main.cpp          setup(): init modules, register scheduler tasks
                       loop():  scheduler_run() only — nothing else may live here
                       dispatch_power_toggle(): the one place allowed to reach
                       across modules — turns a PowerToggle transition into
                       calls on ChirpNotifier/LedFader

lib/Scheduler/         Cooperative task table: scheduler_add_task(fn, period_ms)
                        - period_ms == 0  -> polled every scheduler_run() pass
                        - period_ms > 0   -> polled at least every period_ms
                        - SCHEDULER_MAX_TASKS = 8 (fixed-size table, no heap use)

lib/PowerToggle/        Debounced touch button, polled (D12 has no interrupt);
                        owns the off / on / red-alert state — a tap toggles
                        power, a 3 s hold toggles red alert — and latches a
                        one-shot transition for main.cpp's dispatcher to consume

lib/ChirpNotifier/      Non-blocking chirp playback state machine
                        (IDLE / NOTE_ON / NOTE_GAP) over one of two note
                        sequences (CHIRP_SEQUENCE_ON / _OFF), selected via
                        chirp_notifier_play()

lib/LedFader/           Non-blocking breathing effect over all four LED
                        channels; one step per channel per led_fader_update()
                        call, each channel on its own period / min-max / pause
                        calibration. led_fader_set_scene() picks which channels
                        are lit (OFF / NORMAL / RED_ALERT); a channel the new
                        scene drops ramps down to off rather than cutting out,
                        and one it adds climbs from wherever its brightness
                        ended up
```

### LED fade rate curve

Fade rate is deliberately not constant: it scales linearly with the current PWM
level, from `CAL_LED_MIN_RATE_GAIN_PERCENT` (10%) of full rate at 0 PWM up to
full rate at 255. The gain follows the *absolute* PWM level, not the position
between a channel's own min and max, so every channel shares one curve shape.

Because the rate varies along a ramp, the per-tick step that lands a ramp on its
requested duration is not `range / ticks`; it comes from integrating
`dt = dB / (rate * gain(B))`, which for a gain linear in `B` yields a logarithm.
`LedFader` evaluates that integral in floating point **once per channel in
`led_fader_init()`** and stores an integer Q16 step; the update path is
integer-only. Stepping uses the gain at the brightness each tick starts from
(forward Euler), which runs a few percent long — measured 5.03 s for the 5 s
wings and 10.22 s for the 10 s bridge. That is an accepted trade, not a bug.

`period_ms` is the **whole** cycle, pauses included, so each ramp gets
`(period_ms - pause_top_ms - pause_bottom_ms) / 2`.

### Hold channels (min == max)

A channel calibrated with `min_percent == max_percent` — the saucer — has no
breathing span, so it has no breathing step. It is not a special case in the
state machine; it simply rises to its level and stays there, because the falling
phase clamps at a minimum equal to its maximum. But it still has to be able to
*climb*: from 0 at power on, and from 0 after a ramp-down to off. It therefore
rises at the full-span (`0..max`) rate, the same rate used to ramp down. Its
`period_ms` is what sets that rate — which is the only thing period means on a
channel that never breathes.

### States and scenes

The model has three states, owned by `PowerToggle`, and each maps onto one
`LedFader` scene — a mask in `Calibration.h` saying which channels are lit. A
scene never changes *how* a channel behaves, only whether it runs at all; every
channel keeps its own calibration in every scene.

| State | Wings | Red alert | Saucer | Bridge |
|-------|-------|-----------|--------|--------|
| Off        | dark | dark | dark | dark |
| On         | breathing | dark | hold at 100% | breathing |
| Red alert  | breathing | breathing | hold at 100% | ramps down, dark |

A channel outside the current scene's mask ramps down on the same rate curve as
a power-off — so the bridge fades out when red alert is raised, rather than
cutting. A channel the new scene adds starts climbing from wherever its
brightness happens to be, so entering red alert leaves the wings and saucer
untouched mid-cycle.

### Touch button: tap vs hold

One button, two gestures, split by `CAL_TOUCH_HOLD_MS` (3000 ms):

| Gesture | Off | On | Red alert |
|---------|-----|----|-----------|
| Tap (released under 3 s) | → On | → Off | → Off |
| Hold (still held at 3 s)  | → Red alert | → Red alert | → On |

A hold acts the instant the threshold is crossed, while the finger is still
down; the release that follows is swallowed so it does not also count as a tap.
A tap can only be recognised on release — that is inherent in telling the two
apart, and it is why the action moved from the rising edge to the falling one.
Both edges share one debounce settle window, so bounce neither registers as a
touch nor cuts a hold short.

Holding from off brings the model up straight into red alert; a tap out of red
alert powers down rather than stepping back through normal on.

### Reset state

A reset leaves the model **dark and powered down**. `PowerToggle` starts in the
off state and `LedFader` starts in its off mode with every pin driven low, so the
first touch is always a turn-on. Nothing lights and nothing chirps on power-up.

Because every channel also starts at brightness 0, that first fade-in is
identical to any later toggle back on — there is no special-case power-up path.

Saucer behaviour, end to end: dark at 0% after a reset → fade in to 100% on the
first touch → hold at 100% → fade back out to 0% on the next touch.

Each module follows the same shape: `<module>_init(void)` called once from `setup()`,
`<module>_update(void)` registered with the scheduler and called repeatedly. Modules do
not call each other directly — `main.cpp` (specifically its `dispatch_power_toggle`
task) is the only place that wires them together.

### Currently registered tasks

| Task | Period | Why |
|------|--------|-----|
| `power_toggle_update` | 0 (every pass) | The touch pin is polled, not interrupt-driven — a touch shorter than one pass would be missed entirely; it also times the 3 s hold |
| `dispatch_power_toggle` | 0 (every pass) | Apply a pending transition to ChirpNotifier/LedFader as soon as it appears |
| `chirp_notifier_update` | 0 (every pass) | Note timing runs in tens-of-ms steps; needs tight polling to stay accurate |
| `led_fader_update` | `CAL_LED_TICK_MS` (10 ms) | Fade step rate for all four channels; every period and pause is quantised to this |

4 of 8 scheduler task slots remain free.

## 4. Coding Standards

Governed by `Software standards/C programming/Template` and the project's own
`.clang-format` (LLVM base, Allman braces, 4-space indent, 100-col limit). Run
`clang-format -i` (or the template's `clang_format.bat`) over changed files before
committing.

- File header doxygen block (`@file`, `@brief`) at the top of every `.c`/`.cpp`/`.h`.
- Section comments in source order: `File header` / `System headers` /
  `Third-party header files` / `Project headers` / `Constants, macros, datatypes` /
  `Static variable definitions` / `Static function prototypes` / implementation.
- Functions and variables: `snake_case`. File-scope statics: `s_` prefix.
  Compile-time constants/macros: `SCREAMING_SNAKE_CASE`.
- Library folder and file names: `PascalCase` (matches `lib/README`'s `Foo`/`Bar`
  example), one library per concern, flat `Lib/Lib.h` + `Lib/Lib.cpp` layout.
- No blocking calls (`delay()`, busy-wait loops) inside any `_update()` task —
  use `millis()`-based state machines instead (see `ChirpNotifier` for the pattern).

## 5. Repository Layout

```
enterprise_ncc1701/
├── .clang-format
├── platformio.ini
├── src/
│   └── main.cpp
├── lib/
│   ├── Scheduler/       (Scheduler.h, Scheduler.cpp)
│   ├── PowerToggle/     (PowerToggle.h, PowerToggle.cpp)
│   ├── ChirpNotifier/   (ChirpNotifier.h, ChirpNotifier.cpp)
│   └── LedFader/        (LedFader.h, LedFader.cpp)
├── include/
│   └── Calibration.h    (pins, chirp tables, LED fade parameters)
└── test/                (empty — PlatformIO unit test dir)
```

`platformio.ini` carries `build_flags = -Iinclude` so the `lib/` modules — not
just `src/` — can reach `Calibration.h`.

## 6. Development Workflow

1. Add an entry under **Planned Features** below: what it does, and how you'll know
   it works (acceptance criteria).
2. Implement it as a new `lib/<Module>/` (or an addition to an existing one) following
   the `_init()`/`_update()` shape and the coding standards above. Any tunable number
   it introduces — a pin, a timing, a table — goes in `include/Calibration.h`, not in
   the module.
3. Register the new task in `src/main.cpp` via `scheduler_add_task(...)`, choosing a
   period appropriate to the task's timing needs (`0` only if it genuinely needs
   per-pass polling).
4. Format (`clang-format -i`) and build (`pio run`) to confirm it compiles and fits
   RAM/flash.
5. Move the entry from **Planned Features** to **Implemented Features**, noting the
   date and which library implements it.

## 7. Implemented Features

| Feature | Library | Date |
|---------|---------|------|
| Debounced trigger → 4-note piezo chirp, non-blocking playback | `ChirpNotifier` | 2026-07-25 |
| Breathing LED fade, non-blocking | `LedFader` | 2026-07-25 |
| Cooperative task scheduler | `Scheduler` | 2026-07-25 |
| Button toggles power state: LED ramps down to off / resumes breathing (not an instant cut), distinct on/off chirp sequences (`CHIRP_SEQUENCE_ON`/`_OFF`, off sequence is a placeholder pending real tones) | `PowerToggle`, `ChirpNotifier`, `LedFader` | 2026-07-25 |
| Central calibration header: GPIO pins, chirp note tables, per-channel LED fade parameters and the minimum rate-gain, all in one place with no project-header dependencies | `include/Calibration.h` | 2026-09-26 |
| New pinout: buzzer D11, wings D5/D6, saucer D9, bridge D10, touch button D12 | `Calibration.h`, all modules | 2026-09-26 |
| Touch button polled instead of interrupt-driven, since D12 has no hardware interrupt; same debounce and one-shot transition semantics as before | `PowerToggle` | 2026-09-26 |
| Four independently-calibrated LED channels, each with its own period, min/max brightness and top/bottom pause | `LedFader` | 2026-09-26 |
| Hold channels (`min == max`, the saucer): fade in from 0 to their level, hold, fade back out on power off | `LedFader` | 2026-09-26 |
| Reset state is powered down: all LEDs dark, no chirp, first touch turns on | `PowerToggle`, `LedFader` | 2026-09-26 |
| Brightness-dependent fade rate: linear gain from 10% of full rate at 0 PWM to 100% at full scale, with the per-tick step solved at init so ramps still land on their requested period | `LedFader` | 2026-09-26 |
| Pinout renamed: D5 is the wings (one channel, both wings), D6 is the new red alert strip — replacing the separate left/right wing channels | `Calibration.h`, `LedFader` | 2026-09-26 |
| LED scenes: `led_fader_set_scene(OFF / NORMAL / RED_ALERT)` replaces `led_fader_set_enabled()`. A per-scene channel mask in `Calibration.h` decides who is lit; dropped channels ramp down, added ones climb from their current brightness | `LedFader`, `Calibration.h` | 2026-09-26 |
| Red alert state, entered by holding the touch button for 3 s: red alert strip breathes 0→100% over 1 s with a 2 s bottom pause, wings and saucer carry on as in normal on, bridge fades out. Holding again returns to normal on; a tap powers down | `PowerToggle`, `LedFader`, `main.cpp` | 2026-09-26 |
| Tap/hold gesture split on the one touch button, both edges debounced by a shared settle window; taps act on release, holds act the moment the 3 s threshold is crossed | `PowerToggle` | 2026-09-26 |

## 8. Planned Features

*(none — add the next request here)*

## 9. Constraints & Open Questions

- 2 KB RAM total — keep new modules' static state small; avoid `String`/heap use.
  Current usage: 308 bytes static (15.0%), 7064 bytes flash (23.0%).
- `SCHEDULER_MAX_TASKS` is 8; raise it in `lib/Scheduler/Scheduler.cpp` if a feature
  pushes past that. 4 of 8 slots are in use.
- Only D2/D3 are hardware-interrupt-capable on the Nano, and both are now free.
  The touch button on D12 therefore polls — which is why `power_toggle_update` must
  stay registered at period 0.
- PWM is available only on D3/D5/D6/D9/D10/D11. D11 is the buzzer and D3's PWM is
  lost to `tone()`'s use of Timer2, so all four PWM pins that remain are already
  spoken for by `LedFader`. A fifth fading channel would need software PWM.
- `Calibration.h` must keep including no project headers. It defines its own table
  types for that reason: if it pulled in a module header, every module reading it
  would gain a dependency on that module, and PlatformIO's default library
  dependency finder does not resolve includes nested inside `include/` anyway.
- Fade periods land a few percent long by design (forward-Euler stepping). If exact
  periods ever matter, evaluate the rate gain at the tick midpoint instead.
- `CHIRP_SEQUENCE_OFF` is still a placeholder (two descending notes) pending a real
  "power down" chirp.
- Red alert has no chirp of its own: entering it replays `CHIRP_SEQUENCE_ON` so the
  3 s threshold is audibly confirmed. A dedicated red-alert klaxon sequence would
  be a new table in `Calibration.h` and a third `chirp_sequence_t` value.
- The red alert channel's `period_ms` is 3000, not the 1000 the request named:
  `period_ms` is the whole cycle with pauses included, so 3000 against a 2000 ms
  bottom pause is what yields the asked-for 1 s of fading plus a 2 s pause.
- Leaving red alert is a second 3 s hold. The request only specified how to enter
  it, so this is a chosen symmetry, not a stated requirement.
