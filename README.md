# Paddock

A driving simulator in Rust and [Bevy](https://bevy.org), built around a real
steering wheel and pedals. Tires, force feedback, drivetrain and engine sound
are all modelled from physics rather than canned effects.

The first car is the Subaru BRZ / Toyota GR86 (FA24 2.4 L flat-four, Aisin
six-speed automatic). More cars are planned.

This is a personal project. It runs on Linux and has been tested with a MOZA
R12 wheel base and CRP2 pedals.

## Features

- **Wheel and pedal input** read directly from Linux evdev: steering, pedals
  plugged into the wheel base, and the shift paddles.
- **Force feedback** from the tire model. Steering torque is the front tires'
  self-aligning torque plus their lateral force acting at the caster trail,
  divided by the steering ratio. It goes light as the front tires reach their
  limit. Friction stands in for tire scrub at parking speeds.
- **Vehicle dynamics** use a bicycle model with per-wheel vertical loads.
  - Longitudinal and lateral load transfer, with a lag for the suspension.
  - A Pacejka-style tire with load sensitivity and a friction circle.
  - Transient slip (relaxation length), so the model holds from standstill
    to high speed without switching to a separate low-speed model.
- **Powertrain**
  - FA24 torque curve with throttle and intake manifold lag.
  - Torque converter with lockup clutch, and the Aisin 6AT ratios.
  - Manual mode on the paddles, or an automatic shift schedule. Both blip
    the throttle on downshifts and refuse downshifts that would over-rev.
- **Procedural engine sound**, built as source, transfer path and receiver.
  - Source: per-cylinder exhaust pulses with cycle-to-cycle variation.
  - Exhaust: a digital waveguide network (header runners, collector,
    muffler chambers, tailpipe).
  - Cabin: separate airborne, intake and structure-borne paths into the
    cabin, with cabin modes and a short reverb.
  - Extras: road and wind noise, and an Active Sound Control layer.
- **Track**: a 2 km circuit with asphalt, curbs and grass under each wheel
  separately, plus lap timing.
- **3D chase camera**, with a top-down view as an alternative.

## Requirements

- Linux and a recent stable Rust toolchain (edition 2024).
- ALSA development headers for audio output: `sudo apt install libasound2-dev`.
- Read and write access to the wheel's `/dev/input/event*` node. On desktop
  distributions, `uaccess` usually grants this to the logged-in user. Write
  access is needed for force feedback.
- A MOZA wheel base (USB vendor `0x346e`). The input mapping assumes:
  - `ABS_X` is steering.
  - `ABS_Z` and `ABS_RZ` are the throttle and brake of pedals connected to
    the base.
  - Buttons 301 and 300 are the right and left paddles.

  Other wheels need these constants changed in `src/wheel.rs`.

Set the in-game steering rotation range (Up/Down) to match the one in MOZA
Pit House. The default is 900°.

## Running

```sh
cargo run
```

The dev profile already optimizes dependencies. Force feedback is on from
the start, so rest your hands lightly on the wheel when launching.

## Controls

| Input | Action |
| --- | --- |
| Wheel, throttle, brake | Drive |
| Right / left paddle, or E / Q | Shift up / down (manual mode) |
| A | Automatic mode (the paddles return to manual) |
| R | Reset to the start line |
| C | Chase or top-down camera |
| - / = | Zoom out / in |
| F | Force feedback on / off |
| Left / Right | Force feedback strength |
| Hold T | Force feedback direction test |
| Up / Down | Steering rotation range |
| M | Mute the engine |

## Project layout

| Path | Contents |
| --- | --- |
| `src/wheel.rs` | evdev input: steering, pedals, paddles |
| `src/ffb.rs` | Force feedback output thread and torque mapping |
| `src/vehicle.rs` | Vehicle dynamics |
| `src/tire.rs` | Tire model |
| `src/surface.rs` | Ground surfaces |
| `src/engine.rs` | Engine model |
| `src/powertrain.rs` | Torque converter, gearbox and shift logic |
| `src/engine_sound/` | Engine sound synthesis (source, exhaust waveguides, cabin) |
| `src/audio.rs` | Audio output via cpal |
| `src/track.rs` | Circuit geometry, surfaces, rendering, lap timing |
| `src/view.rs` | 3D scene, car model and camera |
| `src/hud.rs` | On-screen telemetry and wheel and pedal display |

## Tests

```sh
cargo test
```

The tests check the physics against theory and against known behaviour, for
example:

- steady-state cornering against the linear bicycle model;
- the quarter-wave resonance of a waveguide pipe;
- the firing-frequency harmonics of the engine sound;
- yaw acceleration staying within what the tires can produce during a spin.

To hear the engine sound offline, render a scripted drive to a WAV file:

```sh
ENGINE_PREVIEW=engine.wav cargo test render_preview -- --ignored
```

## Roadmap

- More cars, such as the Lexus LFA's V10.
- Wheel spin and lock-up (wheel rotation and slip ratio).
- Curb rumble in the force feedback.

## References

The engine sound design draws on:

- Baldan, Lachambre, Delle Monache, Boussard. *Physically informed car engine
  sound synthesis for virtual and augmented environments.* SIVE, IEEE VR 2015.
- The Sound Design Toolkit, `SDTMotor`: https://github.com/SkAT-VG/SDT
- Jagla, Maillard, Martin. *Sample-based engine noise synthesis using an
  enhanced pitch-synchronous overlap-and-add method.* JASA 132(5), 2012.
- Pieren, Bütler, Heutschi. *Auralization of accelerating passenger cars
  using spectral modeling synthesis.* Applied Sciences 6(1), 2016.
- Michon et al., Sound and Music Computing (SMC) 2016, on sound for driving
  simulators: https://www.wendyju.com/publications/smc2016MichonDriving.pdf
