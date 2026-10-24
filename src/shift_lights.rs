//! Shift lights: the RPM indicator LEDs on a MOZA steering wheel, driven
//! over the wheel base's serial port.
//!
//! MOZA's serial protocol here follows the Boxflat project's
//! reverse-engineered implementation (github.com/Lawstorant/boxflat). A
//! frame is 0x7E, payload length, command group, device id, the payload
//! (command id bytes then value bytes) and a checksum: 0x0D plus the sum of
//! all preceding bytes, modulo 256. The wheel is device 0x17 (0x15 on some
//! wheels), reached through the base. Every request gets a reply with the
//! group's high bit set and the device id's nibbles swapped.
//!
//! Wheels speak one of two generations of the protocol. The newer one has a
//! telemetry mode in which the game sets the lit LEDs as a bitmask; the
//! older one has an RPM indicator mode and its own telemetry command. The
//! wheel is probed for each, and the original telemetry mode is restored on
//! exit.

use std::fs::{self, File, OpenOptions};
use std::io::{Read, Write};
use std::path::PathBuf;
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender};
use std::thread;
use std::time::{Duration, Instant};

use bevy::prelude::*;
use nix::sys::termios::{self, SetArg, SpecialCharacterIndices};

use crate::vehicle::Vehicle;

/// Where udev lists serial devices by name.
const SERIAL_BY_ID: &str = "/dev/serial/by-id";
const LED_COUNT: u32 = 10;
/// Device ids the wheel may answer on.
const WHEEL_DEVICES: [u8; 2] = [0x17, 0x15];

const WRITE_GROUP: u8 = 0x3F;
const READ_GROUP: u8 = 0x40;
/// Newer protocol: LED mode (0 off, 1 telemetry, 2 static), the lit-LED
/// bitmask (little-endian) and the colours used in telemetry mode.
const TELEMETRY_MODE: [u8; 2] = [28, 0];
const TELEMETRY_ON: u8 = 1;
const RPM_TELEMETRY: [u8; 2] = [26, 0];
const RPM_COLORS: [u8; 2] = [25, 0];
/// Older protocol: indicator mode (1 RPM telemetry), a setting read only to
/// detect the protocol, and the bitmask (big-endian) in its own group.
const OLD_INDICATOR_MODE: [u8; 1] = [4];
const OLD_PROBE: [u8; 2] = [24, 0];
const OLD_TELEMETRY_GROUP: u8 = 0x41;
const OLD_RPM_TELEMETRY: [u8; 2] = [0xFD, 0xDE];

/// Telemetry colours, left to right: green, then yellow, then red.
const LED_COLORS: [[u8; 3]; 10] = [
    [0, 255, 0],
    [0, 255, 0],
    [0, 255, 0],
    [0, 255, 0],
    [255, 180, 0],
    [255, 180, 0],
    [255, 180, 0],
    [255, 0, 0],
    [255, 0, 0],
    [255, 0, 0],
];

/// The first LED lights at this fraction of the redline.
const FIRST_LED_FRACTION: f32 = 0.6;
/// All LEDs are lit, and flash, this far below the redline: the shift point, rpm.
const SHIFT_POINT_BELOW_REDLINE: f32 = 300.0;
const FLASH_HZ: f32 = 8.0;
/// How long to wait for the wheel to answer a probe.
const PROBE_TIMEOUT: Duration = Duration::from_millis(300);
/// Longest payload a reply can have; a "frame" claiming more is noise.
const MAX_PAYLOAD: usize = 32;
/// Shortest time between LED updates sent to the wheel.
const UPDATE_INTERVAL: Duration = Duration::from_millis(20);
/// How long to wait before looking for the wheel again.
const RETRY_INTERVAL: Duration = Duration::from_secs(10);

pub struct ShiftLightsPlugin;

impl Plugin for ShiftLightsPlugin {
    fn build(&self, app: &mut App) {
        let (sender, receiver) = mpsc::channel();
        thread::Builder::new()
            .name("shift-lights".into())
            .spawn(move || run_shift_lights(receiver))
            .expect("failed to spawn shift lights thread");
        app.insert_resource(ShiftLights(sender))
            .add_systems(Update, (update_shift_lights, restore_on_exit));
    }
}

enum LightCommand {
    /// Light the LEDs in this bitmask (bit 0 is the leftmost LED).
    Show(u16),
    /// Put the wheel back as it was, then acknowledge.
    Restore(Sender<()>),
}

#[derive(Resource)]
struct ShiftLights(Sender<LightCommand>);

/// LEDs to light at `rpm` for an engine redlining at `redline`, at time
/// `seconds` (for flashing at the shift point).
fn led_mask(rpm: f32, redline: f32, seconds: f32) -> u16 {
    let all = (1u16 << LED_COUNT) - 1;
    let shift_point = redline - SHIFT_POINT_BELOW_REDLINE;
    if rpm >= shift_point {
        let flash_on = ((seconds * FLASH_HZ * 2.0) as u32).is_multiple_of(2);
        return if flash_on { all } else { 0 };
    }
    let first = redline * FIRST_LED_FRACTION;
    let fraction = ((rpm - first) / (shift_point - first)).clamp(0.0, 1.0);
    let lit = (fraction * LED_COUNT as f32).ceil() as u32;
    ((1u32 << lit) - 1) as u16
}

fn update_shift_lights(
    time: Res<Time>,
    vehicle: Res<Vehicle>,
    lights: Res<ShiftLights>,
    mut shown: Local<Option<u16>>,
) {
    let engine = &vehicle.powertrain.engine;
    let mask = led_mask(engine.rpm, engine.spec.redline_rpm, time.elapsed_secs());
    if *shown != Some(mask) {
        *shown = Some(mask);
        // Nothing to do if the lights thread has given up on the wheel.
        let _ = lights.0.send(LightCommand::Show(mask));
    }
}

/// On exit, give the wheel a moment to be put back as it was.
fn restore_on_exit(mut exits: MessageReader<AppExit>, lights: Res<ShiftLights>) {
    if exits.read().next().is_none() {
        return;
    }
    let (done, restored) = mpsc::channel();
    if lights.0.send(LightCommand::Restore(done)).is_ok() {
        let _ = restored.recv_timeout(Duration::from_millis(300));
    }
}

/// Builds one protocol frame.
fn frame(group: u8, device: u8, payload: &[u8]) -> Vec<u8> {
    let mut bytes = vec![0x7E, payload.len() as u8, group, device];
    bytes.extend_from_slice(payload);
    let checksum = bytes.iter().fold(0x0Du32, |sum, &b| sum + b as u32) as u8;
    bytes.push(checksum);
    bytes
}

/// A reply: group, device and payload.
#[derive(Debug, PartialEq)]
struct Reply {
    group: u8,
    device: u8,
    payload: Vec<u8>,
}

/// Takes complete, valid frames off the front of `buffer`, dropping noise.
fn parse_replies(buffer: &mut Vec<u8>) -> Vec<Reply> {
    let mut replies = Vec::new();
    loop {
        let Some(start) = buffer.iter().position(|&b| b == 0x7E) else {
            buffer.clear();
            return replies;
        };
        buffer.drain(..start);
        if buffer.len() < 2 {
            return replies;
        }
        if buffer[1] as usize > MAX_PAYLOAD {
            // Not a real frame start; skip this byte and resync.
            buffer.drain(..1);
            continue;
        }
        let total = buffer[1] as usize + 5;
        if buffer.len() < total {
            return replies;
        }
        let candidate: Vec<u8> = buffer[..total].to_vec();
        let checksum = candidate[..total - 1]
            .iter()
            .fold(0x0Du32, |sum, &b| sum + b as u32) as u8;
        if checksum == candidate[total - 1] {
            replies.push(Reply {
                group: candidate[2],
                device: candidate[3],
                payload: candidate[4..total - 1].to_vec(),
            });
            buffer.drain(..total);
        } else {
            // Not a real frame start; skip this byte and resync.
            buffer.drain(..1);
        }
    }
}

#[derive(Clone, Copy, Debug)]
enum Protocol {
    /// Newer wheels, with the telemetry mode the wheel was in before.
    New {
        original_mode: u8,
    },
    Old,
}

struct Wheel {
    port: File,
    device: u8,
    protocol: Protocol,
    buffer: Vec<u8>,
}

/// The base's serial port, found by name.
fn find_port() -> Option<PathBuf> {
    fs::read_dir(SERIAL_BY_ID)
        .ok()?
        .filter_map(Result::ok)
        .map(|entry| entry.path())
        .find(|path| {
            let name = path.file_name().unwrap_or_default().to_string_lossy();
            name.contains("MOZA") && name.contains("Base")
        })
}

fn open_port(path: &PathBuf) -> std::io::Result<File> {
    let port = OpenOptions::new().read(true).write(true).open(path)?;
    let mut settings = termios::tcgetattr(&port)?;
    // Binary frames: no line editing or byte translation. Reads return
    // after 100 ms if nothing arrives.
    termios::cfmakeraw(&mut settings);
    settings.control_chars[SpecialCharacterIndices::VMIN as usize] = 0;
    settings.control_chars[SpecialCharacterIndices::VTIME as usize] = 1;
    termios::tcsetattr(&port, SetArg::TCSANOW, &settings)?;
    Ok(port)
}

impl Wheel {
    /// Sends a request and waits for its reply, matched by group and the
    /// first payload bytes (the command id).
    fn request(
        port: &mut File,
        buffer: &mut Vec<u8>,
        group: u8,
        device: u8,
        payload: &[u8],
        id_length: usize,
    ) -> std::io::Result<Option<Reply>> {
        port.write_all(&frame(group, device, payload))?;
        let deadline = Instant::now() + PROBE_TIMEOUT;
        let mut chunk = [0u8; 256];
        while Instant::now() < deadline {
            let read = port.read(&mut chunk)?;
            buffer.extend_from_slice(&chunk[..read]);
            for reply in parse_replies(buffer) {
                if reply.group == group | 0x80 && reply.payload.starts_with(&payload[..id_length]) {
                    return Ok(Some(reply));
                }
            }
        }
        Ok(None)
    }

    /// Finds which device id and protocol generation the wheel answers to.
    fn connect(path: &PathBuf) -> std::io::Result<Option<Self>> {
        let mut port = open_port(path)?;
        let mut buffer = Vec::new();
        for device in WHEEL_DEVICES {
            let mut read_mode = TELEMETRY_MODE.to_vec();
            read_mode.push(1);
            if let Some(reply) =
                Self::request(&mut port, &mut buffer, READ_GROUP, device, &read_mode, 2)?
            {
                let original_mode = reply.payload.get(2).copied().unwrap_or(0);
                let protocol = Protocol::New { original_mode };
                return Ok(Some(Self {
                    port,
                    device,
                    protocol,
                    buffer,
                }));
            }
            let mut read_old = OLD_PROBE.to_vec();
            read_old.extend_from_slice(&[0, 1]);
            if Self::request(&mut port, &mut buffer, READ_GROUP, device, &read_old, 2)?.is_some() {
                let protocol = Protocol::Old;
                return Ok(Some(Self {
                    port,
                    device,
                    protocol,
                    buffer,
                }));
            }
        }
        Ok(None)
    }

    fn send(&mut self, group: u8, payload: &[u8]) -> std::io::Result<()> {
        self.port.write_all(&frame(group, self.device, payload))?;
        // Replies aren't needed once connected; keep the input drained.
        let mut chunk = [0u8; 256];
        let _ = self.port.read(&mut chunk);
        self.buffer.clear();
        Ok(())
    }

    /// Hands the LEDs to the game, in game colours.
    fn take_over(&mut self) -> std::io::Result<()> {
        match self.protocol {
            Protocol::New { .. } => {
                self.send(
                    WRITE_GROUP,
                    &[TELEMETRY_MODE[0], TELEMETRY_MODE[1], TELEMETRY_ON],
                )?;
                for half in LED_COLORS.chunks(5).enumerate() {
                    let (first, colors) = half;
                    let mut payload = RPM_COLORS.to_vec();
                    for (offset, rgb) in colors.iter().enumerate() {
                        payload.push((first * 5 + offset) as u8);
                        payload.extend_from_slice(rgb);
                    }
                    self.send(WRITE_GROUP, &payload)?;
                }
                Ok(())
            }
            Protocol::Old => self.send(WRITE_GROUP, &[OLD_INDICATOR_MODE[0], 1]),
        }
    }

    fn show(&mut self, mask: u16) -> std::io::Result<()> {
        match self.protocol {
            Protocol::New { .. } => {
                let [low, high] = mask.to_le_bytes();
                self.send(
                    WRITE_GROUP,
                    &[RPM_TELEMETRY[0], RPM_TELEMETRY[1], low, high],
                )
            }
            Protocol::Old => {
                let mut payload = OLD_RPM_TELEMETRY.to_vec();
                payload.extend_from_slice(&(mask as u32).to_be_bytes());
                self.send(OLD_TELEMETRY_GROUP, &payload)
            }
        }
    }

    fn restore(&mut self) -> std::io::Result<()> {
        self.show(0)?;
        if let Protocol::New { original_mode } = self.protocol {
            self.send(
                WRITE_GROUP,
                &[TELEMETRY_MODE[0], TELEMETRY_MODE[1], original_mode],
            )?;
        }
        Ok(())
    }
}

fn run_shift_lights(commands: Receiver<LightCommand>) {
    loop {
        let wheel = match find_port().map(|path| (Wheel::connect(&path), path)) {
            Some((Ok(Some(wheel)), path)) => {
                info!(
                    "Shift lights on {} (device {:#04x}, {:?} protocol)",
                    path.display(),
                    wheel.device,
                    wheel.protocol
                );
                Some(wheel)
            }
            Some((Ok(None), _)) => {
                warn!("MOZA wheel did not answer on the serial port; shift lights off");
                None
            }
            Some((Err(err), path)) => {
                warn!("Shift lights unavailable: {}: {err}", path.display());
                None
            }
            None => None,
        };
        match wheel {
            Some(mut wheel) => match drive_lights(&mut wheel, &commands) {
                Ok(()) => return,
                Err(err) => warn!("Shift lights stopped: {err}"),
            },
            None => {
                // Keep draining commands while waiting to try again.
                let deadline = Instant::now() + RETRY_INTERVAL;
                while let Some(left) = deadline.checked_duration_since(Instant::now()) {
                    match commands.recv_timeout(left) {
                        Ok(LightCommand::Restore(done)) => {
                            let _ = done.send(());
                        }
                        Ok(LightCommand::Show(_)) | Err(RecvTimeoutError::Timeout) => {}
                        Err(RecvTimeoutError::Disconnected) => return,
                    }
                }
            }
        }
    }
}

/// Shows masks as they come, no faster than the update interval. Returns
/// `Ok` once the game has exited.
fn drive_lights(wheel: &mut Wheel, commands: &Receiver<LightCommand>) -> std::io::Result<()> {
    wheel.take_over()?;
    let mut last_sent = Instant::now() - UPDATE_INTERVAL;
    let mut pending = None;
    loop {
        match commands.recv_timeout(UPDATE_INTERVAL) {
            Ok(LightCommand::Show(mask)) => pending = Some(mask),
            Ok(LightCommand::Restore(done)) => {
                let result = wheel.restore();
                let _ = done.send(());
                return result;
            }
            Err(RecvTimeoutError::Timeout) => {}
            Err(RecvTimeoutError::Disconnected) => return wheel.restore(),
        }
        if let Some(mask) = pending
            && last_sent.elapsed() >= UPDATE_INTERVAL
        {
            wheel.show(mask)?;
            last_sent = Instant::now();
            pending = None;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frames_carry_boxflat_checksums() {
        // Examples computed with Boxflat's checksum formula.
        assert_eq!(
            frame(WRITE_GROUP, 0x17, &[0x1A, 0x00, 0x00, 0x00]),
            [0x7E, 0x04, 0x3F, 0x17, 0x1A, 0x00, 0x00, 0x00, 0xFF]
        );
        assert_eq!(
            frame(WRITE_GROUP, 0x17, &[0x1A, 0x00, 0xFF, 0x03]),
            [0x7E, 0x04, 0x3F, 0x17, 0x1A, 0x00, 0xFF, 0x03, 0x01]
        );
        assert_eq!(
            frame(WRITE_GROUP, 0x17, &[0x1C, 0x00, 0x01]),
            [0x7E, 0x03, 0x3F, 0x17, 0x1C, 0x00, 0x01, 0x01]
        );
    }

    #[test]
    fn replies_are_parsed_and_noise_is_skipped() {
        let reply = frame(READ_GROUP | 0x80, 0x71, &[0x1C, 0x00, 0x02]);
        let mut buffer = vec![0x00, 0x7E, 0x55];
        buffer.extend_from_slice(&reply);
        buffer.extend_from_slice(&[0x7E, 0x04]); // start of an unfinished frame
        let replies = parse_replies(&mut buffer);
        assert_eq!(
            replies,
            [Reply {
                group: 0xC0,
                device: 0x71,
                payload: vec![0x1C, 0x00, 0x02],
            }]
        );
        assert_eq!(buffer, [0x7E, 0x04], "partial frame kept for later");
    }

    #[test]
    fn leds_fill_up_then_flash_at_the_shift_point() {
        let redline = 7400.0;
        assert_eq!(led_mask(2000.0, redline, 0.0), 0);
        let halfway = redline * FIRST_LED_FRACTION
            + (redline - SHIFT_POINT_BELOW_REDLINE - redline * FIRST_LED_FRACTION) * 0.5;
        assert_eq!(led_mask(halfway, redline, 0.0).count_ones(), 5);
        assert_eq!(led_mask(7200.0, redline, 0.0), 0x3FF);
        // The flash alternates every half period.
        let half_period = 1.0 / (FLASH_HZ * 2.0);
        assert_eq!(led_mask(7200.0, redline, half_period + 0.01), 0);
    }
}
