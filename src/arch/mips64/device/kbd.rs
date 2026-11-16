//! The Indy's keyboard, read from the IOC2's 8042-compatible controller.
//!
//! The controller is in the IOC2 (0x1FBD_9800): data at offset 0x40, command
//! and status at 0x44. The IOC2 is an 8-bit device on the low byte lane of a
//! big-endian word, so the bytes are at kseg1 0xBFBD_9843 and 0xBFBD_9847.
//!
//! `init` selects scancode set 2, turns the mouse port off and enables the
//! keyboard interrupt; the PROM is not called for input. `poll` runs from
//! that interrupt (INT3 mappable line 4): it reads what the
//! controller holds and turns make codes into the bytes a terminal sends
//! (ASCII, control characters, and xterm sequences for the cursor and editing
//! keys).
//!
//! Sources: the IOC2 description in the IRIX 7 project's
//! `docs/research/indy-hardware.md` (15.5) and the PS/2 keyboard protocol.

use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};

const DATA: *mut u8 = 0xBFBD_9843 as *mut u8;
const COMMAND: *mut u8 = 0xBFBD_9847 as *mut u8;

const STATUS_OUTPUT_FULL: u8 = 0x01;
const STATUS_INPUT_FULL: u8 = 0x02;
const STATUS_AUX: u8 = 0x20;

const CONTROLLER_READ_COMMAND_BYTE: u8 = 0x20;
const CONTROLLER_WRITE_COMMAND_BYTE: u8 = 0x60;
const CONTROLLER_DISABLE_AUX: u8 = 0xa7;
/// Command byte: keyboard interrupt, mouse interrupt, keyboard clock off,
/// mouse clock off, translation to set 1.
const COMMAND_KEYBOARD_INTERRUPT: u8 = 0x01;
const COMMAND_AUX_INTERRUPT: u8 = 0x02;
const COMMAND_KEYBOARD_DISABLE: u8 = 0x10;
const COMMAND_AUX_DISABLE: u8 = 0x20;
const COMMAND_TRANSLATE: u8 = 0x40;
const KEYBOARD_SET_SCANCODE_SET: u8 = 0xf0;
const KEYBOARD_ENABLE: u8 = 0xf4;
const ACK: u8 = 0xfa;

static ACTIVE: AtomicBool = AtomicBool::new(false);

/// Raw mode, for a program that needs key presses and releases (the
/// console's mode 9900): every scancode byte is sent as two letters, the
/// high nibble as `A`..`P` and the low nibble as `a`..`p`. Letters pass
/// through a terminal's line discipline whatever its settings, which the
/// bytes themselves (0x03, 0x11, 0x1A, 0x1C are all key codes) would not.
/// Control-Alt-Backspace is not sent: it gives the screen back to the
/// console, for a program that died with the screen.
static RAW: AtomicBool = AtomicBool::new(false);

pub fn set_raw(raw: bool) {
    RAW.store(raw, Ordering::Relaxed);
    BREAK.store(false, Ordering::Relaxed);
    EXTENDED.store(false, Ordering::Relaxed);
    MODIFIERS.fetch_and(CAPS, Ordering::Relaxed);
}

// Decoder state. Only the timer tick touches it.
static BREAK: AtomicBool = AtomicBool::new(false);
static EXTENDED: AtomicBool = AtomicBool::new(false);
/// Bit 0 shift, bit 1 control, bit 2 alt, bit 3 caps lock.
static MODIFIERS: AtomicU8 = AtomicU8::new(0);
const SHIFT: u8 = 1;
const CONTROL: u8 = 2;
const ALT: u8 = 4;
const CAPS: u8 = 8;

fn status() -> u8 {
    unsafe { COMMAND.read_volatile() }
}

/// Wait until the controller has taken the last byte written to it.
fn wait_input_empty() -> bool {
    for _ in 0..200_000 {
        if status() & STATUS_INPUT_FULL == 0 {
            return true;
        }
    }
    false
}

/// Read the next byte from the keyboard, dropping mouse bytes.
fn wait_reply() -> Option<u8> {
    for _ in 0..400_000 {
        let status = status();
        if status & STATUS_OUTPUT_FULL != 0 {
            let byte = unsafe { DATA.read_volatile() };
            if status & STATUS_AUX == 0 {
                return Some(byte);
            }
        }
    }
    None
}

/// Send one byte to the keyboard and wait for its acknowledgement.
fn keyboard_command(byte: u8) -> bool {
    if !wait_input_empty() {
        return false;
    }
    unsafe { DATA.write_volatile(byte) };
    // Scancodes typed during the exchange may come first.
    for _ in 0..8 {
        match wait_reply() {
            Some(ACK) => return true,
            Some(_) => {}
            None => return false,
        }
    }
    false
}

/// Take the keyboard over from the PROM. Returns whether the keyboard
/// acknowledged; the poll runs either way.
pub fn init() -> bool {
    // Drop what the controller still holds.
    for _ in 0..64 {
        if status() & STATUS_OUTPUT_FULL == 0 {
            break;
        }
        unsafe { DATA.read_volatile() };
    }
    // The mouse would fill the same output buffer with movement packets.
    if wait_input_empty() {
        unsafe { COMMAND.write_volatile(CONTROLLER_DISABLE_AUX) };
    }
    // Command byte: the keyboard interrupts (INT3 mappable line 4), the
    // mouse does not, and the controller passes set 2 codes through
    // untranslated.
    if wait_input_empty() {
        unsafe { COMMAND.write_volatile(CONTROLLER_READ_COMMAND_BYTE) };
        let old = wait_reply().unwrap_or(0);
        let new = (old | COMMAND_KEYBOARD_INTERRUPT | COMMAND_AUX_DISABLE)
            & !(COMMAND_AUX_INTERRUPT | COMMAND_KEYBOARD_DISABLE | COMMAND_TRANSLATE);
        if wait_input_empty() {
            unsafe { COMMAND.write_volatile(CONTROLLER_WRITE_COMMAND_BYTE) };
            if wait_input_empty() {
                unsafe { DATA.write_volatile(new) };
            }
        }
    }
    let acknowledged = keyboard_command(KEYBOARD_SET_SCANCODE_SET)
        && keyboard_command(2)
        && keyboard_command(KEYBOARD_ENABLE);
    ACTIVE.store(true, Ordering::Relaxed);
    acknowledged
}

pub fn active() -> bool {
    ACTIVE.load(Ordering::Relaxed)
}

/// Set 2 make code to (plain, shifted) character.
fn character(code: u8) -> Option<(u8, u8)> {
    Some(match code {
        0x1c => (b'a', b'A'),
        0x32 => (b'b', b'B'),
        0x21 => (b'c', b'C'),
        0x23 => (b'd', b'D'),
        0x24 => (b'e', b'E'),
        0x2b => (b'f', b'F'),
        0x34 => (b'g', b'G'),
        0x33 => (b'h', b'H'),
        0x43 => (b'i', b'I'),
        0x3b => (b'j', b'J'),
        0x42 => (b'k', b'K'),
        0x4b => (b'l', b'L'),
        0x3a => (b'm', b'M'),
        0x31 => (b'n', b'N'),
        0x44 => (b'o', b'O'),
        0x4d => (b'p', b'P'),
        0x15 => (b'q', b'Q'),
        0x2d => (b'r', b'R'),
        0x1b => (b's', b'S'),
        0x2c => (b't', b'T'),
        0x3c => (b'u', b'U'),
        0x2a => (b'v', b'V'),
        0x1d => (b'w', b'W'),
        0x22 => (b'x', b'X'),
        0x35 => (b'y', b'Y'),
        0x1a => (b'z', b'Z'),
        0x16 => (b'1', b'!'),
        0x1e => (b'2', b'@'),
        0x26 => (b'3', b'#'),
        0x25 => (b'4', b'$'),
        0x2e => (b'5', b'%'),
        0x36 => (b'6', b'^'),
        0x3d => (b'7', b'&'),
        0x3e => (b'8', b'*'),
        0x46 => (b'9', b'('),
        0x45 => (b'0', b')'),
        0x0e => (b'`', b'~'),
        0x4e => (b'-', b'_'),
        0x55 => (b'=', b'+'),
        0x5d => (b'\\', b'|'),
        0x54 => (b'[', b'{'),
        0x5b => (b']', b'}'),
        0x4c => (b';', b':'),
        0x52 => (b'\'', b'"'),
        0x41 => (b',', b'<'),
        0x49 => (b'.', b'>'),
        0x4a => (b'/', b'?'),
        0x29 => (b' ', b' '),
        0x5a => (b'\r', b'\r'),
        0x66 => (0x7f, 0x7f),
        0x0d => (b'\t', b'\t'),
        0x76 => (0x1b, 0x1b),
        // Keypad.
        0x70 => (b'0', b'0'),
        0x69 => (b'1', b'1'),
        0x72 => (b'2', b'2'),
        0x7a => (b'3', b'3'),
        0x6b => (b'4', b'4'),
        0x73 => (b'5', b'5'),
        0x74 => (b'6', b'6'),
        0x6c => (b'7', b'7'),
        0x75 => (b'8', b'8'),
        0x7d => (b'9', b'9'),
        0x71 => (b'.', b'.'),
        0x79 => (b'+', b'+'),
        0x7b => (b'-', b'-'),
        0x7c => (b'*', b'*'),
        _ => return None,
    })
}

/// Set 2 make code after the 0xE0 prefix to the sequence a terminal sends.
fn extended(code: u8) -> Option<&'static [u8]> {
    Some(match code {
        0x75 => b"\x1b[A",
        0x72 => b"\x1b[B",
        0x74 => b"\x1b[C",
        0x6b => b"\x1b[D",
        0x6c => b"\x1b[H",
        0x69 => b"\x1b[F",
        0x70 => b"\x1b[2~",
        0x71 => b"\x1b[3~",
        0x7d => b"\x1b[5~",
        0x7a => b"\x1b[6~",
        0x4a => b"/",
        0x5a => b"\r",
        _ => return None,
    })
}

/// Decode one scancode byte; `emit` receives the bytes of a key press.
fn decode(byte: u8, emit: &mut dyn FnMut(u8)) {
    if RAW.load(Ordering::Relaxed) {
        // Follow the prefixes and Control and Alt far enough to see the
        // escape chord.
        let released = byte != 0xf0 && BREAK.swap(false, Ordering::Relaxed);
        match byte {
            0xf0 => BREAK.store(true, Ordering::Relaxed),
            0xe0 => {}
            0x14 | 0x11 => {
                let bit = if byte == 0x14 { CONTROL } else { ALT };
                if released {
                    MODIFIERS.fetch_and(!bit, Ordering::Relaxed);
                } else {
                    MODIFIERS.fetch_or(bit, Ordering::Relaxed);
                }
            }
            0x66 if !released
                && MODIFIERS.load(Ordering::Relaxed) & (CONTROL | ALT) == CONTROL | ALT =>
            {
                if super::newport::force_text() {
                    return;
                }
            }
            _ => {}
        }
        emit(b'A' + (byte >> 4));
        emit(b'a' + (byte & 0xf));
        return;
    }
    match byte {
        0xe0 => {
            EXTENDED.store(true, Ordering::Relaxed);
            return;
        }
        0xf0 => {
            BREAK.store(true, Ordering::Relaxed);
            return;
        }
        // Acknowledge, resend, self-test and echo replies are not keys.
        0xfa | 0xfe | 0xaa | 0xee | 0x00 | 0xff => return,
        _ => {}
    }
    let released = BREAK.swap(false, Ordering::Relaxed);
    let was_extended = EXTENDED.swap(false, Ordering::Relaxed);
    let modifier = match (was_extended, byte) {
        (false, 0x12) | (false, 0x59) => SHIFT,
        (_, 0x14) => CONTROL,
        (_, 0x11) => ALT,
        _ => 0,
    };
    if modifier != 0 {
        if released {
            MODIFIERS.fetch_and(!modifier, Ordering::Relaxed);
        } else {
            MODIFIERS.fetch_or(modifier, Ordering::Relaxed);
        }
        return;
    }
    if released {
        return;
    }
    if !was_extended && byte == 0x58 {
        MODIFIERS.fetch_xor(CAPS, Ordering::Relaxed);
        return;
    }
    let modifiers = MODIFIERS.load(Ordering::Relaxed);
    if was_extended {
        // 0xE0 0x12 is the fake shift some keys send around their code.
        if let Some(sequence) = extended(byte) {
            for &b in sequence {
                emit(b);
            }
        }
        return;
    }
    let Some((plain, shifted)) = character(byte) else {
        return;
    };
    let mut ch = if modifiers & SHIFT != 0 { shifted } else { plain };
    if modifiers & CAPS != 0 && plain.is_ascii_lowercase() {
        ch = if modifiers & SHIFT != 0 { plain } else { shifted };
    }
    if modifiers & CONTROL != 0 {
        ch = match ch {
            b'a'..=b'z' => ch - b'a' + 1,
            b'@'..=b'_' => ch - b'@',
            b' ' | b'2' => 0,
            b'/' => 0x1f,
            b'?' => 0x7f,
            _ => ch,
        };
    }
    if modifiers & ALT != 0 {
        emit(0x1b);
    }
    emit(ch);
}

/// Read what the controller holds, up to a bound that keeps the tick short.
///
/// Runs without `init` as well (a serial console): the bytes are read and
/// dropped, so that a controller the PROM left interrupting cannot hold its
/// interrupt line up.
pub fn poll(emit: &mut dyn FnMut(u8)) {
    let active = active();
    for _ in 0..16 {
        let status = status();
        if status & STATUS_OUTPUT_FULL == 0 {
            break;
        }
        let byte = unsafe { DATA.read_volatile() };
        if active && status & STATUS_AUX == 0 {
            decode(byte, emit);
        }
    }
}
