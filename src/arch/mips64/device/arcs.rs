//! Calls into the ARCS firmware (the Indy PROM).
//!
//! The PROM leaves a system parameter block at physical 0x1000. Its firmware
//! vector is a table of 32-bit entry points: `GetMemoryDescriptor` is entry
//! 18, `Reboot` 6 and `GetEnvironmentVariable` 30. The kernel asks the PROM
//! for the memory map, environment variables and a reboot. It does not use
//! the PROM's console: `ConsoleOut` only says which device the kernel's own
//! console drives, the Newport screen or Serial Port 1 (`serial(0)`).
//!
//! The PROM is 32-bit o32 code. It needs a 16-byte argument area at the stack
//! pointer and it restores callee-saved registers with `lw`, which loses the
//! upper half of a 64-bit value, so `arcs_call` saves them itself.
//!
//! The PROM is only callable while its own memory is intact, so `start`
//! leaves every range the PROM reports as its own out of the frame allocator.

use core::{
    arch::naked_asm,
    sync::atomic::{AtomicBool, Ordering},
};

use crate::arch::cp0;

const SPB: *const u32 = 0xA000_1000 as *const u32;
const SPB_SIGNATURE: u32 = 0x5343_5241;
/// Word index of the firmware vector pointer in the parameter block.
const SPB_FIRMWARE_VECTOR: usize = 8;

const FN_REBOOT: usize = 6;
const FN_GET_PEER: usize = 9;
const FN_GET_CHILD: usize = 10;
const FN_GET_MEMORY_DESCRIPTOR: usize = 18;
const FN_GET_ENVIRONMENT_VARIABLE: usize = 30;


/// Memory descriptor types as SGI's ARCS numbers them.
pub const MEM_FREE_CONTIGUOUS: u32 = 2;
pub const MEM_FREE: u32 = 3;
pub const MEM_LOADED_PROGRAM: u32 = 5;

#[repr(C)]
pub struct MemoryDescriptor {
    pub kind: u32,
    /// Physical page number, 4 KiB pages.
    pub base_page: u32,
    pub page_count: u32,
}

static PRESENT: AtomicBool = AtomicBool::new(false);
/// The PROM console is the serial port: do not also write the port directly.
static CONSOLE_IS_SERIAL: AtomicBool = AtomicBool::new(false);

#[unsafe(naked)]
unsafe extern "C" fn arcs_call(a0: usize, a1: usize, a2: usize, a3: usize, entry: usize) -> isize {
    naked_asm!(
        "
        .set noreorder
        addiu   $sp, $sp, -112
        sd      $31, 104($sp)
        sd      $16, 16($sp)
        sd      $17, 24($sp)
        sd      $18, 32($sp)
        sd      $19, 40($sp)
        sd      $20, 48($sp)
        sd      $21, 56($sp)
        sd      $22, 64($sp)
        sd      $23, 72($sp)
        sd      $30, 80($sp)
        sd      $28, 88($sp)
        move    $25, $8
        jalr    $25
        nop
        ld      $31, 104($sp)
        ld      $16, 16($sp)
        ld      $17, 24($sp)
        ld      $18, 32($sp)
        ld      $19, 40($sp)
        ld      $20, 48($sp)
        ld      $21, 56($sp)
        ld      $22, 64($sp)
        ld      $23, 72($sp)
        ld      $30, 80($sp)
        ld      $28, 88($sp)
        sll     $2, $2, 0
        jr      $31
        addiu   $sp, $sp, 112
        .set reorder
        "
    )
}

/// Call firmware vector entry `index` with interrupts off: the PROM is not
/// re-entrant and must not be preempted by our own handlers.
unsafe fn call(index: usize, a0: usize, a1: usize, a2: usize, a3: usize) -> isize {
    unsafe {
        let vector = SPB.add(SPB_FIRMWARE_VECTOR).read_volatile() as usize as *const u32;
        let entry = vector.add(index).read_volatile() as usize;
        let status = cp0::read_status();
        cp0::write_status(status & !cp0::status::IE);
        let cache_was_on = crate::arch::cache::scache_pause();
        let ret = arcs_call(a0, a1, a2, a3, entry);
        crate::arch::cache::scache_resume(cache_was_on);
        cp0::write_status(status);
        ret
    }
}

/// Look for the parameter block and read the `ConsoleOut` variable.
pub unsafe fn init() {
    unsafe {
        if SPB.read_volatile() != SPB_SIGNATURE {
            return;
        }
        PRESENT.store(true, Ordering::Relaxed);
        let value = call(
            FN_GET_ENVIRONMENT_VARIABLE,
            b"ConsoleOut\0".as_ptr() as usize,
            0,
            0,
            0,
        ) as usize as *const u8;
        // `serial(0)` or `video()`; headless, the PROM falls back to serial.
        if !value.is_null() && value.read() == b's' {
            CONSOLE_IS_SERIAL.store(true, Ordering::Relaxed);
        }
    }
}

pub fn present() -> bool {
    PRESENT.load(Ordering::Relaxed)
}

/// True when the PROM's console is the graphics board. The kernel then
/// draws the screen and reads the keyboard itself (`newport`, `kbd`); the
/// PROM's `Read` and `Write` are not used.
pub fn console_is_graphics() -> bool {
    present() && !CONSOLE_IS_SERIAL.load(Ordering::Relaxed)
}

/// The descriptor after `previous`, or the first one for `None`.
pub fn memory_descriptor(previous: Option<&MemoryDescriptor>) -> Option<&'static MemoryDescriptor> {
    if !present() {
        return None;
    }
    let previous = previous.map_or(0, |d| d as *const MemoryDescriptor as usize);
    let next = unsafe { call(FN_GET_MEMORY_DESCRIPTOR, previous, 0, 0, 0) } as usize;
    unsafe { (next as *const MemoryDescriptor).as_ref() }
}

/// Ask the firmware to restart the machine (`Reboot`, entry 6): it resets
/// the devices itself and then autoboots if `AutoLoad` says so. Returns only
/// if there is no firmware or the call comes back.
pub unsafe fn reboot() {
    if !present() {
        return;
    }
    unsafe {
        // The PROM's own vectors, interrupts off and masked.
        cp0::write_status(cp0::status::BEV);
        call(FN_REBOOT, 0, 0, 0, 0);
    }
}

/// The inventory as text, `class/type/key'name'` per component, written
/// once at boot by `log_components`.
pub struct Inventory {
    text: [u8; 640],
    len: usize,
}

impl core::fmt::Write for Inventory {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let room = self.text.len() - self.len;
        let n = s.len().min(room);
        self.text[self.len..self.len + n].copy_from_slice(&s.as_bytes()[..n]);
        self.len += n;
        Ok(())
    }
}

static INVENTORY: core::cell::SyncUnsafeCell<Inventory> =
    core::cell::SyncUnsafeCell::new(Inventory { text: [0; 640], len: 0 });

static SECONDARY_CACHE_KEY: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// The key of the secondary cache component, 0 when the PROM lists none.
pub fn secondary_cache_key() -> u32 {
    SECONDARY_CACHE_KEY.load(Ordering::Relaxed)
}

/// The PROM's inventory as recorded at boot.
pub fn inventory() -> &'static str {
    let inventory = unsafe { &*INVENTORY.get() };
    core::str::from_utf8(&inventory.text[..inventory.len]).unwrap_or("")
}

/// One node of the PROM's configuration tree, as the PROM lays it out.
#[repr(C)]
struct Component {
    class: u32,
    kind: u32,
    flags: u32,
    version: u16,
    revision: u16,
    key: u32,
    affinity: u32,
    data_size: u32,
    identifier_len: u32,
    identifier: u32,
}

/// Log the PROM's hardware inventory, the data its `hinv` command prints:
/// class, type and key of every component, with its name. A cache
/// component's key holds its size: the low byte is log2 of the size in
/// 4 KiB pages, the next byte log2 of the line size. The secondary cache of
/// an R4600SC or R5000SC module shows here; nothing else reports it without
/// touching the module.
pub fn log_components() {
    fn walk(node: usize, depth: usize, budget: &mut usize) {
        let mut node = node;
        while node != 0 && *budget != 0 {
            *budget -= 1;
            let component = unsafe { &*(node as *const Component) };
            let mut name = [0u8; 32];
            let len = (component.identifier_len as usize).min(name.len());
            if component.identifier != 0 {
                for (i, byte) in name[..len].iter_mut().enumerate() {
                    *byte = unsafe { (component.identifier as usize as *const u8).add(i).read() };
                }
            }
            let text = core::str::from_utf8(&name[..len]).unwrap_or("?");
            // Class 2 is a cache, type 7 the secondary cache.
            if component.class == 2 && component.kind == 7 {
                SECONDARY_CACHE_KEY.store(component.key, Ordering::Relaxed);
            }
            {
                // Kept for `cachestat`: the log scheme starts later than this.
                use core::fmt::Write;
                let _ = write!(
                    unsafe { &mut *INVENTORY.get() },
                    " {}/{}/{:#x}'{}'",
                    component.class,
                    component.kind,
                    component.key,
                    text.trim_end_matches('\0')
                );
            }
            info!(
                "Indy: PROM component depth {} class {} type {} key {:#x} '{}'",
                depth,
                component.class,
                component.kind,
                component.key,
                text.trim_end_matches('\0')
            );
            if depth < 6 {
                let child = unsafe { call(FN_GET_CHILD, node, 0, 0, 0) } as usize;
                walk(child, depth + 1, budget);
            }
            node = unsafe { call(FN_GET_PEER, node, 0, 0, 0) } as usize;
        }
    }
    if !present() {
        return;
    }
    let root = unsafe { call(FN_GET_CHILD, 0, 0, 0, 0) } as usize;
    walk(root, 0, &mut 48);
}

/// The value of a PROM environment variable. `name` ends with a NUL byte.
/// The string lives in the PROM's memory, which the kernel never reuses.
pub fn env(name: &'static [u8]) -> Option<&'static [u8]> {
    if !present() || name.last() != Some(&0) {
        return None;
    }
    unsafe {
        let value =
            call(FN_GET_ENVIRONMENT_VARIABLE, name.as_ptr() as usize, 0, 0, 0) as usize as *const u8;
        if value.is_null() {
            return None;
        }
        let mut len = 0;
        while len < 128 && value.add(len).read() != 0 {
            len += 1;
        }
        Some(core::slice::from_raw_parts(value, len))
    }
}
