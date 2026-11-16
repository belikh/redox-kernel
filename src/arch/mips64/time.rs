//! Monotonic time from CP0 Count.
//!
//! Count runs at half the pipeline clock. The PROM measures the CPU clock
//! and publishes it in MHz as the `cpufreq` variable (133 on an R4600PC
//! Indy), so the rate comes from there. Without a PROM value the rate is
//! 33 MHz, which is what the emulator runs at by default.

use core::sync::atomic::{AtomicU32, AtomicU64, Ordering};

use super::cp0;
use crate::sync::CleanLockToken;

/// Scheduler tick rate.
pub const TICKS_PER_SECOND: u64 = 100;

const DEFAULT_COUNT_HZ: u64 = 33_000_000;

/// CP0 Count increments per second.
static COUNT_HZ: AtomicU64 = AtomicU64::new(DEFAULT_COUNT_HZ);
static COUNTS_PER_TICK: AtomicU32 = AtomicU32::new((DEFAULT_COUNT_HZ / TICKS_PER_SECOND) as u32);

pub fn count_hz() -> u64 {
    COUNT_HZ.load(Ordering::Relaxed)
}

/// Take the Count rate from the PROM's `cpufreq` (CPU MHz, decimal).
fn calibrate() {
    let mhz = super::device::arcs::env(b"cpufreq\0")
        .and_then(|text| core::str::from_utf8(text).ok())
        .and_then(|text| text.trim().parse::<u64>().ok())
        .filter(|mhz| (20..=400).contains(mhz));
    if let Some(mhz) = mhz {
        let hz = mhz * 1_000_000 / 2;
        COUNT_HZ.store(hz, Ordering::Relaxed);
        COUNTS_PER_TICK.store((hz / TICKS_PER_SECOND) as u32, Ordering::Relaxed);
    }
    info!(
        "Indy: PROM cpufreq {:?} MHz, Count at {} Hz",
        mhz,
        count_hz()
    );
}

/// Count value extended to 64 bits across wraps by the timer interrupt.
static COUNT_HIGH: AtomicU64 = AtomicU64::new(0);
static LAST_COUNT: AtomicU64 = AtomicU64::new(0);

/// Boot deadman, for testing kernels on a machine nobody is sitting at: if
/// nothing writes `disarm` to `/scheme/sys/kstop` within this many seconds
/// of boot, the kernel reboots through the PROM, which loads whatever image
/// the boot server holds by then. The bring-up shell disarms it once its
/// TCP port is open. Zero turns the deadman off.
const DEADMAN_SECONDS: u64 = 300;
static DEADMAN_ARMED: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(true);
static TICKS: AtomicU64 = AtomicU64::new(0);

pub fn disarm_deadman() {
    if DEADMAN_ARMED.swap(false, Ordering::Relaxed) {
        info!("Indy: boot deadman disarmed");
    }
}

/// Called from the timer interrupt: track Count wrap-around and arm the next tick.
pub fn tick() {
    let ticks = TICKS.fetch_add(1, Ordering::Relaxed);
    if DEADMAN_SECONDS != 0
        && ticks == DEADMAN_SECONDS * TICKS_PER_SECOND
        && DEADMAN_ARMED.load(Ordering::Relaxed)
    {
        println!("Indy: boot deadman: no disarm in {} s, rebooting", DEADMAN_SECONDS);
        unsafe { crate::arch::stop::emergency_reset() }
    }
    let now = cp0::read_count() as u64;
    let last = LAST_COUNT.load(Ordering::Relaxed);
    if now < last {
        COUNT_HIGH.fetch_add(1 << 32, Ordering::Relaxed);
    }
    LAST_COUNT.store(now, Ordering::Relaxed);
    // Arm the next tick one period after the last. If the kernel kept
    // interrupts off past that point, Count is already beyond it and the
    // match would not come until Count wraps, about a minute later: start
    // again from now. (Found with kdbg: TICKS stood at 1 a minute after
    // boot.)
    let period = COUNTS_PER_TICK.load(Ordering::Relaxed);
    let mut next = cp0::read_compare().wrapping_add(period);
    let ahead = next.wrapping_sub(cp0::read_count()) as i32;
    if ahead < (period / 16) as i32 {
        next = cp0::read_count().wrapping_add(period);
    }
    unsafe {
        cp0::write_compare(next);
    }
}

/// Start the Count/Compare timer and unmask IP7.
pub unsafe fn init() {
    calibrate();
    unsafe {
        cp0::write_count(0);
        cp0::write_compare(COUNTS_PER_TICK.load(Ordering::Relaxed));
        LAST_COUNT.store(0, Ordering::Relaxed);
        cp0::write_status(cp0::read_status() | cp0::status::IM7);
    }
}

fn count64() -> u64 {
    let now = cp0::read_count() as u64;
    let mut high = COUNT_HIGH.load(Ordering::Relaxed);
    if now < LAST_COUNT.load(Ordering::Relaxed) {
        high += 1 << 32;
    }
    high | now
}

pub fn monotonic_absolute(_token: &mut CleanLockToken) -> u128 {
    // 64-bit arithmetic: the CPU divides 64 bits in one instruction, and a
    // 128-bit division is a library routine on every context switch.
    let (count, hz) = (count64(), count_hz());
    ((count / hz) * 1_000_000_000 + (count % hz) * 1_000_000_000 / hz) as u128
}

pub fn monotonic_resolution() -> u128 {
    1_000_000_000u128 / count_hz() as u128
}

/// A sampling profile of the timer interrupt: where the interrupted code
/// was. Kernel text in 256-byte buckets from the image base, one counter
/// for user mode. `echo profile > /scheme/sys/kstop` logs the busiest
/// buckets and clears the counts.
const PROFILE_BUCKETS: usize = 8192;
static PROFILE: [core::sync::atomic::AtomicU16; PROFILE_BUCKETS] =
    [const { core::sync::atomic::AtomicU16::new(0) }; PROFILE_BUCKETS];
static PROFILE_USER: AtomicU32 = AtomicU32::new(0);
static PROFILE_OTHER: AtomicU32 = AtomicU32::new(0);
const PROFILE_BASE: usize = 0x8804_0000;

/// User-mode samples of one program, chosen by the last four characters
/// of its name: 256-byte buckets of its first 2 MiB.
static PROFILE_USER_TAG: AtomicU32 = AtomicU32::new(u32::from_be_bytes(*b"tack"));

/// Choose the program whose user-mode PC is sampled, by the last four
/// characters of its name (`profile=ebfs` to `/scheme/sys/kstop`).
pub fn profile_program(tag: [u8; 4]) {
    PROFILE_USER_TAG.store(u32::from_be_bytes(tag), Ordering::Relaxed);
    for bucket in PROFILE_USER_PC.iter() {
        bucket.store(0, Ordering::Relaxed);
    }
}
static PROFILE_USER_PC: [core::sync::atomic::AtomicU16; PROFILE_BUCKETS] =
    [const { core::sync::atomic::AtomicU16::new(0) }; PROFILE_BUCKETS];

pub fn profile_sample(epc: usize, user: bool) {
    if user {
        PROFILE_USER.fetch_add(1, Ordering::Relaxed);
        if crate::arch::cache::current_tag() == PROFILE_USER_TAG.load(Ordering::Relaxed)
            && let Some(bucket) = PROFILE_USER_PC.get(epc >> 8)
        {
            bucket.fetch_add(1, Ordering::Relaxed);
        }
    } else if let Some(bucket) = epc.checked_sub(PROFILE_BASE).and_then(|o| PROFILE.get(o >> 8)) {
        bucket.fetch_add(1, Ordering::Relaxed);
    } else {
        PROFILE_OTHER.fetch_add(1, Ordering::Relaxed);
    }
}

pub fn profile_log() {
    let mut top = [(0u16, 0usize); 24];
    let mut total = 0u32;
    for (i, bucket) in PROFILE.iter().enumerate() {
        let count = bucket.swap(0, Ordering::Relaxed);
        total += count as u32;
        if count > top[23].0 {
            top[23] = (count, i);
            top.sort_unstable_by(|a, b| b.0.cmp(&a.0));
        }
    }
    info!(
        "Indy: profile: {} kernel, {} user, {} other samples",
        total,
        PROFILE_USER.swap(0, Ordering::Relaxed),
        PROFILE_OTHER.swap(0, Ordering::Relaxed)
    );
    for chunk in top.chunks(6) {
        let mut line = alloc::string::String::new();
        for (count, bucket) in chunk {
            use core::fmt::Write;
            let _ = write!(line, " {:#x}:{}", PROFILE_BASE + (bucket << 8), count);
        }
        info!("Indy: profile:{}", line);
    }
    // The chosen program's busiest buckets.
    let mut top = [(0u16, 0usize); 48];
    let mut total = 0u32;
    for (i, bucket) in PROFILE_USER_PC.iter().enumerate() {
        let count = bucket.swap(0, Ordering::Relaxed);
        total += count as u32;
        if count > top[47].0 {
            top[47] = (count, i);
            top.sort_unstable_by(|a, b| b.0.cmp(&a.0));
        }
    }
    let tag = PROFILE_USER_TAG.load(Ordering::Relaxed).to_be_bytes();
    info!(
        "Indy: profile: {} samples in the program ending in '{}'",
        total,
        core::str::from_utf8(&tag).unwrap_or("?")
    );
    for chunk in top.chunks(12) {
        let mut line = alloc::string::String::new();
        for (count, bucket) in chunk {
            use core::fmt::Write;
            if *count != 0 {
                let _ = write!(line, " {:#x}:{}", bucket << 8, count);
            }
        }
        info!("Indy: profile: user{}", line);
    }
}
