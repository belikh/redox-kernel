//! Kernel entry from the SGI PROM.
//!
//! `boot -f bootp()redox` loads the ECOFF image at kseg0 0x8804_0000 and
//! jumps to `kstart` with ARCS arguments in `$a0`..`$a2`, caches on, TLB
//! unused by us, interrupts off and `Status.BEV` set. There is no bootloader
//! and no `KernelArgs`, so `kstart` builds them: the kernel's own extent from
//! linker symbols, one memory area sized from the memory controller, an empty
//! environment and no bootstrap image yet.

use core::{arch::naked_asm, cell::SyncUnsafeCell};

use crate::{
    allocator,
    arch::{consts, cp0, device, paging},
    context::context::Kstack,
    devices::graphical_debug,
    startup::{memory::BootloaderMemoryKind, KernelArgs},
};

#[repr(C, align(16))]
struct StackAlign<T>(T);

/// Boot stack, page aligned so it can be registered with the memory map.
#[repr(C, align(4096))]
struct PageAlign<T>(T);

const STACK_SIZE: usize = 64 * 1024;
static STACK: SyncUnsafeCell<PageAlign<[u8; STACK_SIZE]>> =
    SyncUnsafeCell::new(PageAlign([0; STACK_SIZE]));

/// Matches `BootloaderMemoryEntry` in startup::memory.
#[repr(C, packed(8))]
struct AreaEntry {
    base: u64,
    size: u64,
    kind: BootloaderMemoryKind,
}

/// The environment handed to userspace through `sys:env`.
/// `NETADDR` is the address the PROM got from the boot server (`netaddr`),
/// so a netbooted machine needs no address of its own in the image.
const ENV_BASE: &[u8] = b"BOOTSTRAP_LOG_LEVEL=info\n";
static ENV: SyncUnsafeCell<[u8; 128]> = SyncUnsafeCell::new([0; 128]);

/// Fill `ENV`; returns its length.
unsafe fn build_env() -> usize {
    unsafe {
        let env = &mut *ENV.get();
        let mut len = 0;
        let mut push = |bytes: &[u8]| {
            let n = bytes.len().min(env.len() - len);
            env[len..len + n].copy_from_slice(&bytes[..n]);
            len += n;
        };
        push(ENV_BASE);
        if let Some(addr) = device::arcs::env(b"netaddr\0") {
            if !addr.is_empty() && addr.len() <= 15 {
                push(b"NETADDR=");
                push(addr);
                push(b"\n");
            }
        }
        len
    }
}

const MAX_AREAS: usize = 16;

static AREAS: SyncUnsafeCell<[AreaEntry; MAX_AREAS]> = SyncUnsafeCell::new(
    [const {
        AreaEntry {
            base: 0,
            size: 0,
            kind: BootloaderMemoryKind::Null,
        }
    }; MAX_AREAS],
);

unsafe extern "C" {
    static __bss_start: u8;
    static __bss_end: u8;
    static __end: u8;
}

// FIXME use extern "custom"
#[unsafe(naked)]
#[unsafe(no_mangle)]
#[unsafe(link_section = ".text.kstart")]
extern "C" fn kstart() {
    naked_asm!(
        "
        .set noreorder
        .set noat
        // Kernel mode, interrupts off, 32-bit addressing, keep BEV until the
        // vectors are installed.
        li      $8, {status}
        mtc0    $8, $12
        nop
        nop
        move    $28, $zero

        la      $sp, {stack}
        li      $8, {stack_size}
        addu    $sp, $sp, $8
        addiu   $sp, $sp, -16

        la      $8, __bss_start
        la      $9, __bss_end
    1:
        sltu    $10, $8, $9
        beqz    $10, 2f
        nop
        sw      $zero, 0($8)
        b       1b
        addiu   $8, $8, 4
    2:
        la      $25, {start}
        jalr    $25
        nop
    3:
        b       3b
        nop
        .set at
        .set reorder
        ",
        status = const cp0::status::BEV,
        stack = sym STACK,
        stack_size = const STACK_SIZE,
        start = sym start,
    );
}

/// Bytes of DRAM the PROM mapped contiguously from `RAM_PHYS_BASE`.
///
/// `MEMCFG0`/`MEMCFG1` each hold two 16-bit bank fields: bit 13 valid, bits
/// 12..8 size in units minus one, bits 7..0 base in units. The unit is 4 MiB
/// below memory controller revision 5 and 16 MiB from revision 5.
fn ram_bytes() -> usize {
    const MC_SYSID: *const u32 = 0xBFA0_001C as *const u32;
    const MC_MEMCFG0: *const u32 = 0xBFA0_00C4 as *const u32;
    const MC_MEMCFG1: *const u32 = 0xBFA0_00CC as *const u32;
    let (sysid, cfg0, cfg1) = unsafe {
        (
            MC_SYSID.read_volatile(),
            MC_MEMCFG0.read_volatile(),
            MC_MEMCFG1.read_volatile(),
        )
    };
    let shift = if sysid & 0x0f >= 5 { 24 } else { 22 };
    let mut end = consts::RAM_PHYS_BASE as u64;
    let fields = [cfg0 >> 16, cfg0 & 0xffff, cfg1 >> 16, cfg1 & 0xffff];
    for _ in 0..fields.len() {
        for field in fields {
            if field & 0x2000 == 0 {
                continue;
            }
            let base = ((field & 0xff) as u64) << shift;
            let size = ((((field >> 8) & 0x1f) + 1) as u64) << shift;
            if base <= end && base + size > end {
                end = base + size;
            }
        }
    }
    (end - consts::RAM_PHYS_BASE as u64) as usize
}

/// The entry to Rust, all things must be initialized
unsafe extern "C" fn start() -> ! {
    unsafe {
        if rmm::mips::FORCE_UNCACHED {
            // Write the data cache back (32 KiB covers every supported CPU),
            // then make kseg0 uncached: Config.K0 = 2.
            let mut addr = 0x8000_0000usize;
            while addr < 0x8000_8000 {
                core::arch::asm!("cache 1, 0({0})", in(reg) addr, options(nostack));
                addr += 16;
            }
            cp0::write_config((cp0::read_config() & !7) | 2);
        }

        // The PROM console (the screen, or the serial port) and our own
        // driver for Serial Port 1.
        device::arcs::init();
        device::serial::init();
        // With a graphics console the kernel draws the screen and reads the
        // keyboard itself from here on.
        let keyboard = device::arcs::console_is_graphics().then(|| {
            device::newport::init();
            device::kbd::init()
        });

        info!("Redox OS starting...");
        match keyboard {
            Some(answered) => {
                let (width, height) = device::newport::size();
                {
                    let raw = device::newport::RAW.lock();
                    info!(
                        "Indy: VC2 video entry {:#x}, frame table {:x?}, timing table adds up to {} by {}",
                        raw.0, raw.1, raw.2, raw.3
                    );
                }
                info!(
                    "Indy: console on the Newport screen, {} by {}, and the IOC2 keyboard{}",
                    width,
                    height,
                    if answered { "" } else { " (no answer from the keyboard)" }
                );
            }
            None => {
                info!("Indy: console on Serial Port 1");
            }
        }
        if rmm::mips::FORCE_UNCACHED {
            info!("Indy: EXPERIMENT: kseg0 and all mappings uncached; this boot is slow");
        }

        // TLB cleared, exception vectors in kseg0, BEV off: from here a kernel
        // fault prints its registers instead of vanishing into the PROM.
        paging::init();
        info!("Indy: BEV off, exceptions come to the kernel");

        let ram = ram_bytes();
        RAM_SIZE.store(ram, core::sync::atomic::Ordering::Relaxed);
        let kernel_base = consts::RAM_PHYS_BASE + consts::LOW_RESERVED_BYTES;
        let kernel_end_virt = &raw const __end as usize;
        let kernel_size = kernel_end_virt - crate::kernel_executable_offsets::KERNEL_OFFSET();
        info!(
            "Indy: {} MiB RAM at {:#x}, kernel {:#x}+{:#x}, bss {:#x}..{:#x}",
            ram >> 20,
            consts::RAM_PHYS_BASE,
            kernel_base,
            kernel_size,
            &raw const __bss_start as usize,
            &raw const __bss_end as usize,
        );

        // An initfs appended by `ecoff convert --append` sits on the first page
        // boundary after the image: a header page then the payload.
        let (bootstrap_base, bootstrap_size) = {
            let header = ((kernel_end_virt + 4095) & !4095) as *const u8;
            let magic = core::slice::from_raw_parts(header, 8);
            if magic == b"REDOXIFS" {
                let len = u32::from_be_bytes([
                    header.add(8).read(),
                    header.add(9).read(),
                    header.add(10).read(),
                    header.add(11).read(),
                ]) as usize;
                let base = header as usize + 4096 - PHYS_OFFSET;
                (base, (len + 4095) & !4095)
            } else {
                (0, 0)
            }
        };
        info!("Indy: initfs {:#x}+{:#x}", bootstrap_base, bootstrap_size);

        let areas = &mut *AREAS.get();
        areas[0] = AreaEntry {
            base: consts::RAM_PHYS_BASE as u64,
            size: consts::LOW_RESERVED_BYTES as u64,
            kind: BootloaderMemoryKind::Reserved,
        };
        let mut area_count = 1;
        if device::arcs::present() {
            // Take what the PROM calls free, and the image it loaded. Its own
            // ranges stay out of the allocator so its console keeps working.
            let floor = kernel_base as u64;
            let ceiling = (consts::RAM_PHYS_BASE + ram) as u64;
            device::arcs::log_components();
            let mut descriptor = device::arcs::memory_descriptor(None);
            while let Some(d) = descriptor {
                let start = (d.base_page as u64) << 12;
                let end = start + ((d.page_count as u64) << 12);
                info!("Indy: PROM memory type {} at {:#x}..{:#x}", d.kind, start, end);
                let usable = matches!(
                    d.kind,
                    device::arcs::MEM_FREE
                        | device::arcs::MEM_FREE_CONTIGUOUS
                        | device::arcs::MEM_LOADED_PROGRAM
                );
                let (start, end) = (start.max(floor), end.min(ceiling));
                if usable && start < end && area_count < MAX_AREAS {
                    areas[area_count] = AreaEntry {
                        base: start,
                        size: end - start,
                        kind: BootloaderMemoryKind::Free,
                    };
                    area_count += 1;
                }
                descriptor = device::arcs::memory_descriptor(Some(d));
            }
        } else {
            areas[1] = AreaEntry {
                base: kernel_base as u64,
                size: (ram - consts::LOW_RESERVED_BYTES) as u64,
                kind: BootloaderMemoryKind::Free,
            };
            area_count = 2;
        }

        // The CPU module's secondary cache, where the PROM's inventory
        // (read above) lists one: the PROM leaves it off.
        crate::arch::cache::scache(true);

        let env_len = build_env();
        let stack_base = STACK.get() as usize - PHYS_OFFSET;
        let args = KernelArgs {
            kernel_base: kernel_base as u64,
            kernel_size: kernel_size as u64,
            stack_base: stack_base as u64,
            stack_size: STACK_SIZE as u64,
            env_base: (ENV.get() as usize - PHYS_OFFSET) as u64,
            env_size: env_len as u64,
            hwdesc_base: 0,
            hwdesc_size: 0,
            // Read directly, not through phys_to_virt: give the kseg0 address.
            areas_base: AREAS.get() as usize as u64,
            areas_size: (area_count * size_of::<AreaEntry>()) as u64,
            bootstrap_base: bootstrap_base as u64,
            bootstrap_size: bootstrap_size as u64,
        };
        let args = &args;

        graphical_debug::init(args.env());
        args.print();

        // Initialize RMM
        let mut bump_allocator = crate::startup::memory::init(args, None, None);
        info!("Indy: frame allocator ready");

        // The initfs in the boot image is copied into the first process and
        // then freed (memory::release_late_area).
        crate::memory::LATE_AREA
            .get()
            .write((bootstrap_base, bootstrap_size));
        crate::memory::init_mm(&mut bump_allocator);

        crate::arch::misc::init(crate::cpu_set::LogicalCpuId::new(0));
        info!(
            "Indy: percpu block at {:#x}",
            crate::arch::misc::arch_percpu() as usize
        );

        // Setup kernel heap (TLB-mapped kseg2: the first refills happen here)
        allocator::init();
        info!("Indy: kernel heap mapped at {:#x}", crate::kernel_heap_offset());

        // Activate memory logging
        crate::log::init();

        // Initialize devices
        device::init();

        // Initialize all of the non-core devices not otherwise needed to complete initialization
        device::init_noncore();

        // T5 accounting: what the machine gave and what boot took.
        info!(
            "Indy: ram {} KiB, low reserved {} KiB, kernel image {} KiB, free after boot {} KiB ({} frames)",
            ram >> 10,
            consts::LOW_RESERVED_BYTES >> 10,
            kernel_size >> 10,
            (crate::memory::free_frames() * crate::memory::PAGE_SIZE) >> 10,
            crate::memory::free_frames(),
        );

        if args.bootstrap_size == 0 {
            info!("Indy: no initfs in the image, stopping");
            loop {
                super::interrupt::disable();
                super::interrupt::halt();
            }
        }

        let bootstrap = args.bootstrap();

        crate::startup::kmain(
            bootstrap,
            Kstack::new().expect("failed to allocate stack"),
        );
    }
}

const PHYS_OFFSET: usize = <crate::arch::CurrentRmmArch as rmm::Arch>::PHYS_OFFSET;

static RAM_SIZE: core::sync::atomic::AtomicUsize = core::sync::atomic::AtomicUsize::new(0);

/// Bytes of RAM from `RAM_PHYS_BASE`, as sized at boot.
pub fn ram_size() -> usize {
    RAM_SIZE.load(core::sync::atomic::Ordering::Relaxed)
}
