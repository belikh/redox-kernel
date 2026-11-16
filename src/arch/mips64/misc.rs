//! Per-CPU block. The R4400 has no thread-pointer register in kernel mode and
//! the Indy has one CPU, so the block is reached through a static pointer the
//! exception entry also reads.

use core::cell::SyncUnsafeCell;

use crate::{
    cpu_set::LogicalCpuId,
    memory::{RmmA, RmmArch},
    percpu::PercpuBlock,
};

#[repr(C)]
pub struct ArchPercpu {
    // These fields must be kept first and in this order. Assembly in exception.rs depends on it.
    pub tmp: usize,
    /// Kernel stack pointer to load when an exception arrives from user mode:
    /// the top of the current context's kernel stack.
    pub kernel_sp: usize,

    pub percpu: PercpuBlock,
}

/// Address of the single `ArchPercpu`; 0 until `init`.
#[unsafe(no_mangle)]
pub static PERCPU_PTR: SyncUnsafeCell<usize> = SyncUnsafeCell::new(0);

pub fn arch_percpu() -> *mut ArchPercpu {
    unsafe { PERCPU_PTR.get().read_volatile() as *mut ArchPercpu }
}

#[cfg(not(test))]
impl PercpuBlock {
    pub fn current() -> &'static Self {
        let ptr = arch_percpu();
        assert!(!ptr.is_null(), "percpu used before misc::init");
        unsafe { &(*ptr).percpu }
    }
}

#[cold]
pub unsafe fn init(cpu_id: LogicalCpuId) {
    unsafe {
        const {
            assert!(size_of::<ArchPercpu>() <= crate::memory::PAGE_SIZE);
        }
        let must_be_zero = true;
        let frame =
            crate::memory::allocate_frame(must_be_zero).expect("failed to allocate percpu memory");
        let virt = RmmA::phys_to_virt(frame.base()).data() as *mut ArchPercpu;

        virt.write(ArchPercpu {
            tmp: 0,
            kernel_sp: 0,
            percpu: PercpuBlock::init(cpu_id),
        });

        PERCPU_PTR.get().write_volatile(virt as usize);
    }
}
