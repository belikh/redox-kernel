//! Kernel stack traces. n32 code from LLVM keeps a frame pointer in `$30`
//! with `frame-pointer = "always"`, but the saved return address offset is not
//! fixed, so no frame walk is attempted yet: a panic prints the registers only.

pub struct StackTrace {
    pub fp: usize,
    pub pc_ptr: *const usize,
}

impl StackTrace {
    #[inline(always)]
    pub unsafe fn start() -> Option<Self> {
        None
    }

    pub unsafe fn next(self) -> Option<Self> {
        None
    }
}
