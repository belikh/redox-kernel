pub use super::handler::enter_usermode;

/// Nothing to program: `syscall` always traps to the general exception vector.
pub unsafe fn init() {}
