use std::hint;
use std::thread;

/// Behaves like a framework SpinWait, but never voluntarily relinquishes the timeslice to off-core threads.
///
/// There are two big reasons for using this over the regular framework SpinWait:
///
/// 1) The framework spinwait relies on spins for quite a while before resorting to any form of timeslice surrender.
/// Empirically, this is not ideal for the solver — if the sync condition isn't met within several nanoseconds,
/// it will tend to be some microseconds away. This spinwait is much more aggressive about moving to yields.
///
/// 2) After a number of yields, the framework SpinWait will resort to calling Sleep.
/// This widens the potential set of schedulable threads to those not native to the current core.
/// If we permit that transition, it is likely to evict cached solver data.
pub(crate) struct LocalSpinWait {
    pub wait_count: i32,
}

impl LocalSpinWait {
    // Empirically, being pretty aggressive about yielding produces the best results. A single constraint
    // bundle can take hundreds of nanoseconds to finish. That would be a whole lot of spinning that could
    // be used by some other thread. At worst, we're being friendlier to other applications on the system.
    // This thread will likely be rescheduled on the same core, so it's unlikely that we'll lose any cache warmth.
    pub const YIELD_THRESHOLD: i32 = 3;

    #[inline(always)]
    pub fn new() -> Self {
        Self { wait_count: 0 }
    }

    #[inline(always)]
    pub fn spin_once(&mut self) {
        if self.wait_count >= Self::YIELD_THRESHOLD {
            thread::yield_now();
        } else {
            let spin_count = 1 << self.wait_count;
            for _ in 0..spin_count {
                hint::spin_loop();
            }
            self.wait_count += 1;
        }
    }
}
