//! Job filtering for the task scheduling system.
//!
//! Filters determine which jobs are allowed to serve pop requests from the task stack.

/// Determines which jobs are allowed to serve a pop request from the TaskStack.
pub trait IJobFilter {
    /// Determines whether a job with the given tag should be allowed to serve a pop request.
    fn allow_job(&self, job_tag: u64) -> bool;
}

/// A filter that will allow pops from any jobs.
#[derive(Default, Clone, Copy)]
pub struct AllowAllJobs;

impl IJobFilter for AllowAllJobs {
    #[inline(always)]
    fn allow_job(&self, _job_tag: u64) -> bool {
        true
    }
}

/// A job filter that wraps a closure.
pub struct DelegateJobFilter<F>
where
    F: Fn(u64) -> bool,
{
    /// Delegate to use as the filter.
    pub filter: F,
}

impl<F> DelegateJobFilter<F>
where
    F: Fn(u64) -> bool,
{
    /// Creates a job filter that wraps a closure.
    #[inline(always)]
    pub fn new(filter: F) -> Self {
        Self { filter }
    }
}

impl<F> IJobFilter for DelegateJobFilter<F>
where
    F: Fn(u64) -> bool,
{
    #[inline(always)]
    fn allow_job(&self, job_tag: u64) -> bool {
        (self.filter)(job_tag)
    }
}

/// A filter that wraps a function pointer for maximum performance.
#[derive(Clone, Copy)]
pub struct FunctionPointerJobFilter {
    /// Function pointer to use as the filter.
    pub filter: fn(u64) -> bool,
}

impl FunctionPointerJobFilter {
    /// Creates a job filter from a function pointer.
    #[inline(always)]
    pub fn new(filter: fn(u64) -> bool) -> Self {
        Self { filter }
    }
}

impl IJobFilter for FunctionPointerJobFilter {
    #[inline(always)]
    fn allow_job(&self, job_tag: u64) -> bool {
        (self.filter)(job_tag)
    }
}

/// A job filter that requires the job tag to meet or exceed a threshold value.
#[derive(Clone, Copy)]
pub struct MinimumTagFilter {
    /// Minimum tag value that must be met for a job to be allowed.
    pub minimum_tag_value: u64,
}

impl MinimumTagFilter {
    /// Creates a new minimum tag filter.
    #[inline(always)]
    pub fn new(minimum_tag_value: u64) -> Self {
        Self { minimum_tag_value }
    }
}

impl IJobFilter for MinimumTagFilter {
    #[inline(always)]
    fn allow_job(&self, job_tag: u64) -> bool {
        job_tag >= self.minimum_tag_value
    }
}

/// A job filter that requires the job tag to exactly match a specific value.
#[derive(Clone, Copy)]
pub struct EqualTagFilter {
    /// Tag value that must be matched for a job to be allowed.
    pub required_tag: u64,
}

impl EqualTagFilter {
    /// Creates a new exact tag filter.
    #[inline(always)]
    pub fn new(required_tag: u64) -> Self {
        Self { required_tag }
    }
}

impl IJobFilter for EqualTagFilter {
    #[inline(always)]
    fn allow_job(&self, job_tag: u64) -> bool {
        job_tag == self.required_tag
    }
}
