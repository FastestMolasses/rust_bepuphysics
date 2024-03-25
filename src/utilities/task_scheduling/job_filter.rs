/// Defines a trait for filtering jobs based on their tags.
pub trait JobFilter {
    /// Determines whether a job with the given tag should be allowed.
    ///
    /// # Arguments
    ///
    /// * `job_tag` - Tag of the candidate job.
    ///
    /// # Returns
    ///
    /// True if the job should be allowed to serve a request, false otherwise.
    fn allow_job(&self, job_tag: u64) -> bool;
}

/// A filter that allows all jobs.
pub struct AllowAllJobs;

impl JobFilter for AllowAllJobs {
    fn allow_job(&self, _job_tag: u64) -> bool {
        true
    }
}

/// A filter that wraps a Rust closure.
pub struct DelegateJobFilter<F: Fn(u64) -> bool> {
    pub filter: F,
}

impl<F: Fn(u64) -> bool> JobFilter for DelegateJobFilter<F> {
    fn allow_job(&self, job_tag: u64) -> bool {
        (self.filter)(job_tag)
    }
}

/// A filter that wraps a function pointer for high performance.
pub struct FunctionPointerJobFilter {
    pub filter: unsafe extern "C" fn(u64) -> bool,
}

impl JobFilter for FunctionPointerJobFilter {
    fn allow_job(&self, job_tag: u64) -> bool {
        unsafe { (self.filter)(job_tag) }
    }
}

/// A filter that requires the job tag to meet or exceed a threshold value.
pub struct MinimumTagFilter {
    pub minimum_tag_value: u64,
}

impl JobFilter for MinimumTagFilter {
    fn allow_job(&self, job_tag: u64) -> bool {
        job_tag >= self.minimum_tag_value
    }
}

/// A filter that requires the job tag to match a specific value.
pub struct EqualTagFilter {
    pub required_tag: u64,
}

impl JobFilter for EqualTagFilter {
    fn allow_job(&self, job_tag: u64) -> bool {
        job_tag == self.required_tag
    }
}
