/// Defines a loop body function able to handle the equivalent of a foreach's body.
/// Takes a ref parameter for efficiency when dealing with large value types.
pub trait IForEachRef<T> {
    fn loop_body(&mut self, i: &mut T);
}

/// Defines a loop body function able to handle the equivalent of a foreach's body.
pub trait IForEach<T> {
    fn loop_body(&mut self, i: T);
}

pub trait IBreakableForEach<T> {
    /// Executes one execution of the loop body.
    ///
    /// # Arguments
    ///
    /// * `i` - Visited element.
    ///
    /// # Returns
    ///
    /// True if the loop should continue, false otherwise.
    fn loop_body(&mut self, i: T) -> bool;
}

pub trait IBreakableForEachRef<T> {
    /// Executes one execution of the loop body.
    ///
    /// # Arguments
    ///
    /// * `i` - Visited element.
    ///
    /// # Returns
    ///
    /// True if the loop should continue, false otherwise.
    fn loop_body(&mut self, i: &mut T) -> bool;
}
