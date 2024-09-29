pub trait IForEachRef<T> {
    fn loop_body(&mut self, i: &mut T);
}

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
