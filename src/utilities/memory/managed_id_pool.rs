pub struct ManagedIdPool {
    next_index: usize,
    available_ids: Vec<usize>,
}

impl ManagedIdPool {
    pub fn new(initial_capacity: usize) -> Self {
        #[cfg(debug_assertions)]
        debug_assert!(initial_capacity > 0);
        ManagedIdPool {
            next_index: 0,
            available_ids: Vec::with_capacity(initial_capacity),
        }
    }

    #[inline]
    pub fn take(&mut self) -> usize {
        self.available_ids.pop().unwrap_or_else(|| {
            let id = self.next_index;
            self.next_index += 1;
            id
        })
    }

    #[inline]
    pub fn return_id(&mut self, id: usize) {
        self.available_ids.push(id);
    }

    pub fn clear(&mut self) {
        self.next_index = 0;
        self.available_ids.clear();
    }

    pub fn ensure_capacity(&mut self, count: usize) {
        self.available_ids.reserve(count.saturating_sub(self.available_ids.capacity()));
    }

    pub fn compact(&mut self, minimum_count: usize) {
        if minimum_count < self.available_ids.capacity() {
            self.available_ids.shrink_to(minimum_count);
        }
    }

    // Getters
    pub fn highest_possibly_claimed_id(&self) -> isize {
        if self.next_index == 0 {
            -1
        } else {
            (self.next_index - 1) as isize
        }
    }

    pub fn available_id_count(&self) -> usize {
        self.available_ids.len()
    }
}
