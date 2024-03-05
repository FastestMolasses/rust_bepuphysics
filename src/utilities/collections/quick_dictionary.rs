use crate::utilities::memory::buffer::Buffer;
use std::hash::Hash;

use super::equaility_comparer_ref::RefEqualityComparer;

pub struct HashHelper;

impl HashHelper {
    pub fn rehash(hash: i32) -> i32 {
        const A: u32 = 6;
        const B: u32 = 13;
        const C: u32 = 25;

        let uhash = (hash as u32) * 982451653u32;
        let redongled = ((uhash << A) | (uhash >> (32 - A)))
            ^ ((uhash << B) | (uhash >> (32 - B)))
            ^ ((uhash << C) | (uhash >> (32 - C)));

        redongled as i32
    }
}

pub struct QuickDictionary<TKey: 'static, TValue: 'static, TEqualityComparer>
where
    TEqualityComparer: RefEqualityComparer<TKey>,
{
    count: usize,
    table_mask: usize,
    table_power_offset: usize,
    table: Buffer<i32>, // We'll adjust the element type later
    keys: Buffer<TKey>,
    values: Buffer<TValue>,
    equality_comparer: TEqualityComparer,
}

impl<TKey, TValue, TEqualityComparer> QuickDictionary<TKey, TValue, TEqualityComparer>
where
    TKey: 'static + Unpin + Hash + Eq,
    TValue: 'static + Unpin,
    TEqualityComparer: RefEqualityComparer<TKey>,
{
    pub fn get(&self, index: usize) -> Option<(&TKey, &TValue)> {
        if index < self.count {
            Some((&self.keys[index], &self.values[index]))
        } else {
            None
        }
    }

    pub fn set(&mut self, index: usize, value: (TKey, TValue)) {
        if index < self.count {
            self.keys[index] = value.0;
            self.values[index] = value.1;
        }
    }

    // Constructors (To be expanded)
    pub fn new(
        initial_key_span: Buffer<TKey>,
        initial_value_span: Buffer<TValue>,
        initial_table_span: Buffer<i32>, // Placeholder type
        comparer: TEqualityComparer,
        table_power_offset: usize,
    ) -> Self {
        // ... (Span capacity and clearing validation similar to C#)

        QuickDictionary {
            count: 0,
            table_mask: initial_table_span.len() - 1,
            table_power_offset,
            table: initial_table_span,
            keys: initial_key_span,
            values: initial_value_span,
            equality_comparer: comparer,
        }
    }
}
