//! Memory management utilities for high-performance physics simulation.
//!
//! This module provides cache-friendly memory allocation and management primitives
//! designed for physics simulations that require minimal allocation overhead and
//! maximum cache locality.

pub mod allocator;
pub mod buffer;
pub mod buffer_pool;
pub mod managed_id_pool;
pub mod span_helper;
pub mod unmanaged_mempool;
pub mod worker_buffer_pools;

pub use allocator::Allocator;
pub use buffer::Buffer;
pub use buffer_pool::BufferPool;
pub use managed_id_pool::ManagedIdPool;
pub use unmanaged_mempool::UnmanagedMemoryPool;
pub use worker_buffer_pools::WorkerBufferPools;
