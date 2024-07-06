#![no_std]

extern crate alloc;

use alloc::vec;
use alloc::vec::Vec;
use core::alloc::{GlobalAlloc, Layout};

pub struct BumpAllocator {
    mem: Vec<u8>,
    index: usize,
}

impl BumpAllocator {
    pub fn with_capacity(cap: usize) -> Self {
        Self {
            mem: vec![0; cap],
            index: 0,
        }
    }

    pub fn alloc(&mut self, size: usize) -> Option<&mut [u8]> {
        if self.index + size > self.mem.len() {
            return None;
        }

        let old_index = self.index;
        self.index += size;

        Some(&mut self.mem[old_index..self.index])
    }

    pub fn dealloc(&mut self) {
        self.index = 0;
    }
}

unsafe impl GlobalAlloc for BumpAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        todo!()
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        todo!()
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        todo!()
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        todo!()
    }
}