use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

pub fn init_heap() {

     use core::mem::MaybeUninit;
    const HEAP_SIZE: usize = (75)*1024;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
    unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
}

pub fn used_mem()->usize { HEAP.used()}

