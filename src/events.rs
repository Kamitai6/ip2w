use core::{
    cell::{Cell, RefCell, UnsafeCell},
    f32::consts::PI,
    iter::once,
    sync::atomic::{AtomicBool, AtomicI32, AtomicU32, Ordering},
};
use critical_section::{Mutex, with};
use heapless::spsc::Queue;

static EVENT_QUEUE: Mutex<RefCell<Queue<Event, 16>>> = Mutex::new(RefCell::new(Queue::new()));
static EVENT_PENDING: AtomicBool = AtomicBool::new(false);

#[derive(Debug, Clone, Copy)]
pub enum Event {
    DisplayUpdate,
    MotionUpdate,
}

// イベントを発行（割り込みから呼び出し）
pub fn post_event(event: Event) {
    with(|cs| {
        let mut queue_ref = EVENT_QUEUE.borrow(cs).borrow_mut();
        if queue_ref.enqueue(event).is_ok() {
            EVENT_PENDING.store(true, Ordering::SeqCst);
        }
    });
}

// イベントを取得（メインループから呼び出し）
pub fn get_event() -> Option<Event> {
    with(|cs| {
        let mut queue_ref = EVENT_QUEUE.borrow(cs).borrow_mut();
        let event = queue_ref.dequeue();
        if queue_ref.is_empty() {
            EVENT_PENDING.store(false, Ordering::SeqCst);
        }
        event
    })
}

// イベントが待機中かチェック
pub fn has_pending_events() -> bool {
    EVENT_PENDING.load(Ordering::SeqCst)
}
