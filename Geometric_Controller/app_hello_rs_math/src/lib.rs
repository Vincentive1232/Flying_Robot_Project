#![no_std]
#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

use panic_halt as _;
use core::cell::UnsafeCell;

pub mod math;
pub mod stabilizer_types;
pub mod myController_lee;

use stabilizer_types::{control_t, sensorData_t, setpoint_t, state_t};
use myController_lee::controller_Lee;


// Define a global controller variable
static mut Controller: UnsafeCell<Option<controller_Lee>> = UnsafeCell::new(None);


extern "C" {
    pub fn vTaskDelay(ticks: u32);
    pub fn consolePutchar(ch: i32) -> i32;
}

fn console_print(msg: &str) {
    for c in msg.as_bytes() {
        unsafe{ consolePutchar(*c as i32); }
    }
}


#[no_mangle]
pub extern "C" fn appMain() -> i32 {
    console_print("Hello from Rust!\n");

    loop {
        unsafe { vTaskDelay(1000); }
    }
}

#[no_mangle]
pub extern "C" fn controllerOutOfTreeInit() {
    unsafe {
        let controller_ptr = Controller.get();
        *controller_ptr = Some(controller_Lee::controller_Lee_Init());
    }
}

#[no_mangle]
pub extern "C" fn controllerOutOfTreeTest() -> bool {
    return true
}


#[no_mangle]
pub unsafe extern "C" fn controllerOutOfTree (
    control: *mut control_t,
    setpoint: *const setpoint_t,
    sensors: *const sensorData_t,
    state: *const state_t,
    tick: u32
) {
    unsafe {
        if let Some(ref mut controller) = *Controller.get() {
            controller.controller_Lee(control, setpoint, sensors, state, tick);
        }
        
    }
}