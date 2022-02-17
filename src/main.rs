pub mod cpu;
pub mod addressing_mode;
pub mod opcode;

#[macro_use]
extern crate lazy_static;

use cpu::CPU;

fn main() {
    // Create a new CPU.
    let mut cpu = CPU::new();

    println!("Hello, world!");
}
