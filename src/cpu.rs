use crate::addressing_mode::AddressingMode;
use crate::opcode::OP_CODE_MAP;

// Q: Is it worth having a handler object per ops code?
// A: Probably not. The logic is too interwined with the CPU.

// Stack bottom
// 0x0100 = 0000 0001 0000 0000 = 256
// Use this for overflow checks.
const STACK_BOTTOM: u16 = 0x0100;

// Stack top
// 0x01ff = 0000 0001 1111 1111 = 511
// q: in the nes rust tutorial code, why is the stack reset set to 0xFD?
// a: read about it here: https://forums.nesdev.org/viewtopic.php?t=19858
// https://www.nesdev.org/wiki/CPU_power_up_state
// on startup, the stack pointer will init to 0. when cpu follows reset instructions, it treats it like an interrupt.
// it tries to push pc and p onto the stack
// special logic makes it perform reads rather than writes
// the stack pointer is decremented by 3 due to this access

// STACK_TOP is the top of the stack
const STACK_TOP: u16 = 0x01FF;

// STACK_REAL_TOP is the effective top of the stack.
// Use this for underflow checks.
const STACK_REAL_TOP: u16 = 0x01FD;

// STACK_POINTER_RESET is the actual reset location for the stack pointer
// It is 0xFF - 2 
const STACK_POINTER_RESET: u8 = 0xFD;

// TODO: Figure out how to implement the stack
// CPU emulates a 6502 CPU.
pub struct CPU {
	// -------- Registers --------
	// Small
	// Fast access

	// PC - Program counter
	// Holds address for next machine language instruction.
	pub pc: u16,

	// S - Stack Pointer
	// Pointer to hold the address at the top of the stack.
	pub s: u8,

	// A - Accumulator
	// Store result of arithmetic, logic, and memory access operations.
	pub a: u8,

	// X - Index register X
	// It is the offset in specific memory addressing modes.
	pub x: u8,

	// Y - Index register Y
	// Similar to X.
	pub y: u8,

	/* P - Processor Status
	 * 8 bit register represents 7 status flags.
	 * Each one is toggled depending on operation.
	 * 
	 * 7 N Negative - It really just tells you if the 7th bit is toggled. Its up to the next instruction to determine how to interpret this flag.
	 * 6 V Overflow
	 * 5 - (Expansion) (Break 2)
	 * 4 B Break Command
	 * 3 D Decimal
	 * 2 I Interrupt Disable
	 * 1 Z Zero
	 * 0 C Carry
	 */
	pub p: u8,
	
	// -------- Memory --------
	// Slow
	// Larger
	pub mem: [u8; 0xFFFF],
}

impl CPU {
	// new creates a new CPU struct, default every register and memory to zero.
	pub fn new() -> Self {
		CPU {
			pc: 0,
			s: 0,
			a: 0,
			x: 0,
			y: 0,
			p: 0,
			mem: [0; 0xFFFF],
		}
	}

	// reset will restore all of the registers and memory to default states.
	// When a new cartrigee is inserted, the CPU recieves a special signal to:
	// - Reset the state of all registers and flags.
	// - Set the pc to the 16 bit address stored at 0xFFFC.
	// 
	// reset does not change what is stored in memory.
	pub fn reset(&mut self) {
		// Set A register to 0.
		self.a = 0;

		// Set X register to 0.
		self.x = 0;

		// Set Y register to 0.
		self.y = 0;

		// Set Processor Status register to 0.
		self.p = 0;

		// Set Stack Pointer to 0xfd.
		// 0x01ff, 0x00ff, 0x00fe are handled as special cases, and the stack pointer is decremented
		// for more information, please look above for the stack comment
		self.s = STACK_POINTER_RESET;

		// Set PC to the value stored at 0xFFFC.
		self.pc = self.mem_read_u16(0xFFFC);
	}

	// load will load the program's ROM into the designated memory space.
	pub fn load_and_run(&mut self, program: Vec<u8>) {
		self.load(program);
		self.reset();
		self.run();
	}

	// load copies the program into memory space, and resets the pc.
	pub fn load(&mut self, program: Vec<u8>) {
		// Copy the program bytes into the memory space.
		self.mem[0x8000 .. (0x8000 + program.len())].copy_from_slice(&program[..]);

		// Set the address 0xFFFC to the beginning of the program.
		// 0xFFFC is where the NES expects the address to be.
		self.mem_write_u16(0xFFFC, 0x8000);
	}

	// mem_read will read 2 bytes of whats at a memory position.
	fn mem_read(&self, addr: u16) -> u8 {
		self.mem[addr as usize]
	}

	// mem_write will write 2 bytes to a memory position.
	fn mem_write(&mut self, addr: u16, data: u8) {
		self.mem[addr as usize] = data;
	}

	// TODO: Figure out what happens if mem_read_u16 is reading from the last address as the first position.
	// mem_read_u16 will read 4 bytes of whats at 2 memory positions.
	// This function assumes data is stored in little endian.
	fn mem_read_u16(&self, addr: u16) -> u16 {
		let low = self.mem_read(addr) as u16;
		let high = self.mem_read(addr + 1) as u16;

		// q: I don't quite understand this. why?
		(high << 8) | (low as u16)
	}

	// mem_write_u16 will write 4 bytes to 2 memory positions.
	// This function assumes data is stored in little endian.
	fn mem_write_u16(&mut self, addr: u16, value: u16) {
		let high = (value >> 8) as u8;
		let low = (value & 0xff) as u8;
		self.mem_write(addr, low);
		self.mem_write(addr + 1, high);
	}

	// stack_push will push a u8 onto the stack
	fn stack_push(&mut self, value: u8) {
		// Check that the stack is not overflowing or underflowing.
		// Underflowing would be a very odd case that should never occur.
		self.check_stack_overflow(1);

		// set the current (stack pointer location + stack bottom) location to the value
		let addr: u16 = self.stack_current_address();

		self.mem_write(addr, value);

		// decrement the stack pointer
		self.s = self.s.wrapping_sub(1);
	}

	// stack_push_u16 will push a u16 onto the stack
	fn stack_push_u16(&mut self, value: u16) {
		// need to check that we are not overflowing
		// each stack push action will check this

		// get the high value of the u16
		// use a right shift to shift the top 8 bits to the bottom 
		let high: u8 = (value >> 8) as u8;

		// get the low value of the u16
		// use a and with 0xff to get the first 8 bits and set the high 8 bits to 0
		let low: u8 = (value & 0xff) as u8;

		// push the high value onto the stack
		self.stack_push(high);
		
		// push the low value onto the stack
		self.stack_push(low);
	}

	// stack_pop will pop a u8 off of the stack
	fn stack_pop(&mut self) -> u8 {
		// Check that the stack is not underflowing or overflowing.
		// Overflowing would be a very odd case that should never occur.
		// Based off of the implementation, the lowest value that can be fetched
		// from stack_current_address is 0x0100, which is still a valid stack address.
		// In an real world implementation where performance/speed counts, we may
		// not want to check for an overflow. 
		self.check_stack_underflow(1);

		// increment the stack pointer
		self.s = self.s.wrapping_add(1);

		// Get the address to read off of
		let addr: u16 = self.stack_current_address();

		// return the value
		return self.mem_read(addr);
	}

	// stack_pop_u16 will pop a u16 off of the stack
	fn stack_pop_u16(&mut self) -> u16 {
		// need to check that we are not underflowing
		// each stack pop action will check this

		// pop the value, this is the low value
		let low: u8 = self.stack_pop();

		// pop the value, this is the high value
		let high: u8 = self.stack_pop();

		// return the number
		// left shift high bits to higher 8 bits
		// low will just stay low
		return (high as u16) << 8 | (low as u16);
	}

	// stack_current_address will return the current location of the
	// top of the stack. 
	fn stack_current_address(&self) -> u16 {
		return STACK_BOTTOM.wrapping_add(self.s as u16);
	}

	// https://chubakbidpaa.com/retro/2020/12/15/6502-stack-copy.html
	// go to the "Overflow and Underflow" section for more info 

	// check_stack_overflow checks if the stack is overflowing.
	// The stack is overflowing when there is no more space to push onto the stack.
	fn check_stack_overflow(&self, required_change: u8) {
		// If the stack pointer indicates it will become less than 0,
		// then the stack is overflowing.
		if self.s < required_change {
			panic!("stack overflow");
		}
	}

	// check_stack_underflow checks if the stack is underflowing.
	// The stack is underflowing when there is not enough elements to pop.
	fn check_stack_underflow(&self, required_change: u16) {
		// If the stack pointer has less space from the top then the required change,
		// then the stack is underflowing.
		if (STACK_REAL_TOP - self.stack_current_address() as u16) < required_change {
			panic!("stack underflow");
		}
	}

	// get_operand_address determines how an address should be read.
	// It returns an address for the next step to read off of.
	// It is determined based off of the Addressing mode.
	// TODO: Implement this in another MR.
	fn get_operand_address(&self, mode: &AddressingMode) -> u16 {
 		// Check what mode is passed in.
 		match mode {
			// Immediate
			AddressingMode::Immediate => {
				// Return the current value of the pc.
				self.pc
			}

			// Zero Page
			AddressingMode::ZeroPage => {
				// Read the value stored on 1 address.
				// The value of the address is the value of the pc.
				self.mem_read(self.pc) as u16
			}

			// Absolute
			AddressingMode::Absolute => {
				// Read the value stored on 2 adjacent addresses.
				// The value of the first address is the value of the pc.
				self.mem_read_u16(self.pc) 
			}

			// Zero Page X
			AddressingMode::ZeroPageX => {
				// Read 1 value stored on 1 address and add value of x to it.
				// The value of the address is the value of the pc.
				// These are added together. If the value overflows the available byte space, it will restart from 0.
				let pos = self.mem_read(self.pc);
				let addr = pos.wrapping_add(self.x) as u16;
				addr
			}
				
			// Zero Page Y
			AddressingMode::ZeroPageY => {
				// Read 1 value stored on 1 address and add value of y to it.
				// The value of the address is the value of the pc.
				// These are added together. If the value overflows the available byte space, it will restart from 0.
				let pos = self.mem_read(self.pc);
				let addr = pos.wrapping_add(self.y) as u16;
				addr
			}

			// Absolute X
			AddressingMode::AbsoluteX => {
				// Read the value stored on 2 adjacent address and add value of x to it.
				// The value of the first address is the value of the pc.
				// These are added together. If the value overflows the available byte space, it will restart from 0.
				let base = self.mem_read_u16(self.pc);
				let addr = base.wrapping_add(self.x as u16);
				addr
			}

			// Absolute Y
			AddressingMode::AbsoluteY => {
				// Read the value stored on 2 adjacent address and add value of y to it.
				// The value of the first address is the value of the pc.
				// These are added together. If the value overflows the available byte space, it will restart from 0.
				let base = self.mem_read_u16(self.pc);
				let addr = base.wrapping_add(self.y as u16);
				addr
			}

			// Indirect X
			AddressingMode::IndirectX => {
				// Read the value stored on 1 address.
				// The value of the address is the value of the pc.
				let base = self.mem_read(self.pc);
				
				// Add x to the value. If the value overflows the available byte space, it will restart from 0. This will be the pointer.
				let ptr: u8 = (base as u8).wrapping_add(self.x);

				// Read the low value stored on the pointer.
				let low = self.mem_read(ptr as u16);

				// Read the high value stored on the pointer.
				let high = self.mem_read(ptr.wrapping_add(1) as u16);

				// Put the high value on the lower end, and low value on the higher end.
				// This is a little endian.
				// Return this computation.
				((high as u16) << 8) | (low as u16)
			}

			// Indirect Y
			AddressingMode::IndirectY => {
				// Read the value stored on 1 address.
				// The value of the address is the value of the pc.
				let base = self.mem_read(self.pc);
				
				// Add y to the value. If the value overflows the available byte space, it will restart from 0. This will be the pointer.
				let ptr = (base as u8).wrapping_add(self.y);

				// Read the low value stored on the pointer.
				let low = self.mem_read(ptr as u16);

				// Read the high value stored on the pointer.
				let high = self.mem_read(ptr.wrapping_add(1) as u16);

				// Put the high value on the lower end, and low value on the higher end.
				// This is a little endian.
				// Return this computation.
				((high as u16) << 8) | (low as u16)
			}

			// None Addressing
			AddressingMode::NoneAddressing => {
				// Not supported.
				panic!("not supported");
			}
		}
	}

	/*
	 * interpret interprets the incoming instructions.
	 * The basic loop is:
	 * - fetch instruction from instruction memory
	 * - decode the instruction
	 * - execute the instruction
	 * - go back to first step
	 * 
	 * We need a mutable reference to the self, since the
	 * passed in value will change a register.
	 */
	pub fn run(&mut self) {
		// Start the loop.
		loop {
			// Get the ops code from the pc.
			// q: why convert from u16 to usize?
			let opscode = self.mem_read(self.pc);

			// Increment pc.
			self.pc += 1;

			// Execute based off of the ops code.
			let code_info = OP_CODE_MAP.get(&opscode).expect(&format!("OpCode {:x} is not recognized", opscode));

			// TODO: Organize these alphabetically and in categories.
			match code_info.code {
				// Handle ops code LDA.
				0xA9 | 0xA5 | 0xAD | 0xB5 | 0xBD | 0xB9 | 0xA1 | 0xB1 => {
					self.lda(&code_info.mode);
				}

				// Handle ops code LDX.
				0xA2 | 0xA6 | 0xB6 | 0xAE | 0xBE => {
					self.ldx(&code_info.mode);
				}

				// Handle ops code LDY.
				0xA0 | 0xA4 | 0xB4 | 0xAC | 0xBC => {
					self.ldy(&code_info.mode);
				}

				// Handle ops code TAX (0xAA)
				0xAA => self.tax(),

				// Handle ops code TAY (0xA8)
				0xA8 => self.tay(),

				// Handle ops code TXA (0x8A)
				0x8A => self.txa(),

				// Handle ops code TYA (0x98)
				0x98 => self.tya(),

				// Handle ops code DEX (0xca)
				0xCA => self.dex(),

				// Handle ops code DEY (0x88)
				0x88 => self.dey(),

				// Handle ops code INX (0xE8)
				0xE8 => self.inx(),

				// Handle ops code INY (0xC8)
				0xC8 => self.iny(),

				// Handle ops code CLC (0x18)
				0x18 => self.clc(),

				// Handle ops code CLD (0xD8)
				0xD8 => self.cld(),

				// Handle ops code CLV (0xB8)
				0xB8 => self.clv(),

				// Handle ops code SEC (0x38)
				0x38 => self.sec(),

				// Handle ops code SED (0xF8)
				0xF8 => self.sed(),

				// Handle ops code STA
				0x85 | 0x8D | 0x95 | 0x9D | 0x99 => self.sta(&code_info.mode),

				// Handle ops code STX
				0x86 | 0x96 | 0x8E => self.stx(&code_info.mode),
				
				// Handle ops code STY
				0x84 | 0x94 | 0x8C => self.sty(&code_info.mode),

				// Handle ops code TXS
				0x9A => self.txs(),

				// Handle ops code TSX (0xBA)
				0xBA => self.tsx(),

				// Handle ops code PHA (0x48)
				0x48 => self.pha(),

				// Handle ops code PLA (0x68)
				0x68 => self.pla(),

				// Handle ops code PHP (0x08)
				0x08 => self.php(),

				// Handle ops code PLP (0x28)
				0x28 => self.plp(),

				// Handle ops code NOP (0xEA)
				0xEA => {
					// do nothing
				},

				// Handle ops code BRK (0x00).
				// BRK is the break command. It causes an
				// interrupt sequence. The program transfers control to the 
				// interrupt vector.
				0x00 => {
					return;
				}

				// No match was found. 
				// This should be a panic. This should never happen.
				_ => {
					panic!("no ops code match was found");
				}
			}

			// Increment the pc to pc + (opcode.len - 1)
			self.pc += ((code_info.len - 1) as u16);
		}
	}

	// -------- Handle Opscodes --------
	// TODO: organize these alphabetically

	// lda handles ops code LDA.
	// LDA is Load Accumulator.
	fn lda(&mut self, mode: &AddressingMode) {
		// Get the address of where the value is stored.
		let addr = self.get_operand_address(mode);

		// Get the value from the address indicated by the addressing mode.
		let value = self.mem_read(addr);
		
		// Fill the A register with the param.
		self.a = value;

		// Change the Processor Status Flags based off of the new A value
		self.update_processor_flags(self.a);
	}

	// ldx handles ops code LDX.
	// LDX is Load X.
	fn ldx(&mut self, mode: &AddressingMode) {
		// Get the address of where the value is stored.
		let addr = self.get_operand_address(mode);

		// Get the value from the address indicated by the addressing mode.
		let value = self.mem_read(addr);
		
		// Fill the X register with the param.
		self.x = value;

		// Change the Processor Status Flags based off of the new X value
		self.update_processor_flags(self.x);
	}

	// ldy handles ops code LDY.
	// LDY is Load Y.
	fn ldy(&mut self, mode: &AddressingMode) {
		// Get the address of where the value is stored.
		let addr = self.get_operand_address(mode);

		// Get the value from the address indicated by the addressing mode.
		let value = self.mem_read(addr);
		
		// Fill the Y register with the param.
		self.y = value;

		// Change the Processor Status Flags based off of the new Y value
		self.update_processor_flags(self.y);
	}

	// tax handles the ops code TAX (0xAA).
	// TAX copies the value from the A register to the X register.
	fn tax(&mut self) {
		// Copy the value from A register into the X register.
		self.x = self.a;

		// Change the Processor Status Flags based off of the new X value
		self.update_processor_flags(self.x);
	}

	// tay handles the ops code TAY (0xA8).
	// TAY copies the value from the A register to the Y register.
	fn tay(&mut self) {
		// Copy the value from A register into the Y register.
		self.y = self.a;

		// Change the Processor Status Flags based off of the new Y value
		self.update_processor_flags(self.y);
	}

	// txa handles the ops code TXA (0x8A).
	// TXA copies the value from the X register to the A register.
	fn txa(&mut self) {
		// Copy the value from X register into the A register.
		self.a = self.x;

		// Change the Processor Status Flags based off of the new A value
		self.update_processor_flags(self.a);
	}

	// tya handles the ops code TYA (0x98).
	// TYA copies the value from the Y register to the A register.
	fn tya(&mut self) {
		// Copy the value from Y register into the A register.
		self.a = self.y;

		// Change the Processor Status Flags based off of the new A value
		self.update_processor_flags(self.a);
	}

	// dex handles the ops code DEX (0xCA).
	// DEX decrements the X register value by 1.
	fn dex(&mut self) {
		// Decrement X by 1.
		self.x = self.x.wrapping_sub(1);

		// Change the Processor Status Flags based off of the new X value
		self.update_processor_flags(self.x);
	}

	// dey handles the ops code DEY (0x88).
	// DEY decrements the Y register value by 1.
	fn dey(&mut self) {
		// Decrement Y by 1.
		self.y = self.y.wrapping_sub(1);

		// Change the Processor Status Flags based off of the new Y value
		self.update_processor_flags(self.y);
	}

	// inx handles the ops code INX (0xE8).
	// INX increments the X register value by 1.
	fn inx(&mut self) {
		// Increment X by 1.
		self.x = self.x.wrapping_add(1);

		// Change the Processor Status Flags based off of the new X value
		self.update_processor_flags(self.x);
	}

	// iny handles the ops code INY (0xE8).
	// INY increments the Y register value by 1.
	fn iny(&mut self) {
		// Increment Y by 1.
		self.y = self.y.wrapping_add(1);

		// Change the Processor Status Flags based off of the new Y value
		self.update_processor_flags(self.y);
	}

	// clc handles the ops code CLC (0x18).
	// clc clears the carry bit. It sets it to 0.
	fn clc(&mut self) {
		self.p = self.p & 0b1111_1110;
	}

	// cld handles the ops code CLD (0xD8).
	// cld clears the decimal bit. It sets it to 0.
	fn cld(&mut self) {
		self.p = self.p & 0b1111_0111;
	}

	// clv handles the ops code CLV (0xB8).
	// clv clears the overflow bit. It sets it to 0.
	fn clv(&mut self) {
		self.p = self.p & 0b1011_1111;
	}

	// sec handles the ops code SEC (0x38).
	// sec sets the carry bit. It sets it to 1.
	fn sec(&mut self) {
		self.p = self.p | 0b0000_0001;
	}

	// sed handles the ops code SED (0xF8).
	// sec sets the decimal bit. It sets it to 1.
	fn sed(&mut self) {
		self.p = self.p | 0b0000_1000;
	}

	// sta handles the ops code STA.
	// sta stores the A register value into memory.
	fn sta(&mut self, mode: &AddressingMode) {
		let addr = self.get_operand_address(mode);
		self.mem_write(addr, self.a);
	}

	// stx handles the ops code STX.
	// stx stores the X register value into memory.
	fn stx(&mut self, mode: &AddressingMode) {
		let addr = self.get_operand_address(mode);
		self.mem_write(addr, self.x);
	}
	
	// sty handles the ops code STY.
	// sty stores the Y register value into memory.
	fn sty(&mut self, mode: &AddressingMode) {
		let addr = self.get_operand_address(mode);
		self.mem_write(addr, self.y);
	}

	// txs handles the ops code TXS.
	// txs transfers the contents of the X register to the stack pointer
	fn txs(&mut self) {
		// set the stack pointer to the value of the X register
		self.s = self.x;
	}

	// tsx transfers the contents of the stack pointer to the X register
	fn tsx(&mut self) {
		// set the X register to the value of the stack pointer
		self.x = self.s;

		// Change the Processor Status Flags based off of the new X value
		self.update_processor_flags(self.x);
	}

	// pha pushes the A register value onto the stack
	fn pha(&mut self) {
		self.stack_push(self.a);
	}

	// pla pulls (pop) the value from the Stack into the A register
	fn pla(&mut self) {
		// pop the value off the stack onto the A register
		self.a = self.stack_pop();

		// Change the Processor Status Flags based off of the new A value
		self.update_processor_flags(self.a);
	}

	// php pushes the Processor Status value onto the stack, with a modified value
	fn php(&mut self) {
		// Create a clone of the processor status
		let mut status = self.p.clone();

		// Set the Break 1 processor status		
		// Set the Break 2 processor status
		status = status | 0b0011_0000;

		// Push the processor status onto the stack
		self.stack_push(status);
	}

	// plp pulls the Procesor Status value off of the stack, with a modified value
	fn plp(&mut self) {
		// Pull the bits off of the stack onto the Processor status
		self.p = self.stack_pop();

		// Unset the Break 1 processor status		
		self.p = self.p & 0b1110_1111;

		// Set the Break 2 processor status
		self.p = self.p | 0b0010_0000;
	}

	// update_processor_flags change the Processor Status Flags based off of the new A values
	fn update_processor_flags(&mut self, result: u8) {
		// Check if the A register is 0.
		if result == 0 {
			// If 0, set the zero flag to 1.
			self.p = self.p | 0b0000_0010;
		} else {
			// If not, set the zero flag to 0.
			self.p = self.p & 0b1111_1101;
		}

		// Check if the A register is less than 0.
		// It checks if the 7 bit of the a register value is set. If it's set, it's a negative number.
		if result & 0b1000_0000 != 0 {
			// If < 0, set the negative flag to 1.
			self.p = self.p | 0b1000_0000;
		} else {
			// If >= 0, set the negative flag to 0.
			self.p = self.p & 0b0111_1111;
		}
	}
}

// --------- Tests ---------

/* Typically, unit tests are listed below the actual code in the file.
 * This is how rust works. It doesn't work if I put the test in a separate file.
 */

#[cfg(test)]
mod test {
    use super::*;

    // -------- Internal Functions --------

    #[test]
    fn test_reset() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

    	// Call reset on the CPU.
    	cpu.reset();

    	// Check that the PC is set to 0.
    	// In a real execution, self.pc should be 0x8000, but this is an isolated unit test. 
    	assert_eq!(cpu.pc, 0);

    	// Check that the other registers are set to 0.
    	assert_eq!(cpu.a, 0);
    	assert_eq!(cpu.x, 0);
    	assert_eq!(cpu.y, 0);
    	assert_eq!(cpu.p, 0);
    	assert_eq!(cpu.s, 0xfd);
    }

    #[test]
    fn test_load_and_run_happy_path() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

    	// Load and run an empty program.
    	cpu.load_and_run(vec![0x00]);

    	// Check that the PC is set to 0x8000.
    	assert_eq!(cpu.pc, 0x8001);

    	// Check that the other registers are expected.
    	assert_eq!(cpu.a, 0);
    	assert_eq!(cpu.x, 0);
    	assert_eq!(cpu.y, 0);
    	assert_eq!(cpu.p, 0);
    	assert_eq!(cpu.s, 0xfd);
    }

    #[test]	
	#[should_panic(expected = "OpCode f1 is not recognized")]
    fn test_load_and_run_unknown_opcode() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

    	// Load and run a program that has an empty op code.
    	cpu.load_and_run(vec![0xf1]);
    }

    #[test]
    fn test_load_happy_path() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

    	// Create a fake program.
    	let program = vec![0x34, 0x54, 0x12, 0x96];

    	// Need a clone because the ownership of program will be moved into the load function.
    	let program_clone = program.clone();

    	// Load a fake program.
    	cpu.load(program);

    	// Check the contents of whats on memory.
    	for n in 0..3 {
    		assert_eq!(cpu.mem_read(0x8000 + n), program_clone[n as usize]);
    	}

    	// Check the value on the address 0xFCCC.
    	assert_eq!(cpu.mem_read_u16(0xFFFC), 0x8000 as u16);
    }

    #[test]
    fn test_mem_read_happy_path() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

    	// Put a fake program on a single space in memory.
    	cpu.mem[0x5000] = 0x86;

    	// Use mem_read to check the value in the single memory space.
    	assert_eq!(cpu.mem_read(0x5000), 0x86);
    } 

    #[test]
    fn test_mem_write_happy_path() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

    	// Use mem_write to write to a location in the memory space.
    	cpu.mem_write(0x3453, 0x42);

    	// Check the memory in the single memory space.
    	assert_eq!(cpu.mem[0x3453], 0x42)
    }

    #[test]
    fn test_mem_read_u16_happy_path() {
    	// Create a CPU
    	let mut cpu = CPU::new();

    	// Put something in 2 memory spaces.
    	cpu.mem[0x5000] = 0x12;
    	cpu.mem[0x5001] = 0x34;

    	// Use mem_read_u16 to check the value in the 2 memory spaces.
    	assert_eq!(cpu.mem_read_u16(0x5000), 0x3412);
    }

    #[test]
    fn test_mem_write_u16_happy_path() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

    	// Use mem_write_u16 to write to a location in the 2 memory spaces.
    	cpu.mem_write_u16(0x5000, 0x1234);

    	// Check the memory in 2 memory spaces.
    	assert_eq!(cpu.mem[0x5000], 0x34);
    	assert_eq!(cpu.mem[0x5001], 0x12);
	}

    // -------- Operand Addressing --------
    // TODO: For all the expected, use the exact value as the value of expected, rather than the formula.
    // Leave the equation used to get the number as a comment.
    // TODO: Put the actual number on top of each address.

   	#[test]
   	fn test_get_operand_address_immediate_happypath() {
   		// Create a CPU.
   		let mut cpu = CPU::new();

   		// Set the pc to some value.
   		cpu.pc = 0x4343;

   		// Check that the expected value is returned from get_operand_address.
   		assert_eq!(cpu.get_operand_address(&AddressingMode::Immediate), 0x4343);
   	}

	#[test]
	fn test_get_operand_address_zeropage_happypath() {
		// Create a CPU.
		let mut cpu = CPU::new();

		// Set the pc to some value.
		cpu.pc = 0x9043;

		// Set the memory address of the pc to some value.
		cpu.mem[cpu.pc as usize] = 0x34;

   		// Check that the expected value is returned from get_operand_address.
		assert_eq!(cpu.get_operand_address(&AddressingMode::ZeroPage), 0x34 as u16);
	}
			
	#[test]
	fn test_get_operand_address_absolute_happypath() {
		// Create a CPU.
		let mut cpu = CPU::new();

		// Set the pc to some value.
		cpu.pc = 0x5050;

		// Set the memory address of the pc to a value.
		cpu.mem[cpu.pc as usize] = 0x65;

		// Set the adjacent +1 memory address of the pc to a value.
		cpu.mem[(cpu.pc + 1) as usize] = 0x12;

		// Check that the expected value is returned from get_operand_address.
		assert_eq!(cpu.get_operand_address(&AddressingMode::Absolute), 0x1265);
	}

	// TODO: Check if a test needs to be added for absolute addressing if its reading from the last address.

	#[test]
	fn test_get_operand_address_zeropagex_happypath() {
		// Create a CPU.
		let mut cpu = CPU::new();

		// Set the pc to some value.
		cpu.pc = 0x3445;

		// Set the memory address of the pc to a value.
		cpu.mem[cpu.pc as usize] = 0x01;

		// Set the x value to some value, no overflow.
		// Can use a simple "+" as it won't overflow. 
		cpu.x = 0x42;

		// Check that the expected value is returned from get_operand_address.
		// let expected = (cpu.mem[cpu.pc as usize] + cpu.x) as u16;
		// 0x01 + 0x42 = 0x43 = 67
		// let expected = 0x43 as u16;
		let expected = 67 as u16;
		assert_eq!(cpu.get_operand_address(&AddressingMode::ZeroPageX), expected);
	}

	#[test]
	fn test_get_operand_address_zeropagex_overflow() {
		// Create a CPU.
		let mut cpu = CPU::new();

		// Set the pc to some value.
		cpu.pc = 0x4353;

		// Set the memory address of the pc to a value.
		cpu.mem[cpu.pc as usize] = 0xfd;

		// Set the x value to some value that will overflow the u16 bit space.
		cpu.x = 0xfe;

		// Check that the expected value is returned from get_operand_address.
		// Cannot use a simple "+" because it will overflow the space.
		// let expected = cpu.mem[cpu.pc as usize].wrapping_add(cpu.x) as u16;
		// 0xfd + 0xfe = 0xfb = 251
		// 253 + 254 = 507
		// 507 - 256 = 251
		// let expected = 0xfb as u16;
		let expected = 251 as u16;

		assert_eq!(cpu.get_operand_address(&AddressingMode::ZeroPageX), expected);
	}
			
	#[test]
	fn test_get_operand_address_zeropagey_happypath() {
		// Create a CPU.
		let mut cpu = CPU::new();

		// Set the pc to some value.
		cpu.pc = 0x4351;

		// Set the memory address of the pc to a value.
		cpu.mem[cpu.pc as usize] = 0x01;

		// Set the y value to some value, no overflow.
		cpu.y = 0x34;

		// Check that the expected value is returned from get_operand_address.
		// Can use a simple "+" as it won't overflow.
		// 0x01 + 0x34 = 0x35 = 53
		// let expected = 0x35;
		let expected = 53 as u16;
		assert_eq!(cpu.get_operand_address(&AddressingMode::ZeroPageY), expected);
	}

	#[test]
	fn test_get_operand_address_zeropagey_overflow() {
		// Create a CPU.
		let mut cpu = CPU::new();

		// Set the pc to some value.
		cpu.pc = 0x4351;

		// Set the memory address of the pc to a value.
		cpu.mem[cpu.pc as usize] = 0xef;

		// Set the y value to some value that will overflow the u16 bit space.
		cpu.y = 0xf4;

		// Check that the expected value is returned from get_operand_address.
		// Cannot use a simple "+" because it will overflow the space.
		// let expected = cpu.mem[cpu.pc as usize].wrapping_add(cpu.y) as u16;
		// 0xef + 0xf4 = 239 + 244 = 483
		// 483 - 256 = 227 = 0xe3
		// let expected = 0xe3 as u16;
		let expected = 227 as u16;
		assert_eq!(cpu.get_operand_address(&AddressingMode::ZeroPageY), expected);
	}

	#[test]
	fn test_get_operand_address_absolutex_happypath() {
		// Create a CPU.
		let mut cpu = CPU::new();

		// Set the pc to some value.
		cpu.pc = 0x4512;

		// Set the memory address of the pc to some value.
		cpu.mem[cpu.pc as usize] = 0x23;

		// Set the memory address of the pc + 1 to some value.
		cpu.mem[(cpu.pc + 1) as usize] = 0x76;

		// Set the x value to some value, no overflow.
		cpu.x = 0x32;

		// Check that the expected value is returned from get_operand_address.
		// Can use a simple "+" as it won't overflow.
		// let expected = cpu.mem_read_u16(cpu.pc) + (cpu.x as u16);
		// 0x7623 + 0x0032
		// 0111 0110 0010 0011 + 0000 0000 0011 0010
		// 0x7623 = (2 ^ 14 + 2 ^ 13 + 2 ^ 12) + (2 ^ 10 + 2 ^ 9) + (2 ^ 5) + (2 ^ 1 + 2 ^ 0) = 30243
		// 0x0032 = (0) + (0) + (2 ^ 5 + 2 ^ 4) + (2 ^ 1) = 50
		// 30243 + 50 = 30293 = 0x7655
		// let expected = 0x7655;
		let expected = 30293;
		assert_eq!(cpu.get_operand_address(&AddressingMode::AbsoluteX), expected);
	}

	#[test]
	fn test_get_operand_address_absolutex_overflow(){
		// Create a CPU.
		let mut cpu = CPU::new();

		// Set the pc to some value.
		cpu.pc = 0x65;

		// Set the memory address of the pc to some value.
		cpu.mem[cpu.pc as usize] = 0xff;

		// Set the memory address of the pc + 1 to some value.
		cpu.mem[(cpu.pc + 1) as usize] = 0xff;

		// Set the x value to some value that will overflow the u16 bit space.
		cpu.x = 0x8f;

		// Check that the expected value is returned from get_operand_address.
		// Cannot use a simple "+" because it will overflow the space.
		// let expected = cpu.mem_read_u16(cpu.pc).wrapping_add(cpu.x as u16);
		// 0xffff = 1111 1111 1111 1111 = 2 ^ 16 - 1
		// 0x8f = 1000 1111 = (2 ^ 7) + (2 ^ 3 + 2 ^ 2 + 2 ^ 1 + 2 ^ 0) = (128 + 15) = 143
		// 143 + (2 ^ 16 - 1) - (2 ^ 16) = 142 = 0x8e
		// 143 + (2 ^ 16 - 1) - (2 ^ 16) = 142
		// let expected = 0x8e;
		let expected = 142;
		assert_eq!(cpu.get_operand_address(&AddressingMode::AbsoluteX), expected);
	}
				
	#[test]
	fn test_get_operand_address_absolutey_happypath() {
		// Create a CPU.
		let mut cpu = CPU::new();

		// Set the pc to some value.
		cpu.pc = 0x34;

		// Set the memory address of the pc to some value.
		cpu.mem[cpu.pc as usize] = 0x87;

		// Set the memory address of the pc + 1 to some value.
		cpu.mem[(cpu.pc + 1) as usize] = 0x56;

		// Set the y valuee to some value that will not overflow the u16 bit space.
		cpu.y = 0x32;

		// Check that the expected value is returned from get_operand_address.
		// Can use a simple "+" as it won't overflow.
		// let expected = cpu.mem_read_u16(cpu.pc) + (cpu.y as u16);
		// let expected = 0x56b9;
		// 0x5687 = 0101 0110 1000 0111 = (2 ^ 14 + 2 ^ 12) + (2 ^ 10 + 2 ^ 9) + (2 ^ 7) + (2 ^ 2 + 2 ^ 1 + 2 ^ 0) = 22151
		// 0x32 = 0011 0010 = (2 ^ 5 + 2 ^ 4) + (2 ^ 1) = 50
		// 0x5687 + 0x32 = 22151 + 50 = 22201 = 0x56b9
		let expected = 22201;
		assert_eq!(cpu.get_operand_address(&AddressingMode::AbsoluteY), expected);
	}

	#[test]
	fn test_get_operand_address_absolutey_overflow() {
		// Create a CPU.
		let mut cpu = CPU::new();

		// Set the pc to some value.
		cpu.pc = 0x34;

		// Set the memory address of the pc to some value.
		cpu.mem[cpu.pc as usize] = 0x97;

		// Set the memory address of the pc + 1 to some value.
		cpu.mem[(cpu.pc + 1) as usize] = 0xff;

		// Set the y value to some value that will overflow the u16 bit space.
		cpu.y = 0xff;

		// Check that the expected value is returned from get_operand_address.
		// Cannot use a simple "+" as it will overflow the space.
		// let expected = cpu.mem_read_u16(cpu.pc).wrapping_add(cpu.y as u16);
		// 0xff97 = 1111 1111 1001 0111 = (2 ^ 15 + 2 ^ 14 + 2 ^ 13 + 2 ^ 12) + (2 ^ 11 + 2 ^ 10 + 2 ^ 9 + 2 ^ 8) + (2 ^ 7 + 2 ^ 4) + (2 ^ 2 + 2 ^ 1 + 2 ^ 0) = 65431
		// 0xff = 1111 1111 = (2 ^ 8) - 1 = 256 - 1 = 255
		// 65431 + 255 = 65686
		// 65686 - 65536 = 150 = 2 ^ 7 + 2 ^ 4 + 2 ^ 2 + 2 ^ 2 = 1001 0110 = 0x96
		// let expected = 0x96;
		let expected = 150;
		assert_eq!(cpu.get_operand_address(&AddressingMode::AbsoluteY), expected);
	}
				
	#[test]
	fn test_get_operand_address_indirectx_happypath() {
		// Create a CPU.
		let mut cpu = CPU::new();

		// Set the pc to some value.
		cpu.pc = 0x54;

		// Set the address of the pc to some value.
		cpu.mem[cpu.pc as usize] = 0x43;

		// Set the x value to some value that will not overflow the u16 bit space.
		cpu.x = 0x01;

		// Get the pointer.
		// Add x to the value stored on the pc's address.
		// let ptr = cpu.mem[cpu.pc as usize].wrapping_add(cpu.x);
		// 0x43 + 0x01 = 0x44
		let ptr = 0x44 as u8;

		// Set the address of the pc's address value to some value.
		cpu.mem[ptr as usize] = 0x23;

		// Set the address of the pc + 1 address value to some value.
		cpu.mem[(ptr.wrapping_add(1)) as usize] = 0x76;

		// Check that the expected value is returned from get_operand_address.
		// Can use a simple "+" as it won't overflow
		let expected = 0x7623;
		assert_eq!(cpu.get_operand_address(&AddressingMode::IndirectX), expected);
	}

	#[test]
	fn test_get_operand_address_indirectx_overflow() {
		// Create a CPU.
		let mut cpu = CPU::new();

		// Set the pc to some value.
		cpu.pc = 0x45;

		// Set the address of the pc to some value.
		cpu.mem[cpu.pc as usize] = 0xff;

		// Set the x value to some value that will overflow the u16 bit space.
		cpu.x = 0x43;

		// Get the pointer.
		// Add x to the value stored on the pc's address.
		// let ptr = cpu.mem[cpu.pc as usize].wrapping_add(cpu.x);
		// 0xff + 0x43 -> 0x42
		let ptr = 0x42 as u8;

		// Set the address of the pc's address value to some value.
		cpu.mem[ptr as usize] = 0x23; 

		// Set the address of the pc + 1 address value to some value.
		cpu.mem[ptr.wrapping_add(1) as usize] = 0x98; 

		// Check that the expected value is returned from get_operand_address.
		// Cannot use a simple "+" as it will overflow the space.
		let expected = 0x9823;
		assert_eq!(cpu.get_operand_address(&AddressingMode::IndirectX), expected);
	}

	// TODO: overflow ptr + y + 1

	// Indirect Y
	
	#[test]
	fn test_get_operand_address_indirecty_happypath() {
		// Create a CPU.
		let mut cpu = CPU::new();

		// Set pc to some value.
		cpu.pc = 0x43;

		// Set the address of the pc to some value.
		cpu.mem[cpu.pc as usize] = 0x22;

		// Set the y value to some value that will not overflow the u16 bit space.
		cpu.y = 0x01;

		// Get the pointer.
		// Add y to the value stored on the pc's address.
		// let ptr = cpu.mem[cpu.pc as usize].wrapping_add(cpu.y);
		// 0x22 + 0x01 = 0x23
		let ptr = 0x23 as u8;

		// Set the address of the pc + y address value to some value.
		cpu.mem[ptr as usize] = 0x42;

		// Set the address of the pc + y + 1 address value to some value.
		cpu.mem[ptr.wrapping_add(1) as usize] = 0x67;

		// Check that the expected value is returned from get_operand_address.
		// Can use a simple "+" as it won't overflow.
		let expected = 0x6742;
		assert_eq!(cpu.get_operand_address(&AddressingMode::IndirectY), expected);
	}

	// overflow
	#[test]
	fn test_get_operand_address_indirecty_overflow() {
		// Create a CPU.
		let mut cpu = CPU::new();

		// Set pc to some value.
		cpu.pc = 0x12;

		// Set the address of the pc to some value.
		cpu.mem[cpu.pc as usize] = 0xdd;

		// Set the y value to some value that will overflow the u16 bit space.
		cpu.y = 0xff;

		// Get the pointer.
		// Add y to the value stored on the pc's address.
		// let ptr = cpu.mem[cpu.pc as usize].wrapping_add(cpu.y);
		// 0xdd + 0xff -> 0xdc
		let ptr = 0xdc as u8;
		
		// Set the address of the pc + y address value to some value.
		cpu.mem[ptr as usize] = 0x47;

		// Set the address of the pc + y + 1 address value to some value.
		cpu.mem[ptr.wrapping_add(1) as usize] = 0x12;

		// Check that the expected value is returned from get_operand_address.
		// Cannot use a simple "+" as it will overflow the space.
		let expected = 0x1247;
		assert_eq!(cpu.get_operand_address(&AddressingMode::IndirectY), expected);
	}

	// TODO: overflow ptr + y + 1

	// None Addressing
	#[test]
	#[should_panic(expected = "not supported")]
	fn test_get_operand_address_none_happypath() {
		let mut cpu = CPU::new();
		cpu.get_operand_address(&AddressingMode::NoneAddressing);
	}

	// ---------- Stack Operation Tests -----------

	#[test]
	fn test_stack_push_happypath() {
		// create a cpu
		let mut cpu = CPU::new();

		// reset the CPU
		cpu.reset();

		// set a u8 number
		let value: u8 = 0x3e;

		// push the value onto the stack
		cpu.stack_push(value);

		// check the stack pointer value
		assert_eq!(cpu.s, 0xfc);

		// check the value on memory
		assert_eq!(cpu.mem[0x01fd], value);
	}
	
	#[test]
	fn test_stack_push_u16_happypath() {
		// create a cpu
		let mut cpu = CPU::new();

		// reset the CPU
		cpu.reset();

		// set a u16 number
		let value: u16 = 0x4387;

		// push the value onto the stack
		cpu.stack_push_u16(value);

		// check the stack pointer value
		assert_eq!(cpu.s, 0xfb);

		// check the value on the low value
		assert_eq!(cpu.mem[0x01fc], 0x87);

		// check the value on the high value
		assert_eq!(cpu.mem[0x01fd], 0x43);
	}
	
	#[test]
	fn test_stack_pop_happypath() {
		// create a cpu
		let mut cpu = CPU::new();

		// reset the cpu
		cpu.reset();

		// set a u8 number
		let value: u8 = 0x34;

		// push a value onto the stack
		cpu.stack_push(value);

		// pop the value off of the stack
		let actual: u8 = cpu.stack_pop();

		// check the stack pointer value
		assert_eq!(cpu.s, 0xfd);

		// check the value returned
		assert_eq!(actual, value);
	}

	#[test]
	fn test_stack_pop_u16_happypath() {
		// create a cpu
		let mut cpu = CPU::new();

		// reset the cpu
		cpu.reset();

		// ---- push 2 u8 values onto the stack ----

		// set two u8 numbers
		let lo: u8 = 0x43;
		let high: u8 = 0x67;

		// push 2 values onto the stack
		cpu.stack_push(high);
		cpu.stack_push(lo);

		// pop the u16 value off of the stack
		let actual: u16 = cpu.stack_pop_u16();

		// check the stack pointer value
		assert_eq!(cpu.s, 0xfd);

		// check the value returned
		assert_eq!(actual, 0x6743);

		// ---- push 1 u16 value onto the stack ----

		// set a single u16 number
		let value: u16 = 0x1290;

		// push 2 values onto the stack
		cpu.stack_push_u16(value);

		// pop the u16 value off of the stack
		let actual: u16 = cpu.stack_pop_u16();

		// check the stack pointer value
		assert_eq!(cpu.s, 0xfd);

		// check the value returned
		assert_eq!(actual, value);
	}

	#[should_panic(expected = "stack underflow")]
	#[test]
	fn test_stack_pop_underflow() {
		// create a cpu
		let mut cpu = CPU::new();

		// reset the cpu
		cpu.reset();

		// pop the value off of the stack
		// this should trigger the panic immediately
		let actual: u8 = cpu.stack_pop();
	}

	#[should_panic(expected = "stack underflow")]
	#[test]
	fn test_stack_pop_u16_underflow() {
		// create a cpu
		let mut cpu = CPU::new();

		// reset the cpu
		cpu.reset();

		// pop the value off of the stack
		// this should trigger the panic immediately
		let actual: u16 = cpu.stack_pop_u16();
	}

	// Stack pop overflows
	// We will not be checking for these scenarios.
	// The current implementation does not permit this to occur.

	// the implementation actually takes this to be a stack underflow
	#[should_panic(expected = "stack overflow")]
	#[test]
	fn test_stack_push_overflow() {
		// create a cpu
		let mut cpu = CPU::new();

		// reset the cpu
		cpu.reset();

		// set the stack pointer to the bottom of the stack.
		cpu.s = 0;

		let value: u8 = 2;

		// push the value off of the stack
		// this should trigger the panic immediately
		cpu.stack_push(value);
		cpu.stack_push(value);
	}

	#[should_panic(expected = "stack overflow")]
	#[test]
	fn test_stack_push_u16_overflow() {
		// create a cpu
		let mut cpu = CPU::new();

		// reset the cpu
		cpu.reset();

		// set the stack pointer to almost the bottom of the stack
		// push 2 values should lead to this as an overflow state.
		cpu.s = 1;

		let value: u8 = 2;

		// push the value off of the stack
		// this should trigger the panic immediately
		cpu.stack_push(value);
		cpu.stack_push(value);
	}

	// stack push underflow
	// odd, should never happen case
	// We won't check for this case.

	// stack push u16 underflow
	// odd, should never happen case
	// We won't check for this case.

    // ----------------------------------
    // --------- Ops Code Tests ---------
    // ----------------------------------

    // -------- BRK and NOP --------

    #[test]
    fn test_brk_nop_happy_path() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. No Op.
        // 2. Break.
        cpu.load_and_run(vec![0xea, 0x00]);

        // Check the registers have the expected values.
        assert_eq!(cpu.a, 0x00);
        assert_eq!(cpu.x, 0x00);
        assert_eq!(cpu.y, 0x00);
        assert_eq!(cpu.s, 0xfd);
        assert_eq!(cpu.pc, 0x8002);

        // Check the processor status is expected:
        // - Check all flags are not set.
        assert!(cpu.p & 0b1111_1111 == 0b0000_0000);
    }

    // -------- STA --------

    #[test]
    fn test_sta_zeropage_happy_path() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a positive value into A register.
        // 2. Store the value of the A register onto memory.
        // 3. Break.
        cpu.load_and_run(vec![0xa9, 0x05, 0x85, 0x04, 0x00]);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x05);

        // Check that the memory space has the expected value.
        assert_eq!(cpu.mem[0x0004], 0x05);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0000);
    }

    #[test]
    fn test_sta_absolute_happy_path() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a positive value into A register.
        // 2. Store the value of the A register onto memory.
        // 3. Break.
        cpu.load_and_run(vec![0xa9, 0x05, 0x8d, 0x04, 0xaa, 0x00]);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x05);

        // Check that the memory space has the expected value.
        assert_eq!(cpu.mem[0xaa04], 0x05);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0000);
    }

    #[test]
    fn test_sta_zeropagex_happy_path() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a positive value into A register.
        // 2. Load a positive value into X register.
        // 3. Store the value of the A register onto memory.
        // 4. Break.
        cpu.load_and_run(vec![0xa9, 0x05, 0xa2, 0x07, 0x95, 0x04, 0x00]);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x05);

        // Check that the memory space has the expected value.
        assert_eq!(cpu.mem[0x000b], 0x05);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0000);
    }

    #[test]
    fn test_sta_absolutex_happy_path() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a positive value into A register.
        // 2. Load a positive value into X register.
        // 3. Store the value of the A register onto memory.
        // 4. Break.
        cpu.load_and_run(vec![0xa9, 0x05, 0xa2, 0x07, 0x9d, 0x04, 0xea, 0x00]);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x05);

        // Check that the memory space has the expected value.
        assert_eq!(cpu.mem[0xea0b], 0x05);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0000);
    }

    #[test]
    fn test_sta_absolutey_happy_path() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a positive value into A register.
        // 2. Load a positive value into Y register.
        // 3. Store the value of the A register onto memory.
        // 4. Break.
        cpu.load_and_run(vec![0xa9, 0x05, 0xa0, 0x07, 0x99, 0x04, 0xdf, 0x00]);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x05);

        // Check that the memory space has the expected value.
        assert_eq!(cpu.mem[0xdf0b], 0x05);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0000);
    }
    
    // -------- STX --------

    #[test]
    fn test_stx_zeropage_happy_path() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a positive value into X register.
        // 2. Store the value of the X register onto memory.
        // 3. Break.
        cpu.load_and_run(vec![0xa2, 0x05, 0x86, 0x04, 0x00]);

        // Check the X register has the expected value.
        assert_eq!(cpu.x, 0x05);

        // Check that the memory space has the expected value.
        assert_eq!(cpu.mem[0x0004], 0x05);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0000);
    }

	#[test]
    fn test_stx_zeropagey_happy_path() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a positive value into X register.
        // 2. Load a positive value into Y register.
        // 3. Store the value of the X register onto memory.
        // 4. Break.
        cpu.load_and_run(vec![0xa2, 0x05, 0xa0, 0x07, 0x96, 0x04, 0x00]);

        // Check the X register has the expected value.
        assert_eq!(cpu.x, 0x05);

        // Check that the memory space has the expected value.
        assert_eq!(cpu.mem[0x000b], 0x05);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0000);
    }

    #[test]
    fn test_stx_absolute_happy_path() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a positive value into X register.
        // 2. Store the value of the X register onto memory.
        // 3. Break.
        cpu.load_and_run(vec![0xa2, 0x05, 0x8e, 0x04, 0x4a, 0x00]);

        // Check the X register has the expected value.
        assert_eq!(cpu.x, 0x05);

        // Check that the memory space has the expected value.
        assert_eq!(cpu.mem[0x4a04], 0x05);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0000);
    }
    
    // -------- STY --------

    #[test]
    fn test_sty_zeropage_happy_path() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a positive value into Y register.
        // 2. Store the value of the Y register onto memory.
        // 3. Break.
        cpu.load_and_run(vec![0xa0, 0x05, 0x84, 0x04, 0x00]);

        // Check the Y register has the expected value.
        assert_eq!(cpu.y, 0x05);

        // Check that the memory space has the expected value.
        assert_eq!(cpu.mem[0x0004], 0x05);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0000);
    }

    #[test]
    fn test_sty_zeropagex_happy_path() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a positive value into Y register.
        // 2. Load a positive value into X register.
        // 3. Store the value of the Y register onto memory.
        // 4. Break.
        cpu.load_and_run(vec![0xa0, 0x05, 0xa2, 0x6, 0x94, 0x04, 0x00]);

        // Check the Y register has the expected value.
        assert_eq!(cpu.y, 0x05);

        // Check that the memory space has the expected value.
        assert_eq!(cpu.mem[0x000a], 0x05);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0000);
    }

    #[test]
    fn test_sty_absolute_happy_path() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a positive value into Y register.
        // 2. Store the value of the Y register onto memory.
        // 3. Break.
        cpu.load_and_run(vec![0xa0, 0x05, 0x8c, 0x04, 0x4a, 0x00]);

        // Check the Y register has the expected value.
        assert_eq!(cpu.y, 0x05);

        // Check that the memory space has the expected value.
        assert_eq!(cpu.mem[0x4a04], 0x05);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0000);
    }

    // -------- LDA --------

    // -------- Immediate --------

    #[test]
    fn test_lda_immediate_happy_path() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a positive value into A register.
        // 2. Break.
        cpu.load_and_run(vec![0xa9, 0x05, 0x00]);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x05);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0000);
    }

    #[test]
    fn test_lda_immediate_negative_input() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a negative value into A register.
        // 2. Break.
        cpu.load_and_run(vec![0xa9, 0xf5, 0x00]);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0xf5);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is set.
        assert!(cpu.p & 0b1000_0010 == 0b1000_0000);
    }

    #[test]
    fn test_lda_immediate_zero() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load zero into A register.
        // 2. Break.
        cpu.load_and_run(vec![0xa9, 0x00, 0x00]);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x00);

        // Check the processor status is expected:
        // - Check the Zero Flag is set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0010);
    }

    // -------- Zero Page --------

    // TODO: Add a program in the future that loads stuff into RAM formally, not with hack.
    // Q: How do I load stuff into RAM?
    // TODO: Replace the hack to store into memory with STA, STX, and STY

    #[test]
    fn test_lda_zeropage_happy_path() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Set a location on RAM to be some value.
        cpu.mem[0x0005] = 0x43;

        // Load and run a short program.
        // 1. Load a positive value into A register.
        // 2. Break.
        cpu.load_and_run(vec![0xa5, 0x05, 0x00]);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x43);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0000);
    }

    #[test]
    fn test_lda_zeropage_negative_input() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Set a location on RAM to be some value.
        cpu.mem[0x0005] = 0xf6;

        // Load and run a short program.
        // 1. Load a negative value into A register.
        // 2. Break.
        cpu.load_and_run(vec![0xa5, 0x05, 0x00]);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0xf6);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is set.
        assert!(cpu.p & 0b1000_0010 == 0b1000_0000);
    }

    #[test]
    fn test_lda_zeropage_zero() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Set a location on RAM to be some value.
        cpu.mem[0x0005] = 0x00;

        // Load and run a short program.
        // 1. Load zero into A register.
        // 2. Break.
        cpu.load_and_run(vec![0xa5, 0x05, 0x00]);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x00);

        // Check the processor status is expected:
        // - Check the Zero Flag is set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0010);
    }

    // -------- Zero Page X --------

    #[test]
    fn test_lda_zeropagex_happy_path() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a positive value into X register.
        // 2. Load a positive value into memory from the X register.
        // 3. Load a positive value into A register.
        // 4. Break.
        cpu.load_and_run(vec![0xa2, 0x04, 0x86, 0x25, 0xb5, 0x21, 0x00]);

        // Check the X register has the expected value.
        assert_eq!(cpu.x, 0x04);

        // Check the memory address has the expected value.
        assert_eq!(cpu.mem[0x0025], 0x04);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x04);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0000);
    }

    #[test]
    fn test_lda_zeropagex_negative() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a negative value into Y register.
        // 2. Load a negative value into memory from the Y register.
        // 3. Load a positive value into the X register.
        // 4. Load a negative value into A register.
        // 5. Break.
        cpu.load_and_run(vec![0xa0, 0xf4, 0x84, 0x25, 0xa2, 0x04, 0xb5, 0x21, 0x00]);

		// Check the Y register has the expected value.
        assert_eq!(cpu.y, 0xf4);
        
        // Check the X register has the expected value.
        assert_eq!(cpu.x, 0x04);

        // Check the memory address has the expected value.
        assert_eq!(cpu.mem[0x0025], 0xf4);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0xf4);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is set.
        assert!(cpu.p & 0b1000_0010 == 0b1000_0000);
    }

    #[test]
    fn test_lda_zeropagex_zero() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a zero value into Y register.
        // 2. Load a zero value into memory from the Y register.
        // 3. Load a positive value into the X register.
        // 4. Load a zero value into A register.
        // 5. Break.
        cpu.load_and_run(vec![0xa0, 0x00, 0x84, 0x25, 0xa2, 0x04, 0xb5, 0x21, 0x00]);

		// Check the Y register has the expected value.
        assert_eq!(cpu.y, 0x00);
        
        // Check the X register has the expected value.
        assert_eq!(cpu.x, 0x04);

        // Check the memory address has the expected value.
        assert_eq!(cpu.mem[0x0025], 0x00);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x00);

        // Check the processor status is expected:
        // - Check the Zero Flag is set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0010);
    }

    // -------- Absolute --------

    #[test]
    fn test_lda_absolute_happy_path() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a positive value into A register.
        // 2. Break.
        cpu.load_and_run(vec![0xad, 0x07, 0x80, 0x00, 0xea, 0xea, 0xea, 0x57]);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x57);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0000);
    }

    #[test]
    fn test_lda_absolute_negative() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a negative value into A register.
        // 2. Break.
        cpu.load_and_run(vec![0xad, 0x07, 0x80, 0x00, 0xea, 0xea, 0xea, 0xf6]);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0xf6);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is set.
        assert!(cpu.p & 0b1000_0010 == 0b1000_0000);
    }

    #[test]
    fn test_lda_absolute_zero() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load zero into A register.
        // 2. Break.
        cpu.load_and_run(vec![0xad, 0x08, 0x80, 0x00, 0xea, 0xea, 0xea, 0xea, 0x00]);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x00);

        // Check the processor status is expected:
        // - Check the Zero Flag is set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0010);
    }

    // -------- Absolute X --------

    #[test]
    fn test_lda_absolutex_happy_path() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a positive value into A register.
        // 2. Load a positive value from the A register onto memory.
        // 3. Load a zero value into the A register to reset.
        // 4. Load a positive value into X register.
        // 5. Load a positive value into A register, using the address where it is stored.
        // 6. Break.
        cpu.load_and_run(vec![0xa9, 0x04, 0x8d, 0xab, 0xa1, 0xa9, 0x00, 0xa2, 0x08, 0xbd, 0xa3, 0xa1, 0x00]);

        // Check the X register has the expected value.
        assert_eq!(cpu.x, 0x08);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x04);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0000);
    }

    #[test]
    fn test_lda_absolutex_negative() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a negative value into A register.
        // 2. Load a negative value from the A register onto memory.
        // 3. Load a zero value into the A register to reset.
        // 4. Load a positive value into X register.
        // 5. Load a positive value into A register, using the address where it is stored.
        // 6. Break.
        cpu.load_and_run(vec![0xa9, 0xf4, 0x8d, 0xab, 0xa1, 0xa9, 0x00, 0xa2, 0x08, 0xbd, 0xa3, 0xa1, 0x00]);

        // Check the X register has the expected value.
        assert_eq!(cpu.x, 0x08);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0xf4);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is set.
        assert!(cpu.p & 0b1000_0010 == 0b1000_0000);
    }

    #[test]
    fn test_lda_absolutex_zero() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a zero value into A register.
        // 2. Load a zero value from the A register onto memory.
        // 3. Load a non-zero value into the A register to reset.
        // 4. Load a positive value into X register.
        // 5. Load a positive value into A register, using the address where it is stored.
        // 6. Break.
        cpu.load_and_run(vec![0xa9, 0x00, 0x8d, 0xab, 0xa1, 0xa9, 0x01, 0xa2, 0x08, 0xbd, 0xa3, 0xa1, 0x00]);

        // Check the X register has the expected value.
        assert_eq!(cpu.x, 0x08);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x00);

        // Check the processor status is expected:
        // - Check the Zero Flag is set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0010);
    }

    // -------- Absolute Y --------

    #[test]
    fn test_lda_absolutey_happy_path() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a positive value into A register.
        // 2. Load a positive value from the A register onto memory.
        // 3. Load a zero value into the A register to reset.
        // 4. Load a positive value into Y register.
        // 5. Load a positive value into A register, using the address where it is stored.
        // 6. Break.
        cpu.load_and_run(vec![0xa9, 0x04, 0x8d, 0xab, 0xa1, 0xa9, 0x00, 0xa0, 0x08, 0xb9, 0xa3, 0xa1, 0x00]);

        // Check the Y register has the expected value.
        assert_eq!(cpu.y, 0x08);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x04);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0000);
    }

    #[test]
    fn test_lda_absolutey_negative() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a negative value into A register.
        // 2. Load a negative value from the A register onto memory.
        // 3. Load a zero value into the A register to reset.
        // 4. Load a positive value into Y register.
        // 5. Load a positive value into A register, using the address where it is stored.
        // 6. Break.
        cpu.load_and_run(vec![0xa9, 0xf4, 0x8d, 0xab, 0xa1, 0xa9, 0x00, 0xa0, 0x08, 0xb9, 0xa3, 0xa1, 0x00]);

        // Check the Y register has the expected value.
        assert_eq!(cpu.y, 0x08);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0xf4);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is set.
        assert!(cpu.p & 0b1000_0010 == 0b1000_0000);
    }

    #[test]
    fn test_lda_absolutey_zero() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a zero value into A register.
        // 2. Load a zero value from the A register onto memory.
        // 3. Load a non-zero value into the A register to reset.
        // 4. Load a positive value into Y register.
        // 5. Load a positive value into A register, using the address where it is stored.
        // 6. Break.
        cpu.load_and_run(vec![0xa9, 0x00, 0x8d, 0xab, 0xa1, 0xa9, 0x01, 0xa0, 0x08, 0xb9, 0xa3, 0xa1, 0x00]);

        // Check the Y register has the expected value.
        assert_eq!(cpu.y, 0x08);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x00);

        // Check the processor status is expected:
        // - Check the Zero Flag is set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0010);
    }

    // -------- Indirect X --------

    #[test]
    fn test_lda_indirectx_happy_path() {
    	// Create a CPU.
        let mut cpu = CPU::new();

    	// value is on 0x2163
    	// first address is 0x0043 and has 0x63
    	// second address is 0x0044 and 0x21
    	// x is 
    	// something + 31 = 0x6300
    	// 0075

    	// Load and run a short program
    	// 1. Load a positive value into the Y register. This will be the final result.
    	// 2. Store the value from the Y register onto memory.
    	// 3. Load a positive value into the Y register. This will be the first address value stored on the indirect address.
    	// 4. Store the value from the Y register onto memory on an adjacent space.
    	// 3. Load a positive value into the Y register. This will be the second address value stored on the indirect address.
    	// 4. Store the value from the Y register onto memory on an adjacent space.
    	// 5. Load a positive value into the X register.
    	// 6. Load a positive value into the A register, from the value on memory.
    	// 7. Break
    	cpu.load_and_run(vec![0xa0, 0x54, 0x8c, 0x63, 0x21, 0xa0, 0x63, 0x84, 0x43, 0xa0, 0x21, 0x84, 0x44, 0xa2, 0x31, 0xa1, 0x12, 0x00]);

        // Check the Y register has the expected value.
        assert_eq!(cpu.y, 0x21);

        // Check the X register has the expected value.
        assert_eq!(cpu.x, 0x31);
        
        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x54);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0000);
    }

    #[test]
    fn test_lda_indirectx_negative() {
    	// Create a CPU.
        let mut cpu = CPU::new();

    	// value is on 0x2163
    	// first address is 0x0043 and has 0x63
    	// second address is 0x0044 and 0x21
    	// x is 
    	// something + 31 = 0x6300
    	// 0075

    	// Load and run a short program
    	// 1. Load a negative value into the Y register. This will be the final result.
    	// 2. Store the value from the Y register onto memory.
    	// 3. Load a positive value into the Y register. This will be the first address value stored on the indirect address.
    	// 4. Store the value from the Y register onto memory on an adjacent space.
    	// 3. Load a positive value into the Y register. This will be the second address value stored on the indirect address.
    	// 4. Store the value from the Y register onto memory on an adjacent space.
    	// 5. Load a positive value into the X register.
    	// 6. Load a positive value into the A register, from the value on memory.
    	// 7. Break
    	cpu.load_and_run(vec![0xa0, 0xf4, 0x8c, 0x63, 0x21, 0xa0, 0x63, 0x84, 0x43, 0xa0, 0x21, 0x84, 0x44, 0xa2, 0x31, 0xa1, 0x12, 0x00]);

        // Check the Y register has the expected value.
        assert_eq!(cpu.y, 0x21);

        // Check the X register has the expected value.
        assert_eq!(cpu.x, 0x31);
        
        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0xf4);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is set.
        assert!(cpu.p & 0b1000_0010 == 0b1000_0000);
    }

    #[test]
    fn test_lda_indirectx_zero() {
    	// Create a CPU.
        let mut cpu = CPU::new();

    	// value is on 0x2163
    	// first address is 0x0043 and has 0x63
    	// second address is 0x0044 and 0x21
    	// x is 
    	// something + 31 = 0x6300
    	// 0075

    	// Load and run a short program
    	// 1. Load a zero value into the Y register. This will be the final result.
    	// 2. Store the value from the Y register onto memory.
    	// 3. Load a positive value into the Y register. This will be the first address value stored on the indirect address.
    	// 4. Store the value from the Y register onto memory on an adjacent space.
    	// 3. Load a positive value into the Y register. This will be the second address value stored on the indirect address.
    	// 4. Store the value from the Y register onto memory on an adjacent space.
    	// 5. Load a positive value into the X register.
    	// 6. Load a positive value into the A register, from the value on memory.
    	// 7. Break
    	cpu.load_and_run(vec![0xa0, 0x00, 0x8c, 0x63, 0x21, 0xa0, 0x63, 0x84, 0x43, 0xa0, 0x21, 0x84, 0x44, 0xa2, 0x31, 0xa1, 0x12, 0x00]);

        // Check the Y register has the expected value.
        assert_eq!(cpu.y, 0x21);

        // Check the X register has the expected value.
        assert_eq!(cpu.x, 0x31);
        
        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x00);

        // Check the processor status is expected:
        // - Check the Zero Flag is set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0010);
    }

    // -------- Indirect Y --------

    #[test]
    fn test_lda_indirecty_happy_path() {
    	// Create a CPU.
        let mut cpu = CPU::new();

    	// value is on 0x2163
    	// first address is 0x0043 and has 0x63
    	// second address is 0x0044 and 0x21
    	// y is 
    	// something + 31 = 0x6300
    	// 0075

    	// Load and run a short program
    	// 1. Load a positive value into the X register. This will be the final result.
    	// 2. Store the value from the X register onto memory.
    	// 3. Load a positive value into the X register. This will be the first address value stored on the indirect address.
    	// 4. Store the value from the X register onto memory on an adjacent space.
    	// 3. Load a positive value into the X register. This will be the second address value stored on the indirect address.
    	// 4. Store the value from the X register onto memory on an adjacent space.
    	// 5. Load a positive value into the Y register.
    	// 6. Load a positive value into the A register, from the value on memory.
    	// 7. Break
    	cpu.load_and_run(vec![0xa2, 0x54, 0x8e, 0x63, 0x21, 0xa2, 0x63, 0x86, 0x43, 0xa2, 0x21, 0x86, 0x44, 0xa0, 0x31, 0xb1, 0x12, 0x00]);

        // Check the X register has the expected value.
        assert_eq!(cpu.x, 0x21);

        // Check the Y register has the expected value.
        assert_eq!(cpu.y, 0x31);
        
        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x54);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0000);
    }

    #[test]
    fn test_lda_indirecty_negative() {
    	// Create a CPU.
        let mut cpu = CPU::new();

    	// value is on 0x2163
    	// first address is 0x0043 and has 0x63
    	// second address is 0x0044 and 0x21
    	// y is 
    	// something + 31 = 0x6300
    	// 0075

    	// Load and run a short program
    	// 1. Load a negative value into the X register. This will be the final result.
    	// 2. Store the value from the X register onto memory.
    	// 3. Load a positive value into the X register. This will be the first address value stored on the indirect address.
    	// 4. Store the value from the X register onto memory on an adjacent space.
    	// 3. Load a positive value into the X register. This will be the second address value stored on the indirect address.
    	// 4. Store the value from the X register onto memory on an adjacent space.
    	// 5. Load a positive value into the Y register.
    	// 6. Load a positive value into the A register, from the value on memory.
    	// 7. Break
    	cpu.load_and_run(vec![0xa2, 0xf4, 0x8e, 0x63, 0x21, 0xa2, 0x63, 0x86, 0x43, 0xa2, 0x21, 0x86, 0x44, 0xa0, 0x31, 0xb1, 0x12, 0x00]);

        // Check the X register has the expected value.
        assert_eq!(cpu.x, 0x21);

        // Check the Y register has the expected value.
        assert_eq!(cpu.y, 0x31);
        
        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0xf4);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is set.
        assert!(cpu.p & 0b1000_0010 == 0b1000_0000);
    }

    #[test]
    fn test_lda_indirecty_zero() {
    	// Create a CPU.
        let mut cpu = CPU::new();

    	// value is on 0x2163
    	// first address is 0x0043 and has 0x63
    	// second address is 0x0044 and 0x21
    	// y is 
    	// something + 31 = 0x6300
    	// 0075

    	// Load and run a short program
    	// 1. Load a zero value into the X register. This will be the final result.
    	// 2. Store the value from the X register onto memory.
    	// 3. Load a positive value into the X register. This will be the first address value stored on the indirect address.
    	// 4. Store the value from the X register onto memory on an adjacent space.
    	// 3. Load a positive value into the X register. This will be the second address value stored on the indirect address.
    	// 4. Store the value from the X register onto memory on an adjacent space.
    	// 5. Load a positive value into the Y register.
    	// 6. Load a positive value into the A register, from the value on memory.
    	// 7. Break
    	cpu.load_and_run(vec![0xa2, 0x00, 0x8e, 0x63, 0x21, 0xa2, 0x63, 0x86, 0x43, 0xa2, 0x21, 0x86, 0x44, 0xa0, 0x31, 0xb1, 0x12, 0x00]);

        // Check the X register has the expected value.
        assert_eq!(cpu.x, 0x21);

        // Check the Y register has the expected value.
        assert_eq!(cpu.y, 0x31);
        
        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x00);

		// Check the processor status is expected:
        // - Check the Zero Flag is set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0010);
    }

    // -------- LDX --------

    // -------- Immediate --------

    #[test]
    fn test_ldx_immediate_happy_path() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a positive value into X register.
        // 2. Break.
        cpu.load_and_run(vec![0xa2, 0x05, 0x00]);

        // Check the X register has the expected value.
        assert_eq!(cpu.x, 0x05);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0000);
    }

    #[test]
    fn test_ldx_immediate_negative_input() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a negative value into X register.
        // 2. Break.
        cpu.load_and_run(vec![0xa2, 0xf5, 0x00]);

        // Check the X register has the expected value.
        assert_eq!(cpu.x, 0xf5);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is set.
        assert!(cpu.p & 0b1000_0010 == 0b1000_0000);
    }

    #[test]
    fn test_ldx_immediate_zero() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load zero into X register.
        // 2. Break.
        cpu.load_and_run(vec![0xa2, 0x00, 0x00]);

        // Check the X register has the expected value.
        assert_eq!(cpu.x, 0x00);

        // Check the processor status is expected:
        // - Check the Zero Flag is set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0010);
    }

    // -------- Zero Page --------

    #[test]
    fn test_ldx_zeropage_happy_path() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a positive value into A register.
        // 2. Store the positive value onto memory in the first 256 bytes.
        // 3. Load a positive value into the X register.
        // 4. Break.
        cpu.load_and_run(vec![0xa9, 0x54, 0x85, 0x23, 0xa6, 0x23, 0x00]);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x54);

        // Check the X register has the expected value.
        assert_eq!(cpu.x, 0x54);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0000);
    }

    #[test]
    fn test_ldx_zeropage_negative() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a negative value into A register.
        // 2. Store the negative value onto memory in the first 256 bytes.
        // 3. Load a negative value into the X register.
        // 4. Break.
        cpu.load_and_run(vec![0xa9, 0xa5, 0x85, 0x23, 0xa6, 0x23, 0x00]);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0xa5);

        // Check the X register has the expected value.
        assert_eq!(cpu.x, 0xa5);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is set.
        assert!(cpu.p & 0b1000_0010 == 0b1000_0000);
    }

    #[test]
    fn test_ldx_zeropage_zero() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a zero value into A register.
        // 2. Store the zero value onto memory in the first 256 bytes.
        // 3. Load a zero value into the X register.
        // 4. Break.
        cpu.load_and_run(vec![0xa9, 0x00, 0x85, 0x23, 0xa6, 0x23, 0x00]);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x00);

        // Check the X register has the expected value.
        assert_eq!(cpu.x, 0x00);

        // Check the processor status is expected:
        // - Check the Zero Flag is set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0010);
    }

	// -------- Zero Page Y --------

    #[test]
    fn test_ldx_zeropagey_happy_path() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a positive value into the Y register.
        // 2. Load a positive value into A register.
        // 3. Store the positive value onto memory in the first 256 bytes.
        // 4. Load a positive value into the X register.
        // 5. Break.
        cpu.load_and_run(vec![0xa0, 0x01, 0xa9, 0x54, 0x85, 0x23, 0xb6, 0x22, 0x00]);

        // Check the Y register has the expected value.
        assert_eq!(cpu.y, 0x01);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x54);

        // Check the X register has the expected value.
        assert_eq!(cpu.x, 0x54);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0000);
    }

    #[test]
    fn test_ldx_zeropagey_negative() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a positive value into the Y register.
        // 2. Load a negative value into A register.
        // 3. Store the negative value onto memory in the first 256 bytes.
        // 4. Load a negative value into the X register.
        // 5. Break.
        cpu.load_and_run(vec![0xa0, 0x01, 0xa9, 0xa4, 0x85, 0x23, 0xb6, 0x22, 0x00]);

        // Check the Y register has the expected value.
        assert_eq!(cpu.y, 0x01);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0xa4);

        // Check the X register has the expected value.
        assert_eq!(cpu.x, 0xa4);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is set.
        assert!(cpu.p & 0b1000_0010 == 0b1000_0000);
    }

    #[test]
    fn test_ldx_zeropagey_zero() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a positive value into the Y register.
        // 2. Load a zero value into A register.
        // 3. Store the zero value onto memory in the first 256 bytes.
        // 4. Load a zero value into the X register.
        // 5. Break.
        cpu.load_and_run(vec![0xa0, 0x01, 0xa9, 0x00, 0x85, 0x23, 0xb6, 0x22, 0x00]);

        // Check the Y register has the expected value.
        assert_eq!(cpu.y, 0x01);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x00);

        // Check the X register has the expected value.
        assert_eq!(cpu.x, 0x00);

        // Check the processor status is expected:
        // - Check the Zero Flag is set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0010);
    }

	// -------- Absolute --------

    #[test]
    fn test_ldx_absolute_happy_path() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a positive value into A register.
        // 2. Store the positive value onto memory outside the first 256 bytes.
        // 3. Load a positive value into the X register.
        // 4. Break.
        cpu.load_and_run(vec![0xa9, 0x54, 0x8d, 0x45, 0x86, 0xae, 0x45, 0x86, 0x00]);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x54);

        // Check the X register has the expected value.
        assert_eq!(cpu.x, 0x54);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0000);
    }

    #[test]
    fn test_ldx_absolute_negative() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a negative value into A register.
        // 2. Store the negative value onto memory outside the first 256 bytes.
        // 3. Load a negative value into the X register.
        // 4. Break.
        cpu.load_and_run(vec![0xa9, 0x8a, 0x8d, 0x45, 0x86, 0xae, 0x45, 0x86, 0x00]);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x8a);

        // Check the X register has the expected value.
        assert_eq!(cpu.x, 0x8a);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is set.
        assert!(cpu.p & 0b1000_0010 == 0b1000_0000);
    }

    #[test]
    fn test_ldx_absolute_zero() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a zero value into A register.
        // 2. Store the zero value onto memory outside the first 256 bytes.
        // 3. Load a zero value into the X register.
        // 4. Break.
        cpu.load_and_run(vec![0xa9, 0x00, 0x8d, 0x45, 0x86, 0xae, 0x45, 0x86, 0x00]);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x00);

        // Check the X register has the expected value.
        assert_eq!(cpu.x, 0x00);

        // Check the processor status is expected:
        // - Check the Zero Flag is set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0010);
    }

	// -------- Absolute Y --------

    #[test]
    fn test_ldx_absolutey_happy_path() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a positive value into the Y register
        // 2. Load a positive value into A register.
        // 3. Store the positive value onto memory outside the first 256 bytes.
        // 4. Load a positive value into the X register.
        // 5. Break.
        cpu.load_and_run(vec![0xa0, 0x01, 0xa9, 0x54, 0x8d, 0x45, 0x86, 0xbe, 0x44, 0x86, 0x00]);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x54);

        // Check the X register has the expected value.
        assert_eq!(cpu.x, 0x54);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0000);
    }

    #[test]
    fn test_ldx_absolutey_negative() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a positive value into the Y register
        // 2. Load a negative value into A register.
        // 3. Store the negative value onto memory outside the first 256 bytes.
        // 4. Load a negative value into the X register.
        // 5. Break.
        cpu.load_and_run(vec![0xa0, 0x01, 0xa9, 0x82, 0x8d, 0x45, 0x86, 0xbe, 0x44, 0x86, 0x00]);
        
        // Check the Y register has the expected value.
        assert_eq!(cpu.y, 0x01);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x82);

        // Check the X register has the expected value.
        assert_eq!(cpu.x, 0x82);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is set.
        assert!(cpu.p & 0b1000_0010 == 0b1000_0000);
    }

    #[test]
    fn test_ldx_absolutey_zero() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a positive value into the Y register
        // 2. Load a zero value into A register.
        // 3. Store the zero value onto memory outside the first 256 bytes.
        // 4. Load a zero value into the X register.
        // 5. Break.
        cpu.load_and_run(vec![0xa0, 0x01, 0xa9, 0x00, 0x8d, 0x45, 0x86, 0xbe, 0x44, 0x86, 0x00]);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x00);

        // Check the X register has the expected value.
        assert_eq!(cpu.x, 0x00);

        // Check the processor status is expected:
        // - Check the Zero Flag is set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0010);
    }

    // -------- LDY --------

    // -------- Immediate --------

    #[test]
    fn test_ldy_immediate_happy_path() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a positive value into A register.
        // 2. Break.
        cpu.load_and_run(vec![0xa0, 0x05, 0x00]);

        // Check the A register has the expected value.
        assert_eq!(cpu.y, 0x05);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0000);
    }

    #[test]
    fn test_ldy_immediate_negative_input() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a negative value into A register.
        // 2. Break.
        cpu.load_and_run(vec![0xa0, 0xf5, 0x00]);

        // Check the A register has the expected value.
        assert_eq!(cpu.y, 0xf5);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is set.
        assert!(cpu.p & 0b1000_0010 == 0b1000_0000);
    }

    #[test]
    fn test_ldy_immediate_zero() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load zero into A register.
        // 2. Break.
        cpu.load_and_run(vec![0xa0, 0x00, 0x00]);

        // Check the A register has the expected value.
        assert_eq!(cpu.y, 0x00);

        // Check the processor status is expected:
        // - Check the Zero Flag is set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0010);
    }

    // -------- Zero Page --------

    #[test]
    fn test_ldy_zeropage_happy_path() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a positive value into A register.
        // 2. Store the positive value onto memory in the first 256 bytes.
        // 3. Load a positive value into the Y register.
        // 4. Break.
        cpu.load_and_run(vec![0xa9, 0x54, 0x85, 0x23, 0xa4, 0x23, 0x00]);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x54);

        // Check the Y register has the expected value.
        assert_eq!(cpu.y, 0x54);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0000);
    }

    #[test]
    fn test_ldy_zeropage_negative() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a negative value into A register.
        // 2. Store the negative value onto memory in the first 256 bytes.
        // 3. Load a negative value into the Y register.
        // 4. Break.
        cpu.load_and_run(vec![0xa9, 0xa5, 0x85, 0x23, 0xa4, 0x23, 0x00]);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0xa5);

        // Check the Y register has the expected value.
        assert_eq!(cpu.y, 0xa5);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is set.
        assert!(cpu.p & 0b1000_0010 == 0b1000_0000);
    }

    #[test]
    fn test_ldy_zeropage_zero() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a zero value into A register.
        // 2. Store the zero value onto memory in the first 256 bytes.
        // 3. Load a zero value into the Y register.
        // 4. Break.
        cpu.load_and_run(vec![0xa9, 0x00, 0x85, 0x23, 0xa4, 0x23, 0x00]);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x00);

        // Check the X register has the expected value.
        assert_eq!(cpu.y, 0x00);

        // Check the processor status is expected:
        // - Check the Zero Flag is set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0010);
    }

    // -------- Zero Page X --------

    #[test]
    fn test_ldy_zeropagex_happy_path() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a positive value into the X register.
        // 2. Load a positive value into A register.
        // 3. Store the positive value onto memory in the first 256 bytes.
        // 4. Load a positive value into the Y register.
        // 5. Break.
        cpu.load_and_run(vec![0xa2, 0x01, 0xa9, 0x54, 0x85, 0x23, 0xb4, 0x22, 0x00]);

        // Check the X register has the expected value.
        assert_eq!(cpu.x, 0x01);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x54);

        // Check the Y register has the expected value.
        assert_eq!(cpu.y, 0x54);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0000);
    }

    #[test]
    fn test_ldy_zeropagex_negative() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a positive value into the X register.
        // 2. Load a negative value into A register.
        // 3. Store the negative value onto memory in the first 256 bytes.
        // 4. Load a negative value into the Y register.
        // 5. Break.
        cpu.load_and_run(vec![0xa2, 0x01, 0xa9, 0xa4, 0x85, 0x23, 0xb4, 0x22, 0x00]);

        // Check the X register has the expected value.
        assert_eq!(cpu.x, 0x01);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0xa4);

        // Check the Y register has the expected value.
        assert_eq!(cpu.y, 0xa4);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is set.
        assert!(cpu.p & 0b1000_0010 == 0b1000_0000);
    }

    #[test]
    fn test_ldy_zeropagex_zero() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a positive value into the X register.
        // 2. Load a zero value into A register.
        // 3. Store the zero value onto memory in the first 256 bytes.
        // 4. Load a zero value into the Y register.
        // 5. Break.
        cpu.load_and_run(vec![0xa2, 0x01, 0xa9, 0x00, 0x85, 0x23, 0xb4, 0x22, 0x00]);

        // Check the X register has the expected value.
        assert_eq!(cpu.x, 0x01);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x00);

        // Check the Y register has the expected value.
        assert_eq!(cpu.y, 0x00);

        // Check the processor status is expected:
        // - Check the Zero Flag is set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0010);
    }

    // -------- Absolute --------

    #[test]
    fn test_ldy_absolute_happy_path() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a positive value into A register.
        // 2. Store the positive value onto memory outside the first 256 bytes.
        // 3. Load a positive value into the Y register.
        // 4. Break.
        cpu.load_and_run(vec![0xa9, 0x54, 0x8d, 0x45, 0x86, 0xac, 0x45, 0x86, 0x00]);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x54);

        // Check the Y register has the expected value.
        assert_eq!(cpu.y, 0x54);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0000);
    }

    #[test]
    fn test_ldy_absolute_negative() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a negative value into A register.
        // 2. Store the negative value onto memory outside the first 256 bytes.
        // 3. Load a negative value into the Y register.
        // 4. Break.
        cpu.load_and_run(vec![0xa9, 0x8a, 0x8d, 0x45, 0x86, 0xac, 0x45, 0x86, 0x00]);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x8a);

        // Check the Y register has the expected value.
        assert_eq!(cpu.y, 0x8a);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is set.
        assert!(cpu.p & 0b1000_0010 == 0b1000_0000);
    }

    #[test]
    fn test_ldy_absolute_zero() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a zero value into A register.
        // 2. Store the zero value onto memory outside the first 256 bytes.
        // 3. Load a zero value into the Y register.
        // 4. Break.
        cpu.load_and_run(vec![0xa9, 0x00, 0x8d, 0x45, 0x86, 0xac, 0x45, 0x86, 0x00]);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x00);

        // Check the Y register has the expected value.
        assert_eq!(cpu.y, 0x00);

        // Check the processor status is expected:
        // - Check the Zero Flag is set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0010);
    }

    // -------- Absolute X --------

    #[test]
    fn test_ldy_absolutex_happy_path() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a positive value into the X register
        // 2. Load a positive value into A register.
        // 3. Store the positive value onto memory outside the first 256 bytes.
        // 4. Load a positive value into the Y register.
        // 5. Break.
        cpu.load_and_run(vec![0xa2, 0x01, 0xa9, 0x54, 0x8d, 0x45, 0x86, 0xbc, 0x44, 0x86, 0x00]);

        // Check the A register has the expected value.
        assert_eq!(cpu.x, 0x01);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x54);

        // Check the Y register has the expected value.
        assert_eq!(cpu.y, 0x54);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0000);
    }

    #[test]
    fn test_ldy_absolutex_negative() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a positive value into the X register
        // 2. Load a negative value into A register.
        // 3. Store the negative value onto memory outside the first 256 bytes.
        // 4. Load a negative value into the Y register.
        // 5. Break.
        cpu.load_and_run(vec![0xa2, 0x01, 0xa9, 0xa4, 0x8d, 0x45, 0x86, 0xbc, 0x44, 0x86, 0x00]);

        // Check the A register has the expected value.
        assert_eq!(cpu.x, 0x01);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0xa4);

        // Check the Y register has the expected value.
        assert_eq!(cpu.y, 0xa4);

        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is set.
        assert!(cpu.p & 0b1000_0010 == 0b1000_0000);
    }

    #[test]
    fn test_ldy_absolutex_zero() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Load and run a short program.
        // 1. Load a positive value into the X register
        // 2. Load a zero value into A register.
        // 3. Store the zero value onto memory outside the first 256 bytes.
        // 4. Load a positive value into the Y register.
        // 5. Break.
        cpu.load_and_run(vec![0xa2, 0x01, 0xa9, 0x00, 0x8d, 0x45, 0x86, 0xbc, 0x44, 0x86, 0x00]);

        // Check the A register has the expected value.
        assert_eq!(cpu.x, 0x01);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x00);

        // Check the Y register has the expected value.
        assert_eq!(cpu.y, 0x00);

        // Check the processor status is expected:
        // - Check the Zero Flag is set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0010);
    }

    // -------- TAX --------

    #[test]
    fn test_tax_happy_path() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

        // Load and run a short program.
    	// 1. Load a positive value into the A register.
    	// 2. Copy value from A register into X register.
    	// 3. Break.
    	cpu.load_and_run(vec![0xa9, 0x05, 0xaa, 0x00]);

    	// Check the X register has the expected value.
    	assert_eq!(cpu.x, 0x05);
        
        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0000);
    }

    #[test]
    fn test_tax_negative() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

        // Load and run a short program.
    	// 1. Load a negative value into the A register.
    	// 2. Copy value from A register into X register.
    	// 3. Break.
    	cpu.load_and_run(vec![0xa9, 0xfd, 0xaa, 0x00]);

    	// Check the X register has the expected value.
    	assert_eq!(cpu.x, 0xfd);
        
        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is set.
        assert!(cpu.p & 0b1000_0010 == 0b1000_0000);
    }

    #[test]
    fn test_tax_zero() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

        // Load and run a short program.
    	// 1. Load 0 into the A register.
    	// 2. Copy value from A register into X register.
    	// 3. Break.
    	cpu.load_and_run(vec![0xa9, 0x00, 0xaa, 0x00]);

    	// Check the X register has the expected value.
    	assert_eq!(cpu.x, 00);
        
        // Check the processor status is expected:
        // - Check the Zero Flag is set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0010);
    }

    // -------- TAY --------

    #[test]
    fn test_tay_happy_path() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

        // Load and run a short program.
    	// 1. Load a positive value into the A register.
    	// 2. Copy value from A register into Y register.
    	// 3. Break.
    	cpu.load_and_run(vec![0xa9, 0x05, 0xa8, 0x00]);

    	// Check the Y register has the expected value.
    	assert_eq!(cpu.y, 0x05);
        
        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0000);
    }

    #[test]
    fn test_tay_negative() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

        // Load and run a short program.
    	// 1. Load a negative value into the A register.
    	// 2. Copy value from A register into Y register.
    	// 3. Break.
    	cpu.load_and_run(vec![0xa9, 0xfd, 0xa8, 0x00]);

    	// Check the Y register has the expected value.
    	assert_eq!(cpu.y, 0xfd);
        
        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is set.
        assert!(cpu.p & 0b1000_0010 == 0b1000_0000);
    }

    #[test]
    fn test_tay_zero() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

        // Load and run a short program.
    	// 1. Load 0 into the A register.
    	// 2. Copy value from A register into Y register.
    	// 3. Break.
    	cpu.load_and_run(vec![0xa9, 0x00, 0xa8, 0x00]);

    	// Check the Y register has the expected value.
    	assert_eq!(cpu.y, 0x00);
        
        // Check the processor status is expected:
        // - Check the Zero Flag is set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0010);
    }

    // -------- TXA --------

    #[test]
    fn test_txa_happy_path() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

        // Load and run a short program.
    	// 1. Load a positive value into the X register.
    	// 2. Copy value from X register into A register.
    	// 3. Break.
    	cpu.load_and_run(vec![0xa2, 0x05, 0x8a, 0x00]);

    	// Check the A register has the expected value.
    	assert_eq!(cpu.a, 0x05);
        
        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0000);
    }

    #[test]
    fn test_txa_negative() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

        // Load and run a short program.
    	// 1. Load a negative value into the X register.
    	// 2. Copy value from X register into A register.
    	// 3. Break.
    	cpu.load_and_run(vec![0xa2, 0xfd, 0x8a, 0x00]);

    	// Check the A register has the expected value.
    	assert_eq!(cpu.a, 0xfd);
        
        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is set.
        assert!(cpu.p & 0b1000_0010 == 0b1000_0000);
    }

    #[test]
    fn test_txa_zero() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

        // Load and run a short program.
    	// 1. Load 0 into the X register.
    	// 2. Copy value from X register into A register.
    	// 3. Break.
    	cpu.load_and_run(vec![0xa2, 0x00, 0x8a, 0x00]);

    	// Check the A register has the expected value.
    	assert_eq!(cpu.a, 0x00);
        
        // Check the processor status is expected:
        // - Check the Zero Flag is set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0010);
    }

    // -------- TYA --------

    #[test]
    fn test_tya_happy_path() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

        // Load and run a short program.
    	// 1. Load a positive value into the Y register.
    	// 2. Copy value from Y register into A register.
    	// 3. Break.
    	cpu.load_and_run(vec![0xa0, 0x05, 0x98, 0x00]);

    	// Check the A register has the expected value.
    	assert_eq!(cpu.a, 0x05);
        
        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0000);
    }

    #[test]
    fn test_tya_negative() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

        // Load and run a short program.
    	// 1. Load a negative value into the Y register.
    	// 2. Copy value from Y register into A register.
    	// 3. Break.
    	cpu.load_and_run(vec![0xa0, 0xfd, 0x98, 0x00]);

    	// Check the A register has the expected value.
    	assert_eq!(cpu.a, 0xfd);
        
        // Check the processor status is expected:
        // - Check the Zero Flag is not set.
        // - Check the Negative Flag is set.
        assert!(cpu.p & 0b1000_0010 == 0b1000_0000);
    }

    #[test]
    fn test_tya_zero() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

        // Load and run a short program.
    	// 1. Load 0 into the Y register.
    	// 2. Copy value from Y register into A register.
    	// 3. Break.
    	cpu.load_and_run(vec![0xa0, 0x00, 0x98, 0x00]);

    	// Check the A register has the expected value.
    	assert_eq!(cpu.a, 0x00);
        
        // Check the processor status is expected:
        // - Check the Zero Flag is set.
        // - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0010);
    }

    // -------- DEX --------

    #[test]
    fn test_dex_happy_path() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

    	// Load and run a short program.
    	// 1. Load a positive value into the X register.
    	// 2. Decrement X.
    	// 3. Break
    	cpu.load_and_run(vec![0xa2, 0x34, 0xca, 0x00]);

    	// Check that the X register has the expected value.
    	assert_eq!(cpu.x, 0x33);

    	// Check that the processor status is expected.
    	// - Check the Zero Flag is not set.
    	// - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0000);
    }

    #[test]
    fn test_dex_negative_to_negative() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

    	// Load and run a short program.
    	// 1. Load an 8 bit value with the 8th bit set into the X register.
    	// 2. Decrement X.
    	// 3. Break
    	cpu.load_and_run(vec![0xa2, 0xf4, 0xca, 0x00]);

    	// Check that the X register has the expected value.
    	assert_eq!(cpu.x, 0xf3);

    	// Check that the processor status is expected.
    	// - Check the Zero Flag is not set.
    	// - Check the Negative Flag is set.
        assert!(cpu.p & 0b1000_0010 == 0b1000_0000);
    }

    #[test]
    fn test_dex_negative_to_zero() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

    	// Load and run a short program.
    	// 1. Load 0 into the X register.
    	// 2. Decrement X.
    	// 3. Break
    	cpu.load_and_run(vec![0xa2, 0x00, 0xca, 0x00]);

    	// Check that the X register has the expected value.
    	assert_eq!(cpu.x, 0xff);

    	// Check that the processor status is expected.
    	// - Check the Zero Flag is not set.
    	// - Check the Negative Flag is set.
        assert!(cpu.p & 0b1000_0010 == 0b1000_0000);
    }

    #[test]
    fn test_dex_positive_to_zero() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

    	// Load and run a short program.
    	// 1. Load 1 into the X register.
    	// 2. Decrement X.
    	// 3. Break
    	cpu.load_and_run(vec![0xa2, 0x01, 0xca, 0x00]);

    	// Check that the X register has the expected value.
    	assert_eq!(cpu.x, 0x00);

    	// Check that the processor status is expected.
    	// - Check the Zero Flag is set.
    	// - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0010);
    }

    // -------- DEY --------

    #[test]
    fn test_dey_happy_path() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

    	// Load and run a short program.
    	// 1. Load a positive value into the Y register.
    	// 2. Decrement Y.
    	// 3. Break
    	cpu.load_and_run(vec![0xa0, 0x34, 0x88, 0x00]);

    	// Check that the X register has the expected value.
    	assert_eq!(cpu.y, 0x33);

    	// Check that the processor status is expected.
    	// - Check the Zero Flag is not set.
    	// - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0000);
    }

	#[test]
    fn test_dey_negative_to_negative() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

    	// Load and run a short program.
    	// 1. Load an 8 bit value with the 8th bit set into the Y register.
    	// 2. Decrement Y.
    	// 3. Break
    	cpu.load_and_run(vec![0xa0, 0xf4, 0x88, 0x00]);

    	// Check that the Y register has the expected value.
    	assert_eq!(cpu.y, 0xf3);

    	// Check that the processor status is expected.
    	// - Check the Zero Flag is not set.
    	// - Check the Negative Flag is set.
        assert!(cpu.p & 0b1000_0010 == 0b1000_0000);
    }

    #[test]
    fn test_dey_negative_to_zero() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

    	// Load and run a short program.
    	// 1. Load 0 into the Y register.
    	// 2. Decrement Y.
    	// 3. Break
    	cpu.load_and_run(vec![0xa0, 0x00, 0x88, 0x00]);

    	// Check that the Y register has the expected value.
    	assert_eq!(cpu.y, 0xff);

    	// Check that the processor status is expected.
    	// - Check the Zero Flag is not set.
    	// - Check the Negative Flag is set.
        assert!(cpu.p & 0b1000_0010 == 0b1000_0000);
    }

    #[test]
    fn test_dey_positive_to_zero() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

    	// Load and run a short program.
    	// 1. Load 1 into the Y register.
    	// 2. Decrement Y.
    	// 3. Break
    	cpu.load_and_run(vec![0xa0, 0x01, 0x88, 0x00]);

    	// Check that the Y register has the expected value.
    	assert_eq!(cpu.y, 0x00);

    	// Check that the processor status is expected.
    	// - Check the Zero Flag is set.
    	// - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0010);
    }

    // -------- INX --------

    #[test]
    fn test_inx_happy_path() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

    	// Load and run a short program.
    	// 1. Load a positive value into the X register.
    	// 2. Increment X.
    	// 3. Break
    	cpu.load_and_run(vec![0xa2, 0x34, 0xe8, 0x00]);

    	// Check that the X register has the expected value.
    	assert_eq!(cpu.x, 0x35);

    	// Check that the processor status is expected.
    	// - Check the Zero Flag is not set.
    	// - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0000);
    }

	#[test]
    fn test_inx_negative_to_negative() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

    	// Load and run a short program.
    	// 1. Load an 8 bit value with the 8th bit set into the X register.
    	// 2. Increment X.
    	// 3. Break
    	cpu.load_and_run(vec![0xa2, 0xf4, 0xe8, 0x00]);

    	// Check that the X register has the expected value.
    	assert_eq!(cpu.x, 0xf5);

    	// Check that the processor status is expected.
    	// - Check the Zero Flag is not set.
    	// - Check the Negative Flag is set.
        assert!(cpu.p & 0b1000_0010 == 0b1000_0000);
    }

    #[test]
    fn test_inx_negative_to_zero() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

    	// Load and run a short program.
    	// 1. Load the max value into the X register.
    	// 2. Increment X.
    	// 3. Break
    	cpu.load_and_run(vec![0xa2, 0xff, 0xe8, 0x00]);

    	// Check that the X register has the expected value.
    	assert_eq!(cpu.x, 0x00);

    	// Check that the processor status is expected.
    	// - Check the Zero Flag is set.
    	// - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0010);
    }

    #[test]
    fn test_inx_zero_to_positive() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

    	// Load and run a short program.
    	// 1. Load 0 into the X register.
    	// 2. Increment X.
    	// 3. Break
    	cpu.load_and_run(vec![0xa2, 0x00, 0xe8, 0x00]);

    	// Check that the X register has the expected value.
    	assert_eq!(cpu.x, 0x01);

    	// Check that the processor status is expected.
    	// - Check the Zero Flag is not set.
    	// - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0000);
    }

    // -------- INY --------

    #[test]
    fn test_iny_happy_path() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

    	// Load and run a short program.
    	// 1. Load a positive value into the Y register.
    	// 2. Increment Y.
    	// 3. Break
    	cpu.load_and_run(vec![0xa0, 0x34, 0xc8, 0x00]);

    	// Check that the X register has the expected value.
    	assert_eq!(cpu.y, 0x35);

    	// Check that the processor status is expected.
    	// - Check the Zero Flag is not set.
    	// - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0000);
    }

    #[test]
    fn test_iny_negative_to_negative() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

    	// Load and run a short program.
    	// 1. Load an 8 bit value with the 8th bit set into the Y register.
    	// 2. Increment Y.
    	// 3. Break
    	cpu.load_and_run(vec![0xa0, 0xf4, 0xc8, 0x00]);

    	// Check that the Y register has the expected value.
    	assert_eq!(cpu.y, 0xf5);

    	// Check that the processor status is expected.
    	// - Check the Zero Flag is not set.
    	// - Check the Negative Flag is set.
        assert!(cpu.p & 0b1000_0010 == 0b1000_0000);
    }

    #[test]
    fn test_iny_negative_to_zero() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

    	// Load and run a short program.
    	// 1. Load the max value into the Y register.
    	// 2. Increment Y.
    	// 3. Break
    	cpu.load_and_run(vec![0xa0, 0xff, 0xc8, 0x00]);

    	// Check that the Y register has the expected value.
    	assert_eq!(cpu.y, 0x00);

    	// Check that the processor status is expected.
    	// - Check the Zero Flag is set.
    	// - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0010);
    }

    #[test]
    fn test_iny_zero_to_positive() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

    	// Load and run a short program.
    	// 1. Load 0 into the Y register.
    	// 2. Increment Y.
    	// 3. Break
    	cpu.load_and_run(vec![0xa0, 0x00, 0xc8, 0x00]);

    	// Check that the Y register has the expected value.
    	assert_eq!(cpu.y, 0x01);

    	// Check that the processor status is expected.
    	// - Check the Zero Flag is not set.
    	// - Check the Negative Flag is not set.
        assert!(cpu.p & 0b1000_0010 == 0b0000_0000);
    }


    // -------- CLC --------

    // TODO: Add a proper program to check as well. This one is a bit hacky.
    #[test]
    fn test_clc_happy_path() {
    	// Create a CPU.
    	let mut cpu = CPU::new();
    	cpu.reset();

    	// Set the processor flag to have every bit set.
    	cpu.p = 0b1111_1111;
    	
    	// Load and run a short program.
    	// 1. Clear Carry bit.
    	// 2. Break
    	cpu.load(vec![0x18, 0x00]);
		cpu.pc = cpu.mem_read_u16(0xFFFC);
    	cpu.run();

    	// Check that the processor status is expected.
    	// - Carry bit is not set.
    	// - All other bits are set.
        assert!(cpu.p & 0b1111_1111 == 0b1111_1110);   	
    }

    // -------- CLD --------

	// TODO: Add a proper program to check as well. This one is a bit hacky.
    #[test]
    fn test_cld_happy_path() {
    	// Create a CPU.
    	let mut cpu = CPU::new();
    	cpu.reset();

    	// Set the processor flag to have every bit set.
    	cpu.p = 0b1111_1111;
    	
    	// Load and run a short program.
    	// 1. Clear Decimal bit.
    	// 2. Break
    	cpu.load(vec![0xd8, 0x00]);
		cpu.pc = cpu.mem_read_u16(0xFFFC);
    	cpu.run();

    	// Check that the processor status is expected.
    	// - Decimal bit is not set.
    	// - All other bits are set.
        assert!(cpu.p & 0b1111_1111 == 0b1111_0111);   	
    }

    // -------- CLV --------

    // TODO: Add a proper program to check as well. This one is a bit hacky.
    #[test]
    fn test_clv_happy_path() {
    	// Create a CPU.
    	let mut cpu = CPU::new();
    	cpu.reset();

    	// Set the processor flag to have every bit set.
    	cpu.p = 0b1111_1111;
    	
    	// Load and run a short program.
    	// 1. Clear Overflow bit.
    	// 2. Break
    	cpu.load(vec![0xb8, 0x00]);
		cpu.pc = cpu.mem_read_u16(0xFFFC);
    	cpu.run();

    	// Check that the processor status is expected.
    	// - Decimal bit is not set.
    	// - All other bits are set.
        assert!(cpu.p & 0b1111_1111 == 0b1011_1111);   	
    }

    // -------- SEC --------

    #[test]
    fn test_sec_happy_path() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

    	// Load and run a short program.
    	// 1. Set Carry bit.
    	// 2. Break
    	cpu.load_and_run(vec![0x38, 0x00]);

    	// Check that the processor status is expected.
    	// - Decimal bit is not set.
    	// - All other bits are set.
        assert!(cpu.p & 0b1111_1111 == 0b0000_0001);   	
    }

    // -------- SED --------
    
    #[test]
    fn test_sed_happy_path() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

    	// Load and run a short program.
    	// 1. Set Decimal bit.
    	// 2. Break
    	cpu.load_and_run(vec![0xf8, 0x00]);

    	// Check that the processor status is expected.
    	// - Decimal bit is not set.
    	// - All other bits are set.
        assert!(cpu.p & 0b1111_1111 == 0b0000_1000);   	
    }

    // -------- TXS --------

    #[test]
    fn test_txs_happy_path() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

    	// Load and run a short program.
    	// 1. Load X with a value.
    	// 2. Transfer the value of X to the stack pointer.
    	// 3. Break.
    	cpu.load_and_run(vec![0xa2, 0x34, 0x9a, 0x00]);

    	// Check that the x register is expected.
    	assert_eq!(cpu.x, 0x34);

    	// Check that the s register is expected.
    	assert_eq!(cpu.s, 0x34);

    	// Check that the processor status is expected.
    	// None of the bits are set.
        assert!(cpu.p & 0b1111_1111 == 0b0000_0000);   	
    }

    // -------- TSX --------

	#[test]
    fn test_tsx_happy_path() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

    	// Load and run a short program.
    	// 1. Transfer the value of the stack pointer to the x register.
    	// 2. Break.
    	cpu.load_and_run(vec![0xba, 0x00]);

    	// Check that the x register is expected.
    	assert_eq!(cpu.x, 0xfd);

    	// Check that the s register is expected.
    	assert_eq!(cpu.s, 0xfd);

    	// Check that the processor status is expected.
    	// The negative bit is set.
    	// None of the other bits are set.
        assert!(cpu.p & 0b1111_1111 == 0b1000_0000);
    }

    // -------- PHA --------

	#[test]
	fn test_pha_happy_path() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

    	// Load and run a short program.
    	// 1. Load A with a value.
    	// 2. Push the value of A onto the stack.
    	// 3. Break.
    	cpu.load_and_run(vec![0xa9, 0x34, 0x48, 0x00]);

    	// Check that the A register is expected.
    	assert_eq!(cpu.a, 0x34);

    	// Check that the s register is expected.
    	assert_eq!(cpu.s, 0xfc);

    	// Check that the stack has a value pushed onto it.
    	assert_eq!(cpu.mem[0x01FD], 0x34);

    	// Check that the processor status is expected.
    	// None of the bits are set.
        assert!(cpu.p & 0b1111_1111 == 0b0000_0000);
    }

    // -------- PLA --------

	#[test]
	fn test_pla_happy_path() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

    	// Load and run a short program.
    	// 1. Load A with a value.
    	// 2. Push the value of A onto the stack.
    	// 3. Load 0 into A to reset A.
    	// 4. Pop the value onto A.
    	// 5. Break.
    	cpu.load_and_run(vec![0xa9, 0x34, 0x48, 0xa9, 0x00, 0x68, 0x00]);

    	// Check that the A register is expected.
    	assert_eq!(cpu.a, 0x34);

    	// Check that the s register is expected.
    	assert_eq!(cpu.s, 0xfd);

    	// Check that the stack has a value pushed onto it.
    	assert_eq!(cpu.mem[0x01FD], 0x34);

    	// Check that the processor status is expected.
    	// None of the bits are set.
        assert!(cpu.p & 0b1111_1111 == 0b0000_0000);
    }

	#[test]
	fn test_pla_negative() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

    	// Load and run a short program.
    	// 1. Load A with a negative value.
    	// 2. Push the value of A onto the stack.
    	// 3. Load 0 into A to reset A.
    	// 4. Pop the value onto A.
    	// 5. Break.
    	cpu.load_and_run(vec![0xa9, 0xf6, 0x48, 0xa9, 0x00, 0x68, 0x00]);

    	// Check that the A register is expected.
    	assert_eq!(cpu.a, 0xf6);

    	// Check that the s register is expected.
    	assert_eq!(cpu.s, 0xfd);

    	// Check that the stack has a value pushed onto it.
    	assert_eq!(cpu.mem[0x01FD], 0xf6);

    	// Check that the processor status is expected.
    	// - Check the Negative Flag is set.
    	// - None of the bits are set.
        assert!(cpu.p & 0b1111_1111 == 0b1000_0000);
    }

	#[test]
    fn test_pla_zero() {
    	// Create a CPU.
    	let mut cpu = CPU::new();

    	// Load and run a short program.
    	// 1. Load A with a zero value.
    	// 2. Push the value of A onto the stack.
    	// 3. Load positive into A to reset A.
    	// 4. Pop the value onto A.
    	// 5. Break.
    	cpu.load_and_run(vec![0xa9, 0x00, 0x48, 0xa9, 0x23, 0x68, 0x00]);

    	// Check that the A register is expected.
    	assert_eq!(cpu.a, 0x00);

    	// Check that the s register is expected.
    	assert_eq!(cpu.s, 0xfd);

    	// Check that the stack has a value pushed onto it.
    	assert_eq!(cpu.mem[0x01FD], 0x00);

    	// Check that the processor status is expected.
    	// - Check the Zero Flag is set.
    	// - None of the other bits are set.
        assert!(cpu.p & 0b1111_1111 == 0b0000_0010);
    }

    // -------- PHP --------

	#[test]
    fn test_php_happy_path() {
		// create a cpu
    	let mut cpu = CPU::new();

		// Load and run a short program.
    	// 1. Push the processor status flags onto the stack.
    	// 2. Break.
    	cpu.load_and_run(vec![0x08, 0x00]);

    	// Check that the p register is expected.
    	assert_eq!(cpu.p, 0b0000_0000);

    	// Check that the stack pointer is expected.
    	assert_eq!(cpu.s, 0xFC);

    	// Check that the stack content is expected
    	assert_eq!(cpu.mem[0x01FD], 0b0011_0000);
	}

	#[test]
    fn test_php_negative() {
		// create a cpu
    	let mut cpu = CPU::new();

		// Load and run a short program.
		// 1. Load a negative value into A.
    	// 2. Push the processor status flags onto the stack.
    	// 3. Break.
    	cpu.load_and_run(vec![0xa9, 0xf1, 0x08, 0x00]);

    	// Check that the p register is expected.
    	assert_eq!(cpu.p, 0b1000_0000);

    	// Check that the stack pointer is expected.
    	assert_eq!(cpu.s, 0xFC);

    	// Check that the stack content is expected
    	assert_eq!(cpu.mem[0x01FD], 0b1011_0000);
	}

	#[test]
    fn test_php_zero() {
		// create a cpu
    	let mut cpu = CPU::new();

		// Load and run a short program.
		// 1. Load a zero value into A.
    	// 2. Push the processor status flags onto the stack.
    	// 3. Break.
    	cpu.load_and_run(vec![0xa9, 0x00, 0x08, 0x00]);

    	// Check that the p register is expected.
    	assert_eq!(cpu.p, 0b0000_0010);

    	// Check that the stack pointer is expected.
    	assert_eq!(cpu.s, 0xFC);

    	// Check that the stack content is expected
    	assert_eq!(cpu.mem[0x01FD], 0b0011_0010);
	}

    // -------- PLP --------

	#[test]
    fn test_plp_happy_path() {
		// create a cpu
    	let mut cpu = CPU::new();

		// Load and run a short program.
    	// 1. Push the processor status flags onto the stack.
    	// 2. Pull the processor status flags off the stack.
    	// 3. Break.
    	cpu.load_and_run(vec![0x08, 0x28, 0x00]);

    	// Check that the p register is expected.
    	assert_eq!(cpu.p, 0b0010_0000);

    	// Check that the stack pointer is expected.
    	assert_eq!(cpu.s, 0xFD);

    	// Check that the stack content is expected
    	assert_eq!(cpu.mem[0x01FD], 0b0011_0000);
	}

	#[test]
    fn test_plp_negative() {
		// create a cpu
    	let mut cpu = CPU::new();

		// Load and run a short program.
		// 1. Load a negative value into A.
    	// 2. Push the processor status flags onto the stack.
    	// 3. Pull the processor status flags off the stack.
    	// 4. Break.
    	cpu.load_and_run(vec![0xa9, 0xf1, 0x08, 0x28, 0x00]);

    	// Check that the p register is expected.
    	assert_eq!(cpu.p, 0b1010_0000);

    	// Check that the stack pointer is expected.
    	assert_eq!(cpu.s, 0xFD);

    	// Check that the stack content is expected
    	assert_eq!(cpu.mem[0x01FD], 0b1011_0000);
	}

	#[test]
    fn test_plp_zero() {
		// create a cpu
    	let mut cpu = CPU::new();

		// Load and run a short program.
		// 1. Load a zero value into A.
    	// 2. Push the processor status flags onto the stack.
    	// 3. Pull the processor status flags off the stack.
    	// 4. Break.
    	cpu.load_and_run(vec![0xa9, 0x00, 0x08, 0x28, 0x00]);

    	// Check that the p register is expected.
    	assert_eq!(cpu.p, 0b0010_0010);

    	// Check that the stack pointer is expected.
    	assert_eq!(cpu.s, 0xFD);

    	// Check that the stack content is expected
    	assert_eq!(cpu.mem[0x01FD], 0b0011_0010);
	}
}