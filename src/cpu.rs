// TODO: Implement this in another MR.
// Addressing Modes
	// List of Addressing Modes in Enum listing

// A Table-like reference to hold information about ops codes.
// TODO: Implement this in another MR.

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
	 * 7 N Negative
	 * 6 V Overflow
	 * 5 - (Expansion)
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


	// TODO: Consider having a handler object per ops code.
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
	// TODO: Implement this in another MR.

	// load will load the program's ROM into the designated memory space.
	// TODO: Implement this in another MR.

	// mem_read will read 2 bytes of whats at a memory position.
	// TODO: Implement this in another MR.

	// mem_write will write 2 bytes to a memory position.
	// TODO: Implement this in another MR.

	// mem_read_u16 will read 4 bytes of whats at a memory position.
	// This function assumes data is stored in little endian.
	// TODO: Implement this in another MR.

	// mem_write_u16 will write  4 bytes to a memory position.
	// This function assumes data is stored in little endian.
	// TODO: Implement this in another MR.

	// get_operand_address determines how an address should be read.
	// It is determined based off of the Addressing mode.
	// TODO: Implement this in another MR.

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
	pub fn interpret(&mut self, program: Vec<u8>) {
		// Set the pc to 0, the start point.
		self.pc = 0;

		// Start the loop.
		loop {
			// Get the ops code from the pc.
			// q: why convert from u16 to usize?
			let opscode = program[self.pc as usize];

			// Increment pc.
			self.pc += 1;

			// Execute based off of the ops code.
			match opscode {
				// Handle ops code LDA (0xA9).
				// LDA is Load Accumulator.
				0xA9 => {
					// Get the param input from the next instruction.
					let param = program[self.pc as usize];

					// Increment pc.
					self.pc += 1;

					// Fill the A register with the param.
					self.a = param;

					// ---- Change the Processor Status Flags based off of the new A value -----
					
					// Check if the A register is 0.
					if self.a == 0 {
						// If 0, set the zero flag to 1.
						self.p = self.p | 0b0000_0010;
					} else {
						// If not, set the zero flag to 0.
						self.p = self.p & 0b1111_1101;
					}

					// Check if the A register is less than 0.
					// q: I don't understand this fully.
					if self.a & 0b1000_0000 != 0 {
						// If < 0, set the negative flag to 1.
						self.p = self.p | 0b1000_0000;
					} else {
						// If >= 0, set the negative flag to 0.
						self.p = self.p & 0b0111_1111;
					}
				}

				// Handle ops code BRK (0x00).
				// BRK is the break command. It causes an
				// interrupt sequence. The program transfers control to the 
				// interrupt vector.
				0x00 => {
					return;
				}

				// Handle ops code 2.
				_ => {}
			}
		}
	}
}

/* Typically, unit tests are listed below the actual code in the file.
 * This is how rust works. It doesn't work if I put the test in a separate file.
 */

#[cfg(test)]
mod test {
    // import a super library?
    use super::*;

    // test LDA happy path
    #[test]
    fn test_LDA_happy_path() {
        // Create a CPU.
        let mut cpu = CPU::new();

        // Interpret a short program.
        // 1. Load a value into A register.
        // 2. Break.
        cpu.interpret(vec![0xa9, 0x05, 0x00]);

        // Check the A register has the expected value.
        assert_eq!(cpu.a, 0x05);

        // Check the Zero Flag is not set.
        // Q: What is the difference between 0b00 and 0? 
        assert!(cpu.p & 0b0000_0010 == 0b00);

        // Check the Negative Flag is set.
        assert!(cpu.p & 0b1000_0000 == 0);
    }


    // test LDA zero flag
        // Create a CPU.

        // Interpret a short program.
        // 1. Load zero into A register.
        // 2. Break.

        // Check the A register has the expected value.

        // Check the Zero Flag is set.

        // Check the Negative Flag is not set.
}