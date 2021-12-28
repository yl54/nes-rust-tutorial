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

	// P - Processor Status
	// 8 bit register represents 7 status flags.
	// Each one is toggled depending on operation.
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

		// Start the loop.
			// Get the ops code from the pc.

			// Increment the pc.

			// Execute based off of the ops code
				// Handle ops code 1

				// Handle ops code 2
	}
}
