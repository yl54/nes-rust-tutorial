 // TODO: Implement this in another MR.
// Addressing Modes
pub enum AddressingMode {
	// List of Addressing Modes in Enum listing

	// Immediate
	Immediate,

	// Zero Page
	ZeroPage,

	// Zero Page X
	ZeroPageX,

	// Zero Page Y
	ZeroPageY,

	// Absolute
	Absolute,

	// Absolute X
	AbsoluteX,

	// Absolute Y
	AbsoluteY,

	// Indirect X
	IndirectX,

	// Indirect Y
	IndirectY,

	// None Addressing
	NoneAddressing,
}

// A Table-like reference to hold information about ops codes.
// TODO: Implement this in another MR.

// Q: Is it worth having a handler object per ops code?
// A: Probably not. The logic is too interwined with the CPU.

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

	// mem_read_u16 will read 4 bytes of whats at 2 memory positions.
	// This function assumes data is stored in little endian.
	fn mem_read_u16(&mut self, addr: u16) -> u16 {
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

	// get_operand_address determines how an address should be read.
	// It is determined based off of the Addressing mode.
	// TODO: Implement this in another MR.
		// Check what mode is passed in.

			// Immediate
				// Return the current value of the pc.

			// Zero Page
				// Read the value stored on 1 address.
				// The value of the address is the value of the pc.

			// Absolute
				// Read the value stored on 2 adjacent addresses.
				// The value of the first address is the value of the pc. 

			// Zero Page X
				// Read 1 value stored on 1 address and add value of x to it.
				// The value of the address is the value of the pc.
				// These are added together. If the value overflows the available byte space, it will restart from 0.
				
			// Zero Page Y
				// Read 1 value stored on 1 address and add value of y to it.
				// The value of the address is the value of the pc.
				// These are added together. If the value overflows the available byte space, it will restart from 0.

			// Absolute X
				// Read the value stored on 2 adjacent address and add value of x to it.
				// The value of the first address is the value of the pc.
				// These are added together. If the value overflows the available byte space, it will restart from 0.

			// Absolute Y
				// Read the value stored on 2 adjacent address and add value of y to it.
				// The value of the first address is the value of the pc.
				// These are added together. If the value overflows the available byte space, it will restart from 0.

			// Indirect X
				// Read the value stored on 1 address.
				// The value of the address is the value of the pc.
				
				// Add x to the value. If the value overflows the available byte space, it will restart from 0. This will be the pointer.

				// Read the low value stored on the pointer.

				// Read the high value stored on the pointer.

				// Put the high value on the lower end, and low value on the higher end.
				// This is a little endian.
				// Return this computation.


			// Indirect Y
				// Read the value stored on 1 address.
				// The value of the address is the value of the pc.
				
				// Add y to the value. If the value overflows the available byte space, it will restart from 0. This will be the pointer.

				// Read the low value stored on the pointer.

				// Read the high value stored on the pointer.

				// Put the high value on the lower end, and low value on the higher end.
				// This is a little endian.
				// Return this computation.

			// None Addressing
				// Not supported.

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
			match opscode {
				// Handle ops code LDA (0xA9).
				0xA9 => {
					// Get the param input from the next instruction.
					let param = self.mem_read(self.pc);

					// Increment pc.
					self.pc += 1;

					self.lda(param);
				}

				// Handle ops code TAX (0xAA)
				0xAA => self.tax(),

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

	// -------- Handle Opscodes --------

	// lda handles ops code LDA (0xA9).
	// LDA is Load Accumulator.
	fn lda(&mut self, value: u8) {
		// Fill the A register with the param.
		self.a = value;

		// Change the Processor Status Flags based off of the new A value
		self.update_processor_flags(self.a);
	}

	// tax handles the ops code TAX (0xAA).
	// TAX copies the value from the A register to the X register.
	fn tax(&mut self) {
		// Copy the value from A register into the X register.
		self.x = self.a;

		// Change the Processor Status Flags based off of the new X value
		self.update_processor_flags(self.x);
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

   	// Immediate

	// Zero Page
			
	// Absolute
			
	// Zero Page X
			
	// Zero Page Y
			
	// Absolute X
				
	// Absolute Y
				
	// Indirect X

	// Indirect Y
			
	// None Addressing

    // -------- LDA --------

    #[test]
    fn test_lda_happy_path() {
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
    fn test_lda_negative_input() {
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
    fn test_lda_zero() {
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
}