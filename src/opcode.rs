// import
use crate::addressing_mode::AddressingMode;

// opcode struct
pub struct OpCode {
	// code
	// This is the exact number that represents the opcode in 6502.
	pub code: u8,

	// mnemonic
	// This is the string representation of the op code.
	pub mnemonic: String,

	// len
	// this is the number of bytes read
	// this is used to figure out how much to increment the pc
	pub len: u8,

	// cycles
	// this is not really used, but we'll just have it here as a record
	pub cycles: u8,
	
	// addressing mode
	// This is the addressing mode enum associated with the op code
	pub mode: AddressingMode,
}

// impl
impl OpCode {
	// new function
	pub fn new(code: u8, mnemonic: String, len: u8, cycles: u8, mode: AddressingMode) -> Self {
		OpCode {
			code: code,
			mnemonic: mnemonic,
			len: len,
			cycles: cycles,
			mode: mode,
		}
	}
}

// lazy static inits
	// opcode table

	// function to create hash table