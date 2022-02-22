// import
use crate::addressing_mode::AddressingMode;
use std::collections::HashMap;

// opcode struct
pub struct OpCode {
	// code
	// This is the exact number that represents the opcode in 6502.
	pub code: u8,

	// mnemonic
	// This is the string representation of the op code.
	pub mnemonic: &'static str,

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
	pub fn new(code: u8, mnemonic: &'static str, len: u8, cycles: u8, mode: AddressingMode) -> Self {
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
lazy_static! {
	// opcode table
	pub static ref CPU_OP_CODES: Vec<OpCode> = vec![
		OpCode::new(0x00, "BRK", 1, 7, AddressingMode::NoneAddressing),

		// --------- Loads -----------

		OpCode::new(0xa9, "LDA", 2, 2, AddressingMode::Immediate),
		OpCode::new(0xa5, "LDA", 2, 3, AddressingMode::ZeroPage),

		OpCode::new(0xa2, "LDX", 2, 2, AddressingMode::Immediate),

		OpCode::new(0xa0, "LDY", 2, 2, AddressingMode::Immediate),

		// --------- Register Flags -----------

		OpCode::new(0xaa, "TAX", 1, 2, AddressingMode::NoneAddressing),
		OpCode::new(0xa8, "TAY", 1, 2, AddressingMode::NoneAddressing),
		OpCode::new(0x8a, "TXA", 1, 2, AddressingMode::NoneAddressing),
	];

	// function to create hash table from code to OpCode
	pub static ref OP_CODE_MAP: HashMap<u8, &'static OpCode> = {
		let mut map: HashMap<u8, &'static OpCode> = HashMap::new();
		for op_code in &*CPU_OP_CODES {
			map.insert(op_code.code, op_code);
		}
		map 
	};
}
