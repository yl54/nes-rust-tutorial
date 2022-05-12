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
		OpCode::new(0xea, "NOP", 1, 2, AddressingMode::NoneAddressing),

		// --------- Processor Status Instructions ----------

		OpCode::new(0x18, "CLC", 1, 2, AddressingMode::NoneAddressing),
		OpCode::new(0xd8, "CLD", 1, 2, AddressingMode::NoneAddressing),
		OpCode::new(0xb8, "CLV", 1, 2, AddressingMode::NoneAddressing),

		OpCode::new(0x38, "SEC", 1, 2, AddressingMode::NoneAddressing),
		OpCode::new(0xf8, "SED", 1, 2, AddressingMode::NoneAddressing),

		// --------- Loads -----------

		OpCode::new(0xa9, "LDA", 2, 2, AddressingMode::Immediate),
		OpCode::new(0xa5, "LDA", 2, 3, AddressingMode::ZeroPage),
		OpCode::new(0xad, "LDA", 3, 4, AddressingMode::Absolute),
		OpCode::new(0xb5, "LDA", 2, 4, AddressingMode::ZeroPageX),
		OpCode::new(0xbd, "LDA", 3, 4, AddressingMode::AbsoluteX),
		OpCode::new(0xb9, "LDA", 3, 4, AddressingMode::AbsoluteY),
		OpCode::new(0xa1, "LDA", 2, 6, AddressingMode::IndirectX),
		OpCode::new(0xb1, "LDA", 2, 5, AddressingMode::IndirectY),

		OpCode::new(0xa2, "LDX", 2, 2, AddressingMode::Immediate),
		OpCode::new(0xa6, "LDX", 2, 3, AddressingMode::ZeroPage),
		OpCode::new(0xb6, "LDX", 2, 4, AddressingMode::ZeroPageY),
		OpCode::new(0xae, "LDX", 3, 4, AddressingMode::Absolute),
		OpCode::new(0xbe, "LDX", 3, 4, AddressingMode::AbsoluteY),

		OpCode::new(0xa0, "LDY", 2, 2, AddressingMode::Immediate),
		OpCode::new(0xa4, "LDY", 2, 3, AddressingMode::ZeroPage),
		OpCode::new(0xb4, "LDY", 2, 4, AddressingMode::ZeroPageX),
		OpCode::new(0xac, "LDY", 3, 4, AddressingMode::Absolute),
		OpCode::new(0xbc, "LDY", 3, 4, AddressingMode::AbsoluteX),


		// --------- Register Flags -----------

		OpCode::new(0xaa, "TAX", 1, 2, AddressingMode::NoneAddressing),
		OpCode::new(0xa8, "TAY", 1, 2, AddressingMode::NoneAddressing),
		OpCode::new(0x8a, "TXA", 1, 2, AddressingMode::NoneAddressing),
		OpCode::new(0x98, "TYA", 1, 2, AddressingMode::NoneAddressing),
		OpCode::new(0xca, "DEX", 1, 2, AddressingMode::NoneAddressing),
		OpCode::new(0x88, "DEY", 1, 2, AddressingMode::NoneAddressing),
		OpCode::new(0xe8, "INX", 1, 2, AddressingMode::NoneAddressing),
		OpCode::new(0xc8, "INY", 1, 2, AddressingMode::NoneAddressing),

		// STA
		OpCode::new(0x85, "STA", 2, 3, AddressingMode::ZeroPage),
		OpCode::new(0x95, "STA", 2, 4, AddressingMode::ZeroPageX),
		OpCode::new(0x8d, "STA", 3, 4, AddressingMode::Absolute),
		OpCode::new(0x9d, "STA", 3, 5, AddressingMode::AbsoluteX),
		OpCode::new(0x99, "STA", 3, 5, AddressingMode::AbsoluteY),

		// STX
		OpCode::new(0x86, "STX", 2, 3, AddressingMode::ZeroPage),
		OpCode::new(0x96, "STX", 2, 4, AddressingMode::ZeroPageY),
		OpCode::new(0x8e, "STX", 3, 4, AddressingMode::Absolute),

		// STY
		OpCode::new(0x84, "STY", 2, 3, AddressingMode::ZeroPage),
		OpCode::new(0x94, "STY", 2, 4, AddressingMode::ZeroPageX),
		OpCode::new(0x8C, "STY", 3, 4, AddressingMode::Absolute),

		// --------- Stack Instructions -----------
		
		// TXS
		OpCode::new(0x9A, "TXS", 1, 2, AddressingMode::NoneAddressing),

		// TSX
		OpCode::new(0xBA, "TSX", 1, 2, AddressingMode::NoneAddressing),

		// PHA
		// PLA
		// PHP
		// PLP

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
