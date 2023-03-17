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
		
		OpCode::new(0x9A, "TXS", 1, 2, AddressingMode::NoneAddressing),
		OpCode::new(0xBA, "TSX", 1, 2, AddressingMode::NoneAddressing),
		OpCode::new(0x48, "PHA", 1, 3, AddressingMode::NoneAddressing),
		OpCode::new(0x68, "PLA", 1, 4, AddressingMode::NoneAddressing),
		OpCode::new(0x08, "PHP", 1, 3, AddressingMode::NoneAddressing),
		OpCode::new(0x28, "PLP", 1, 4, AddressingMode::NoneAddressing),

		// Shift Instructions

		// ASL
		OpCode::new(0x0A, "ASL", 1, 2, AddressingMode::NoneAddressing),
		OpCode::new(0x06, "ASL", 2, 5, AddressingMode::ZeroPage),
		OpCode::new(0x16, "ASL", 2, 6, AddressingMode::ZeroPageX),
		OpCode::new(0x0E, "ASL", 3, 6, AddressingMode::Absolute),
		OpCode::new(0x1E, "ASL", 3, 7, AddressingMode::AbsoluteX),

		// LSR
		OpCode::new(0x4A, "LSR", 1, 2, AddressingMode::NoneAddressing),
		OpCode::new(0x46, "LSR", 2, 5, AddressingMode::ZeroPage),
		OpCode::new(0x56, "LSR", 2, 6, AddressingMode::ZeroPageX),
		OpCode::new(0x4E, "LSR", 3, 6, AddressingMode::Absolute),
		OpCode::new(0x5E, "LSR", 3, 7, AddressingMode::AbsoluteX),

		// ROL
		OpCode::new(0x2A, "ROL", 1, 2, AddressingMode::NoneAddressing),
		OpCode::new(0x26, "ROL", 2, 5, AddressingMode::ZeroPage),
		OpCode::new(0x36, "ROL", 2, 6, AddressingMode::ZeroPageX),
		OpCode::new(0x2E, "ROL", 3, 6, AddressingMode::Absolute),
		OpCode::new(0x3E, "ROL", 3, 7, AddressingMode::AbsoluteX),

		// ROR
		OpCode::new(0x6A, "ROR", 1, 2, AddressingMode::NoneAddressing),
		OpCode::new(0x66, "ROR", 2, 5, AddressingMode::ZeroPage),
		OpCode::new(0x76, "ROR", 2, 6, AddressingMode::ZeroPageX),
		OpCode::new(0x6E, "ROR", 3, 6, AddressingMode::Absolute),
		OpCode::new(0x7E, "ROR", 3, 7, AddressingMode::AbsoluteX),

		// CMP
		OpCode::new(0xC9, "CMP", 2, 2, AddressingMode::Immediate),
		OpCode::new(0xC5, "CMP", 2, 3, AddressingMode::ZeroPage),
		OpCode::new(0xD5, "CMP", 2, 4, AddressingMode::ZeroPageX),
		OpCode::new(0xCD, "CMP", 3, 4, AddressingMode::Absolute),
		OpCode::new(0xDD, "CMP", 3, 4, AddressingMode::AbsoluteX),
		OpCode::new(0xD9, "CMP", 3, 4, AddressingMode::AbsoluteY),
		OpCode::new(0xC1, "CMP", 2, 6, AddressingMode::IndirectX),
		OpCode::new(0xD1, "CMP", 2, 5, AddressingMode::IndirectY),

		// CPX
		OpCode::new(0xE0, "CPX", 2, 2, AddressingMode::Immediate),
		OpCode::new(0xE4, "CPX", 2, 3, AddressingMode::ZeroPage),
		OpCode::new(0xEC, "CPX", 3, 4, AddressingMode::Absolute),

		// CPY
		OpCode::new(0xC0, "CPY", 2, 2, AddressingMode::Immediate),
		OpCode::new(0xC4, "CPY", 2, 3, AddressingMode::ZeroPage),
		OpCode::new(0xCC, "CPY", 3, 4, AddressingMode::Absolute),

		// ORA

		// 0x09
		// immediate
		// 2
		// 2
		OpCode::new(0x09, "ORA", 2, 2, AddressingMode::Immediate),

		// 0x05
		// zero page
		// 2
		// 3
		OpCode::new(0x05, "ORA", 2, 3, AddressingMode::ZeroPage),

		// 0x15
		// zero page x
		// 2
		// 4
		OpCode::new(0x15, "ORA", 2, 4, AddressingMode::ZeroPageX),

		// 0x0D
		// absolute
		// 3
		// 4
		OpCode::new(0x0D, "ORA", 3, 4, AddressingMode::Absolute),

		// 0x1D
		// absolute x
		// 3
		// 4+
		OpCode::new(0x1D, "ORA", 3, 4, AddressingMode::AbsoluteX),

		// 0x19
		// absolute y
		// 3
		// 4+
		OpCode::new(0x19, "ORA", 3, 4, AddressingMode::AbsoluteY),

		// 0x01
		// indirect x
		// 2
		// 6
		OpCode::new(0x01, "ORA", 2, 6, AddressingMode::IndirectX),

		// 0x11
		// indirect y
		// 2
		// 5+
		OpCode::new(0x11, "ORA", 2, 5, AddressingMode::IndirectY),
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
