// Addressing Mode Enums
pub enum AddressingMode {
	// List of Addressing Modes in Enum listing

	// Immediate
	Immediate,

	// Zero Page
	// Lookup directly to first page of RAM
	// First page of RAM = first 256 bytes
	// Only the first page of RAM are accessible
	// Supposed to be somewhat fast, since only need to lookup 1 byte.
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
