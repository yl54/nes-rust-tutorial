// cpu struct
pub struct CPU {
	// --- Registers ---
	// P - Program counter

	// Stack Pointer

	// A - Accumulator

	// X - Index register X

	// Y - Index register Y

	// P - Processor Status
	
	// Memory
}

// cpu impl
impl CPU {
	// new
	pub fn new() -> Self {
		// init the fields to zero/default values
		CPU {

		}
	}

	// interpret 
	// We need a mutable reference to the self, since the
	// passed in value will change a register.
	pub fn interpret(&mut self, program: Vec<u8>) {
		// fill it in
	}
}
