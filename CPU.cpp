#include "CPU.h"

//Helper function to convert a 32 bitset with 2s complement representation into a 32 bit signed int 
int32_t bitsetConverter(const bitset<32>& bits) {
    if (bits[31] == 1) { // Check the MSB for neg. sign (31st index for 32 bits)
        // Compute the two's complement
        return static_cast<int32_t>(bits.to_ulong()) - (1 << 32);
    }
	else
	{
    	return static_cast<int32_t>(bits.to_ulong());
	}
}

///Helper function to sign extend size 8 bitset into 32 bit signed integer, for SB/LB purposes
int32_t signExtendBitset(const bitset<8>& bits) {
    int32_t value = static_cast<int32_t>(bits.to_ulong());
    // Check if the sign bit (bit 7) is set
    if (bits[7]) {
        // If bit 7 is 1, perform sign extension
        value |= 0xFFFFFF00;  // Set the upper 24 bits to 1
    }
    return value;
}

CPU::CPU()
{
	PC = 0; //set PC to 0
	for (int i = 0; i < 4096; i++) //default constructor, fill with 0s
	{
		instMem[i] = (0);
	}
	// Initialize dmemory to 0
	for (int i = 0; i < 4096; ++i) {
		dmemory[i] = 0b00000000; // Set each bitset to zero
	}

	for (int i = 0; i < 32; ++i) {
		registers[i] = 0; 
	}
}

CPU::CPU(bitset<8>* instr_array)
{
	PC = 0; //set PC to 0 
	//copy instrMEM
	for (int i = 0; i < 4096; i++)
	{
		instMem[i] = instr_array[i];
	}
	// Initialize dmemory to 0
	for (int i = 0; i < 4096; ++i) {
		dmemory[i] = 0b00000000; // Set each bitset to zero
	}

	for (int i = 0; i < 32; ++i) {
		registers[i] = 0; 
	}
}

unsigned long CPU::readPC()
{
	return PC;
}

//normal PC next step 
void CPU::incPC()
{
	PC+=4;
}

//Make sure your on the current and right instruction 
void CPU::instrFetch()
{
	instruction = (instMem[PC+3].to_ullong() << 24) | (instMem[PC + 2].to_ullong() << 16) | (instMem[PC + 1].to_ullong() << 8) | (instMem[PC].to_ullong());
}

void CPU::instrDecode()
{
	//R-type segmentation
	opcode = (instruction.to_ulong() & 0x7F);
	rd = (instruction.to_ulong() & 0xf80) >> 7; 
	funct3 = (instruction.to_ulong() & 0x7000) >> 12;
	rs1 = ((instruction.to_ulong() >> 15) & 0x1f);
	rs2 = ((instruction.to_ulong() >> 20) & 0x1f);
	funct7 = ((instruction.to_ulong() >> 25) & 0x7F);
}

void CPU::ControlPass()
{
	//R type instr
	if(opcode == bitset<7>(0b0110011))
	{
		instr_type = 'R'; 
		control.branch_bit = 0; 
		control.memRead_bit = 0; 
		control.memToReg_bit = 0; 
		//r-type instr opcode for aluop
		control.aluop_bit = bitset<2>(0b10);
		control.memwrite_bit = 0;  
		control.alusrc_bit = 0;  
		control.regwrite_bit = 1; 

		control.upper_imm = 0; 
	}
	//I-type
	else if(opcode == bitset<7>(0b0010011))
	{
		instr_type = 'I'; 
		control.branch_bit = 0; 
		control.memRead_bit = 0; 
		control.memToReg_bit = 0; 
		//i-type instr opcode for aluop
		control.aluop_bit = bitset<2>(0b11);
		control.memwrite_bit = 0;  
		control.alusrc_bit = 1;  
		control.regwrite_bit = 1; 

		control.upper_imm = 0; 
	}
	//load instr
	else if(opcode == bitset<7>(0b0000011)){ 
		instr_type = 'L'; 
		control.branch_bit = 0; 
		control.memRead_bit = 1; 
		control.memToReg_bit = 1; 
		//load instr opcode for aluop
		control.aluop_bit = bitset<2>(0b00);
		control.memwrite_bit = 0;  
		control.alusrc_bit = 1;  
		control.regwrite_bit = 1; 

		control.upper_imm = 0; 
	}
	//store instr
	else if(opcode == bitset<7>(0b0100011)){ 
		instr_type = 'S'; 
		control.branch_bit = 0; 
		control.memRead_bit = 0; 
		control.memToReg_bit = 0; 
		//store instr opcode for aluop
		control.aluop_bit = bitset<2>(0b00);
		control.memwrite_bit = 1;  
		control.alusrc_bit = 1;  
		control.regwrite_bit = 0; 

		control.upper_imm = 0; 
	}
	//beq instr
	else if(opcode == bitset<7>(0b1100011)){ 
		instr_type = 'B';  
		control.branch_bit = 1; 
		control.memRead_bit = 0; 
		control.memToReg_bit = 0; 
		//beq aluop, subtract rs1 from rs2 and see if 0 value is produced (which means true and proceed with jump)
		control.aluop_bit = bitset<2>(0b01);
		control.memwrite_bit = 0;  
		control.alusrc_bit = 0;  
		control.regwrite_bit = 0; 

		control.upper_imm = 0; 
	}

	//jump instr 
	else if(opcode == bitset<7>(0b1101111)){ 
		instr_type = 'J'; 
		control.branch_bit = 1; 
		control.memRead_bit = 0; 
		control.memToReg_bit = 0; 
		//aluop OMITTED (doesn't matter, want to just jump directly to given immediate while storing pc+4 into target register)
		control.aluop_bit = bitset<2>(0b00);
		control.memwrite_bit = 0;  
		control.alusrc_bit = 0;  
		control.regwrite_bit = 1; 

		control.upper_imm = 0; 
	}

	//LUI instr 
	else if(opcode == bitset<7>(0b0110111)){ 
		instr_type = 'U'; 
		control.branch_bit = 0; 
		control.memRead_bit = 0; 
		control.memToReg_bit = 0; 
		//aluop bit OMITTED (doesn't matter, want to just load upper imm to register)
		control.aluop_bit = bitset<2>(0b00); 
		control.memwrite_bit = 0;  
		control.alusrc_bit = 0;  
		control.regwrite_bit = 1; 
		//do nothing on every compute unit in the CPU, but make sure to extract 20 bits using mask below and write the result to the register before we move to next instr
		control.upper_imm = 1; 
	}
}

//convert bitset to unsigned integer, access from register based on u_int index 
void CPU::regreads()
{
	reg1_value = registers[rs1.to_ulong()]; 
	reg2_value = registers[rs2.to_ulong()]; 
}

//write to given register if regwrite_bit is active
void CPU::regwrite()
{
	//don't write to 0th register as this is reserved, but write otherwise 
	if(control.regwrite_bit == 1 && (!(rd.to_ulong() == 0)))
	{ 
		registers[rd.to_ulong()] = regwrite_buffer; 
	}
}

//pad to 32 bits, extract bits based on instr_type
void CPU::set_immgen()
{
	//LUI, ORI, SRAI, JAL, BEQ, LB, LW, SB, SW 
	//LUI
	if(instr_type == 'U')
	{
		immgen_result = (instruction.to_ulong() & (0b11111111111111111111 << 12));
	}
	//ORI and SRAI 
	if(instr_type == 'I')
	{//differentiate between ORI/SRAI through funct3 field, ORI immediate need sign extension 
	//ORI
		if(funct3 == bitset<3>(0b110))
		{
			immgen_result = (instruction.to_ulong() & (0b111111111111 << 20)) >> 20;
			//sign extension (current size of 12 bits)
			int immt; 
			immt = (static_cast<int>(immgen_result.to_ulong()) << 20) >> 20; 
			immgen_result = bitset<32>(immt); 
		}
	//SRAI 
		else if(funct3 == bitset<3>(0b101))
		{
			immgen_result = (instruction.to_ulong() & (0b11111 << 20)) >> 20; 
		}
	}
	//JAL (immediate needs sign extension)
	if(instr_type == 'J')
	{
		bool bit20 = instruction.to_ulong() & (0b1 << 31);
		bool bit11 = instruction.to_ulong() & (0b1 << 20);
		bitset<8> eightbits((instruction.to_ulong() & (0b11111111 << 12)) >> 12); 
		bitset<10> tenbits((instruction.to_ulong() & (0b1111111111 << 21)) >> 21); 
		int immt; 
		//sign extension (current size of 21 bits)
		immgen_result = (bit20 << 20) | (bit11 << 11) | (eightbits.to_ulong() << 12) | (tenbits.to_ulong() << 1);
		immt = (static_cast<int>(immgen_result.to_ulong()) << 11) >> 11;
		immgen_result = bitset<32>(immt);

	}
	//BEQ (immediate needs sign extension)
	if(instr_type == 'B')
	{
		bool bit12 = instruction.to_ulong() & (0b1 << 31);
		bool bit11 = instruction.to_ulong() & (0b1 << 7);
		//set all bits to 0 
		immgen_result.reset(); 
		immgen_result |= bit11 << 11;
		immgen_result |= bit12 << 12;
		immgen_result |= ((instruction.to_ulong() & (0b1111<<8)) >> 8) << 1;        
		immgen_result |= ((instruction.to_ulong() & (0b111111<<25)) >> 25) << 5;    
		//sign extension (current size of 13 bits)
		int immt; 
		immt = (static_cast<int>(immgen_result.to_ulong()) << 19) >> 19; 
		immgen_result = bitset<32>(immt); 
	}
	//load instr (immediate needs sign extension)
	if(instr_type == 'L')
	{
		immgen_result = (instruction.to_ulong() & (0b111111111111 << 20)) >> 20;
		//sign extension (current size of 12 bits)
		int immt; 
		immt = (static_cast<int>(immgen_result.to_ulong()) << 20) >> 20; 
		immgen_result = bitset<32>(immt); 
	}
	//store instr (immediate needs sign extension)
	if(instr_type == 'S')
	{
		immgen_result = ((instruction.to_ulong() & (0b11111 << 7)) >> 7) | ((instruction.to_ulong() & (0b1111111 << 25)) >> 20);
		//sign extension (current size of 12 bits)
		int immt; 
		immt = (static_cast<int>(immgen_result.to_ulong()) << 20) >> 20; 
		immgen_result = bitset<32>(immt); 
	}

}

//take input from ALUOp and query table to output correct signal to ALU 
void CPU::set_aluControl()
{
	//ALUOp == 00, ld and sd
	//ALUOp == 01, beq should be subtract operation 
	//ALUOp == 10, consult R-type table for funct3/funct7 field 
	//AluOp == 11, consult I-type table for funct/funct7 field.
	const unsigned long ldsdpattern = 0b00; 
	const unsigned long beqpattern = 0b01; 
	const unsigned long rtypepattern = 0b10; 
	const unsigned long itypepattern = 0b11; 
	switch(control.aluop_bit.to_ulong())
	{
		case ldsdpattern: 
		//LB, LW, SB, SW instr 
			alu_control_input = bitset<4>(0b0010); 
			break;
		case beqpattern: 
		//BEQ instr 
			alu_control_input = bitset<4>(0b0110); 
			break;
		case rtypepattern:
		//consult r type table  
			//for now, we implement just the add for R-type
			if(funct3 == (bitset<3>(0b000)))
			{
				alu_control_input = bitset<4>(0b0010);  
			}
			//XOR instr 
			else if(funct3 == (bitset<3>(0b100)))
			{
				alu_control_input = bitset<4>(0b0011); 
			}
			break;
		case itypepattern:
		//consult I type table 
			//ORI instr 
			if(funct3 == (bitset<3>(0b110)))
			{
				alu_control_input = bitset<4>(0b0001); 
			}
			//SRAI instr 
			else if(funct3 == (bitset<3>(0b101)))
			{
				alu_control_input = bitset<4>(0b0111); 
			}
			break;
	}
}


int32_t CPU::alu_compute()
{
	//perform operations based on ALU Control Input
	//add 
	if(alu_control_input == bitset<4>(0b0010))
	{
		//if 1 in MUX, pass generated imm 
		if(control.alusrc_bit == 1)
		{
			//use 2s complement bitset converter for immgen_result and add 
			int32_t cleaned_immediate = bitsetConverter(immgen_result); 
			int32_t sum = reg1_value + cleaned_immediate; 
			//returning int 32 from ALU 
			return sum; 
		}
		//if 0, pass in data from rs2 
		else if(control.alusrc_bit == 0)
		{
			int32_t sum = reg1_value + reg2_value; 
			return sum; 
		}
	}

	//subtract 
	if(alu_control_input == bitset<4>(0b0110))
	{
		//if 1 in MUX, pass generated imm 
		if(control.alusrc_bit == 1)
		{
			int32_t cleaned_immediate = bitsetConverter(immgen_result); 
			int32_t output = reg1_value - cleaned_immediate; 
			return output; 
		}
		else if(control.alusrc_bit == 0)
		{
			int32_t output = reg1_value - reg2_value; 
			return output; 
		}
	}

	//ideally, should implement AND here 

	//OR
	if(alu_control_input == bitset<4>(0b0001))
	{
		//if 1 in MUX, pass generated imm 
		if(control.alusrc_bit == 1)
		{
			int32_t cleaned_immediate = bitsetConverter(immgen_result); 
			int32_t output = reg1_value | cleaned_immediate; 
			return output;  
		}
		else if(control.alusrc_bit == 0)
		{
			int32_t output = reg1_value | reg2_value; 
			return output; 
		}
	}

	//XOR 
	if(alu_control_input == bitset<4>(0b0011))
	{
		//if 1 in MUX, pass generated imm 
		if(control.alusrc_bit == 1)
		{
			int32_t cleaned_immediate = bitsetConverter(immgen_result); 
			int32_t output = reg1_value ^ cleaned_immediate; 
			return output; 
		}
		else if(control.alusrc_bit == 0)
		{
			int32_t output = reg1_value ^ reg2_value; 
			return output; 
		}
	}

	//SRAI 
	if(alu_control_input == bitset<4>(0b0111))
	{
		//if 1 in MUX, pass generated imm 
		if(control.alusrc_bit == 1)
		{
			int32_t cleaned_immediate = bitsetConverter(immgen_result); 
			int32_t output = reg1_value >> cleaned_immediate; 
			return output; 
		}
		else if(control.alusrc_bit == 0)
		{
			int32_t output = reg1_value >> reg2_value;
			return output; 
		}
	}

}


void CPU::datamemoryunit_compute(int32_t aluresult)
{
	//perform read from memory (load instr)
	if(control.memRead_bit == 1)
	{
		//loading byte (LB)
		if(funct3 == bitset<3>(0b000))
		{
			//need to sign extend byte 
			int32_t output = signExtendBitset(dmemory[aluresult]);
			regwrite_buffer = output;
		}
		//loading word (LW)
		else if(funct3 == bitset<3>(0b010))
		{
			unsigned long holder = (dmemory[aluresult].to_ulong()) | ((dmemory[aluresult+1].to_ulong()) << 8) | ((dmemory[aluresult+2].to_ulong()) << 16) | ((dmemory[aluresult+3].to_ulong()) << 24);
			regwrite_buffer = static_cast<int>(holder);
		}
	}

	//perform a write to memory (store instr)
	if(control.memwrite_bit == 1)
	{
		//storing byte (SB)
		if(funct3 == bitset<3>(0b000))
		{
			//reg2_value is of int32_t type
			bitset<32> bits(reg2_value);
			//extracting first byte of reg2_value
			bitset<8> first8bits(bits.to_string().substr(24,32));
			dmemory[aluresult] = first8bits; 
		}
		//storing word (SW)
		else if(funct3 == bitset<3>(0b010))
		{
			bitset<32> bits(reg2_value);
			//store 4 bytes 
			bitset<8> first8bits(bits.to_string().substr(24,32));
			bitset<8> second8bits(bits.to_string().substr(16,24));
			bitset<8> third8bits(bits.to_string().substr(8,16));
			bitset<8> fourth8bits(bits.to_string().substr(0,8));
			dmemory[aluresult] = first8bits; 
			dmemory[aluresult + 1] = second8bits; 
			dmemory[aluresult + 2] = third8bits; 
			dmemory[aluresult + 3] = fourth8bits; 
		}
	}
}

void CPU::regwrite_mux(int32_t aluresult)
{
	//for load instr, read from memory address. however, we need memtoreg bit to be active to write to memory. so check on memtoreg bit, if its 1 we do nothing. if its 
	//0 we have to clean regwrite_buffer and just give it aluresult instead. 
	if(control.memToReg_bit == 1)
	{
		return; 
	}
	else if(control.memToReg_bit == 0)
	{
		regwrite_buffer = aluresult; 
		return; 
	}
}

//if jal instruction, we want to write to register the value PC + 4 
void CPU::regwrite_jal_mux()
{
	if(instr_type == 'J')
	{
		int32_t output = static_cast<int>(PC + 4); 
		regwrite_buffer = output; 
	}
	return; 
}

//if LUI instruction, we want to write to dest. register 
void CPU::regwrite_lui_mux()
{
	if(instr_type == 'U')
	{
		int32_t output = bitsetConverter(immgen_result); 
		regwrite_buffer = output; 
	}
	return; 
}

bool CPU::branch_and_jal_wrapper(int32_t aluresult)
{
	//if branch instruction
	if(instr_type == 'B' && control.branch_bit == 1)
	{
		if(aluresult == 0)
		{
			//jump based on pc + imm we read, assume that negative imm can be possible
			int32_t imm_clean = bitsetConverter(immgen_result); 
			int32_t pc_convert = static_cast<int>(PC);
			int32_t result = pc_convert + imm_clean; 
			PC = static_cast<unsigned long>(result); 
			return false; 
		}
		else
			return true; 
	}
	//if jal instruction 
	else if(instr_type == 'J' && control.branch_bit == 1)
	{
		//jump based on pc + imm we read, assume that negative imm can be possible 
		int32_t imm_clean = bitsetConverter(immgen_result); 
		int32_t pc_convert = static_cast<int>(PC);
		int32_t result = pc_convert + imm_clean; 
		PC = static_cast<unsigned long>(result); 
		return false; 
	}
	//not branch/jump instruction, so always return false in this case 
	return true; 
}


void CPU::instrConsoleOut(int lines)
{
	for(int i = 0; i < lines; i++)
	{
	cout << "Line " << i << ": Bitset = " << instMem[i] 
				<<  endl;
	}
}


int32_t CPU::get_a0_regvalue()
{
	return registers[10]; 
}

int32_t CPU::get_a1_regvalue(){
	return registers[11];
}