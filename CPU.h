#include <iostream>
#include <bitset>
#include <stdio.h>
#include<stdlib.h>
#include <string>
using namespace std;


struct ControlUnit
{
	//Controller specific parameters 
	bitset<1> branch_bit; 
	bitset<1> memRead_bit;
	bitset<1> memToReg_bit; 
	bitset<2> aluop_bit; 
	bitset<1> memwrite_bit; 
	bitset<1> alusrc_bit; 
	bitset<1> regwrite_bit; 

	//for LUI instructions 
	bitset<1> upper_imm; 
};

class CPU {
private:
	bitset<8> dmemory[4096]; //data memory byte addressable in little endian fashion; assuming data memory has only 4096 8 bit slots, ideally should be up to 2^32 
	bitset<8> instMem[4096]; //holds read in instructions 
	unsigned long PC; //pc 
	int32_t registers[32]; //32 registers that store 32 bit data
	
	//bit sections 
	bitset<32> instruction; //32 instruction bits
	bitset<7> opcode; 		//bits 0-6
	bitset<5> rd;           //bits 7-11
    bitset<3> funct3;       //bits 12-14
    bitset<5> rs1;          //bits 15-19
    bitset<5> rs2;          //bits 20-24
    bitset<7> funct7;       //bits 25-31

	ControlUnit control; 
	bitset<4> alu_control_input; 
	int32_t reg1_value; 
	int32_t reg2_value; 
	int32_t regwrite_buffer;
	char instr_type; 
	bitset<32> immgen_result; 


public:
	CPU();
	CPU(bitset<8>* instr_array);
	unsigned long readPC();
	void incPC();
	void instrFetch(); 
	void instrDecode(); 
	void regreads(); 
	void ControlPass(); 
	void set_immgen(); 
	void set_aluControl();
	int32_t alu_compute(); 
	//grab result from alu_compute and pass it here
	void datamemoryunit_compute(int32_t aluresult);
	//for load instr, read from memory address. however, we need memtoreg bit to be active to write to memory. so check on memtoreg bit, if its 1 we do nothing. if its 
	//0 we have to clean regwrite_buffer and just give it aluresult instead. 
	void regwrite_mux(int32_t aluresult); 
	//JAL needs to write to register, so add another MUX 
	void regwrite_jal_mux();
	//LUI needs to write to register, so add another MUX 
	void regwrite_lui_mux(); 
	void regwrite();  
	//returns a true/false bool on whether or not we need to call incPC(). true means call, false means don't. 
	bool branch_and_jal_wrapper(int32_t aluresult);
	void instrConsoleOut(int lines); 
	int32_t get_a0_regvalue(); 
	int32_t get_a1_regvalue(); 
	
};


//Helper function to convert a 32 bitset with 2s complement representation into a 32 bit signed int 
int32_t bitsetConverter(const bitset<32>& bits);

///Helper function to sign extend size 8 bitset into 32 bit signed integer, for SB/LB purposes
int32_t signExtendBitset(const bitset<8>& bits);