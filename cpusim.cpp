#include "CPU.h"

#include <iostream>
#include <bitset>
#include <stdio.h>
#include<stdlib.h>
#include <string>
#include<fstream>
#include <sstream>
using namespace std;

/*
Add all the required standard and developed libraries here
*/

/*
Put/Define any helper function/definitions you need here
*/
int main(int argc, char* argv[])
{
	/* This is the front end of your project.
	You need to first read the instructions that are stored in a file and load them into an instruction memory.
	*/

	/* Each cell should store 1 byte. You can define the memory either dynamically, or define it as a fixed size with size 4KB (i.e., 4096 lines). Each instruction is 32 bits (i.e., 4 lines, saved in little-endian mode).
	Each line in the input file is stored as an hex and is 1 byte (each four lines are one instruction). You need to read the file line by line and store it into the memory. You may need a mechanism to convert these values to bits so that you can read opcodes, operands, etc.
	*/

	bitset<8> instMem[4096];


	if (argc < 2) {
		//cout << "No file name entered. Exiting...";
		return -1;
	}

	ifstream infile(argv[1]); //open the file
	if (!(infile.is_open() && infile.good())) {
		cout<<"error opening file\n";
		return 0; 
	}
	
	string line; 
	int i = 0;
	while (infile) {
			infile>>line;
			stringstream line2(line);
			unsigned int hexinput; 
			line2>> hex >> hexinput;
			//store current line in an 8 lengthed bitset
			instMem[i] = bitset<8>(hexinput);

			i++;
		}
	int maxPC= i; 

	/* Instantiate your CPU object here.  CPU class is the main class in this project that defines different components of the processor.
	CPU class also has different functions for each stage (e.g., fetching an instruction, decoding, etc.).
	*/

	CPU myCPU(instMem);  // call the approriate constructor here to initialize the processor...  
	// make sure to create a variable for PC and resets it to zero (e.g., unsigned int PC = 0); 
	//TEST 
	// myCPU.instrConsoleOut(100); 

	/* OPTIONAL: Instantiate your Instruction object here. */
	//Instruction myInst; 
	
	bool done = true;
	while (done == true) // processor's main loop. Each iteration is equal to one clock cycle.  
	{
		//fetch
		myCPU.instrFetch(); 
		
		// decode
		myCPU.instrDecode(); 

		//reading registers 
		myCPU.regreads(); 

		//setting bits in Control Unit
		myCPU.ControlPass(); 

		//generating immediates based on instr type 
		myCPU.set_immgen(); 

		//setting aluControl signal for ALU 
		myCPU.set_aluControl(); 
		
		//ALU Compute 
		int32_t result = myCPU.alu_compute(); 

		myCPU.datamemoryunit_compute(result);

		//controlling what is written to write register through MUX
		myCPU.regwrite_mux(result); 

		//if we encounter JAL instruction, must store PC + 4 in write register 
		myCPU.regwrite_jal_mux();

		//if we encounter LUI instruction, must store generated immediate to write register 
		myCPU.regwrite_lui_mux(); 

		//write to register if signaled 
		myCPU.regwrite();

		//perform JUMPS/BEQ here, if we need to jump do nothing to PC. if we don't need to, increment by 4 like normal
		if(myCPU.branch_and_jal_wrapper(result))
		{
			myCPU.incPC(); 
		}
		else{
			//do nothing 
		}
		

		if (myCPU.readPC() > maxPC)
			break;
	}

	int a0 = myCPU.get_a0_regvalue();
	int a1 = myCPU.get_a1_regvalue();  
	// print the results (you should replace a0 and a1 with your own variables that point to a0 and a1)
	  cout << "(" << a0 << "," << a1 << ")" << endl;
	
	return 0;

}