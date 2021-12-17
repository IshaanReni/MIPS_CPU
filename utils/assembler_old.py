#MIPS to Hex Assembler
#Made by: Robert H, Marco L, Ishaan R

import re
import sys
import os

# passing on arguments to program
asm_dir = sys.argv[1]
hex_dir = sys.argv[2]

opcodes = {'ADDIU'  : '001001',
           'ADDU'   : '000000',
           'AND'    : '000000',
           'ANDI'   : '001100',
           'BEQ'    : '000100',
           'BGEZ'   : '000001',
           'BGEZAL' : '000001',
           'BGTZ'   : '000111',
           'BLEZ'   : '000110',
           'BLTZ'   : '000001',
           'BLTZAL' : '000001',
           'BNE'    : '000101',
           'DIV'    : '000000',
           'DIVU'   : '000000',
           'J'      : '000010',
           'JALR'   : '000000',
           'JAL'    : '000011',
           'JR'     : '000000',
           'LB'     : '100000',
           'LBU'    : '100100',
           'LH'     : '100001',
           'LHU'    : '100101',
           'LUI'    : '001111',
           'LW'     : '100011',
           'LWL'    : '100010',
           'LWR'    : '100110',
           'MFHI'   : '000000',
           'MFLO'   : '000000',
           'MTHI'   : '000000',
           'MTLO'   : '000000',
           'MULT'   : '000000',
           'MULTU'  : '000000',
           'OR'     : '000000',
           'ORI'    : '001101',
           'SB'     : '101000',
           'SH'     : '101001',
           'SLL'    : '000000',
           'SLLV'   : '000000',
           'SLT'    : '000000',
           'SLTI'   : '001010',
           'SLTIU'  : '001011',
           'SLTU'   : '000000',
           'SRA'    : '000000',
           'SRAV'   : '000000',
           'SRL'    : '000000',
           'SRLV'   : '000000',
           'SUBU'   : '000000',
           'SW'     : '101011',
           'XOR'    : '000000',
           'XORI'   : '001110'
           }

fn_codes = {'ADDU'  : '100001',
            'AND'   : '100100',
            'DIV'   : '011010',
            'DIVU'  : '011011',
            'JALR'  : '001001',
            'JR'    : '001000',
            'MFHI'  : '010000',
            'MFLO'  : '010010',
            'MTHI'  : '010001',
            'MTLO'  : '010011',
            'MULT'  : '011000',
            'MULTU' : '011001',
            'OR'    : '100101',
            'SLL'   : '000000',
            'SLLV'  : '000100',
            'SLT'   : '101010',
            'SLTU'  : '101011',
            'SRA'   : '000011',
            'SRAV'  : '000111',
            'SRL'   : '000010',
            'SRLV'  : '000110',
            'SUBU'  : '100011',
            'XOR'   : '100110'
            }

# source 2/ dest 2
branch_s2_codes = {
        'BGEZ'   : '00001',
        'BGEZAL' : '10001',
        'BLTZ'   : '00000',
        'BLTZAL' : '10000',
        'BGTZ'   : '00000',
        'BLEZ'   : '00000'
        }

#converts an integer into a binary equivalent (l defined bitlength)
def to_bin(num, l):
    bin_num = bin(int(num))
    bin_num = bin_num[2:]
    while len(bin_num) < l:
        
        bin_num = "0" + bin_num
    return bin_num

# break by spaces - the first element of the return should be the opcode
def break_instr_line(i_line):
    ##what's re?
    line = i_line
    line = line.replace('(', " ")
    line = line.replace(')', "")
    line = line.replace('$', "")
    line = line.replace('\n', "")

    return re.split(', | ', line)



def assembly_to_hex(asm_dir, hex_dir):

    file_list = []
    dir_path = asm_dir

    # checks for files with extensions .asm.txt and stores them in an array
    ext = ('.asm.txt')
    for file_name in os.listdir(dir_path):
        if file_name.endswith(ext):
            file_list.append(file_name) 
        else:
            continue
        
    # iterates through all files in the array
    for i in range(len(file_list)):
        
        # opens the asm file to get access to the lines
        asm_file = open((asm_dir + file_list[i]), 'r')
        asm_lines = asm_file.readlines()
        
        # creates a .hex.txt file to write the hex-lines to 
        #hexed_file = open(file_name + ".hex.txt", "w")
        
        for line in asm_lines:

            # breaks down line into vector of opcode and other important values
            i_line = break_instr_line(line)
             
            #Retrieves values of the machine code from directory
            op = opcodes[i_line[0].upper()]
            
            if i_line[0].upper() in fn_codes:
                func_code = fn_codes[i_line[0].upper()]
            else:
                func_code = "000000"
                
            rs = to_bin(0, 5)
            rt = to_bin(0, 5)
            rd = to_bin(0, 5)
            shift = to_bin(0, 5)
            i_data = to_bin(0, 16)
            j_addr = to_bin(0, 26)

            #3 reg instructions
            if i_line[0].upper() in ['AND', 'ADDU', 'OR', 'XOR', 'SUBU','SLT','SLTU', 'SLLV', 'SRAV', 'SRLV']:
                rs = to_bin(i_line[1],5)
                rt = to_bin(i_line[2],5)
                rd = to_bin(i_line[3],5)
            #2 reg instructions (rs, rt, with a possible immediate value)
            elif i_line[0].upper() in ['ADDIU', 'ANDI', 'ORI', 'SLTI', 'SLTIU', 'XORI', 'MULT', 'MULTU', 'DIV', 'DIVU', 'BEQ', 'BNE', 'LB', 'LBU', 'LH', 'LHU', 'LW', 'LWL', 'LWR', 'SB', 'SH', 'SW']:
                rs = to_bin(i_line[1],5)
                rt = to_bin(i_line[2],5)
                i_data = to_bin(i_line[3],16) if len(i_line) == 4 else to_bin(0, 16)
            
            #2 reg (rt, rd, shift)
            elif i_line[0].upper() in ['SLL', 'SRA', 'SRL']:
                rt = to_bin(i_line[1],5)
                rd = to_bin(i_line[2],5)
                shift = to_bin(i_line[3],5)
            #1 reg (rs)
            elif i_line[0].upper() in ['MTHI', 'MTLO', 'JR']:
                rs = to_bin(i_line[1],5)
                i_data = to_bin(i_line[2],16) if len(i_line) == 3 else to_bin(0, 16)
            #1 reg (rt)
            elif i_line[0].upper() in ['MFHI', 'MFLO']:
                rd = to_bin(i_line[1],5)
                i_data = to_bin(i_line[2],16) if len(i_line) == 3 else to_bin(0, 16) 
            
            #1 reg (rd)
            elif i_line[0].upper() == 'LUI':
                rt = to_bin(i_line[1],5) 
                i_data = to_bin(i_line[2],16) if len(i_line) == 3 else to_bin(0, 16) 
            
            #branch z instructions
            elif i_line[0].upper() in ['BGEZ', 'BGEZAL', 'BLTZ', 'BLTZAL', 'BGTZ', 'BLEZ']:
                rs = to_bin(i_line[1],5)
                rt = branch_s2_codes[i_line[0]]
                i_data = to_bin(i_line[2],16) if len(i_line) == 3 else to_bin(0, 16)

            #j-type instructions   
            elif i_line[0].upper() in ['J', 'JAL']:
                j_addr = to_bin(i_line[1],26)
            
            elif i_line[0].upper() == 'JALR':
                rs = to_bin(i_line[1],5)
                rd = to_bin(i_line[2],5)
            
            #Assemble machine code with defined values 
            machine_code = ""

            #r-type
            if op == "000000":
                print(op)
                print(rs)
                print(rt)
                print(shift)
                print(func_code)
                machine_code = str(op) + rs + rt + rd + shift + func_code
            #j-type
            elif op[:5] == "00001":
                machine_code = str(op) + j_addr
            #i-type
            else:
                machine_code = str(op) + rs + rt + i_data

            print(machine_code)
            #convert into hex
            hex_machine_code = ""

            for j in range(0, 32, 4):
                hex_machine_code = hex_machine_code + hex(int(machine_code[j:j+4], 2))[2:]

            print(hex_machine_code)

            #Write to file
            filename = file_list[i][:-8] + '.hex.txt'
            with open(os.path.join(hex_dir, filename), 'a') as hexed_file:
                hexed_file.write(hex_machine_code + "\n")
            hexed_file.close()
            






assembly_to_hex(asm_dir, hex_dir)

