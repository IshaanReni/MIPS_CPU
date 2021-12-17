#!/bin/bash

#turn on warnings
set -eou pipefail


#Important parameters
MIPS_RAM_DIR="test/0-MIPS_tb_RAM"
TESTCASE_DIR="test/1-testcase_assembly"
BIN_DIR="test/2-bin"
SIMULATOR_DIR="test/3-simulator"
OUT_DIR="test/4-tb_outputs"
REF_DIR="test/5-reference"


#INPUTS
SOURCE_DIR="$1" #rtl or similar
TESTFOLDER="$2"
FILENAME="$3" #Name of FILE under test 
INSTRUCTION="$4" #instruction tested in file, can be test_all!
TESTFILE="$5" #Path where FILE is stored

#Removes all previous outputs of a specific testcase - discards warnings
rm -rf test/3-simulator/${TESTFOLDER}/mips_cpu_bus_tb_${FILENAME}*
rm -rf test/4-tb_outputs/${TESTFOLDER}/mips_cpu_bus_tb_${FILENAME}*


set +e
#Compiles a specific simulator and testbench - -P command used to modify the RAM_INIT_FILE parameter on the test-bench at compile-time
iverilog -g 2012 \
${SOURCE_DIR}/mips_cpu_*.v ${MIPS_RAM_DIR}/mips_cpu_bus_tb.v ${MIPS_RAM_DIR}/RAM_8x_40000_avalon.v \
-s mips_cpu_bus_tb \
-P mips_cpu_bus_tb.RAM_INIT_FILE=\"./${TESTFILE}\" \
-o ${SIMULATOR_DIR}/${TESTFOLDER}/mips_cpu_bus_tb_${FILENAME}
set -e


#runs the program, caputes all output into a file +e to disable automatic script failure
set +e
./${SIMULATOR_DIR}/${TESTFOLDER}/mips_cpu_bus_tb_${FILENAME} > ${OUT_DIR}/${TESTFOLDER}/mips_cpu_bus_tb_${FILENAME}.stdout
set -e



FIRST_LINE=$(head -n 1 "${REF_DIR}/${TESTFOLDER}/${FILENAME}.out")
#echo "${FIRST_LINE}"
PATTERN="TB : INFO : register_v0="${FIRST_LINE}
#echo "${PATTERN}"

if grep -Fxq "${PATTERN}" ${OUT_DIR}/${TESTFOLDER}/mips_cpu_bus_tb_${FILENAME}.stdout 
then
    echo "${FILENAME}  ${INSTRUCTION} Pass"
else
    echo "${FILENAME}  ${INSTRUCTION}    Fail    Error in Execution"
fi

#Test_one_testcase
#Rob