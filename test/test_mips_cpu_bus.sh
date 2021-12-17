#!/bin/bash

# no u to allow for unbounded/ undefined variables
set -eou pipefail

./test/build_utils.sh

#Input Parameters
SOURCE_DIRECTORY="$1"
INSTRUCTION="${2:-test_all}"

#valid Testcases
HEX_TESTFOLDERS="test/2-bin"

if [[ ${INSTRUCTION} == "test_all" ]]; then

    for i in ${HEX_TESTFOLDERS}/*; do
        TESTFOLDER=$(basename ${i})
        for j in ${i}/*.hex.txt; do   
        # gets filename from file
            FILENAME=$(basename ${j} .hex.txt) 
        #extracts the instruction from filename
            TEST_INSTRUCTION=${FILENAME%%_*} 
            ./test/test_one_testcase.sh ${SOURCE_DIRECTORY} ${TESTFOLDER} ${FILENAME} ${TEST_INSTRUCTION} ${j}
        done
    done
   

else
    #loops through all folders to find files with INSTRUCTION prefix
    for k in ${HEX_TESTFOLDERS}/*; do
        TESTFOLDER=$(basename ${k})
        for q in ${k}/${INSTRUCTION}*; do
            # gets filename from file
            FILENAME=$(basename ${q} .hex.txt)
            #checks whether instruction is in the filename, otherwise breaks the search
            if [[ ${FILENAME} == "${INSTRUCTION}*" ]]; then
                break
            fi
            ./test/test_one_testcase.sh ${SOURCE_DIRECTORY} ${TESTFOLDER} ${FILENAME} ${INSTRUCTION} ${q}
        done

    done
    
fi


#test_mips_cpu_bus.sh
#Robert Hoppe
