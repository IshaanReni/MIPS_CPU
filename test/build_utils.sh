#!/bin/bash

set -eou pipefail

# Removes all previos outputs out of bin folder
BIN_DIR="2-bin/"
rm -rf test/2-bin/0-fundamental_testing/*
rm -rf test/2-bin/1-arithmetic_testing/*
rm -rf test/2-bin/2-conditionals/*
rm -rf test/2-bin/3-other_testcases/*

python3 utils/assembler.py test/1-testcase_assembly/0-fundamental_testing/ test/2-bin/0-fundamental_testing/
python3 utils/assembler.py test/1-testcase_assembly/1-arithmetic_testing/ test/2-bin/1-arithmetic_testing/
python3 utils/assembler.py test/1-testcase_assembly/2-conditionals/ test/2-bin/2-conditionals/
python3 utils/assembler.py test/1-testcase_assembly/3-other_testcases/ test/2-bin/3-other_testcases/

# build utils
# Robert 