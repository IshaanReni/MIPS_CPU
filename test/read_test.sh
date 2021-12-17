#!/bin/bash

set -eou pipefail

FIRST_LINE=$(head -n 1 "test/5-reference/1-arithmetic_testing/xori_5.out")
echo "${FIRST_LINE}"
PATTERN="TB : INFO : register_v0="${FIRST_LINE}
echo "${PATTERN}"

if grep -Fxq "${PATTERN}" test/4-tb_outputs/1-arithmetic_testing/mips_cpu_bus_tb_xori_5.stdout 
then
    echo "success"
else
    echo "fail"
fi