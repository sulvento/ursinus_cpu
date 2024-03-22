#!/bin/bash

# general purpose registers
iverilog -o registers.vvp registers_tb.v
vvp registers.vvp
echo

# program segment/pointer registers.
iverilog -o isr_test.vvp InstructionSegmentRegister_tb.v
vvp isr_test.vvp 
echo

iverilog -o ssr_test.vvp StaticSegmentRegister_tb.v
vvp ssr_test.vvp
echo

iverilog -o dsr_test.vvp DynamicSegmentRegister_tb.v
vvp dsr_test.vvp
echo

iverilog -o ip_test.vvp InstructionPointer_tb.v 
vvp ip_test.vvp
echo

iverilog -o sp_test.vvp StaticPointer_tb.v
vvp sp_test.vvp
echo

iverilog -o dp_test.vvp DynamicPointer_tb.v
vvp dp_test.vvp
echo

# increment registers
iverilog -o icr_test.vvp instruction_count_register_tb.v 
vvp icr_test.vvp
echo

iverilog -o mar_test.vvp memory_access_register_tb.v
vvp mar_test.vvp
echo

iverilog -o mcr_test.vvp memory_correction_register_tb.v
vvp mcr_test.vvp
echo


read -p "Press y to produce a GTKWave output or any other key to skip: " answer
if [[ $answer == "y" || $answer == "Y" ]]; then
   gtkwave
else
   echo "GTKWave skipped"
fi
