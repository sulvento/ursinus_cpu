#!/bin/bash

iverilog -o registers.vvp registers_tb.v
vvp registers.vvp

read -p "Press y to produce a GTKWave output or any other key to skip: " answer
if [[ $answer == "y" || $answer == "Y" ]]; then
   gtkwave
else
   echo "GTKWave skipped"
fi

