#!/bin/bash

for i in $(seq 1 $1)
do
    python3 ../inventory-codegen/meow.py && python3 labelmaker.py -i barcode.png -F True -a True -C 98:6E:E8:43:E1:0C
    sleep 1
done