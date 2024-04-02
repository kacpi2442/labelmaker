#!/bin/bash

# if $2 defined
if [ -z "$2" ]
then
    cable=0
else
    cable=1
    text=$2
fi

for i in $(seq 1 $1)
do
    python3 ../inventory-codegen/meow.py $cable "$text" && python3 labelmaker.py -i barcode.png -F True -a True -C 98:6E:E8:43:E1:0C
    sleep 1
done