#!/bin/bash

if [ "$#" -ne 2 ]; then
    echo "To execute the script, run: ./run_tests.sh 1 10"
    exit 1
fi

START=$1
END=$2

for i in $(seq $START $END); do
    INPUT="instances/sdp.$i.inp"
    OUTPUT="out/sdp.$i.out"
    
    echo "RUN INSTANCE $INPUT..."
    ./sdp < "$INPUT" > "$OUTPUT"
    
    cat "$INPUT" "$OUTPUT" | ./checker
    echo "-----------------------------------"
done

echo "Ok!"