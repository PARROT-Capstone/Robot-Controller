#!/bin/sh

# Build the c++ code
cd build
cmake ..
make

# Run the python code in the background
cd ..
python3 mainProgram.py &

# delay for 5 seconds
sleep 5

# send sigint to the python code
kill -2 $(pgrep -f mainProgram.py)

# delay for 2 seconds
sleep 2
``
# run the python code again
python3 mainProgram.py