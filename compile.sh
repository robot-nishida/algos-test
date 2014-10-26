#!/bin/sh

echo "Ready...OK."
g++ -w -o exe_algos main.cpp -I ~/Software/local/include/ `ode-config --cflags --libs` -ldrawstuff -framework GLUT -framework OpenGL || { echo oops!; exit 1; }
echo "Done...GO.\n>>> - - - "
./exe_algos