#!/bin/sh

PROGRAM=test

rm $PROGRAM
g++ -x c -std=c99 -Wall -Wextra -pedantic -O3 -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -lSDL2 -o $PROGRAM $PROGRAM.c && ./$PROGRAM
