#!/bin/sh

rm test
g++ -x c -std=c99 -Wall -Wextra -pedantic -O3 -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -o test test.c && ./test
