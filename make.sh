#!/bin/sh

rm test_sdl
g++ -x c -std=c99 -Wall -Wextra -pedantic -O3 -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -lSDL2 -o test_sdl test_sdl.c && ./test_sdl
