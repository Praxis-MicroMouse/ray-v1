#!/usr/bin/env bash
# Builds maze_cli against the real ../../src/maze.cpp (plus
# config/motion_tuning.h for a couple of estimate constants - plain
# macros, no Arduino dependency). Run from anywhere; output goes to
# ./maze_cli next to this script.
set -euo pipefail
cd "$(dirname "$0")"

g++ -std=c++17 -O2 -Wall \
    -I../../include \
    maze_cli.cpp ../../src/maze.cpp \
    -o maze_cli

echo "Built ./maze_cli"
