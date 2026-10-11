cmake_minimum_required(VERSION 3.22)
# Convenience entry; board/ remains the authoritative firmware project.
project(board LANGUAGES C CXX ASM)
add_subdirectory(board)
