
# Toolchain file for cross-compiling to 64‑bit Windows with MinGW‑w64
# Usage:
#   cmake -B build-win -S . -DCMAKE_TOOLCHAIN_FILE=toolchain-mingw64.cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DCMAKE_WINDOWS_EXPORT_ALL_SYMBOLS=ON
#
# If you need 32‑bit output, replace x86_64 with i686 everywhere below.

set(CMAKE_SYSTEM_NAME Windows)

# --- Compilers -----------------------------------------------------------
set(CMAKE_C_COMPILER   x86_64-w64-mingw32-gcc)
set(CMAKE_CXX_COMPILER x86_64-w64-mingw32-g++)
set(CMAKE_RC_COMPILER  x86_64-w64-mingw32-windres)

# --- Paths ---------------------------------------------------------------
# Adjust if your MinGW‑w64 prefix lives elsewhere
set(CMAKE_FIND_ROOT_PATH /usr/x86_64-w64-mingw32)

# Prefer libraries and headers from the target root path
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
