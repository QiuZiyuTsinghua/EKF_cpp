
# Windows DLL Build Pack for EKF Library

This bundle contains everything you need to build **libekf.dll** on Ubuntu (or any Linux) without leaving the host OS.

## Contents

| File | Purpose |
|------|---------|
| `toolchain-mingw64.cmake` | CMake toolchain definition that tells CMake to use the MinGW‑w64 cross‑compiler and produce Windows binaries. |
| `build-win.sh` | Convenience shell script that configures and builds the DLL in `build-win/`. |

## Quick Start

1. **Install the cross‑compiler once:**
   ```bash
   sudo apt-get update
   sudo apt-get install mingw-w64 cmake make
   ```

2. **Place these files in the root of your EKF source tree** (same level as `CMakeLists.txt`).

3. **Run the build helper:**
   ```bash
   bash build-win.sh
   ```
   After it completes you will find
   ```
   build-win/libekf.dll
   build-win/libekf.dll.a
   ```
   ready to copy onto a Windows machine.

4. **Linking from Windows C++ (MinGW):**
   ```bash
   g++ main.cpp -Lekf\build-win -lekf -o app.exe
   ```

   **Linking from MSVC:** generate an import library first
   ```bash
   x86_64-w64-mingw32-dlltool -d exports.def -l libekf_msvc.lib -D libekf.dll
   ```
   then add `libekf_msvc.lib` to your Visual Studio project.

## Notes

* `Eigen` is header‑only, so no extra Windows libraries are required.
* We enable `CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS` to auto‑export all public symbols.  
  If you want a tight API surface, add explicit `__declspec(dllexport)` decorators or a `.def` file.
* For 32‑bit apps, replace every `x86_64` with `i686` in the toolchain file and installer command (`mingw-w64` provides both).

---

Generated on 2025-05-15 06:01:20.
