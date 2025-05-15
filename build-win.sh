#!/usr/bin/env bash
# Simple helper script to cross‑compile EKF as Windows DLL from Ubuntu
set -euo pipefail

# Install prerequisites (run once):
#   sudo apt-get update && sudo apt-get install mingw-w64 cmake make
#
# Then execute this script from the project root:
#   bash build-win.sh

TOOLCHAIN_FILE="toolchain-mingw64.cmake"
BUILD_DIR="build-win"
EIGEN_VERSION="3.4.0"
EIGEN_DIR="./external/eigen-${EIGEN_VERSION}"

# Download and extract Eigen3 if not already available
if [ ! -d "${EIGEN_DIR}" ]; then
    echo "Downloading Eigen3 ${EIGEN_VERSION}..."
    mkdir -p external
    wget -q https://gitlab.com/libeigen/eigen/-/archive/${EIGEN_VERSION}/eigen-${EIGEN_VERSION}.tar.gz -O eigen.tar.gz
    tar -xzf eigen.tar.gz -C external
    rm eigen.tar.gz
    echo "Eigen3 downloaded and extracted to ${EIGEN_DIR}"
fi

cmake -B "${BUILD_DIR}" -S . \
      -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN_FILE}" \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_SHARED_LIBS=ON \
      -DCMAKE_WINDOWS_EXPORT_ALL_SYMBOLS=ON \
      -DEIGEN3_INCLUDE_DIR="${EIGEN_DIR}"

cmake --build "${BUILD_DIR}" --parallel

echo ""
echo "==========================================="
echo "Build finished."
echo "Artifacts:"
echo "  $(realpath ${BUILD_DIR}/libekf.dll)"
echo "  $(realpath ${BUILD_DIR}/libekf.dll.a)  (import library for MinGW)"
echo "If you need an import library for MSVC, run:"
echo "  x86_64-w64-mingw32-dlltool -d exports.def -l libekf_msvc.lib -D libekf.dll"
echo "==========================================="
