#!/bin/bash
# Compile script for Extended EKF S-Function
# This script compiles the ekf_sfun.cpp for use in Simulink

echo "=== Extended EKF S-Function Compilation ==="

# Check if MATLAB is available
if ! command -v matlab &> /dev/null; then
    echo "Warning: MATLAB not found in PATH"
    echo "Please ensure MATLAB is installed and accessible"
fi

# Compilation command (adjust paths as needed)
echo "Compilation command:"
echo "mex ekf_sfun.cpp -I../external/eigen-3.4.0 -I../../ -L../../build -lekf"

# Instructions
echo ""
echo "=== Instructions ==="
echo "1. Open MATLAB"
echo "2. Navigate to the directory containing ekf_sfun.cpp"
echo "3. Run the mex command above (adjust paths as needed)"
echo "4. Ensure the EKF library is built and accessible"
echo ""
echo "Required files:"
echo "  - ekf_sfun.cpp (this S-function)"
echo "  - ekf.h (EKF library header)"
echo "  - libekf.a or libekf.so (compiled EKF library)"
echo "  - Eigen library headers"
echo ""
echo "Example directory structure:"
echo "  project_root/"
echo "  ├── ekf.h"
echo "  ├── ekf.cpp"
echo "  ├── build/"
echo "  │   └── libekf.a"
echo "  ├── examples/simulink/"
echo "  │   ├── ekf_sfun.cpp"
echo "  │   └── test_extended_ekf_sfun.m"
echo "  └── external/"
echo "      └── eigen-3.4.0/"
echo ""
echo "Key features of the Extended EKF S-Function:"
echo "  - 7-dimensional state vector [v_x, v_y, γ, Fy_fl, Fy_fr, Fy_rl, Fy_rr]"
echo "  - 5-dimensional control input [δ, Fx_fl, Fx_fr, Fx_rl, Fx_rr]"
echo "  - 7-dimensional measurement vector [ax, ay, γ, v_fl, v_fr, v_rl, v_rr]"
echo "  - Individual wheel dynamics with steering transformation"
echo "  - First-order tire force dynamics"
echo "  - Advanced vehicle state and tire force estimation"
