#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"
quadtest_dir="${repo_root}/quadtest"
tinympc_dir="${quadtest_dir}/tinympc/TinyMPC"
build_dir="${tinympc_dir}/build"
px4_dir="${PX4_DIR:-${repo_root}/third_party/PX4-Autopilot}"

echo "TinyMPC-PX4 repo: ${repo_root}"

if ! command -v cmake >/dev/null 2>&1; then
  echo "cmake is required but was not found on PATH." >&2
  exit 1
fi

cmake_args=(-S "${tinympc_dir}" -B "${build_dir}")
if command -v ninja >/dev/null 2>&1 && [ ! -f "${build_dir}/CMakeCache.txt" ]; then
  cmake_args+=(-G Ninja)
fi

cmake "${cmake_args[@]}"
cmake --build "${build_dir}"

echo
echo "TinyMPC native build complete:"
echo "  ${build_dir}/src/tinympc/libtinympcstatic.a"

echo
echo "Building TinyMPC Simulink-wrapper smoke test..."
smoke_binary="$(mktemp "${TMPDIR:-/tmp}/tinympc_interface_smoke.XXXXXX")"
trap 'rm -f "${smoke_binary}"' EXIT

c++ -std=c++17 \
  -I"${tinympc_dir}/src" \
  -I"${tinympc_dir}/include/Eigen" \
  -I"${quadtest_dir}/wrapper" \
  "${quadtest_dir}/wrapper/tinympc_interface.cpp" \
  "${quadtest_dir}/wrapper/tinympc_interface_smoke.cpp" \
  "${build_dir}/src/tinympc/libtinympcstatic.a" \
  -o "${smoke_binary}"

"${smoke_binary}"

if command -v matlab >/dev/null 2>&1 && \
   [ -d "${px4_dir}/build/px4_sitl_default" ]; then
  echo
  echo "Running MATLAB/Simulink/PX4 environment check..."
  PX4_DIR="${px4_dir}" matlab -batch "cd('${quadtest_dir}'); status = setup_tinympc_px4(); if ~status.modelLoads, exit(2); end"
elif command -v matlab >/dev/null 2>&1; then
  echo
  echo "Skipping the MATLAB/PX4 check because no built px4_sitl_default tree was found:"
  echo "  ${px4_dir}/build/px4_sitl_default"
  echo "Run ./scripts/setup_px4_firmware.sh and ./scripts/build_px4_sitl.sh, then rerun this command."
else
  echo
  echo "MATLAB was not found on PATH."
  echo "Install MATLAB/Simulink, then run:"
  echo "  matlab -batch \"cd('${quadtest_dir}'); run_tinympc_px4_demo('update')\""
fi
