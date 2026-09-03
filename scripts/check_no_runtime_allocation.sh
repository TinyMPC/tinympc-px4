#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"
quadtest_dir="${repo_root}/quadtest"
tinympc_dir="${quadtest_dir}/tinympc/TinyMPC"

test_binary="$(mktemp "${TMPDIR:-/tmp}/tinympc_no_runtime_malloc.XXXXXX")"
embedded_test_binary="$(mktemp "${TMPDIR:-/tmp}/tinympc_embedded_no_runtime_malloc.XXXXXX")"
trap 'rm -f "${test_binary}" "${embedded_test_binary}"' EXIT

c++ -std=c++17 -O2 -DEIGEN_RUNTIME_NO_MALLOC \
  -I"${tinympc_dir}/src" \
  -I"${tinympc_dir}/include/Eigen" \
  -I"${quadtest_dir}/wrapper" \
  "${quadtest_dir}/wrapper/tinympc_interface.cpp" \
  "${quadtest_dir}/wrapper/tinympc_no_runtime_malloc.cpp" \
  "${tinympc_dir}/src/tinympc/admm.cpp" \
  "${tinympc_dir}/src/tinympc/tiny_api.cpp" \
  "${tinympc_dir}/src/tinympc/rho_benchmark.cpp" \
  -o "${test_binary}"

"${test_binary}"

c++ -std=c++17 -O2 -DEIGEN_RUNTIME_NO_MALLOC \
  -DTINY_DISABLE_ADAPTIVE_RHO \
  -DTINY_MPC_HARDWARE_HOLD_ONLY \
  -DTINY_MPC_SINGLE_PRECISION \
  -DTINY_MPC_USE_STATIC_CACHE \
  -I"${tinympc_dir}/src" \
  -I"${tinympc_dir}/include/Eigen" \
  -I"${quadtest_dir}/wrapper" \
  "${quadtest_dir}/wrapper/tinympc_interface.cpp" \
  "${quadtest_dir}/wrapper/tinympc_no_runtime_malloc.cpp" \
  "${tinympc_dir}/src/tinympc/admm.cpp" \
  "${tinympc_dir}/src/tinympc/tiny_api.cpp" \
  -o "${embedded_test_binary}"

"${embedded_test_binary}"
