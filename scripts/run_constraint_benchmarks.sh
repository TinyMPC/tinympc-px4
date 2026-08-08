#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"
quadtest_dir="${repo_root}/quadtest"
tinympc_dir="${quadtest_dir}/tinympc/TinyMPC"
build_dir="${tinympc_dir}/build"

cmake -S "${tinympc_dir}" -B "${build_dir}" -DCMAKE_BUILD_TYPE=Release
cmake --build "${build_dir}"

benchmark_binary="$(mktemp "${TMPDIR:-/tmp}/tinympc_constraint_benchmark.XXXXXX")"
trap 'rm -f "${benchmark_binary}"' EXIT

c++ -std=c++17 -O2 -DNDEBUG \
  -I"${tinympc_dir}/src" \
  -I"${tinympc_dir}/include/Eigen" \
  -I"${quadtest_dir}/wrapper" \
  "${quadtest_dir}/wrapper/tinympc_interface.cpp" \
  "${quadtest_dir}/wrapper/tinympc_constraint_benchmark.cpp" \
  "${build_dir}/src/tinympc/libtinympcstatic.a" \
  -o "${benchmark_binary}"

"${benchmark_binary}"
