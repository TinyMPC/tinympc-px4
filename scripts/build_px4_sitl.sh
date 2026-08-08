#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"

px4_dir="${PX4_DIR:-${repo_root}/third_party/PX4-Autopilot}"
venv_dir="${PX4_VENV_DIR:-${repo_root}/third_party/px4-venv}"
target="${PX4_SITL_TARGET:-px4_sitl_default}"

if [ ! -d "${px4_dir}/.git" ]; then
  echo "PX4 checkout not found at ${px4_dir}." >&2
  echo "Run ./scripts/setup_px4_firmware.sh first." >&2
  exit 1
fi

if [ ! -d "${venv_dir}" ]; then
  python3 -m venv "${venv_dir}"
fi

if ! "${venv_dir}/bin/python" -c "import menuconfig, kconfiglib, em, jinja2, numpy, pymavlink" >/dev/null 2>&1; then
  "${venv_dir}/bin/python" -m pip install --disable-pip-version-check -r "${px4_dir}/Tools/setup/requirements.txt"
fi

# PX4's version header generation expects NuttX release tags to exist.
if ! git -C "${px4_dir}/platforms/nuttx/NuttX/nuttx" tag | grep -q '^nuttx-[0-9]'; then
  git -C "${px4_dir}/platforms/nuttx/NuttX/nuttx" fetch --tags
fi

# The Simulink model compiles TinyMPC source files directly into the PX4 app.
# Remove any stale static-library link left by older setup scripts.
simulink_app_cmake="${px4_dir}/src/modules/px4_simulink_app/CMakeLists.txt"
if [ -f "${simulink_app_cmake}" ] && grep -q 'libtinympcstatic\.a' "${simulink_app_cmake}"; then
  sed -i '/target_link_libraries(modules__px4_simulink_app PRIVATE .*libtinympcstatic\.a)/d' "${simulink_app_cmake}"
  echo "Removed stale TinyMPC static-library link from px4_simulink_app CMakeLists."
fi

source "${venv_dir}/bin/activate"
EXTERNAL_MODULES_LOCATION="${repo_root}/px4_external" \
  make -C "${px4_dir}" "${target}"

echo
echo "PX4 SITL build complete:"
echo "  ${px4_dir}/build/${target}/bin/px4"
echo "  includes SITL-only tinympc_fullstate and tinympc_chicane external modules"
