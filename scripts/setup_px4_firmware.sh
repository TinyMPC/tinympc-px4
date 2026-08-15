#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"

px4_version="${PX4_VERSION:-v1.15.3}"
px4_dir="${PX4_DIR:-${repo_root}/third_party/PX4-Autopilot}"

echo "PX4 version: ${px4_version}"
echo "PX4 directory: ${px4_dir}"

if [ ! -d "${px4_dir}/.git" ]; then
  mkdir -p "$(dirname "${px4_dir}")"
  git clone --branch "${px4_version}" --depth 1 --recurse-submodules --shallow-submodules https://github.com/PX4/PX4-Autopilot.git "${px4_dir}"
else
  if ! requested_commit="$(git -C "${px4_dir}" rev-parse "${px4_version}^{commit}" 2>/dev/null)"; then
    git -C "${px4_dir}" fetch --tags origin "${px4_version}"
    requested_commit="$(git -C "${px4_dir}" rev-parse "${px4_version}^{commit}")"
  fi

  current_commit="$(git -C "${px4_dir}" rev-parse HEAD)"
  if [ "${current_commit}" != "${requested_commit}" ]; then
    if ! git -C "${px4_dir}" diff --quiet --ignore-submodules=all -- || \
       ! git -C "${px4_dir}" diff --cached --quiet --ignore-submodules=all --; then
      echo "PX4 checkout has local changes and is not at ${px4_version}." >&2
      echo "Commit or remove those changes, or choose a separate PX4_DIR." >&2
      exit 1
    fi
    git -C "${px4_dir}" checkout --detach "${px4_version}"
  fi
  git -C "${px4_dir}" submodule update --init --recursive --depth 1
fi

actual_version="$(git -C "${px4_dir}" describe --tags --always)"
echo "PX4 checkout reports: ${actual_version}"

# Native TinyMPC is the default public path and does not require generated
# MATLAB sources. Enable the legacy Simulink app only when explicitly asked;
# otherwise a clean native-only build must remain independent of MATLAB.
enable_simulink_app="${ENABLE_SIMULINK_APP:-0}"
if [ "${enable_simulink_app}" != "0" ] && [ "${enable_simulink_app}" != "1" ]; then
  echo "ENABLE_SIMULINK_APP must be 0 or 1." >&2
  exit 1
fi

sitl_board="${px4_dir}/boards/px4/sitl/default.px4board"
if [ "${enable_simulink_app}" = "1" ]; then
  if [ -f "${sitl_board}" ] && ! grep -q 'CONFIG_MODULES_PX4_SIMULINK_APP=y' "${sitl_board}"; then
    echo "CONFIG_MODULES_PX4_SIMULINK_APP=y" >> "${sitl_board}"
    echo "Enabled legacy px4_simulink_app module in SITL board config."
  fi
else
  if [ -f "${sitl_board}" ] && grep -q 'CONFIG_MODULES_PX4_SIMULINK_APP=y' "${sitl_board}"; then
    echo "Legacy px4_simulink_app is already enabled in this PX4 checkout."
    echo "Its generated sources must exist, or use a clean PX4 checkout for the native-only build."
  else
    echo "Leaving legacy px4_simulink_app disabled (native TinyMPC build)."
  fi
fi

echo
echo "To install the full PX4 system toolchain on Ubuntu 22.04, run:"
echo "  cd '${px4_dir}/Tools/setup'"
echo "  bash ./ubuntu.sh"
echo
echo "That toolchain step uses sudo, installs system packages, and usually requires a reboot."
echo
echo "For the repo-local SITL build path, run:"
echo "  ./scripts/build_px4_sitl.sh"
