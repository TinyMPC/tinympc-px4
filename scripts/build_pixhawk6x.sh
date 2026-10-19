#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"

px4_dir="${PX4_DIR:-${repo_root}/third_party/PX4-Autopilot}"
venv_dir="${PX4_VENV_DIR:-${repo_root}/third_party/px4-venv}"
target="${PX4_TARGET:-px4_fmu-v6x_multicopter}"
required_px4_version="${PX4_VERSION:-v1.15.3}"

if [ ! -d "${px4_dir}/.git" ]; then
  echo "PX4 checkout not found at ${px4_dir}." >&2
  echo "Run ./scripts/setup_px4_firmware.sh first, or set PX4_DIR." >&2
  exit 1
fi

actual_px4_version="$(git -C "${px4_dir}" describe --tags --exact-match HEAD 2>/dev/null || true)"
if [ "${actual_px4_version}" != "${required_px4_version}" ]; then
  echo "Pixhawk build requires PX4 ${required_px4_version}; checkout reports '${actual_px4_version:-untagged}'." >&2
  echo "Run ./scripts/setup_px4_firmware.sh or point PX4_DIR at the pinned checkout." >&2
  exit 1
fi

if ! command -v arm-none-eabi-g++ >/dev/null 2>&1; then
  for gcc_dir in "${HOME}/opt/gcc-arm-none-eabi-9-2020-q2-update/bin" \
                 /opt/gcc-arm-none-eabi-9-2020-q2-update/bin; do
    if [ -x "${gcc_dir}/arm-none-eabi-g++" ]; then
      export PATH="${gcc_dir}:${PATH}"
      break
    fi
  done
fi

if ! command -v arm-none-eabi-g++ >/dev/null 2>&1; then
  echo "arm-none-eabi-g++ is required for the Pixhawk build." >&2
  echo "Install the PX4 toolchain described by Tools/setup/ubuntu.sh." >&2
  exit 1
fi

if [ ! -d "${venv_dir}" ]; then
  python3 -m venv "${venv_dir}"
fi

if ! "${venv_dir}/bin/python" -c \
  "import menuconfig, kconfiglib, em, jinja2, numpy, pymavlink, pkg_resources" >/dev/null 2>&1; then
  # Python 3.12 venvs no longer ship setuptools; PX4's UAVCAN DSDL generator
  # still imports pkg_resources, which newer setuptools releases remove.
  "${venv_dir}/bin/python" -m pip install --disable-pip-version-check \
    -r "${px4_dir}/Tools/setup/requirements.txt" "setuptools<81"
fi

source "${venv_dir}/bin/activate"
EXTERNAL_MODULES_LOCATION="${repo_root}/px4_external" \
  make -C "${px4_dir}" "${target}"

firmware="${px4_dir}/build/${target}/${target}.px4"
elf="${px4_dir}/build/${target}/${target}.elf"

if [ ! -f "${firmware}" ] || [ ! -f "${elf}" ]; then
  echo "PX4 reported success, but the expected ${target} artifacts are missing." >&2
  exit 1
fi

echo
echo "${target} firmware cross-build complete (not flashed):"
echo "  ${firmware}"
echo "  ${elf}"
arm-none-eabi-size "${elf}"
sha256sum "${firmware}"
echo
echo "This image is software-tested only. Start with monitor mode and props removed."
