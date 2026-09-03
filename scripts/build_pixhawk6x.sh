#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"

px4_dir="${PX4_DIR:-${repo_root}/third_party/PX4-Autopilot}"
venv_dir="${PX4_VENV_DIR:-${repo_root}/third_party/px4-venv}"
target="px4_fmu-v6x_default"
required_px4_version="${PX4_VERSION:-v1.15.3}"

if [ ! -d "${px4_dir}/.git" ]; then
  echo "PX4 checkout not found at ${px4_dir}." >&2
  echo "Run ./scripts/setup_px4_firmware.sh first, or set PX4_DIR." >&2
  exit 1
fi

actual_px4_version="$(git -C "${px4_dir}" describe --tags --exact-match HEAD 2>/dev/null || true)"
if [ "${actual_px4_version}" != "${required_px4_version}" ]; then
  echo "Pixhawk 6X build requires PX4 ${required_px4_version}; checkout reports '${actual_px4_version:-untagged}'." >&2
  echo "Run ./scripts/setup_px4_firmware.sh or point PX4_DIR at the pinned checkout." >&2
  exit 1
fi

if ! command -v arm-none-eabi-g++ >/dev/null 2>&1; then
  echo "arm-none-eabi-g++ is required for the Pixhawk 6X build." >&2
  echo "Install the PX4 toolchain described by Tools/setup/ubuntu.sh." >&2
  exit 1
fi

if [ ! -d "${venv_dir}" ]; then
  python3 -m venv "${venv_dir}"
fi

if ! "${venv_dir}/bin/python" -c \
  "import menuconfig, kconfiglib, em, jinja2, numpy, pymavlink" >/dev/null 2>&1; then
  "${venv_dir}/bin/python" -m pip install --disable-pip-version-check \
    -r "${px4_dir}/Tools/setup/requirements.txt"
fi

source "${venv_dir}/bin/activate"
EXTERNAL_MODULES_LOCATION="${repo_root}/px4_external" \
  make -C "${px4_dir}" "${target}"

firmware="${px4_dir}/build/${target}/${target}.px4"
elf="${px4_dir}/build/${target}/${target}.elf"

if [ ! -f "${firmware}" ] || [ ! -f "${elf}" ]; then
  echo "PX4 reported success, but the expected 6X artifacts are missing." >&2
  exit 1
fi

echo
echo "Pixhawk 6X firmware cross-build complete (not flashed):"
echo "  ${firmware}"
echo "  ${elf}"
arm-none-eabi-size "${elf}"
sha256sum "${firmware}"
echo
echo "This image is software-tested only. Start with monitor mode and props removed."
