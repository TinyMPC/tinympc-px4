#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
patch_file="${script_dir}/patches/mw_uorb_write_timestamp_offboard.patch"
migration_patch_file="${script_dir}/patches/mw_uorb_write_offboard_v1_to_v2.patch"

matlab_release="${MATLAB_RELEASE:-R2026a}"
default_support_root="${HOME}/Documents/MATLAB/SupportPackages/${matlab_release}"
support_root="${MATLAB_SUPPORT_ROOT:-${default_support_root}}"
source_dir="${support_root}/toolbox/target/supportpackages/px4/core/src"
target_file="${source_dir}/MW_uORB_Write.cpp"

if [ ! -f "${target_file}" ]; then
  echo "MathWorks PX4 support-package source was not found:" >&2
  echo "  ${target_file}" >&2
  echo "Set MATLAB_SUPPORT_ROOT to the ${matlab_release} support-package root." >&2
  exit 1
fi

if grep -q 'TinyMPC-PX4 support-package patch v2' "${target_file}"; then
  echo "MathWorks PX4 support-package patch is already applied."
  exit 0
fi

if [ ! -w "${target_file}" ]; then
  chmod u+w "${target_file}"
fi

selected_patch="${patch_file}"
if grep -q 'Offboard heartbeat: when the model publishes' "${target_file}"; then
  selected_patch="${migration_patch_file}"
fi

if ! patch --batch --forward --dry-run -d "${source_dir}" -p0 < "${selected_patch}" >/dev/null; then
  echo "The patch does not apply cleanly to ${target_file}." >&2
  echo "Confirm MATLAB ${matlab_release} and an unmodified PX4 support package." >&2
  exit 1
fi

patch --batch --forward -d "${source_dir}" -p0 < "${selected_patch}"
echo "Patched ${target_file}"
