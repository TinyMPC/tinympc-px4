#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"
world_name="${GZ_WORLD:-default}"
course="${1:-figure_eight}"

case "${course}" in
  figure_eight|figure_eight_soc|figure8)
    model_name="tinympc_figure_eight_course"
    sdf_file="${repo_root}/quadtest/gazebo/tinympc_figure_eight_course.sdf"
    ;;
  corridor)
    model_name="tinympc_corridor"
    sdf_file="${repo_root}/quadtest/gazebo/tinympc_corridor.sdf"
    ;;
  *)
    echo "Unknown course '${course}'. Use figure_eight or corridor." >&2
    exit 2
    ;;
esac

gz service -s "/world/${world_name}/create" \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req "sdf_filename: \"${sdf_file}\", name: \"${model_name}\""

echo "Spawned ${model_name} in Gazebo world ${world_name}."
