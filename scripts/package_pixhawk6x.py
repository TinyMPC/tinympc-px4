#!/usr/bin/env python3
"""Stage a built 6X image, its provenance/size manifest, and SHA256 checksums."""

import argparse
import base64
import hashlib
import json
import re
import shutil
import shlex
import subprocess
import zlib
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_hashes():
    # Match the module CMake content identifier; paths are deliberately relative.
    patterns = ["px4_external/**/*.cpp", "px4_external/**/*.msg", "px4_external/**/*.h.in",
                "px4_external/**/CMakeLists.txt", "quadtest/wrapper/**/*.h", "quadtest/wrapper/**/*.hpp",
                "quadtest/wrapper/tinympc_interface.cpp", "quadtest/tinympc/TinyMPC/src/tinympc/**/*.cpp",
                "quadtest/tinympc/TinyMPC/src/tinympc/**/*.hpp"]
    paths = sorted({p for pattern in patterns for p in ROOT.glob(pattern)})
    return {str(p.relative_to(ROOT)): sha256(p) for p in paths}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--px4-dir", type=Path, default=ROOT / "third_party/PX4-Autopilot")
    parser.add_argument("--target", default="px4_fmu-v6x_multicopter")
    parser.add_argument("--output", type=Path, default=ROOT / "firmware")
    parser.add_argument("--size-tool", default="arm-none-eabi-size")
    parser.add_argument("--validation-json", type=Path, help="Embed software test evidence with a matching source_id")
    args = parser.parse_args()
    if args.target != "px4_fmu-v6x_multicopter":
        raise SystemExit("This handoff packager is restricted to the staged Pixhawk 6X multicopter target.")
    build = args.px4_dir / "build" / args.target
    image_path = build / f"{args.target}.px4"
    elf = build / f"{args.target}.elf"
    image = json.loads(image_path.read_text())
    raw = zlib.decompress(base64.b64decode(image["image"]))
    if image["board_id"] != 53 or len(raw) != image["image_size"]:
        raise SystemExit("Unexpected board ID or image size; refusing to package.")
    header = (build / "external_modules/tinympc_chicane/tinympc_build_info.h").read_text()
    source_id = re.search(r'TINYMPC_SOURCE_ID "([a-f0-9]+)"', header).group(1)
    hashes = source_hashes()
    expected_id = hashlib.sha256("".join(f"{path}:{digest}\n" for path, digest in hashes.items()).encode()).hexdigest()[:16]
    if source_id != expected_id:
        raise SystemExit(f"Stale build: image source {source_id}, current source {expected_id}. Rebuild first.")
    validation = json.loads(args.validation_json.read_text()) if args.validation_json else None
    if validation and validation.get("source_id") != source_id:
        raise SystemExit("Validation evidence belongs to a different source build.")
    if source_id.encode() not in raw or b"tinympc_status" not in raw:
        raise SystemExit("The image does not contain its source identifier and metrics topic.")
    size_output = subprocess.check_output([args.size_tool, str(elf)], text=True)
    compiler = Path(shutil.which(args.size_tool) or args.size_tool).with_name("arm-none-eabi-g++")
    text, data, bss = map(int, size_output.splitlines()[-1].split()[:3])
    args.output.mkdir(parents=True, exist_ok=True)
    destination = args.output / image_path.name
    old_size = json.loads(destination.read_text())["image_size"] if destination.exists() else None
    patch = subprocess.check_output(["git", "-C", str(ROOT), "diff", "--binary", "HEAD"], text=True)
    commands = json.loads((build / "compile_commands.json").read_text())
    module = next(entry for entry in commands if entry["file"].endswith("/tinympc_chicane/tinympc_chicane.cpp"))
    definitions = [arg[2:] for arg in shlex.split(module["command"]) if arg.startswith(("-DTINY", "-DEIGEN"))]
    manifest = {
        "built_at_utc": datetime.now(timezone.utc).isoformat(), "target": args.target,
        "board_id": image["board_id"], "px4_version": subprocess.check_output(
            ["git", "-C", str(args.px4_dir), "describe", "--tags", "--exact-match", "HEAD"], text=True).strip(),
        "repository_commit": subprocess.check_output(["git", "-C", str(ROOT), "rev-parse", "HEAD"], text=True).strip(),
        "source_id": source_id, "tracked_diff_sha256": hashlib.sha256(patch.encode()).hexdigest(),
        "source_files_sha256": hashes,
        "compiler_version": subprocess.check_output([str(compiler), "--version"], text=True).splitlines()[0],
        "module_compile_definitions": definitions,
        "software_validation": validation,
        "metrics_schema": 1, "firmware_sha256": sha256(image_path), "elf_sha256": sha256(elf),
        "image_size_bytes": image["image_size"], "previous_packaged_image_size_bytes": old_size,
        "image_size_delta_bytes": image["image_size"] - old_size if old_size is not None else None,
        "elf_text_bytes": text, "elf_data_bytes": data, "elf_bss_bytes": bss,
        "elf_static_ram_bytes": data + bss,
        "logger_registration_sha256": sha256(args.px4_dir / "src/modules/logger/logged_topics.cpp"),
        "diagnostic_schema_sha256": sha256(ROOT / "px4_external/msg/TinympcStatus.msg"),
        "hardware_validation": "not run on a physical Pixhawk",
        "hardware_modes": ["monitor", "hold"],
    }
    shutil.copyfile(image_path, destination)
    manifest_path = args.output / "build_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    (args.output / "SHA256SUMS").write_text("".join(
        f"{sha256(path)}  {path.name}\n" for path in (destination, manifest_path)))
    print(f"Packaged {destination}\nsource_id={source_id}\n{size_output.strip()}")


if __name__ == "__main__":
    main()
