#!/usr/bin/env python3
"""Register the external TinyMPC topic in the pinned PX4 logger, idempotently."""

import argparse
from pathlib import Path


def prepare(px4_dir: Path) -> None:
    path = px4_dir / "src/modules/logger/logged_topics.cpp"
    contents = path.read_text()
    marker = '\t// TinyMPC external diagnostics: keep these even with a custom topic file.\n'
    addition = marker + '\tadd_topic("tinympc_status", 0);\n\tadd_topic("control_allocator_status", 20);\n'
    if marker in contents:
        if addition not in contents:
            raise RuntimeError(f"Unexpected existing TinyMPC logger registration in {path}")
        return
    anchor = '\treturn _subscriptions.count > 0;\n'
    start = contents.index('bool LoggedTopics::initialize_logged_topics(')
    before, after = contents[:start], contents[start:]
    if after.count(anchor) != 1:
        raise RuntimeError(f"Unsupported PX4 logger layout in {path}; expected v1.15.3")
    path.write_text(before + after.replace(anchor, addition + '\n' + anchor, 1))
    print("Enabled default ULog recording of tinympc_status and allocator status.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("px4_dir", type=Path)
    prepare(parser.parse_args().px4_dir)
