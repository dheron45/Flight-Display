"""Write manifest.json for the files in a build folder.

    python tools/make_manifest.py build v1.2.0

The board's updater downloads every file listed and checks each one's size and
CRC32 before installing any of them.
"""

import json
import sys
import zlib
from pathlib import Path


def main():
    build_dir, version = Path(sys.argv[1]), sys.argv[2]
    files = []
    for path in sorted(build_dir.rglob("*")):
        if not path.is_file() or path.name == "manifest.json":
            continue
        data = path.read_bytes()
        files.append({
            "path": path.relative_to(build_dir).as_posix(),
            "size": len(data),
            "crc32": zlib.crc32(data) & 0xFFFFFFFF,
        })
    if not files:
        sys.exit("No files in " + str(build_dir))
    manifest = {"version": version, "files": files}
    (build_dir / "manifest.json").write_text(json.dumps(manifest, indent=1) + "\n")
    print(json.dumps(manifest, indent=1))


if __name__ == "__main__":
    main()
