#!/usr/bin/env python3
"""Package installed CRAFT nodes and resources, without source/build symlinks."""
import argparse
import json
import platform
import shutil
import subprocess
import tarfile
import tempfile
from pathlib import Path

PACKAGES = (
    'coin_d4_driver', 'craft_interfaces', 'craft_description', 'lidar_processor',
    'craft_control', 'craft_perception', 'craft_sensors', 'craft_planning', 'craft_bringup',
)


def main():
    repo = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--install', type=Path, default=repo / 'install_runtime')
    parser.add_argument('--output', type=Path, default=repo / 'dist/craft-runtime.tar.gz')
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='craft-runtime-') as temporary:
        root = Path(temporary) / 'craft-runtime'
        installed = root / 'install'
        installed.mkdir(parents=True)
        for package in PACKAGES:
            src = args.install / package
            if not (src / 'share/ament_index/resource_index/packages' / package).is_file():
                raise RuntimeError(f'Package not installed: {package}. Run tools/build_runtime.sh first.')
            # Dereference colcon --symlink-install; the archive must not need the source checkout.
            shutil.copytree(src, installed / package, symlinks=False)
        for src in args.install.iterdir():
            if src.is_file():
                shutil.copy2(src, installed / src.name)
        shutil.copy2(repo / 'docs/DEPLOYMENT.md', root / 'README.md')
        revision = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=repo, text=True).strip()
        dirty = bool(subprocess.check_output(['git', 'status', '--porcelain'], cwd=repo, text=True).strip())
        (root / 'manifest.json').write_text(json.dumps({
            'source_revision': revision, 'source_dirty': dirty,
            'architecture': platform.machine(), 'packages': PACKAGES,
            'scope': 'Existing implementation; ObservePucks/PlacePuck/VerifyLocation remain unimplemented',
        }, ensure_ascii=False, indent=2) + '\n')
        with tarfile.open(args.output, 'w:gz') as archive:
            archive.add(root, arcname=root.name)
    print(args.output.resolve())


if __name__ == '__main__':
    main()
