#!/usr/bin/env python3
"""Development-only export. Runtime perception is C++; no Python model loader is used."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tempfile


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('weights', type=Path)
    parser.add_argument('--output', type=Path,
                        default=Path(__file__).resolve().parents[1] / 'models' / 'knob_sign')
    args = parser.parse_args()
    from ultralytics import YOLO, __version__
    # Export from a temporary copy, preserving the training weight directory.
    with tempfile.TemporaryDirectory(prefix='craft-ncnn-') as temp:
        copied = Path(temp) / args.weights.name
        shutil.copy2(args.weights, copied)
        model = YOLO(copied)
        exported = Path(model.export(format='ncnn', imgsz=640, device='cpu'))
        args.output.mkdir(parents=True, exist_ok=True)
        for name in ('model.ncnn.param', 'model.ncnn.bin', 'metadata.yaml'):
            shutil.copy2(exported / name, args.output / name)
        provenance = {
            'source_weights': args.weights.name,
            'source_sha256': hashlib.sha256(args.weights.read_bytes()).hexdigest(),
            'ultralytics': __version__, 'input': [1, 3, 640, 640],
            'class_names': model.names,
            'output_contract': '[4+class_count,anchors], decoded cxcywh + probabilities; class-aware NMS',
        }
        (args.output / 'provenance.json').write_text(json.dumps(provenance, indent=2) + '\n')
        print('NCNN export saved:', args.output)
        print('Match class_names in config/perception.yaml to:', model.names)


if __name__ == '__main__':
    main()
