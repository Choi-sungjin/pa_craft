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
    parser.add_argument('--format', choices=('ncnn', 'onnx'), default='ncnn',
                        help='onnx: input for trtexec on the Jetson (TensorRT backend)')
    args = parser.parse_args()
    from ultralytics import YOLO, __version__
    # Export from a temporary copy, preserving the training weight directory.
    with tempfile.TemporaryDirectory(prefix='craft-ncnn-') as temp:
        copied = Path(temp) / args.weights.name
        shutil.copy2(args.weights, copied)
        model = YOLO(copied)
        args.output.mkdir(parents=True, exist_ok=True)
        if args.format == 'onnx':
            # nms=None keeps the one-to-many head: output0 [1,4+classes,anchors], same as ncnn.
            # (nms=False selects the end-to-end head, nms=True appends NMS; decode_yolo accepts neither.)
            exported = Path(model.export(format='onnx', imgsz=640, device='cpu', nms=None, opset=17))
            import onnx
            graph = onnx.load(exported).graph
            dims = [d.dim_value for d in graph.output[0].type.tensor_type.shape.dim]
            if len(graph.output) != 1 or dims[:2] != [1, 4 + len(model.names)]:
                raise SystemExit(f'unexpected ONNX output {graph.output[0].name} {dims}')
            shutil.copy2(exported, args.output / 'model.onnx')
            print('ONNX', graph.input[0].name, '->', graph.output[0].name, dims)
            print('On the Jetson: trtexec --onnx=model.onnx --saveEngine=model.engine --fp16')
            return
        exported = Path(model.export(format='ncnn', imgsz=640, device='cpu'))
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
