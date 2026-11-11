#!/usr/bin/env bash
# Rebuild model/ (V186 full-robot URDF + visual meshes, 38 MB, not in git) from the split delivery in ~.
# Usage: ./get_model.sh [delivery folder containing PARTS_MANIFEST.json]
set -e
cd "$(dirname "$(readlink -f "$0")")"
SRC=${1:-$(dirname "$(ls -d ~/V186_RL_THREE_SETS_*/V186_RL_THREE_SETS_*/PARTS_MANIFEST.json | head -1)")}
TMP=$(mktemp -d); trap 'rm -rf "$TMP"' EXIT
/usr/bin/python3 -I - "$SRC" "$TMP" <<'PY'
import hashlib, json, pathlib, sys
src, out = map(pathlib.Path, sys.argv[1:])
info = json.load(open(src / 'PARTS_MANIFEST.json'))['V186_C_full_robot.zip']
data = b''.join((src / p['file']).read_bytes() for p in info['parts'])
if hashlib.sha256(data).hexdigest() != info['sha256']:
    sys.exit('V186_C_full_robot.zip: SHA-256 mismatch')
(out / 'c.zip').write_bytes(data)
PY
unzip -q "$TMP/c.zip" 'V186_DELIVERY/C_full_robot/urdf/*' 'V186_DELIVERY/C_full_robot/meshes/visual/*' -d "$TMP"
rm -rf model && mkdir -p model/meshes
mv "$TMP/V186_DELIVERY/C_full_robot/urdf" model/ && mv "$TMP/V186_DELIVERY/C_full_robot/meshes/visual" model/meshes/
echo "model/ ready: $(du -sh model | cut -f1)"
