#!/usr/bin/env bash
# Run the voice node straight from this folder, without a colcon build: for a robot whose workspace other
# people are building in. Only /opt/ros is needed; craft_interfaces / craft_perception messages are used
# when the robot workspace is already built (control status, YOLO objects, pucks), and left out otherwise.
#
#   scripts/run_voice.sh                         # config/voice_robot.yaml
#   scripts/run_voice.sh config/voice.yaml       # another parameter file
#   CRAFT_VOICE_PREPARE=0 scripts/run_voice.sh   # skip synthesizing the fixed sentences first
#
# Environment: ROS_DOMAIN_ID (default 61), CRAFT_WS (default ~/craft), CRAFT_VOICE_PYTHON (default python3),
# OPENAI_API_KEY or a key file (see craft_voice/envfile.py).
set -e
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PARAMS="${1:-$HERE/config/voice_robot.yaml}"
[ $# -gt 0 ] && shift
set +u
source "/opt/ros/${ROS_DISTRO:-lyrical}/setup.bash"
WS="${CRAFT_WS:-$HOME/craft}"
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-61}"
export PYTHONPATH="$HERE:$PYTHONPATH"
export PYTHONUNBUFFERED=1
PY="${CRAFT_VOICE_PYTHON:-python3}"
cd "$HERE"

value() {  # value of one parameter in the parameter file ('' if it is not there)
  "$PY" - "$PARAMS" "$1" <<'PYEOF'
import re, sys
for line in open(sys.argv[1], encoding='utf-8'):
    match = re.match(r'\s+' + re.escape(sys.argv[2]) + r':\s*(.*?)\s*(#.*)?$', line)
    if match:
        print(match.group(1).strip('"').strip("'"))
        break
PYEOF
}

if [ "${CRAFT_VOICE_PREPARE:-1}" = "1" ]; then
  # Fixed sentences (state changes, blocked path, faults, "응!") are synthesized once and kept, so they
  # play at once and also without the network. Already kept sentences are skipped; a failure is not fatal.
  STYLE="$(value style)"; MODEL="$(value tts_model)"; VOICE="$(value tts_voice)"; HOW="$(value tts_instructions)"
  [ "$(value variety)" = "true" ] && VARIETY=--variety || VARIETY=
  "$PY" -m craft_voice.prepare --style "${STYLE:-polite}" $VARIETY ${MODEL:+--tts-model "$MODEL"} ${VOICE:+--voice "$VOICE"} \
      ${HOW:+--tts-instructions "$HOW"} --step-names "$(value step_names)" || echo "prepare: some sentences were not synthesized" >&2
fi
exec "$PY" -m craft_voice.voice_node --ros-args --params-file "$PARAMS" "$@"
