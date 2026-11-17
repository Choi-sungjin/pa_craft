// Literal defaults from demo/puck_pick/execute.py, kept separate from runtime YAML/JSON.
// Runtime CLI overrides are applied by configure().
#pragma once
#include <nlohmann/json.hpp>
namespace puck::execute {
inline nlohmann::ordered_json default_constants() {
 return nlohmann::ordered_json::parse(R"PORTJSON({
 "ARM": [
  "slide_joint",
  "shoulder_joint",
  "elbow_joint",
  "wrist_pitch_joint",
  "tool_roll_joint"
 ],
 "STATUS": {
  "0": "RUNNING",
  "1": "SUCCEEDED",
  "2": "FAILED",
  "3": "STOPPING",
  "4": "STOPPED",
  "5": "REJECTED"
 },
 "LOAD_URDF_DEG": [
  15.68,
  4.85,
  -107.42,
  0.0
 ],
 "LOAD_SLIDE": 0.0052,
 "RELEASE_GRIPPER_DEG": 147.9,
 "BACK_ROLL_URDF_DEG": -90.9,
 "LOAD_SLOT": 12,
 "BIGBRO_SLOTS": [
  12
 ],
 "BIGBRO_SLOT": 12,
 "NOMINAL_RAW_DEG": [
  561.4,
  53.9,
  178.0,
  58.4,
  0.0,
  73.1
 ],
 "VIA_RAW_DEG": [
  561.4,
  93.3,
  177.6,
  61.3,
  -0.4,
  73.1
 ],
 "CARRY_VIA_RAW_DEG": [
  561.4,
  92.5,
  182.9,
  104.6,
  90.0,
  141.9
 ],
 "CARRY_SHOULDER_OUT_DEG": 5.0,
 "PICK_VIA_RAW_DEG": [
  548.2,
  121.8,
  97.0,
  17.1,
  0.0,
  134.3
 ],
 "PRE_GRASP_RAW_DEG": [
  548.2,
  92.06,
  118.9,
  132.96,
  90.0,
  90.0
 ],
 "PICK_LIFT_RAW_DEG": [
  548.2,
  104.2,
  137.8,
  146.3,
  0.0,
  168.4
 ],
 "SLOW_SECONDS": {},
 "GOTO_DEG_S": 10.0,
 "VIA_DEG_S": 20.0,
 "STAND_VIA_DEG_S": 40.0,
 "APPROACH_DEG": 4.0,
 "APPROACH_DEG_S": 1.5,
 "APPROACH_MIN_S": 2.0,
 "GOTO_MIN_S": 3.0,
 "STEP_SECONDS": {
  "sis2 1:": 0.0,
  "sis2 2:": 0.0,
  "sis2 4:": 0.0,
  "sis2 6:": 0.0,
  "sis2 7:": 0.0,
  "sis2 8:": 0.0,
  "sis2 9:": 0.0,
  "sis2 10:": 0.0,
  "sis2 12:": 0.0,
  "sis2 13:": 0.0,
  "sis2 14:": 0.0,
  "put down on the": 2.0,
  "bigbro pick: descend": 2.0,
  "stand pick: descend": 2.0
 },
 "PLACE_RAW_DEG": [
  593.2,
  99.5,
  269.3,
  -68.1,
  90.0
 ],
 "PLACE_ABOVE_RAW_DEG": [
  593.2,
  99.9,
  256.6,
  -80.3,
  90.0
 ],
 "PLACE_RELEASE_ROLL_RAW_DEG": 0.0,
 "DICE_ROWS": {
  "far": {
   "0": [
    115.0,
    225.23,
    -84.97
   ],
   "40": [
    117.58,
    203.59,
    -100.54
   ],
   "75": [
    111.8,
    196.69,
    -101.72
   ]
  },
  "middle": {
   "0": [
    97.45,
    257.61,
    -70.73
   ],
   "40": [
    100.38,
    237.77,
    -87.65
   ],
   "75": [
    100.36,
    215.89,
    -102.05
   ]
  },
  "near": {
   "0": [
    84.62,
    274.57,
    -68.02
   ],
   "40": [
    86.61,
    257.32,
    -83.27
   ],
   "75": [
    89.31,
    231.59,
    -102.3
   ]
  }
 },
 "DICE_COLS_LM_RAW_DEG_TAUGHT": {
  "left": 141.3,
  "centre": 593.35,
  "right": 1045.4
 },
 "DICE_ROWS_1107_1417": {
  "far": {
   "0": [
    128.65,
    212.37,
    -95.9
   ],
   "40": [
    119.75,
    210.33,
    -98.83
   ],
   "75": [
    110.2,
    212.18,
    -97.54
   ]
  },
  "middle": {
   "0": [
    103.7,
    263.61,
    -69.61
   ],
   "40": [
    106.61,
    234.35,
    -95.95
   ],
   "75": [
    97.07,
    227.14,
    -101.71
   ]
  },
  "near": {
   "0": [
    88.22,
    282.87,
    -65.83
   ],
   "40": [
    89.83,
    256.87,
    -90.22
   ],
   "75": [
    84.13,
    239.95,
    -103.84
   ]
  }
 },
 "DICE_ROWS_0329": {
  "far": {
   "0": [
    130.43,
    213.48,
    -93.01
   ],
   "40": [
    122.4,
    206.69,
    -99.83
   ],
   "75": [
    113.7,
    205.16,
    -101.06
   ]
  },
  "middle": {
   "0": [
    107.02,
    258.68,
    -71.22
   ],
   "40": [
    110.62,
    227.68,
    -98.61
   ],
   "75": [
    100.79,
    222.06,
    -103.07
   ]
  },
  "near": {
   "0": [
    88.49,
    281.42,
    -67.01
   ],
   "40": [
    90.36,
    255.07,
    -91.49
   ],
   "75": [
    84.93,
    237.75,
    -105.24
   ]
  }
 },
 "DICE_COLS_LM_RAW_DEG_0329": {
  "left": 142.0,
  "centre": 548.3,
  "right": 1046.2
 },
 "DICE_EX_MM": [
  -9.83,
  -0.0298,
  12.65
 ],
 "DICE_EY_MM": {
  "far": 6.5,
  "middle": 10.5,
  "near": 1.0
 },
 "DICE_BOX_X_MM": 2.8,
 "DICE_GRID_MM": 60.0,
 "DICE_COL_STEPS": {
  "left": -1,
  "centre": 0,
  "right": 1
 },
 "DICE_CAL_LM_RAW_DEG": {
  "away": 142.0,
  "mid": 594.1,
  "motor": 1046.2
 },
 "DICE_PLACE_ROLL_RAW_DEG": 0.0,
 "DICE_FACES": {
  "1": [
   [
    "middle",
    "centre"
   ]
  ],
  "2": [
   [
    "near",
    "right"
   ],
   [
    "far",
    "left"
   ]
  ],
  "3": [
   [
    "near",
    "right"
   ],
   [
    "middle",
    "centre"
   ],
   [
    "far",
    "left"
   ]
  ],
  "4": [
   [
    "near",
    "left"
   ],
   [
    "near",
    "right"
   ],
   [
    "far",
    "left"
   ],
   [
    "far",
    "right"
   ]
  ],
  "5": [
   [
    "near",
    "left"
   ],
   [
    "near",
    "right"
   ],
   [
    "middle",
    "centre"
   ],
   [
    "far",
    "left"
   ],
   [
    "far",
    "right"
   ]
  ],
  "6": [
   [
    "near",
    "left"
   ],
   [
    "near",
    "right"
   ],
   [
    "middle",
    "left"
   ],
   [
    "middle",
    "right"
   ],
   [
    "far",
    "left"
   ],
   [
    "far",
    "right"
   ]
  ],
  "cal": [
   [
    "far",
    "mid"
   ],
   [
    "middle",
    "mid"
   ],
   [
    "near",
    "mid"
   ]
  ]
 },
 "DICE_FACE": 6,
 "DICE_CAL_COL": "mid",
 "_ROW_ORDER": {
  "near": 0,
  "middle": 1,
  "far": 2
 },
 "_COL_ORDER": {
  "left": 0,
  "centre": 1,
  "right": 2
 },
 "DICE_SLOTS": [
  12,
  1,
  2,
  3,
  4,
  5
 ],
 "PICK_MID_RAW_DEG": [
  548.2,
  88.23,
  137.1,
  146.83,
  90.0
 ],
 "PICK_ABOVE_RAW_DEG": [
  548.2,
  98.87,
  140.13,
  146.5,
  90.0
 ],
 "PICK_DESCEND_FROM_RAW_DEG": [
  548.2,
  98.87,
  140.13,
  148.0,
  90.0
 ],
 "PICK_OPEN_GRIPPER_DEG": 94.0,
 "LM_BACKLASH": 0.002,
 "UNLOAD_DEPTH": 0.03,
 "UNLOAD_ABOVE": 0.06,
 "PUT_DOWN_ABOVE": 0.01,
 "NO_STAGING": [
  "arm over the slot",
  "straight down to the grasp",
  "straight up ",
  "descend",
  "lift the puck",
  "put down",
  "rise above the puck",
  "sis2 ",
  "dice ",
  "bigbro ",
  "stand ",
  "standcal ",
  "standfind ",
  "lie ",
  "lie regrasp",
  "via pose:",
  "via pose (off",
  "to the nominal pose"
 ],
 "STAGE_SEG": 1000,
 "LYING_HIGH": 0.1,
 "LYING_SWING": 20,
 "PLAN_FILES": [
  "plan.yaml"
 ],
 "PLAN_FILE": "plan.yaml",
 "DICE_LINE_STEP": 0.015,
 "DICE_FINAL_DROP": 0.01,
 "DICE_TILT_MAX_DEG": 30.0,
 "STAND_ROLL_RAW_DEG": 90.0,
 "STAND_GRASP_ROWS": {
  "far": [
   112.8,
   238.9,
   -73.5
  ],
  "middle": [
   96.9,
   267.4,
   -61.5
  ],
  "near": [
   84.4,
   283.8,
   -59.0
  ]
 },
 "STAND_RELEASE_MM": 10.0,
 "LOAD_BACK_MM": 2.0,
 "STAND_BASE": {},
 "STAND_GRASP_TAUGHT": {
  "2": "stand_grasp_c2",
  "5": "stand_grasp_c5",
  "8": "stand_grasp_c8"
 },
 "STAND_GRASP_AHEAD_FIX_MM": -10.0,
 "STAND_X_FIX_TABLE": [
  [
   42.1,
   -10.0
  ],
  [
   97.0,
   -2.0
  ],
  [
   157.0,
   -10.0
  ]
 ],
 "STAND_APPROACH_X_FIX_TABLE": [
  [
   52.5,
   0.0
  ],
  [
   97.0,
   0.0
  ]
 ],
 "STAND_Y_FIX_TABLE": [
  [
   42.1,
   4.0
  ],
  [
   97.0,
   0.0
  ]
 ],
 "STAND_LM_Y_FIX_TABLE": [
  [
   -40.0,
   -2.4
  ],
  [
   12.2,
   0.0
  ]
 ],
 "STAND_YAW_Y_FIX_TABLE": {
  "97.0": [
   [
    -26.0,
    1.0
   ],
   [
    0.0,
    0.0
   ],
   [
    26.0,
    4.0
   ]
  ],
  "157.0": [
   [
    -26.0,
    1.0
   ],
   [
    0.0,
    0.0
   ],
   [
    26.0,
    2.0
   ]
  ]
 },
 "STAND_YAW_X_FIX_TABLE": [
  [
   -26.0,
   6.0
  ],
  [
   0.0,
   0.0
  ],
  [
   26.0,
   3.0
  ]
 ],
 "STAND_GRASP_Y_FIX_M": -0.01,
 "ROW_AHEAD_MM": {
  "near": 52.5,
  "middle": 112.5,
  "far": 172.5
 },
 "CELL_ROW": {
  "2": "far",
  "5": "middle",
  "8": "near"
 },
 "STANDCAL_STEP_MM": 10.0,
 "STANDCAL_SHOTS": {},
 "UPPER_AHEAD_BIAS_MM": 20.1,
 "UPPER_LATERAL_BIAS_MM": -12.0,
 "FIND_AHEAD_RANGE_MM": [
  47.5,
  177.5
 ],
 "FIND_MAX_FIX_MM": 25.0,
 "REDOCK_LM_MARGIN_MM": 5.0,
 "REDOCK_MAX_MM": 60.0,
 "FIND": {},
 "STAND_ABOVE_TAUGHT": {
  "2": "stand_above_c2",
  "5": "stand_above_c5",
  "8": "stand_above_c8"
 },
 "STAND_LINE_MM": 40.0,
 "STAND_LINE_PIECES": 4,
 "STAND_DESCEND_PIECE_S": 0.6,
 "STAND_GRASP_UP_MM": 0.0,
 "STAND_APPROACH_UP_MM": 3.0,
 "POLICY_CANDIDATES": [],
 "FIND_FINAL_MAX_MM": 6.0,
 "FIND_FINAL_MIN_MM": 1.5,
 "LIE_GRASP_TAUGHT": {
  "2": "lie_grasp_c2",
  "5": "lie_grasp_c5",
  "8": "lie_grasp_c8"
 },
 "LIE_ABOVE_TAUGHT": {},
 "LIE_LINE_MM": 30.0,
 "LIE_PLACE_CELL": 8,
 "LIE_PICK_CELLS": [
  2,
  5
 ],
 "LIE_REGRASP_ABOVE": "lie_regrasp_above_c8",
 "LIE_REGRASP_ROLL_RAW_DEG": 180.0,
 "LIE_PLACE_YAW_DEG": 90.0,
 "LIE_MAX_LM_FIX_MM": 25.0,
 "LIE_PICK_Y_FIX_MM": 14.0,
 "BIGBRO_SLOT_LM": -0.002,
 "BIGBRO_RELEASE_LOWER_MM": 18.0,
 "AUTO": false,
 "VIA_TOLERANCE_DEG": 3.0,
 "LOOSE_STEPS": [
  "via pose",
  "up to 75 mm, one move",
  "shoulder I off"
 ],
 "TRANSIT_TOLERANCE_DEG": 1.5,
 "TRANSIT_STEPS": [
  "approach",
  "over cell",
  "over clock slot",
  "up to 40 mm over the slot",
  "to the nominal pose",
  "lie regrasp 2"
 ],
 "I_GAINS_BASE": [
  130,
  100,
  200,
  100
 ],
 "STEP_I_GAINS": {
  "pick: up to 40 mm over the slot": [
   100,
   100,
   200,
   100
  ],
  "put down on the": [
   130,
   80,
   160,
   100
  ],
  "up to 75 mm, one move": [
   130,
   80,
   160,
   100
  ],
  "shoulder I off": [
   0,
   100,
   200,
   100
  ]
 },
 "STEP_CONTACT": {},
 "DESCEND_PASS_DEG": 1.0,
 "DESCEND_PIECE_S": 1.0,
 "LM_START_TOL": 0.02,
 "STANDFIND": "standfind",
 "STANDCAL": "standcal",
 "PAUSE": "pause",
 "LIEYAW": "lieyaw",
 "STANDYAW": "standyaw",
 "STAND_JAW_OFF_M": 0.004,
 "STAND_ROLL_PER_YAW": 1.0,
 "STAND_YAW_MIN_DEG": 1.0,
 "GRIPCHECK": "gripcheck",
 "HOLD_BEFORE_CLOSE_S": 0.0,
 "LOOK_STEPS": [
  "bigbro pick: move 1",
  "bigbro pick: move 2",
  "bigbro fix"
 ],
 "GRIPCHECK_MM": 10.0,
 "GRIPFIX_MM": 3.0,
 "NO_GRIPFIX": false,
 "MAX_REACH": 0.15,
 "GRIPFIX_MAX_MM": 35.0,
 "GRIPFIX_TRIES": 3,
 "GRIPFIX_DZ_MM": 4.0,
 "GRIP_Z_PER_SIDE": 0.375,
 "GRIP_DZ_TO_UP": 1.08,
 "GRIPFIX_ROLL_DEG": 3.0,
 "GRIPCAL": "gripcal",
 "GRIPCAL_MM": 8.0,
 "GRIP_TURN_SIGN": 1.0,
 "GRIP_TURN_ZERO_DEG": 0.0,
 "GRIP_TURN_MIN_DEG": 6.0,
 "HELD_CHECK": true,
 "GRIPZERO": "gripzero"
})PORTJSON");
}
}
