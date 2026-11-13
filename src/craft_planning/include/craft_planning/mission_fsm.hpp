#pragma once
// Mission state machine of the dice memory mission (발제 2026-11-04, v5). One level above StateMachine (which runs
// one planning task): it decides what the robot does next and hands out one MissionAction at a time; whoever runs
// it (mission_master.cpp) reports each action back with action_done(). No ROS, no clock of its own.
//
//   Idle -> WatchSignal -> ReadBarcode -> Collect -> Allocate -> Deliver -> Verify -> Return -> Done
//   any state -> Fault (unregistered barcode, invalid signal, a timeout, a failed action, no allocation)
//
// Collect: every start box in turn: drive up, look, pick its pucks into the clock (12 slots hold all 12 pucks, so
//          nothing is left behind and the allocation is made with every puck on board).
// Deliver: L1, L2, L3 in turn: drive up, take the group's pucks out of the clock onto the dice spots.
// Verify:  every location again: drive up, look, record what stands there (a new observation, not the plan).
// Return:  to the start zone, stand still.
#include "craft_planning/mission_logic.hpp"

#include <array>
#include <deque>
#include <map>
#include <string>
#include <vector>

namespace craft_planning::mission {

enum class MissionState {Idle, WatchSignal, ReadBarcode, Collect, Allocate, Deliver, Verify, Return, Done, Fault};
const char *name(MissionState s);

struct MissionAction {
  enum class Kind {
    None,
    ResetPerception,  // clear the blink counter, the barcode and the puck survey
    ApproachBox,      // drive up to `box` and dock (planning /planning/approach_box)
    LeaveBox,         // back straight out of the dock at `box` (planning /planning/leave_box); handed out by
                      // tick() before the next drive (ApproachBox / GoHome) whenever the robot is docked
    ObservePucks,     // at start box `box`: look at its pucks (id, colour, shape, position)
    PickPuck,         // at start box `box`: pick its puck number `index` into clock slot `slot`
    PlacePuck,        // at location `box`: take puck `puck_id` out of clock slot `slot` onto grid spot `spot`
    VerifyLocation,   // at location `box`: look and record the pucks standing there
    GoHome,           // drive back into the start zone
    HoldStill,        // stand still for `seconds`
  };
  Kind kind = Kind::None;
  std::string box;
  int index = 0, slot = 0, puck_id = -1;
  std::array<int, 2> spot{0, 0};  // dice_spots() grid steps
  double seconds = 0;
  std::string describe() const;
};
const char *name(MissionAction::Kind k);

struct MissionConfig {
  std::vector<std::string> start_boxes{"S1", "S2", "S3", "S4", "S5", "S6"};  // in visiting order
  std::array<std::string, 3> locations{"L1", "L2", "L3"};
  int pucks_per_box = 2;
  double barcode_timeout_s = 0, signal_timeout_s = 0;  // 0 = wait without a limit
  double home_still_s = 3.0;
  // Seconds an action may take before the mission faults, by kind name (name(Kind)); missing or 0 = no limit.
  std::map<std::string, double> action_timeout_s;
  // true: Verify drives to every location again after all three are placed (the brief's order).
  // false: each location is verified right after its pucks are placed, before leaving it.
  bool verify_after_all = true;
  // The pick does not report which puck it took (PickPuck is a skeleton): hand the surveyed ids out to the unknown
  // slots, lowest first, so the rest of the mission can be walked through. false: an unknown slot is a fault.
  bool assume_unknown_picks = false;
  // Drive-only run (no arm, no barcode; user 2026-11-12): start -> WatchSignal, and once the signal is complete
  // drive up to every start box, then every location, then home. No ObservePucks / PickPuck / PlacePuck /
  // VerifyLocation, no allocation; the signal is still recorded and checked.
  bool drive_only = false;
};

// What the mission sees of the world; copied in on every tick.
struct MissionInputs {
  std::string barcode;        // confirmed Code 128 text; empty = not read yet
  bool signal_complete = false, signal_valid = false;
  SignalCounts counts;
  std::vector<uint8_t> sequence;
  std::vector<Puck> survey;   // pucks identified so far
};

struct Placement {
  std::string location;
  Group group = Group::Red;
  int puck_id = -1, slot = 0;
  std::array<int, 2> spot{0, 0};
};

class MissionFsm {
public:
  explicit MissionFsm(MissionConfig config = {});
  // Idle / Done / Fault -> WatchSignal; false while a mission is running.
  bool start(double now);
  void fail(const std::string &reason);
  // The next action to run, or None while one is running / the mission waits / it is over.
  MissionAction tick(const MissionInputs &inputs, double now);
  // Result of the action tick() handed out last. puck_id: the puck a PickPuck took (-1 = not known).
  void action_done(bool ok, const std::string &reason = {}, int puck_id = -1);

  MissionState state() const {return state_;}
  bool running() const {return state_ != MissionState::Idle && state_ != MissionState::Done && state_ != MissionState::Fault;}
  bool busy() const {return busy_;}
  const MissionAction &action() const {return action_;}
  const std::string &reason() const {return reason_;}
  std::string label() const;  // state, and the action being run
  // Decisions so far (valid once the state that makes them is passed).
  const std::string &barcode() const {return barcode_;}
  std::optional<SortOrder> order() const {return order_;}
  const SignalCounts &counts() const {return counts_;}
  const std::vector<uint8_t> &sequence() const {return sequence_;}
  const std::array<Group, 3> &groups() const {return groups_;}        // group of L1, L2, L3
  const Allocation &allocation() const {return allocation_;}
  const std::map<int, int> &inventory() const {return inventory_;}    // clock slot -> puck id (-1 = not known)
  const std::vector<Placement> &placements() const {return placements_;}
  const std::string &docked_at() const {return docked_at_;}           // box docked at, empty = not docked

private:
  void enter(MissionState state, double now);
  void plan_collect();
  bool plan_deliver(const MissionInputs &inputs);
  void push(MissionAction::Kind kind, const std::string &box = {});

  MissionConfig config_;
  MissionState state_ = MissionState::Idle;
  double since_ = 0, action_since_ = 0;
  std::deque<MissionAction> queue_;
  MissionAction action_;
  bool busy_ = false;
  std::string reason_, barcode_, docked_at_;
  std::optional<SortOrder> order_;
  SignalCounts counts_;
  std::vector<uint8_t> sequence_;
  std::array<Group, 3> groups_{Group::Red, Group::Yellow, Group::Green};
  Allocation allocation_;
  std::map<int, int> inventory_;
  std::vector<Placement> placements_;
};

}  // namespace craft_planning::mission
