#include "craft_planning/mission_fsm.hpp"

#include <algorithm>
#include <set>
#include <stdexcept>

namespace craft_planning::mission {

const char *name(MissionState s) {
  switch (s) {
    case MissionState::Idle: return "Idle";
    case MissionState::ReadBarcode: return "ReadBarcode";
    case MissionState::WatchSignal: return "WatchSignal";
    case MissionState::Collect: return "Collect";
    case MissionState::Allocate: return "Allocate";
    case MissionState::Deliver: return "Deliver";
    case MissionState::Verify: return "Verify";
    case MissionState::Return: return "Return";
    case MissionState::Done: return "Done";
    case MissionState::Fault: return "Fault";
  }
  return "Fault";
}

const char *name(MissionAction::Kind k) {
  using Kind = MissionAction::Kind;
  switch (k) {
    case Kind::None: return "none";
    case Kind::ResetPerception: return "reset_perception";
    case Kind::ApproachBox: return "approach_box";
    case Kind::LeaveBox: return "leave_box";
    case Kind::ObservePucks: return "observe_pucks";
    case Kind::PickPuck: return "pick_puck";
    case Kind::PlacePuck: return "place_puck";
    case Kind::VerifyLocation: return "verify_location";
    case Kind::GoHome: return "go_home";
    case Kind::HoldStill: return "hold_still";
  }
  return "none";
}

std::string MissionAction::describe() const {
  std::string text = name(kind);
  if (!box.empty()) text += " " + box;
  if (kind == Kind::PickPuck) text += " #" + std::to_string(index) + " -> slot " + std::to_string(slot);
  if (kind == Kind::PlacePuck) {
    text += " puck " + std::to_string(puck_id) + " from slot " + std::to_string(slot) + " -> spot (" +
      std::to_string(spot[0]) + ", " + std::to_string(spot[1]) + ")";
  }
  if (kind == Kind::HoldStill) text += " " + std::to_string(seconds).substr(0, 4) + " s";
  return text;
}

MissionFsm::MissionFsm(MissionConfig config) : config_(std::move(config)) {
  if (config_.start_boxes.empty() || config_.pucks_per_box < 1 ||
      config_.start_boxes.size() * config_.pucks_per_box != 12) {
    throw std::invalid_argument("mission: start boxes x pucks per box must be 12");
  }
  std::set<std::string> names(config_.start_boxes.begin(), config_.start_boxes.end());
  names.insert(config_.locations.begin(), config_.locations.end());
  if (names.size() != config_.start_boxes.size() + 3 || names.count("")) {
    throw std::invalid_argument("mission: start boxes and locations must be distinct, nonempty names");
  }
  if (config_.barcode_timeout_s < 0 || config_.signal_timeout_s < 0 || config_.home_still_s < 0) {
    throw std::invalid_argument("mission: timeouts must not be negative");
  }
}

void MissionFsm::enter(MissionState state, double now) {state_ = state; since_ = now;}

void MissionFsm::push(MissionAction::Kind kind, const std::string &box) {
  MissionAction a; a.kind = kind; a.box = box; queue_.push_back(a);
}

bool MissionFsm::start(double now) {
  if (running()) return false;
  queue_.clear(); busy_ = false; action_ = {}; reason_.clear(); barcode_.clear(); order_.reset(); docked_at_.clear();
  counts_ = {}; sequence_.clear(); allocation_ = {}; inventory_.clear(); placements_.clear();
  push(MissionAction::Kind::ResetPerception);
  enter(MissionState::WatchSignal, now);  // user 2026-11-13: the signal first, then the barcode
  return true;
}

void MissionFsm::fail(const std::string &reason) {
  if (state_ == MissionState::Fault) return;  // the first cause stays
  reason_ = std::string(name(state_)) + ": " + reason;
  state_ = MissionState::Fault; queue_.clear(); busy_ = false;
}

void MissionFsm::plan_collect() {
  int slot = 1;
  for (const auto &box : config_.start_boxes) {
    push(MissionAction::Kind::ApproachBox, box);
    push(MissionAction::Kind::ObservePucks, box);
    for (int i = 0; i < config_.pucks_per_box; ++i) {
      MissionAction a; a.kind = MissionAction::Kind::PickPuck; a.box = box; a.index = i; a.slot = slot++;
      queue_.push_back(a);
    }
  }
}

bool MissionFsm::plan_deliver(const MissionInputs &inputs) {
  if (inputs.survey.size() != 12) {
    fail("puck survey has " + std::to_string(inputs.survey.size()) + " of 12 pucks"); return false;
  }
  allocation_ = allocate(inputs.survey, counts_);
  if (!allocation_.ok) {fail("no allocation: " + allocation_.error); return false;}
  // Which slot holds which puck.
  std::set<int> known;
  int unknown = 0;
  for (const auto &[slot, id] : inventory_) {if (id < 0) ++unknown; else known.insert(id);}
  if (inventory_.size() != 12) {fail(std::to_string(inventory_.size()) + " pucks on board, not 12"); return false;}
  if (unknown && !config_.assume_unknown_picks) {
    fail(std::to_string(unknown) + " picked pucks were not identified"); return false;
  }
  if (known.size() + unknown != 12) {fail("a puck id is on board twice"); return false;}
  std::vector<int> rest;
  for (const auto &p : inputs.survey) if (!known.count(p.id)) rest.push_back(p.id);
  std::sort(rest.begin(), rest.end());
  if (static_cast<int>(rest.size()) != unknown) {fail("a picked puck is not in the survey"); return false;}
  std::size_t next = 0;
  std::map<int, int> slot_of;
  for (auto &[slot, id] : inventory_) {if (id < 0) id = rest[next++]; slot_of[id] = slot;}
  for (int i = 0; i < 3; ++i) {
    const auto &ids = allocation_.of(groups_[i]);
    const auto spots = dice_spots(static_cast<int>(ids.size()));
    push(MissionAction::Kind::ApproachBox, config_.locations[i]);
    for (std::size_t k = 0; k < ids.size(); ++k) {
      MissionAction a; a.kind = MissionAction::Kind::PlacePuck; a.box = config_.locations[i];
      a.puck_id = ids[k]; a.slot = slot_of.at(ids[k]); a.spot = spots[k];
      queue_.push_back(a);
      placements_.push_back({config_.locations[i], groups_[i], a.puck_id, a.slot, a.spot});
    }
    if (!config_.verify_after_all) push(MissionAction::Kind::VerifyLocation, config_.locations[i]);
  }
  return true;
}

MissionAction MissionFsm::tick(const MissionInputs &inputs, double now) {
  if (!running()) return {};
  if (busy_) {
    const auto limit = config_.action_timeout_s.find(name(action_.kind));
    if (limit != config_.action_timeout_s.end() && limit->second > 0 && now - action_since_ >= limit->second) {
      fail(action_.describe() + ": timeout after " + std::to_string(static_cast<int>(limit->second)) + " s");
    }
    return {};
  }
  while (running()) {
    if (!queue_.empty()) {
      const auto next = queue_.front().kind;
      if (!docked_at_.empty() && (next == MissionAction::Kind::ApproachBox || next == MissionAction::Kind::GoHome)) {
        // Docked 20 mm from a face no route can start: back out first.
        action_ = {}; action_.kind = MissionAction::Kind::LeaveBox; action_.box = docked_at_;
        busy_ = true; action_since_ = now;
        return action_;
      }
      action_ = queue_.front(); queue_.pop_front(); busy_ = true; action_since_ = now;
      return action_;
    }
    switch (state_) {
      case MissionState::WatchSignal: {
        if (!inputs.signal_complete) {
          if (config_.signal_timeout_s > 0 && now - since_ >= config_.signal_timeout_s) fail("signal not complete");
          return {};
        }
        counts_ = inputs.counts; sequence_ = inputs.sequence;
        const auto counted = count_sequence(sequence_);
        std::string error = check_signal(counts_);
        if (error.empty() && !inputs.signal_valid) error = "perception marked the signal invalid";
        if (error.empty() && (!counted || counted->red != counts_.red || counted->yellow != counts_.yellow ||
                              counted->green != counts_.green)) error = "signal sequence and counts disagree";
        if (!error.empty()) {fail(error); return {};}
        if (config_.drive_only) {
          for (const auto &box : config_.start_boxes) push(MissionAction::Kind::ApproachBox, box);
          enter(MissionState::Collect, now);
        } else {
          enter(MissionState::ReadBarcode, now);
        }
        break;
      }
      case MissionState::ReadBarcode:
        if (inputs.barcode.empty()) {
          if (config_.barcode_timeout_s > 0 && now - since_ >= config_.barcode_timeout_s) fail("no barcode read");
          return {};
        }
        barcode_ = inputs.barcode; order_ = order_from_barcode(barcode_);
        if (!order_) {fail("unregistered barcode '" + barcode_ + "'"); return {};}
        groups_ = location_groups(counts_, *order_);
        plan_collect();
        enter(MissionState::Collect, now);
        break;
      case MissionState::Collect: enter(MissionState::Allocate, now); break;
      case MissionState::Allocate:
        if (config_.drive_only) {
          for (const auto &location : config_.locations) push(MissionAction::Kind::ApproachBox, location);
        } else if (!plan_deliver(inputs)) {
          return {};
        }
        enter(MissionState::Deliver, now);
        break;
      case MissionState::Deliver:
        if (config_.verify_after_all && !config_.drive_only) {
          for (const auto &location : config_.locations) {
            push(MissionAction::Kind::ApproachBox, location);
            push(MissionAction::Kind::VerifyLocation, location);
          }
        }
        enter(MissionState::Verify, now);
        break;
      case MissionState::Verify: {
        push(MissionAction::Kind::GoHome);
        MissionAction still; still.kind = MissionAction::Kind::HoldStill; still.seconds = config_.home_still_s;
        queue_.push_back(still);
        enter(MissionState::Return, now);
        break;
      }
      case MissionState::Return: enter(MissionState::Done, now); break;
      default: return {};
    }
  }
  return {};
}

void MissionFsm::action_done(bool ok, const std::string &reason, int puck_id) {
  if (!busy_) return;
  const auto done = action_;
  busy_ = false; action_ = {};
  if (!ok) {fail(done.describe() + " failed" + (reason.empty() ? "" : ": " + reason)); return;}
  if (done.kind == MissionAction::Kind::PickPuck) inventory_[done.slot] = puck_id;
  if (done.kind == MissionAction::Kind::ApproachBox) docked_at_ = done.box;
  if (done.kind == MissionAction::Kind::LeaveBox) docked_at_.clear();
}

std::string MissionFsm::label() const {
  std::string text = name(state_);
  if (busy_) text += ": " + action_.describe();
  return text;
}

}  // namespace craft_planning::mission
