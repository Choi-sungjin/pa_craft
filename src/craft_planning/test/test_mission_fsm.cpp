#include "craft_planning/mission_fsm.hpp"
#include <gtest/gtest.h>
#include <set>
using namespace craft_planning::mission;
using Kind = MissionAction::Kind;

// Q1 of the brief: R G R R G R G G Y G Y G, P2-01; 6 red + 6 green, cuboids 0-2 / 6-8, cylinders 3-5 / 9-11.
static MissionInputs q1() {
  MissionInputs in;
  in.barcode = "P2-01"; in.signal_complete = in.signal_valid = true;
  in.sequence = {0, 2, 0, 0, 2, 0, 2, 2, 1, 2, 1, 2}; in.counts = {4, 2, 6};
  for (int id = 0; id < 12; ++id)
    in.survey.push_back({id, id < 6 ? PuckColor::Red : PuckColor::Green,
                         id % 6 < 3 ? PuckShape::Cuboid : PuckShape::Cylinder});
  return in;
}
// Runs the mission to its end, every action succeeding; pick(slot) = the puck id the pick reports.
template <class Pick> static std::vector<MissionAction> run(MissionFsm &f, const MissionInputs &in, Pick pick) {
  std::vector<MissionAction> done;
  double t = 0;
  for (int i = 0; i < 1000 && f.running(); ++i) {
    const auto a = f.tick(in, t += 0.1);
    if (a.kind == Kind::None) continue;
    done.push_back(a);
    f.action_done(true, "", a.kind == Kind::PickPuck ? pick(a.slot) : -1);
  }
  return done;
}

TEST(MissionFsm, WaitsForSignalThenBarcode) {  // user 2026-11-13: the signal comes before the barcode
  MissionFsm f; ASSERT_TRUE(f.start(0)); EXPECT_FALSE(f.start(0));
  EXPECT_EQ(f.tick({}, 0.1).kind, Kind::ResetPerception);
  EXPECT_EQ(f.tick({}, 0.2).kind, Kind::None);  // the reset is still running
  f.action_done(true);
  MissionInputs in; in.barcode = "P2-02";  // a barcode already read does not skip the signal
  EXPECT_EQ(f.tick(in, 0.3).kind, Kind::None); EXPECT_EQ(f.state(), MissionState::WatchSignal);
  in = q1(); in.barcode.clear();
  EXPECT_EQ(f.tick(in, 0.4).kind, Kind::None); EXPECT_EQ(f.state(), MissionState::ReadBarcode);
  in.barcode = "P2-02";
  const auto a = f.tick(in, 0.5);
  EXPECT_EQ(f.order(), SortOrder::Descending);
  EXPECT_EQ(f.state(), MissionState::Collect);
  EXPECT_EQ(a.kind, Kind::ApproachBox); EXPECT_EQ(a.box, "S1");
  EXPECT_EQ(f.groups(), (std::array<Group, 3>{Group::Green, Group::Red, Group::Yellow}));
}

TEST(MissionFsm, FullMissionQ1) {
  MissionFsm f; f.start(0);
  const auto done = run(f, q1(), [](int slot) {return slot - 1;});
  ASSERT_EQ(f.state(), MissionState::Done) << f.reason();
  // ascending: Y(2) -> L1, R(4) -> L2, G(6) -> L3
  EXPECT_EQ(f.groups(), (std::array<Group, 3>{Group::Yellow, Group::Red, Group::Green}));
  int picks = 0, places = 0, verifies = 0;
  std::set<int> slots, placed;
  std::map<std::string, int> per_location;
  for (const auto &a : done) {
    if (a.kind == Kind::PickPuck) {++picks; EXPECT_TRUE(slots.insert(a.slot).second);}
    if (a.kind == Kind::PlacePuck) {
      ++places; ++per_location[a.box];
      EXPECT_TRUE(placed.insert(a.puck_id).second);
      EXPECT_EQ(a.slot, a.puck_id + 1);  // the slot the pick put that puck in
    }
    if (a.kind == Kind::VerifyLocation) ++verifies;
  }
  EXPECT_EQ(picks, 12); EXPECT_EQ(places, 12); EXPECT_EQ(verifies, 3);
  EXPECT_EQ(slots.size(), 12u); EXPECT_EQ(*slots.begin(), 1); EXPECT_EQ(*slots.rbegin(), 12);
  EXPECT_EQ(per_location["L1"], 2); EXPECT_EQ(per_location["L2"], 4); EXPECT_EQ(per_location["L3"], 6);
  // every pick of a box comes after driving up to it; the mission ends at home, standing still 3 s
  EXPECT_EQ(done[0].kind, Kind::ResetPerception);
  EXPECT_EQ(done[1].kind, Kind::ApproachBox); EXPECT_EQ(done[2].kind, Kind::ObservePucks);
  // the arm works docked; leaving comes only after the box's last pick
  EXPECT_EQ(done[5].kind, Kind::LeaveBox); EXPECT_EQ(done[5].box, "S1"); EXPECT_EQ(done[6].box, "S2");
  EXPECT_EQ(done[3].kind, Kind::PickPuck); EXPECT_EQ(done[4].kind, Kind::PickPuck);
  EXPECT_EQ(done[done.size() - 2].kind, Kind::GoHome);
  EXPECT_EQ(done.back().kind, Kind::HoldStill); EXPECT_DOUBLE_EQ(done.back().seconds, 3.0);
  // Y is one shape
  for (const auto &p : f.placements()) if (p.group == Group::Yellow) EXPECT_LT(p.puck_id % 6, 3);
}

TEST(MissionFsm, VerifyRightAfterPlacing) {
  MissionConfig c; c.verify_after_all = false;
  MissionFsm f(c); f.start(0);
  const auto done = run(f, q1(), [](int slot) {return slot - 1;});
  ASSERT_EQ(f.state(), MissionState::Done) << f.reason();
  int approaches = 0;
  for (std::size_t i = 0; i < done.size(); ++i) {
    if (done[i].kind == Kind::ApproachBox && done[i].box[0] == 'L') ++approaches;
    if (done[i].kind == Kind::VerifyLocation) EXPECT_EQ(done[i - 1].kind, Kind::PlacePuck);
  }
  EXPECT_EQ(approaches, 3);  // no second visit
}

TEST(MissionFsm, UnknownPicksFaultUnlessAssumed) {
  MissionFsm strict; strict.start(0);
  run(strict, q1(), [](int) {return -1;});
  EXPECT_EQ(strict.state(), MissionState::Fault);
  EXPECT_NE(strict.reason().find("not identified"), std::string::npos);
  MissionConfig c; c.assume_unknown_picks = true;
  MissionFsm loose(c); loose.start(0);
  run(loose, q1(), [](int) {return -1;});
  EXPECT_EQ(loose.state(), MissionState::Done) << loose.reason();
  EXPECT_EQ(loose.placements().size(), 12u);
}

TEST(MissionFsm, Faults) {
  {  // unregistered barcode
    MissionFsm f; f.start(0); auto in = q1(); in.barcode = "P2-09";
    run(f, in, [](int) {return -1;});
    EXPECT_EQ(f.state(), MissionState::Fault); EXPECT_NE(f.reason().find("unregistered"), std::string::npos);
  }
  {  // 12 blinks but a colour 7 times
    MissionFsm f; f.start(0); auto in = q1(); in.counts = {7, 1, 4};
    run(f, in, [](int) {return -1;});
    EXPECT_EQ(f.state(), MissionState::Fault);
  }
  {  // sequence and counts disagree
    MissionFsm f; f.start(0); auto in = q1(); in.sequence[0] = 2;
    run(f, in, [](int) {return -1;});
    EXPECT_EQ(f.state(), MissionState::Fault);
  }
  {  // survey incomplete
    MissionFsm f; f.start(0); auto in = q1(); in.survey.pop_back();
    run(f, in, [](int slot) {return slot - 1;});
    EXPECT_EQ(f.state(), MissionState::Fault); EXPECT_NE(f.reason().find("11 of 12"), std::string::npos);
  }
  {  // the same puck reported by two picks
    MissionFsm f; f.start(0);
    run(f, q1(), [](int slot) {return slot == 12 ? 0 : slot - 1;});
    EXPECT_EQ(f.state(), MissionState::Fault);
  }
  {  // a failed action, and the first cause stays
    MissionFsm f; f.start(0); const auto in = q1(); double t = 0;
    MissionAction a;
    while ((a = f.tick(in, t += 0.1)).kind != Kind::PickPuck) f.action_done(true);
    f.action_done(false, "no grasp");
    EXPECT_EQ(f.state(), MissionState::Fault); EXPECT_NE(f.reason().find("no grasp"), std::string::npos);
    const auto reason = f.reason(); f.fail("later"); EXPECT_EQ(f.reason(), reason);
    EXPECT_EQ(f.tick(in, t += 0.1).kind, Kind::None);
    EXPECT_TRUE(f.start(t));  // a new run from Fault
  }
}

TEST(MissionFsm, Timeouts) {
  MissionConfig c; c.barcode_timeout_s = 5; c.signal_timeout_s = 30; c.action_timeout_s["approach_box"] = 10;
  {
    MissionFsm f(c); f.start(0); f.tick({}, 0); f.action_done(true);
    auto in = q1(); in.barcode.clear();  // the signal done at t=1: the barcode timer starts there
    f.tick(in, 1); f.tick(in, 5.9); EXPECT_EQ(f.state(), MissionState::ReadBarcode);
    f.tick(in, 6.0); EXPECT_EQ(f.state(), MissionState::Fault);
  }
  {
    MissionFsm f(c); f.start(0); f.tick({}, 0); f.action_done(true);
    MissionInputs in; in.barcode = "P2-01";
    f.tick(in, 1); f.tick(in, 29.9); EXPECT_EQ(f.state(), MissionState::WatchSignal);
    f.tick(in, 30); EXPECT_EQ(f.state(), MissionState::Fault);
  }
  {
    MissionFsm f(c); f.start(0); f.tick({}, 0); f.action_done(true);
    const auto in = q1();
    EXPECT_EQ(f.tick(in, 1).kind, Kind::ApproachBox);
    f.tick(in, 10.9); EXPECT_EQ(f.state(), MissionState::Collect);
    f.tick(in, 11); EXPECT_EQ(f.state(), MissionState::Fault);
    EXPECT_NE(f.reason().find("timeout"), std::string::npos);
  }
}

TEST(MissionFsm, ConfigIsChecked) {
  MissionConfig c; c.start_boxes.pop_back();
  EXPECT_THROW(MissionFsm{c}, std::invalid_argument);
  MissionConfig d; d.locations[0] = "S1";
  EXPECT_THROW(MissionFsm{d}, std::invalid_argument);
}

TEST(MissionFsm, DrivesToBarcodeViewOnlyWhenNotReadYet) {
  MissionConfig c; c.drive_only = true; c.view_barcode = true;
  MissionInputs in;
  in.signal_complete = in.signal_valid = true;
  in.sequence = {0, 2, 0, 0, 2, 0, 2, 2, 1, 2, 1, 2}; in.counts = {4, 2, 6};
  MissionFsm f(c); ASSERT_TRUE(f.start(0));
  EXPECT_EQ(f.tick({}, 0.1).kind, Kind::ResetPerception); f.action_done(true);
  const auto view = f.tick(in, 0.2);  // signal complete, no barcode -> drive to the view pose
  EXPECT_EQ(view.kind, Kind::ViewBarcode); EXPECT_EQ(f.state(), MissionState::ReadBarcode);
  EXPECT_EQ(f.tick(in, 0.3).kind, Kind::None);  // still driving
  f.action_done(true);
  EXPECT_EQ(f.tick(in, 0.4).kind, Kind::None); EXPECT_EQ(f.state(), MissionState::ReadBarcode);  // waits there
  in.barcode = "P2-01";
  EXPECT_EQ(f.tick(in, 0.5).kind, Kind::ApproachBox);
  ASSERT_TRUE(f.order()); EXPECT_EQ(*f.order(), SortOrder::Ascending);
  // Already read before the signal completed: no drive.
  MissionFsm g(c); g.start(0); g.tick({}, 0.1); g.action_done(true);
  EXPECT_EQ(g.tick(in, 0.2).kind, Kind::ApproachBox);
}

TEST(MissionFsm, DriveOnlyReadsBarcodeSkipsArm) {
  MissionConfig c; c.drive_only = true;
  MissionFsm f(c); ASSERT_TRUE(f.start(0));
  EXPECT_EQ(f.tick({}, 0.1).kind, Kind::ResetPerception); f.action_done(true);
  MissionInputs in;
  EXPECT_EQ(f.tick(in, 0.2).kind, Kind::None); EXPECT_EQ(f.state(), MissionState::WatchSignal);
  in.signal_complete = in.signal_valid = true;
  in.sequence = {0, 2, 0, 0, 2, 0, 2, 2, 1, 2, 1, 2}; in.counts = {4, 2, 6};
  // The upper-cam barcode is still waited for (user 2026-11-14).
  EXPECT_EQ(f.tick(in, 0.3).kind, Kind::None); EXPECT_EQ(f.state(), MissionState::ReadBarcode);
  EXPECT_EQ(f.tick(in, 0.4).kind, Kind::None); EXPECT_EQ(f.state(), MissionState::ReadBarcode);
  in.barcode = "P2-02";
  const auto done = run(f, in, [](int) {return -1;});
  EXPECT_EQ(f.state(), MissionState::Done) << f.reason();
  std::vector<std::string> boxes;
  for (const auto &a : done) {
    EXPECT_TRUE(a.kind == Kind::ApproachBox || a.kind == Kind::LeaveBox || a.kind == Kind::GoHome ||
                a.kind == Kind::HoldStill) << name(a.kind);
    if (a.kind == Kind::ApproachBox) boxes.push_back(a.box);
  }
  EXPECT_EQ(boxes, (std::vector<std::string>{"S1", "S2", "S3", "S4", "S5", "S6", "L1", "L2", "L3"}));
  EXPECT_EQ(done[done.size() - 2].kind, Kind::GoHome);
  EXPECT_EQ(f.counts().green, 6);
  ASSERT_TRUE(f.order()); EXPECT_EQ(*f.order(), SortOrder::Descending);
  // Every drive after a dock starts by backing out of that dock: 9 docks, 9 leaves, each right before the next drive.
  int leaves = 0;
  for (std::size_t i = 0; i < done.size(); ++i) {
    if (done[i].kind != Kind::LeaveBox) continue;
    ++leaves;
    ASSERT_GT(i, 0u); ASSERT_LT(i + 1, done.size());
    EXPECT_EQ(done[i - 1].kind, Kind::ApproachBox); EXPECT_EQ(done[i].box, done[i - 1].box);
    EXPECT_TRUE(done[i + 1].kind == Kind::ApproachBox || done[i + 1].kind == Kind::GoHome);
  }
  EXPECT_EQ(leaves, 9);
  // An invalid signal still faults.
  MissionFsm g(c); g.start(0); g.tick({}, 0.1); g.action_done(true);
  in.counts = {7, 1, 4}; in.sequence = {0, 0, 0, 0, 0, 0, 0, 1, 2, 2, 2, 2};
  g.tick(in, 0.2);
  EXPECT_EQ(g.state(), MissionState::Fault);
  // An unregistered barcode faults too.
  MissionFsm h(c); h.start(0); h.tick({}, 0.1); h.action_done(true);
  in.counts = {4, 2, 6}; in.sequence = {0, 2, 0, 0, 2, 0, 2, 2, 1, 2, 1, 2}; in.barcode = "P9-99";
  h.tick(in, 0.2); h.tick(in, 0.3);
  EXPECT_EQ(h.state(), MissionState::Fault);
}
