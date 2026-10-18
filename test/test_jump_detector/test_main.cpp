#include <unity.h>

#include <vector>

#include "jump_detector.h"

namespace {
constexpr uint64_t kPeriodUs = 5000;  // 200 Hz

struct Segment {
  float g;
  uint32_t durationMs;
};

struct Outcome {
  std::vector<JumpResult> jumps;
  std::vector<const char*> rejects;
};

Outcome run(const std::vector<Segment>& trace) {
  JumpDetector detector;
  Outcome outcome;
  uint64_t t = 0;
  for (const Segment& segment : trace) {
    for (uint32_t elapsed = 0; elapsed < segment.durationMs * 1000; elapsed += kPeriodUs) {
      t += kPeriodUs;
      const JumpDetector::Event event = detector.update(t, segment.g);
      if (event == JumpDetector::Event::Jump) outcome.jumps.push_back(detector.lastJump());
      if (event == JumpDetector::Event::Rejected) outcome.rejects.push_back(JumpDetector::describe(detector.lastRejectReason()));
    }
  }
  return outcome;
}

// Stand, countermovement dip, push-off, flight, landing impact, recover.
std::vector<Segment> countermovementJump(uint32_t flightMs, float impactG = 4.5f, uint32_t dipMs = 90) {
  return {{1.0f, 600}, {0.3f, dipMs}, {2.4f, 200}, {0.05f, flightMs},
          {impactG, 40}, {1.6f, 150}, {1.0f, 600}};
}

void test_height_formula() {
  // h = g t^2 / 8: 0.5 s flight -> 30.65 cm
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 30.65f, JumpDetector::heightCmFromFlightSeconds(0.5f));
}

void test_detects_countermovement_jump() {
  const Outcome outcome = run(countermovementJump(500));
  TEST_ASSERT_EQUAL(1, outcome.jumps.size());
  // Dip shorter than 150 ms is rejected, but must not block the real jump.
  TEST_ASSERT_EQUAL(1, outcome.rejects.size());
  TEST_ASSERT_FLOAT_WITHIN(5.0f, 500.0f, outcome.jumps[0].flightTimeMs);
  TEST_ASSERT_FLOAT_WITHIN(0.7f, 30.65f, outcome.jumps[0].heightCm);
  TEST_ASSERT_FLOAT_WITHIN(0.3f, 30.65f / 2.54f, outcome.jumps[0].heightIn);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 4.5f, outcome.jumps[0].peakImpactG);
}

void test_rejects_flight_under_150ms() {
  const Outcome outcome = run(countermovementJump(120));
  TEST_ASSERT_EQUAL(0, outcome.jumps.size());
}

void test_rejects_landing_without_impact() {
  const Outcome outcome = run(countermovementJump(400, 1.3f));
  TEST_ASSERT_EQUAL(0, outcome.jumps.size());
  TEST_ASSERT_EQUAL_STRING("no landing impact", outcome.rejects.back());
}

void test_rejects_long_free_fall() {
  const Outcome outcome = run({{1.0f, 600}, {0.0f, 1500}, {6.0f, 30}, {1.0f, 600}});
  TEST_ASSERT_EQUAL(0, outcome.jumps.size());
  TEST_ASSERT_EQUAL_STRING("free fall too long", outcome.rejects.front());
}

void test_requires_stillness_before_arming() {
  // Jumping without ever standing still (e.g. device being handled) is ignored.
  const Outcome outcome = run({{1.8f, 600}, {0.05f, 400}, {4.0f, 40}, {1.8f, 300}});
  TEST_ASSERT_EQUAL(0, outcome.jumps.size());
}

void test_mid_air_jolt_does_not_split_flight() {
  // Recorded failure: a harness jolt ~114 ms into a big jump ended the flight
  // early and the remainder was measured as a new 245 ms jump.
  const Outcome outcome = run({{1.0f, 600}, {0.3f, 90}, {2.4f, 200}, {0.05f, 115}, {4.5f, 15},
                               {0.05f, 470}, {4.5f, 40}, {1.6f, 150}, {1.0f, 600}});
  TEST_ASSERT_EQUAL(1, outcome.jumps.size());
  TEST_ASSERT_FLOAT_WITHIN(6.0f, 600.0f, outcome.jumps[0].flightTimeMs);
}

void test_consecutive_jumps_are_counted() {
  std::vector<Segment> trace = countermovementJump(450);
  const std::vector<Segment> second = countermovementJump(550);
  trace.insert(trace.end(), second.begin(), second.end());
  const Outcome outcome = run(trace);
  TEST_ASSERT_EQUAL(2, outcome.jumps.size());
  TEST_ASSERT_EQUAL_UINT32(2, outcome.jumps[1].number);
  TEST_ASSERT_TRUE(outcome.jumps[1].flightTimeMs > outcome.jumps[0].flightTimeMs);
}
}  // namespace

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_height_formula);
  RUN_TEST(test_detects_countermovement_jump);
  RUN_TEST(test_rejects_flight_under_150ms);
  RUN_TEST(test_rejects_landing_without_impact);
  RUN_TEST(test_rejects_long_free_fall);
  RUN_TEST(test_requires_stillness_before_arming);
  RUN_TEST(test_mid_air_jolt_does_not_split_flight);
  RUN_TEST(test_consecutive_jumps_are_counted);
  return UNITY_END();
}
