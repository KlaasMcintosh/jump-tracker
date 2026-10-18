#pragma once

// Hardware-independent takeoff/landing detector.
//
// Feed it one acceleration-magnitude sample at a time (in g, orientation
// independent) with a monotonic timestamp. It has no Arduino dependency so it
// can be unit tested on the host (`pio test -e native`).
//
// State machine:
//
//   Settling --(still for stillTimeUs)--> Armed
//   Armed    --(|a| < freefallEnterG)---> Flight      (takeoff time interpolated)
//   Flight   --(|a| > freefallExitG)----> Landing     (touchdown time interpolated)
//            --(> maxFlightUs)----------> Settling    (dropped / thrown device)
//   Landing  --(back in free fall soon)--> Flight     (mid-air jolt, keep takeoff time)
//            --(too short)--------------> Armed       (e.g. countermovement dip)
//            --(impactWindowUs elapsed)--> Settling   (report jump if peak >= impactG)

#include <cstdint>

struct JumpDetectorConfig {
  float freefallEnterG = 0.35f;        // below this: airborne
  float freefallExitG = 0.60f;         // above this: touchdown (hysteresis vs. enter)
  float impactG = 2.0f;                // landing must produce a spike at least this big
  uint32_t impactWindowUs = 150000;    // window after touchdown to find the peak
  // A "touchdown" followed by renewed free fall within this window was a
  // mid-air jolt (loose harness, cable tug); a real landing never returns to ~0 g.
  uint32_t resumeWindowUs = 80000;
  uint8_t resumeSamples = 2;           // consecutive samples below freefallEnterG
  uint32_t minFlightUs = 150000;       // shorter flights are rejected as false triggers
  uint32_t maxFlightUs = 1000000;      // ~1.2 m jump; longer is not a human jump
  float stillToleranceG = 0.25f;       // | |a| - 1g | below this counts as standing
  uint32_t stillTimeUs = 300000;       // stand still this long before (re)arming
};

struct JumpResult {
  uint32_t number;      // 1-based count of accepted jumps
  float flightTimeMs;
  float heightCm;
  float heightIn;
  float peakImpactG;
};

class JumpDetector {
 public:
  enum class State : uint8_t { Settling, Armed, Flight, Landing };
  enum class Event : uint8_t { None, Armed, Takeoff, Jump, Rejected };
  enum class RejectReason : uint8_t { None, FlightTooShort, FreeFallTooLong, NoLandingImpact };

  explicit JumpDetector(const JumpDetectorConfig& config = JumpDetectorConfig());

  // Process one sample. Timestamps must be monotonic (microseconds).
  Event update(uint64_t timeUs, float magnitudeG);
  void reset();

  State state() const { return state_; }
  const JumpResult& lastJump() const { return lastJump_; }
  RejectReason lastRejectReason() const { return rejectReason_; }
  float lastRejectedFlightMs() const { return rejectedFlightMs_; }

  // h = g * t^2 / 8, returned in centimetres.
  static float heightCmFromFlightSeconds(float flightSeconds);
  static const char* describe(RejectReason reason);

 private:
  Event reject(RejectReason reason, float flightMs, State next);
  static double crossingTimeUs(uint64_t t0, float m0, uint64_t t1, float m1, float threshold);

  JumpDetectorConfig config_;
  State state_ = State::Settling;
  bool hasPrevious_ = false;
  uint64_t prevTimeUs_ = 0;
  float prevMagnitudeG_ = 1.0f;

  uint64_t stillSinceUs_ = 0;
  bool still_ = false;
  double takeoffUs_ = 0;
  double touchdownUs_ = 0;
  float peakG_ = 0;
  uint8_t freefallRunLength_ = 0;

  uint32_t jumpCount_ = 0;
  JumpResult lastJump_ = {};
  RejectReason rejectReason_ = RejectReason::None;
  float rejectedFlightMs_ = 0;
};
