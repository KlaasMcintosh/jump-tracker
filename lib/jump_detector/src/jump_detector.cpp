#include "jump_detector.h"

#include <cmath>

namespace {
constexpr float kGravity = 9.80665f;
constexpr float kCmPerInch = 2.54f;
}  // namespace

JumpDetector::JumpDetector(const JumpDetectorConfig& config) : config_(config) {}

void JumpDetector::reset() {
  const JumpDetectorConfig config = config_;
  *this = JumpDetector(config);
}

float JumpDetector::heightCmFromFlightSeconds(float flightSeconds) {
  return kGravity * flightSeconds * flightSeconds / 8.0f * 100.0f;
}

// Linear interpolation of when the signal crossed `threshold` between two
// samples. Gives sub-sample timing (5 ms samples would otherwise cost up to
// ~0.6 cm of height resolution on a 30 cm jump).
double JumpDetector::crossingTimeUs(uint64_t t0, float m0, uint64_t t1, float m1, float threshold) {
  const float span = m1 - m0;
  if (std::fabs(span) < 1e-6f) return static_cast<double>(t1);
  double fraction = (threshold - m0) / span;
  if (fraction < 0) fraction = 0;
  if (fraction > 1) fraction = 1;
  return static_cast<double>(t0) + fraction * static_cast<double>(t1 - t0);
}

const char* JumpDetector::describe(RejectReason reason) {
  switch (reason) {
    case RejectReason::None: return "none";
    case RejectReason::FlightTooShort: return "flight too short";
    case RejectReason::FreeFallTooLong: return "free fall too long";
    case RejectReason::NoLandingImpact: return "no landing impact";
  }
  return "?";
}

JumpDetector::Event JumpDetector::reject(RejectReason reason, float flightMs, State next) {
  rejectReason_ = reason;
  rejectedFlightMs_ = flightMs;
  state_ = next;
  still_ = false;
  return Event::Rejected;
}

JumpDetector::Event JumpDetector::update(uint64_t timeUs, float magnitudeG) {
  const uint64_t t0 = hasPrevious_ ? prevTimeUs_ : timeUs;
  const float m0 = hasPrevious_ ? prevMagnitudeG_ : magnitudeG;
  hasPrevious_ = true;
  prevTimeUs_ = timeUs;
  prevMagnitudeG_ = magnitudeG;

  switch (state_) {
    case State::Settling: {
      if (std::fabs(magnitudeG - 1.0f) > config_.stillToleranceG) {
        still_ = false;
        return Event::None;
      }
      if (!still_) {
        still_ = true;
        stillSinceUs_ = timeUs;
      }
      if (timeUs - stillSinceUs_ >= config_.stillTimeUs) {
        state_ = State::Armed;
        return Event::Armed;
      }
      return Event::None;
    }

    case State::Armed: {
      if (magnitudeG < config_.freefallEnterG) {
        takeoffUs_ = crossingTimeUs(t0, m0, timeUs, magnitudeG, config_.freefallEnterG);
        state_ = State::Flight;
        return Event::Takeoff;
      }
      return Event::None;
    }

    case State::Flight: {
      const double elapsedUs = static_cast<double>(timeUs) - takeoffUs_;
      if (magnitudeG > config_.freefallExitG) {
        touchdownUs_ = crossingTimeUs(t0, m0, timeUs, magnitudeG, config_.freefallExitG);
        peakG_ = magnitudeG;
        freefallRunLength_ = 0;
        state_ = State::Landing;
        return Event::None;
      }
      if (elapsedUs > config_.maxFlightUs) {
        return reject(RejectReason::FreeFallTooLong, static_cast<float>(elapsedUs / 1000.0), State::Settling);
      }
      return Event::None;
    }

    case State::Landing: {
      if (magnitudeG > peakG_) peakG_ = magnitudeG;
      const double sinceTouchdownUs = static_cast<double>(timeUs) - touchdownUs_;
      if (sinceTouchdownUs < config_.resumeWindowUs) {
        freefallRunLength_ = magnitudeG < config_.freefallEnterG ? freefallRunLength_ + 1 : 0;
        if (freefallRunLength_ >= config_.resumeSamples) state_ = State::Flight;
        return Event::None;
      }
      const double flightUs = touchdownUs_ - takeoffUs_;
      if (flightUs < config_.minFlightUs) {
        // Typically the unweighting dip of a countermovement: the athlete is
        // still on the ground, so stay armed for the real takeoff.
        return reject(RejectReason::FlightTooShort, static_cast<float>(flightUs / 1000.0), State::Armed);
      }
      if (sinceTouchdownUs < config_.impactWindowUs) {
        return Event::None;
      }
      const float flightSeconds = static_cast<float>(flightUs / 1e6);
      if (peakG_ < config_.impactG) {
        return reject(RejectReason::NoLandingImpact, flightSeconds * 1000.0f, State::Settling);
      }
      lastJump_.number = ++jumpCount_;
      lastJump_.flightTimeMs = flightSeconds * 1000.0f;
      lastJump_.heightCm = heightCmFromFlightSeconds(flightSeconds);
      lastJump_.heightIn = lastJump_.heightCm / kCmPerInch;
      lastJump_.peakImpactG = peakG_;
      state_ = State::Settling;
      still_ = false;
      return Event::Jump;
    }
  }
  return Event::None;
}
