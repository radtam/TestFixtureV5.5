/*
 * fixture_hal.h — Thin hardware abstraction for future actuators/sensors.
 * Stepper and load cell remain the concrete implementations today; new devices
 * can follow the same interfaces without rewriting the command layer.
 */
#pragma once

#include <Arduino.h>

/** Bump when LINK_UART line format changes; display ESP32 can gate on this. */
constexpr uint8_t LINK_PROTO_VERSION = 1;

/** Forward declarations — defined in the main sketch */
class AccelStepper;
class HX711_MP;

/** Minimal actuator: enable and periodic poll (stepper uses run()/runSpeed() in its task). */
struct IActuator {
  virtual ~IActuator() = default;
  virtual void setEnabled(bool on) = 0;
};

/** Normalized or engineering-unit reading. */
struct ISensor {
  virtual ~ISensor() = default;
  virtual float readPrimary(int samples = 1) = 0;
};

/** Wraps AccelStepper for enable pin semantics used by this fixture. */
template<typename StepperT>
struct StepperActuator final : IActuator {
  StepperT&    stepper;
  const int    enablePin;
  StepperActuator(StepperT& s, int enPin) : stepper(s), enablePin(enPin) {}
  void setEnabled(bool on) override {
    if (enablePin < 0) return;
    digitalWrite(enablePin, on ? LOW : HIGH);
  }
};

/** Wraps HX711_MP calibrated read. Caller must hold lcMutex if concurrent access. */
template<typename LcT>
struct LoadCellSensor final : ISensor {
  LcT& lc;
  explicit LoadCellSensor(LcT& l) : lc(l) {}
  float readPrimary(int samples = 1) override { return lc.get_units(samples); }
};

/*
 * ServoActuator — drives a hobby servo via the ESP32 LEDC peripheral at 50 Hz.
 *
 * Pulse width <-> angle mapping uses a configurable [minPulseUs..maxPulseUs] range
 * so each physical servo can be calibrated to its true 0..180 deg endpoints
 * without losing range (datasheets vary; common range is 500..2500 us).
 *
 * The LEDC API differs between Arduino-ESP32 v2.x and v3.x; both are supported.
 * On v2.x we use an explicit channel; on v3.x the pin IS the handle.
 *
 * setEnabled(false) stops emitting pulses (pin held LOW) — useful to silence a
 * twitchy/hot servo when you don't need to hold position.
 */
struct ServoActuator final : IActuator {
  const int      pin;
  const uint32_t freqHz;
  const uint8_t  resBits;
#if !(defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3))
  const uint8_t  channel;
#endif
  bool           attached = false;
  uint16_t       lastUs   = 1500;

  ServoActuator(int p, uint32_t f, uint8_t r
#if !(defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3))
                , uint8_t ch
#endif
                )
    : pin(p), freqHz(f), resBits(r)
#if !(defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3))
      , channel(ch)
#endif
  {}

  bool begin() {
    if (pin < 0) return false;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    attached = ledcAttach(pin, freqHz, resBits);
#else
    ledcSetup(channel, freqHz, resBits);
    ledcAttachPin(pin, channel);
    attached = true;
#endif
    return attached;
  }

  /* Write a raw pulse width in microseconds. */
  void writeMicroseconds(uint16_t us) {
    if (!attached) return;
    const uint32_t periodUs   = 1000000UL / freqHz;
    const uint32_t maxDutyVal = (1UL << resBits) - 1UL;
    if (us > periodUs) us = periodUs;
    uint32_t duty = ((uint32_t)us * maxDutyVal) / periodUs;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWrite(pin, duty);
#else
    ledcWrite(channel, duty);
#endif
    lastUs = us;
  }

  /* Map degrees -> microseconds using calibrated end-points and write. */
  void writeDegrees(float deg, uint16_t minUs, uint16_t maxUs) {
    if (deg < 0.f) deg = 0.f;
    if (deg > 180.f) deg = 180.f;
    float us = (float)minUs + (deg / 180.f) * (float)(maxUs - minUs);
    writeMicroseconds((uint16_t)(us + 0.5f));
  }

  /* IActuator: setEnabled(false) parks the pin LOW (no pulses). */
  void setEnabled(bool on) override {
    if (!attached) return;
    if (on) {
      writeMicroseconds(lastUs);
    } else {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
      ledcWrite(pin, 0);
#else
      ledcWrite(channel, 0);
#endif
    }
  }
};

/*
 * BreakBeamSensor — digital input with configurable active level.
 *
 * readPrimary() returns 1.0f when the beam is *broken*, 0.0f when clear,
 * so the value can be averaged or thresholded by callers expecting a float.
 * isBroken() / rawLevel() are the typical entry points.
 *
 * usePullup is honored only if the chosen pin supports it (input-only pins
 * 34/35/36/39 do not — caller is responsible for an external pull-up there).
 */
struct BreakBeamSensor final : ISensor {
  const int pin;
  bool      activeLow = true;
  bool      usePullup = false;

  BreakBeamSensor(int p, bool actLow, bool pullup)
    : pin(p), activeLow(actLow), usePullup(pullup) {}

  void begin() {
    if (pin < 0) return;
    pinMode(pin, usePullup ? INPUT_PULLUP : INPUT);
  }

  int rawLevel() const { return (pin < 0) ? -1 : digitalRead(pin); }

  bool isBroken() const {
    if (pin < 0) return false;
    int lvl = digitalRead(pin);
    return activeLow ? (lvl == LOW) : (lvl == HIGH);
  }

  float readPrimary(int /*samples*/ = 1) override {
    return isBroken() ? 1.0f : 0.0f;
  }
};

inline void linkAnnounceProtocol(HardwareSerial& uart, uint8_t ver) {
  uart.printf("proto=%u\n", ver);
}
