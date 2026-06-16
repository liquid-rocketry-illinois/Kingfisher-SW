//
// DataFilter.h — exponential moving average (EMA) filter for rocket sensor data.
//
// alpha (0–1): smoothing factor.
//   alpha near 0 = heavy smoothing (slow response)
//   alpha near 1 = light smoothing (follows signal closely)
//
// No sample rate or cutoff frequency needed.
//

#ifndef KINGFISHER_SW_DATAFILTER_H
#define KINGFISHER_SW_DATAFILTER_H

// ── EmaFilter ─────────────────────────────────────────────────────────────────
// Single-channel EMA. Seeds state on first call to prevent startup transients.
class EmaFilter {
public:
    explicit EmaFilter(float alpha) : alpha_(alpha) {}

    float update(float x) {
        if (!seeded_) { y_ = x; seeded_ = true; }
        y_ = alpha_ * x + (1.0f - alpha_) * y_;
        return y_;
    }

    void seed(float value) { y_ = value; seeded_ = true; }

private:
    float alpha_;
    float y_      = 0.0f;
    bool  seeded_ = false;
};

// ── EmaFilter3 ────────────────────────────────────────────────────────────────
// Three independent EMA channels sharing the same alpha. Operates in-place.
class EmaFilter3 {
public:
    explicit EmaFilter3(float alpha) : fx(alpha), fy(alpha), fz(alpha) {}

    void update(float& x, float& y, float& z) {
        x = fx.update(x);
        y = fy.update(y);
        z = fz.update(z);
    }

    void seed(float x, float y, float z) { fx.seed(x); fy.seed(y); fz.seed(z); }

private:
    EmaFilter fx, fy, fz;
};

// ── SensorFilters ─────────────────────────────────────────────────────────────
class SensorFilters {
public:
    SensorFilters();
    void apply();

private:
    static constexpr float DEG_TO_RAD = 3.14159265358979323846f / 180.0f;

    EmaFilter  pressureFilter;  // alpha = 0.2  — heavy baro smoothing
    EmaFilter3 accelFilter;     // alpha = 0.5
    EmaFilter3 gyroFilter;      // alpha = 0.5
};

extern SensorFilters g_filters;

#endif // KINGFISHER_SW_DATAFILTER_H