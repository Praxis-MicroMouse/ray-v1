#include "filter.h"

void filter_init(filter_t *f) {
    f->estimate = 0.0f;
    f->last_raw = 0.0f;
    f->initialized = false;
}

float filter_update(filter_t *f, float raw) {
    if (!f->initialized) {
        f->estimate = raw;
        f->last_raw = raw;
        f->initialized = true;
        return f->estimate;
    }

    float delta = raw - f->estimate;
    float abs_delta = (delta < 0) ? -delta : delta;

    if (abs_delta > FILTER_SPIKE_THRESHOLD_MM) {
        float confirm_delta = raw - f->last_raw;
        float abs_confirm = (confirm_delta < 0) ? -confirm_delta : confirm_delta;

        if (abs_confirm > FILTER_SPIKE_THRESHOLD_MM) {
            // Doesn't agree with the previous raw reading either - most
            // likely a one-off spike. Hold the estimate steady, but keep
            // this raw value so a second, consistent jump next time gets
            // through instead of being rejected forever.
            f->last_raw = raw;
            return f->estimate;
        }
        // Two readings in a row agree on a big jump - treat it as real
        // and fall through to blend it in below.
    }

    f->last_raw = raw;
    f->estimate += FILTER_EMA_ALPHA * delta;
    return f->estimate;
}
