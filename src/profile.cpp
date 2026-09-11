#include "profile.h"

#include <math.h>

void profile_reset(profile_t *p) {
    p->position = 0.0f;
    p->speed = 0.0f;
    p->target_speed = 0.0f;
    p->final_speed = 0.0f;
    p->final_position = 0.0f;
    p->acceleration = 0.0f;
    p->one_over_acc = 1.0f;
    p->sign = 1;
    p->state = PROFILE_IDLE;
}

bool profile_is_finished(const profile_t *p) {
    return p->state == PROFILE_FINISHED;
}

void profile_start(profile_t *p, float distance, float top_speed, float final_speed, float acceleration) {
    p->sign = (distance < 0.0f) ? -1 : 1;
    if (distance < 0.0f) {
        distance = -distance;
    }
    if (distance < 1.0f) {
        p->state = PROFILE_FINISHED;
        return;
    }
    if (final_speed > top_speed) {
        final_speed = top_speed;
    }

    p->position = 0.0f;
    p->final_position = distance;
    p->target_speed = p->sign * fabsf(top_speed);
    p->final_speed = p->sign * fabsf(final_speed);
    p->acceleration = fabsf(acceleration);
    p->one_over_acc = (p->acceleration >= 1.0f) ? (1.0f / p->acceleration) : 1.0f;
    p->state = PROFILE_ACCELERATING;
}

void profile_stop(profile_t *p) {
    p->target_speed = 0.0f;
    profile_finish(p);
}

void profile_finish(profile_t *p) {
    p->speed = p->target_speed;
    p->state = PROFILE_FINISHED;
}

void profile_set_speed(profile_t *p, float speed) {
    p->speed = speed;
}

void profile_set_target_speed(profile_t *p, float speed) {
    p->target_speed = speed;
}

void profile_adjust_position(profile_t *p, float delta) {
    p->position += delta;
}

void profile_set_position(profile_t *p, float position) {
    p->position = position;
}

float profile_position(const profile_t *p) {
    return p->position;
}

float profile_speed(const profile_t *p) {
    return p->speed;
}

float profile_acceleration(const profile_t *p) {
    return p->acceleration;
}

float profile_get_braking_distance(const profile_t *p) {
    return fabsf(p->speed * p->speed - p->final_speed * p->final_speed) * 0.5f * p->one_over_acc;
}

void profile_update(profile_t *p, float dt_s) {
    if (p->state == PROFILE_IDLE) {
        return;
    }

    float delta_v = p->acceleration * dt_s;
    float remaining = fabsf(p->final_position) - fabsf(p->position);

    if (p->state == PROFILE_ACCELERATING) {
        if (remaining < profile_get_braking_distance(p)) {
            p->state = PROFILE_BRAKING;
            if (p->final_speed == 0.0f) {
                // Small nonzero nudge (numerically ~ the acceleration
                // itself) so floating-point rounding can't stall the
                // profile just short of its target forever.
                p->target_speed = p->sign * 5.0f;
            } else {
                p->target_speed = p->final_speed;
            }
        }
    }

    if (p->speed < p->target_speed) {
        p->speed += delta_v;
        if (p->speed > p->target_speed) {
            p->speed = p->target_speed;
        }
    }
    if (p->speed > p->target_speed) {
        p->speed -= delta_v;
        if (p->speed < p->target_speed) {
            p->speed = p->target_speed;
        }
    }

    p->position += p->speed * dt_s;

    if (p->state != PROFILE_FINISHED && remaining < 0.125f) {
        p->state = PROFILE_FINISHED;
        p->target_speed = p->final_speed;
    }
}
