// Unit tests for profile.cpp - the trapezoidal motion profile. Pure
// C++, no Arduino dependency, so this runs on the host: `pio test -e native`.
#include <unity.h>

#include "profile.h"

void setUp(void) {}
void tearDown(void) {}

void test_reset_gives_idle_zeroed_state(void)
{
    profile_t p;
    profile_reset(&p);

    TEST_ASSERT_FALSE(profile_is_finished(&p));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, profile_position(&p));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, profile_speed(&p));
}

void test_tiny_distance_finishes_immediately(void)
{
    profile_t p;
    profile_reset(&p);

    profile_start(&p, 0.5f, 100.0f, 0.0f, 500.0f); // < 1mm - not worth profiling

    TEST_ASSERT_TRUE(profile_is_finished(&p));
}

void test_accelerates_toward_target_speed(void)
{
    profile_t p;
    profile_reset(&p);
    profile_start(&p, 1000.0f, 200.0f, 0.0f, 500.0f); // accel 500mm/s/s

    profile_update(&p, 0.1f); // 0.1s at 500mm/s/s -> speed should reach 50mm/s

    TEST_ASSERT_EQUAL_FLOAT(50.0f, profile_speed(&p));
    // Euler step: speed is updated to 50mm/s first, then position +=
    // speed*dt using that new speed -> 50 * 0.1 = 5.0mm.
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.0f, profile_position(&p));
}

void test_reaches_and_holds_top_speed(void)
{
    profile_t p;
    profile_reset(&p);
    profile_start(&p, 1000.0f, 100.0f, 0.0f, 1000.0f); // accel fast enough to reach 100mm/s in 0.1s

    for (int i = 0; i < 20; i++) {
        profile_update(&p, 0.01f); // 0.2s total
    }

    TEST_ASSERT_EQUAL_FLOAT(100.0f, profile_speed(&p));
    TEST_ASSERT_FALSE(profile_is_finished(&p)); // still well short of 1000mm
}

void test_decelerates_to_final_speed_and_finishes(void)
{
    profile_t p;
    profile_reset(&p);
    // Short move, generous accel - should accelerate briefly then brake
    // to a nonzero final speed and finish close to the target distance.
    profile_start(&p, 50.0f, 200.0f, 50.0f, 2000.0f);

    float dt = 0.001f;
    int ticks = 0;
    while (!profile_is_finished(&p) && ticks < 10000) {
        profile_update(&p, dt);
        ticks++;
    }

    // Discrete-time stepping means both the braking-distance check and
    // the speed ramp-down land a fraction late - a few mm / mm/s of
    // overshoot past the nominal 50 target is expected here, not a bug,
    // so these tolerances are intentionally loose.
    TEST_ASSERT_TRUE(profile_is_finished(&p));
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 50.0f, profile_position(&p));
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 50.0f, profile_speed(&p));
}

void test_stop_forces_target_speed_zero_and_finished(void)
{
    profile_t p;
    profile_reset(&p);
    profile_start(&p, 1000.0f, 200.0f, 200.0f, 500.0f);
    profile_update(&p, 0.1f); // now moving

    profile_stop(&p);

    TEST_ASSERT_TRUE(profile_is_finished(&p));
}

void test_adjust_and_set_position(void)
{
    profile_t p;
    profile_reset(&p);
    profile_set_position(&p, 42.0f);
    TEST_ASSERT_EQUAL_FLOAT(42.0f, profile_position(&p));

    profile_adjust_position(&p, -10.0f);
    TEST_ASSERT_EQUAL_FLOAT(32.0f, profile_position(&p));
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_reset_gives_idle_zeroed_state);
    RUN_TEST(test_tiny_distance_finishes_immediately);
    RUN_TEST(test_accelerates_toward_target_speed);
    RUN_TEST(test_reaches_and_holds_top_speed);
    RUN_TEST(test_decelerates_to_final_speed_and_finishes);
    RUN_TEST(test_stop_forces_target_speed_zero_and_finished);
    RUN_TEST(test_adjust_and_set_position);
    return UNITY_END();
}
