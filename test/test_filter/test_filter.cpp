// Unit tests for filter.cpp - the despike + EMA smoothing filter. Pure
// C++, no Arduino dependency, so this runs on the host: `pio test -e native`.
#include <unity.h>

#include "filter.h"

void setUp(void) {}
void tearDown(void) {}

void test_init_state(void)
{
    filter_t f;
    filter_init(&f);

    TEST_ASSERT_EQUAL_FLOAT(0.0f, f.estimate);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, f.last_raw);
    TEST_ASSERT_FALSE(f.initialized);
}

void test_first_reading_snaps_to_raw(void)
{
    filter_t f;
    filter_init(&f);

    float out = filter_update(&f, 200.0f);

    TEST_ASSERT_EQUAL_FLOAT(200.0f, out);
    TEST_ASSERT_TRUE(f.initialized);
}

void test_small_change_blends_via_ema(void)
{
    filter_t f;
    filter_init(&f);
    filter_update(&f, 200.0f); // primes estimate at 200

    // delta = 10mm, well under FILTER_SPIKE_THRESHOLD_MM (150) -
    // blends in at FILTER_EMA_ALPHA (0.35): 200 + 0.35*10 = 203.5
    float out = filter_update(&f, 210.0f);

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 203.5f, out);
}

void test_isolated_spike_is_rejected(void)
{
    filter_t f;
    filter_init(&f);
    filter_update(&f, 200.0f); // estimate = last_raw = 200

    // One-off jump of 300mm (> the 150mm threshold) that the previous
    // raw reading doesn't corroborate - estimate must hold steady.
    float out = filter_update(&f, 500.0f);

    TEST_ASSERT_EQUAL_FLOAT(200.0f, out);
}

void test_confirmed_jump_passes_through_on_second_sample(void)
{
    filter_t f;
    filter_init(&f);
    filter_update(&f, 200.0f); // estimate = 200, last_raw = 200
    filter_update(&f, 500.0f); // rejected spike - estimate held at 200, last_raw now 500

    // A second raw reading close to the first "spike" (within threshold
    // of it) confirms it's a real change, not noise - should blend in:
    // 200 + 0.35*(505-200) = 306.75
    float out = filter_update(&f, 505.0f);

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 306.75f, out);
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_state);
    RUN_TEST(test_first_reading_snaps_to_raw);
    RUN_TEST(test_small_change_blends_via_ema);
    RUN_TEST(test_isolated_spike_is_rejected);
    RUN_TEST(test_confirmed_jump_passes_through_on_second_sample);
    return UNITY_END();
}
