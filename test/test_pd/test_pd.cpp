// Unit tests for pd.cpp - the generic PD controller. Pure C++, no
// Arduino dependency, so this runs on the host: `pio test -e native`.
#include <unity.h>

#include "pd.h"

void setUp(void) {}
void tearDown(void) {}

void test_init_sets_gains_and_clears_state(void)
{
    pd_ctrl_t pd;
    pd_init(&pd, 1.5f, 0.25f);

    TEST_ASSERT_EQUAL_FLOAT(1.5f, pd.kp);
    TEST_ASSERT_EQUAL_FLOAT(0.25f, pd.kd);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pd.prev_error);
    TEST_ASSERT_FALSE(pd.has_prev);
}

void test_first_update_has_no_derivative_kick(void)
{
    // kd nonzero, but the very first update() has no prev_error yet -
    // the derivative term must be 0, not a spike off an undefined value.
    pd_ctrl_t pd;
    pd_init(&pd, 0.0f, 3.0f);

    float out = pd_update(&pd, 10.0f, 4.0f, 0.1f);

    TEST_ASSERT_EQUAL_FLOAT(0.0f, out);
    TEST_ASSERT_TRUE(pd.has_prev);
    TEST_ASSERT_EQUAL_FLOAT(6.0f, pd.prev_error);
}

void test_proportional_term(void)
{
    pd_ctrl_t pd;
    pd_init(&pd, 2.0f, 0.0f);

    float out = pd_update(&pd, 10.0f, 4.0f, 0.1f); // error = 6

    TEST_ASSERT_EQUAL_FLOAT(12.0f, out); // kp * error
}

void test_derivative_term_uses_previous_error(void)
{
    pd_ctrl_t pd;
    pd_init(&pd, 0.0f, 1.0f); // kd = 1

    pd_update(&pd, 10.0f, 0.0f, 1.0f); // error = 10, first call, no derivative
    float out = pd_update(&pd, 10.0f, 4.0f, 1.0f); // error = 6, derivative = (6-10)/1 = -4

    TEST_ASSERT_EQUAL_FLOAT(-4.0f, out);
}

void test_update_error_skips_the_setpoint_measurement_subtraction(void)
{
    // drive_controller.cpp's rotation loop feeds a precomputed
    // accumulated error directly, rather than a setpoint/measurement
    // pair - pd_update_error() must treat it as the error as-is.
    pd_ctrl_t pd;
    pd_init(&pd, 2.0f, 0.0f);

    float out = pd_update_error(&pd, -5.0f, 0.1f);

    TEST_ASSERT_EQUAL_FLOAT(-10.0f, out); // kp * error, error used verbatim
}

void test_reset_clears_state_but_keeps_gains(void)
{
    pd_ctrl_t pd;
    pd_init(&pd, 1.0f, 1.0f);
    pd_update(&pd, 10.0f, 0.0f, 1.0f); // dirties prev_error/has_prev

    pd_reset(&pd);

    TEST_ASSERT_EQUAL_FLOAT(0.0f, pd.prev_error);
    TEST_ASSERT_FALSE(pd.has_prev);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, pd.kp); // gains untouched
}

void test_set_gains_does_not_reset_state(void)
{
    pd_ctrl_t pd;
    pd_init(&pd, 1.0f, 1.0f);
    pd_update(&pd, 10.0f, 0.0f, 1.0f); // has_prev becomes true, prev_error becomes 10

    pd_set_gains(&pd, 2.0f, 2.0f);

    float kp, kd;
    pd_get_gains(&pd, &kp, &kd);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, kp);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, kd);
    TEST_ASSERT_TRUE(pd.has_prev); // state carried over - caller must pd_reset() separately
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_sets_gains_and_clears_state);
    RUN_TEST(test_first_update_has_no_derivative_kick);
    RUN_TEST(test_proportional_term);
    RUN_TEST(test_derivative_term_uses_previous_error);
    RUN_TEST(test_update_error_skips_the_setpoint_measurement_subtraction);
    RUN_TEST(test_reset_clears_state_but_keeps_gains);
    RUN_TEST(test_set_gains_does_not_reset_state);
    return UNITY_END();
}
