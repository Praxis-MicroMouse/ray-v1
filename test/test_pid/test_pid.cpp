// Unit tests for pid.cpp - the generic PID controller. Pure C++, no
// Arduino dependency, so this runs on the host: `pio test -e native`.
#include <unity.h>

#include "pid.h"

void setUp(void) {}
void tearDown(void) {}

void test_init_sets_gains_and_clears_state(void)
{
    pid_ctrl_t pid;
    pid_init(&pid, 1.5f, 0.5f, 0.25f, 100.0f);

    TEST_ASSERT_EQUAL_FLOAT(1.5f, pid.kp);
    TEST_ASSERT_EQUAL_FLOAT(0.5f, pid.ki);
    TEST_ASSERT_EQUAL_FLOAT(0.25f, pid.kd);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, pid.integral_limit);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.integral);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.prev_error);
    TEST_ASSERT_FALSE(pid.has_prev);
}

void test_first_update_has_no_derivative_kick(void)
{
    // kd nonzero, but the very first update() has no prev_error yet -
    // the derivative term must be 0, not a spike off an undefined value.
    pid_ctrl_t pid;
    pid_init(&pid, 0.0f, 0.0f, 3.0f, 0.0f);

    float out = pid_update(&pid, 10.0f, 4.0f, 0.1f);

    TEST_ASSERT_EQUAL_FLOAT(0.0f, out);
    TEST_ASSERT_TRUE(pid.has_prev);
    TEST_ASSERT_EQUAL_FLOAT(6.0f, pid.prev_error);
}

void test_proportional_term(void)
{
    pid_ctrl_t pid;
    pid_init(&pid, 2.0f, 0.0f, 0.0f, 0.0f); // integral_limit 0 = unlimited

    float out = pid_update(&pid, 10.0f, 4.0f, 0.1f); // error = 6

    TEST_ASSERT_EQUAL_FLOAT(12.0f, out); // kp * error
    // Integral still accumulates internally even with ki == 0.
    TEST_ASSERT_EQUAL_FLOAT(0.6f, pid.integral); // error * dt
}

void test_integral_anti_windup_clamp(void)
{
    pid_ctrl_t pid;
    pid_init(&pid, 0.0f, 1.0f, 0.0f, 5.0f); // ki = 1, clamp at +-5

    float out = pid_update(&pid, 10.0f, 0.0f, 1.0f); // error = 10, integral would be 10
    TEST_ASSERT_EQUAL_FLOAT(5.0f, pid.integral);      // clamped
    TEST_ASSERT_EQUAL_FLOAT(5.0f, out);

    // Stays clamped, doesn't keep climbing on repeated saturation.
    out = pid_update(&pid, 10.0f, 0.0f, 1.0f);
    TEST_ASSERT_EQUAL_FLOAT(5.0f, pid.integral);
    TEST_ASSERT_EQUAL_FLOAT(5.0f, out);
}

void test_derivative_term_uses_previous_error(void)
{
    pid_ctrl_t pid;
    pid_init(&pid, 0.0f, 0.0f, 1.0f, 0.0f); // kd = 1

    pid_update(&pid, 10.0f, 0.0f, 1.0f); // error = 10, first call, no derivative
    float out = pid_update(&pid, 10.0f, 4.0f, 1.0f); // error = 6, derivative = (6-10)/1 = -4

    TEST_ASSERT_EQUAL_FLOAT(-4.0f, out);
}

void test_reset_clears_state_but_keeps_gains(void)
{
    pid_ctrl_t pid;
    pid_init(&pid, 1.0f, 1.0f, 1.0f, 0.0f);
    pid_update(&pid, 10.0f, 0.0f, 1.0f); // dirties integral/prev_error/has_prev

    pid_reset(&pid);

    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.integral);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.prev_error);
    TEST_ASSERT_FALSE(pid.has_prev);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, pid.kp); // gains untouched
}

void test_set_gains_does_not_reset_state(void)
{
    pid_ctrl_t pid;
    pid_init(&pid, 1.0f, 1.0f, 1.0f, 0.0f);
    pid_update(&pid, 10.0f, 0.0f, 1.0f); // integral becomes nonzero

    pid_set_gains(&pid, 2.0f, 2.0f, 2.0f);

    TEST_ASSERT_EQUAL_FLOAT(2.0f, pid.kp);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, pid.ki);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, pid.kd);
    TEST_ASSERT_TRUE(pid.integral > 0.0f); // state carried over - caller must pid_reset() separately
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_sets_gains_and_clears_state);
    RUN_TEST(test_first_update_has_no_derivative_kick);
    RUN_TEST(test_proportional_term);
    RUN_TEST(test_integral_anti_windup_clamp);
    RUN_TEST(test_derivative_term_uses_previous_error);
    RUN_TEST(test_reset_clears_state_but_keeps_gains);
    RUN_TEST(test_set_gains_does_not_reset_state);
    return UNITY_END();
}
