// Unit tests for maze.cpp - the maze grid + flood-fill/Dijkstra search.
// Pure C++, no Arduino/sensor/motor dependency, so this runs on the
// host: `pio test -e native`.
#include <unity.h>
#include <cstdint>

#include "maze.h"

void setUp(void) {}
void tearDown(void) {}

void test_turn_left_right_opposite(void)
{
    TEST_ASSERT_EQUAL_INT(MAZE_WEST, (int)maze_turn_left(MAZE_NORTH));
    TEST_ASSERT_EQUAL_INT(MAZE_EAST, (int)maze_turn_right(MAZE_NORTH));
    TEST_ASSERT_EQUAL_INT(MAZE_SOUTH, (int)maze_opposite(MAZE_NORTH));
    TEST_ASSERT_EQUAL_INT(MAZE_NORTH, (int)maze_opposite(MAZE_SOUTH));
}

void test_step_moves_one_cell_per_direction(void)
{
    int x, y;

    x = 5; y = 5; maze_step(&x, &y, MAZE_NORTH); TEST_ASSERT_EQUAL_INT(5, x); TEST_ASSERT_EQUAL_INT(6, y);
    x = 5; y = 5; maze_step(&x, &y, MAZE_EAST);  TEST_ASSERT_EQUAL_INT(6, x); TEST_ASSERT_EQUAL_INT(5, y);
    x = 5; y = 5; maze_step(&x, &y, MAZE_SOUTH); TEST_ASSERT_EQUAL_INT(5, x); TEST_ASSERT_EQUAL_INT(4, y);
    x = 5; y = 5; maze_step(&x, &y, MAZE_WEST);  TEST_ASSERT_EQUAL_INT(4, x); TEST_ASSERT_EQUAL_INT(5, y);
}

void test_in_bounds_and_index(void)
{
    TEST_ASSERT_TRUE(maze_in_bounds(0, 0));
    TEST_ASSERT_TRUE(maze_in_bounds(MAZE_WIDTH - 1, MAZE_HEIGHT - 1));
    TEST_ASSERT_FALSE(maze_in_bounds(-1, 0));
    TEST_ASSERT_FALSE(maze_in_bounds(MAZE_WIDTH, 0));
    TEST_ASSERT_FALSE(maze_in_bounds(0, MAZE_HEIGHT));

    TEST_ASSERT_EQUAL_INT(0, maze_index(0, 0));
    TEST_ASSERT_EQUAL_INT(1, maze_index(1, 0));
    TEST_ASSERT_EQUAL_INT(MAZE_WIDTH, maze_index(0, 1));
}

void test_init_sets_only_boundary_walls(void)
{
    maze_t m;
    maze_init(&m);

    uint8_t corner = m.walls[maze_index(0, 0)];
    TEST_ASSERT_TRUE(corner & MAZE_WALL_SOUTH);
    TEST_ASSERT_TRUE(corner & MAZE_WALL_WEST);
    TEST_ASSERT_FALSE(corner & MAZE_WALL_NORTH);
    TEST_ASSERT_FALSE(corner & MAZE_WALL_EAST);

    uint8_t far_corner = m.walls[maze_index(MAZE_WIDTH - 1, MAZE_HEIGHT - 1)];
    TEST_ASSERT_TRUE(far_corner & MAZE_WALL_NORTH);
    TEST_ASSERT_TRUE(far_corner & MAZE_WALL_EAST);

    TEST_ASSERT_EQUAL_INT(0, m.walls[maze_index(5, 5)]); // interior cell: no walls yet
}

void test_set_and_clear_wall_update_both_cells(void)
{
    maze_t m;
    maze_init(&m);

    maze_set_wall(&m, 5, 5, MAZE_NORTH);
    TEST_ASSERT_TRUE(m.walls[maze_index(5, 5)] & MAZE_WALL_NORTH);
    TEST_ASSERT_TRUE(m.walls[maze_index(5, 6)] & MAZE_WALL_SOUTH); // neighbor's matching side

    maze_clear_wall(&m, 5, 5, MAZE_NORTH);
    TEST_ASSERT_FALSE(m.walls[maze_index(5, 5)] & MAZE_WALL_NORTH);
    TEST_ASSERT_FALSE(m.walls[maze_index(5, 6)] & MAZE_WALL_SOUTH);
}

void test_flood_fill_matches_manhattan_distance_on_open_grid(void)
{
    maze_t m;
    maze_init(&m); // boundary walls only - fully open interior

    maze_cell_t goals[1] = { {0, 0} };
    maze_flood_fill(&m, goals, 1);

    TEST_ASSERT_EQUAL_INT16(0, m.dist[maze_index(0, 0)]);
    TEST_ASSERT_EQUAL_INT16(1, m.dist[maze_index(1, 0)]);
    TEST_ASSERT_EQUAL_INT16(MAZE_WIDTH - 1 + MAZE_HEIGHT - 1,
                             m.dist[maze_index(MAZE_WIDTH - 1, MAZE_HEIGHT - 1)]);
}

void test_flood_fill_leaves_walled_off_cell_unreachable(void)
{
    maze_t m;
    maze_init(&m);
    // Wall off cell (1,1) on all four sides.
    maze_set_wall(&m, 1, 1, MAZE_NORTH);
    maze_set_wall(&m, 1, 1, MAZE_EAST);
    maze_set_wall(&m, 1, 1, MAZE_SOUTH);
    maze_set_wall(&m, 1, 1, MAZE_WEST);

    maze_cell_t goals[1] = { {0, 0} };
    maze_flood_fill(&m, goals, 1);

    TEST_ASSERT_EQUAL_INT16(INT16_MAX, m.dist[maze_index(1, 1)]);
}

void test_choose_next_direction_picks_lowest_distance_neighbor(void)
{
    maze_t m;
    maze_init(&m);
    maze_cell_t goals[1] = { {0, 0} };
    maze_flood_fill(&m, goals, 1);

    // From (1,1): south->(1,0) dist 1, west->(0,1) dist 1 (tied),
    // north->(1,2) dist 3, east->(2,1) dist 3. Direction order iterated
    // is N,E,S,W and only a strictly-lower distance replaces the current
    // best, so the first cell at the winning distance (south) wins the tie.
    maze_dir_t chosen = maze_choose_next_direction(&m, 1, 1, MAZE_NORTH);
    TEST_ASSERT_EQUAL_INT(MAZE_SOUTH, (int)chosen);
}

void test_choose_next_direction_falls_back_to_heading_when_boxed_in(void)
{
    maze_t m;
    maze_init(&m);
    maze_set_wall(&m, 1, 1, MAZE_NORTH);
    maze_set_wall(&m, 1, 1, MAZE_EAST);
    maze_set_wall(&m, 1, 1, MAZE_SOUTH);
    maze_set_wall(&m, 1, 1, MAZE_WEST);

    maze_dir_t chosen = maze_choose_next_direction(&m, 1, 1, MAZE_EAST);
    TEST_ASSERT_EQUAL_INT(MAZE_EAST, (int)chosen); // no open neighbor - keeps current heading
}

void test_plan_min_turn_path_returns_zero_when_already_at_goal(void)
{
    maze_t m;
    maze_init(&m);
    maze_cell_t goals[1] = { {0, 0} };
    maze_action_t actions[8];

    int n = maze_plan_min_turn_path(&m, maze_cell_t{0, 0}, MAZE_NORTH, goals, 1, actions, 8);
    TEST_ASSERT_EQUAL_INT(0, n);
}

void test_plan_min_turn_path_straight_line_needs_no_turns(void)
{
    maze_t m;
    maze_init(&m); // open grid
    maze_cell_t goals[1] = { {0, 3} };
    maze_action_t actions[8];

    // Already facing the goal (north) with nothing in the way -
    // the cheapest path should be three plain forward moves.
    int n = maze_plan_min_turn_path(&m, maze_cell_t{0, 0}, MAZE_NORTH, goals, 1, actions, 8);

    TEST_ASSERT_EQUAL_INT(3, n);
    for (int i = 0; i < n; i++)
    {
        TEST_ASSERT_EQUAL_INT((int)MAZE_ACTION_FORWARD, (int)actions[i]);
    }
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_turn_left_right_opposite);
    RUN_TEST(test_step_moves_one_cell_per_direction);
    RUN_TEST(test_in_bounds_and_index);
    RUN_TEST(test_init_sets_only_boundary_walls);
    RUN_TEST(test_set_and_clear_wall_update_both_cells);
    RUN_TEST(test_flood_fill_matches_manhattan_distance_on_open_grid);
    RUN_TEST(test_flood_fill_leaves_walled_off_cell_unreachable);
    RUN_TEST(test_choose_next_direction_picks_lowest_distance_neighbor);
    RUN_TEST(test_choose_next_direction_falls_back_to_heading_when_boxed_in);
    RUN_TEST(test_plan_min_turn_path_returns_zero_when_already_at_goal);
    RUN_TEST(test_plan_min_turn_path_straight_line_needs_no_turns);
    return UNITY_END();
}
