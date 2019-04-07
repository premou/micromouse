#ifndef MAZE_H_
#define MAZE_H_

#include "stm32f7xx_hal.h"
#include <stdbool.h>

// DEFINE
#define MAX_MAZE_DEPTH 10

#ifdef DEFINE_VARIABLES
#define EXTERN /* nothing */
#else
#define EXTERN extern
#endif /* DEFINE_VARIABLES */

// All the maze case type
typedef enum {
    CASE_UNKNOWN     = 0x00,  // 0000b
    CASE_START       = 0x01,  // 0001b
    CASE_END         = 0x02,  // 0010b
    CASE_VISITED     = 0x04,  // 0100b
    CASE_RE_VISITED  = 0x08,  // 1000b
    CASE_WALL_LEFT   = 0x10,
    CASE_WALL_RIGHT  = 0x20,
    CASE_WALL_FRONT  = 0x40
} maze_case_t ;

// All the robot direction
typedef enum {
	RIGHT = 0,
	UP,
	LEFT,
	DOWN
} robot_direction_t;

// STRUCT
typedef struct {
	// The maze array:               x              y
	maze_case_t maze_array[MAX_MAZE_DEPTH][MAX_MAZE_DEPTH];

	// current direction:
	robot_direction_t current_direction;

    // current position
	int32_t current_x;
	int32_t current_y;

	// start position
	int32_t start_x;
	int32_t start_y;

	// end position
	int32_t end_x;
	int32_t end_y;

} maze_ctx_t;

// GLOBAL DEFINITION
EXTERN maze_ctx_t the_maze_ctx ;

// PROTOTYPES
char     *robot_direction_to_txt(robot_direction_t dir);
void      maze_ctx_init(maze_ctx_t *pCtx);
action_t  get_next_action(maze_case_t wall_sensor);
void      update_maze_ctx(maze_ctx_t *pCtx, maze_case_t wall_sensor);
void      display_maze_ctx(maze_ctx_t *pCtx);

#endif // MAZE_H_

// End of file
