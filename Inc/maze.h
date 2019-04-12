#ifndef _MAZE_INC
#define _MAZE_INC

// Define & Macro
/////////////////
#define MAX_MAZE_DEPTH 10

#define IS_SET(value, flag, mask)       ( (value) & ((mask) & (flag)) )
#define IS_NOT_SET(value, flag, mask)  !( IS_SET( (value),(flag),(mask) ) )

#define MAZE_CASE_STATE_MASK    0x00FF
#define MAZE_CASE_NUMBER_MASK   0xFF00
#define MAZE_CASE_NUMBER_OFFSET 8

// Enum and Typedef
///////////////////

// All the maze case type
typedef enum {
    CASE_UNKNOWN     = 0x0000,  // 0000b : 0
    CASE_START       = 0x0001,  // 0001b : 1
    CASE_END         = 0x0002,  // 0010b : 3
    CASE_VISITED     = 0x0004,  // 0100b : 4
    CASE_RE_VISITED  = 0x0008,  // 1000b : 8
    CASE_WALL_N      = 0x0010,  //       : 16
    CASE_WALL_S      = 0x0020,  //       : 32
    CASE_WALL_E      = 0x0040,  //       : 64
    CASE_WALL_W      = 0x0080   //       : 128
} maze_case_t ;

// All the robot direction: do not change the order
typedef enum {
    DIR_N = 0,
    DIR_S = 1,
    DIR_E = 2,
    DIR_W = 3,
} robot_direction_t;

typedef struct {
  int               x;
  int               y;
  int               update_direction_flag;
  robot_direction_t direction;
} algo_update_t ;

typedef struct {
    maze_case_t  wall_test_1;
    action_t     action_test_1;
    const char * test_1_txt;
    maze_case_t  wall_test_2;
    action_t     action_test_2;
    const char * test_2_txt;
    maze_case_t  wall_test_3;
    action_t     action_test_3;
    const char * test_3_txt;
    action_t     action_test_4;
    const char * test_4_txt;
} algo_next_action_ctx_t;

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

// Prototypes
/////////////
char     *robot_direction_to_txt(robot_direction_t dir);
void      maze_ctx_init(maze_ctx_t *pCtx);
action_t  get_next_action(maze_ctx_t *pCtx, maze_case_t wall_sensor);
void      upadte_connex_case(maze_ctx_t *pCtx);
action_t  update_maze_ctx(maze_ctx_t *pCtx, maze_case_t wall_sensor);
void      display_maze_ctx(maze_ctx_t *pCtx);
void      display_case(maze_ctx_t *pCtx, int x, int y);
int       is_it_the_end(maze_ctx_t *pCtx);
void      maze_ctx_start(maze_ctx_t *pCtx);
maze_case_t get_wall_state(maze_ctx_t *pCtx);


#endif // _MAZE_INC

// end of maze.h file
