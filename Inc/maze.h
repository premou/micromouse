#ifndef _MAZE_INC
#define _MAZE_INC

// Define & Macro
/////////////////
#define MAX_MAZE_DEPTH 20

#define IS_SET(value, flag, mask)       ( (value) & ((mask) & (flag)) )
#define IS_NOT_SET(value, flag, mask)  !( IS_SET( (value),(flag),(mask) ) )
#define GET_CASE_NUMBER(case)           ( (MAZE_CASE_NUMBER_MASK & (case)) >>  MAZE_CASE_NUMBER_OFFSET)
#define GET_WALL_STATE(case)            ( MAZE_CASE_WALL_MASK & (case) )

#define WALL_STATE_MASK         0xF
#define MAZE_CASE_STATE_MASK    0x00FF
#define MAZE_CASE_WALL_MASK     0x000F
#define MAZE_CASE_NUMBER_MASK   0xFF00
#define MAZE_CASE_NUMBER_OFFSET 8


// Enum
///////

// All the maze case type
typedef enum {
    CASE_UNKNOWN     = 0x00,  // 00000000b : 0
    CASE_WALL_N      = 0x01,  // 00000001b : 1
    CASE_WALL_S      = 0x02,  // 00000010b : 2
    CASE_WALL_E      = 0x04,  // 00000100b : 4
    CASE_WALL_W      = 0x08,  // 00001000b : 8
    CASE_START       = 0x10,  // 00010000b : 16
    CASE_END         = 0x20,  // 00100000b : 32
    CASE_VISITED     = 0x40   // 01000000b : 64
} maze_case_t ;

// All the robot direction: do not change the order
typedef enum {
    DIR_N = 0,
    DIR_S = 1,
    DIR_E = 2,
    DIR_W = 3,
} robot_direction_t;

// Learn or solve mode
typedef enum {
    LEARN = 0,
    SOLVE = 1
} maze_mode_t;

// Left or right hand
typedef enum {
    LEFT_HAND  = 0,
    RIGHT_HAND = 1
} maze_algo_mode_t;

// Typedef
//////////

typedef struct {
    int               x;
    int               y;
    robot_direction_t direction;
} algo_update_t;

typedef struct {
    maze_case_t  test_1_wall   ;
    action_t     test1_action  ;
    maze_case_t  test_2_wall   ;
    action_t     test_2_action ;
    maze_case_t  test_3_wall   ;
    action_t     test_3_action ;
    action_t     test_4_action ;
} algo_next_action_ctx_t       ;

//To store matrix cell cordinates
typedef struct
{
    int x;
    int y;
} point_t;

// A Data Structure for queue used in BFS
typedef struct
{
    point_t  pt;    // The cordinates of a cell
    int      dist;  // cell's distance of from the source
    void    *pNext; // next node in queue
} queue_node_t;

typedef struct
{
    int   nbElem;
    void *pFront;
} queue_t;

typedef struct {
	// maze mode: learn or solve
	maze_mode_t mode;

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

	// The maze array:               x              y
    maze_case_t maze_array[MAX_MAZE_DEPTH][MAX_MAZE_DEPTH];

    // For solve mode
    maze_case_t solve_array[MAX_MAZE_DEPTH][MAX_MAZE_DEPTH];

    // For solve mode
    maze_case_t shortest_array[MAX_MAZE_DEPTH][MAX_MAZE_DEPTH];

    // Min distance for shortest way
    int min_dist;

    uint8_t switch_led;
} maze_ctx_t;

// Prototypes
/////////////
char        *robot_direction_to_txt(robot_direction_t dir);
void         maze_ctx_init(maze_ctx_t *pCtx);
action_t     get_next_action(maze_ctx_t *pCtx, maze_case_t wall_sensor, maze_algo_mode_t algo);
void         upadte_connex_case(maze_ctx_t *pCtx);
action_t     update_maze_ctx(maze_ctx_t *pCtx);
void         display_maze_ctx(maze_ctx_t *pCtx);
void         display_case(maze_ctx_t *pCtx, int x, int y);
int          is_it_the_end(maze_ctx_t *pCtx);
void         maze_ctx_start(maze_ctx_t *pCtx);
maze_case_t  get_wall_state(robot_direction_t current_direction);
void         findShortestPath(maze_ctx_t *pCtx,
                      int i, int j, // current position
                      int x, int y, // exit
                      int dist);
int          isValid(int x, int y);
int          isSafe(maze_ctx_t *pCtx, int x, int y);

#endif // _MAZE_INC

// end of maze.h file
