#ifndef _MAZE_INC
#define _MAZE_INC

// Define & Macro
/////////////////
#define MAX_MAZE_DEPTH  (24)
#define MAX_ACTION      (50)
#define MAX_INTER       (30)
#define MAX_INT         (2147483647)

#define IS_SET(value, flag, mask)       ( (value) & ((mask) & (flag)) )
#define IS_NOT_SET(value, flag, mask)  !( IS_SET( (value),(flag),(mask) ) )
#define GET_WALL_STATE(case)            ( MAZE_CASE_WALL_MASK & (case) )

#define MAZE_CASE_STATE_MASK    0x00FF
#define MAZE_CASE_WALL_MASK     0x000F

// Enum
///////

// All the maze case type
typedef enum {
	CASE_UNKNOWN     = 0x00,  // 00000000b :  0
	CASE_WALL_N      = 0x01,  // 00000001b :  1
	CASE_WALL_S      = 0x02,  // 00000010b :  2
	CASE_WALL_E      = 0x04,  // 00000100b :  4
	CASE_WALL_W      = 0x08,  // 00001000b :  8
	CASE_START       = 0x10,  // 00010000b : 16
	CASE_END         = 0x20,  // 00100000b : 32
	CASE_VISITED     = 0x40,  // 01000000b : 64
	CASE_TO_VISIT    = 0x80,  // 10000000b :128
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
	// Left / rigth is free and case is not visited
	maze_case_t  test_1_wall   ;
	int          test_1_x      ;
	int          test_1_y      ;
	action_t     test1_action  ;

	// Ahead is free and case is not visited
	maze_case_t  test_2_wall   ;
	int          test_2_x      ;
	int          test_2_y      ;
	action_t     test_2_action ;

	// Rigth / left is free and case is not visited
	maze_case_t  test_3_wall   ;
	int          test_3_x      ;
	int          test_3_y      ;
	action_t     test_3_action ;

	// Cul de sac => must turn around
	maze_case_t  test_4_wall   ;
	action_t     test_4_action ;
} algo_next_action_ctx_t       ;

typedef struct {
	action_t          action;
	int               x;
	int               y;
	robot_direction_t dir;
} action_ctx ;

typedef struct {
	int enable;
	int x;
	int y;
} inter_t ;

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

	// set to 1 for left hand algo
	maze_algo_mode_t algo;

	// Min distance for shortest way
	int min_dist;

	// Number max of action
	int nb_action;

	// Current index in the action array
	int current_action_index;

	// Number max of intersection to visit
	int nb_inter;

	// Intesection list
	inter_t inter_list[MAX_INTER];

	// Action list
	action_ctx action_list[MAX_ACTION];

	// The maze array:               x              y
	maze_case_t maze_array[MAX_MAZE_DEPTH][MAX_MAZE_DEPTH];

	// For solve mode
	maze_case_t solve_array[MAX_MAZE_DEPTH][MAX_MAZE_DEPTH];

	// For solve mode
	maze_case_t shortest_array[MAX_MAZE_DEPTH][MAX_MAZE_DEPTH];

    // For intersection
    maze_case_t inter_array[MAX_MAZE_DEPTH][MAX_MAZE_DEPTH];

} maze_ctx_t;

// Prototypes
/////////////
void         maze_ctx_init(maze_ctx_t *pCtx);
void         maze_ctx_start(maze_ctx_t *pCtx);
void         init_action_array(maze_ctx_t *pCtx);
void         init_inter_array(maze_ctx_t *pCtx);
void         display_maze_ctx(maze_ctx_t *pCtx);
void         display_action_list(maze_ctx_t *pCtx);

action_t     get_next_action(maze_ctx_t *pCtx, maze_case_t wall_sensor);
void         update_related_case(maze_ctx_t *pCtx);
action_t     update_maze_ctx(maze_ctx_t *pCtx);
int          is_it_the_end(maze_ctx_t *pCtx);
maze_case_t  get_wall_state(robot_direction_t current_direction);
void         find_shortest_path(maze_ctx_t *pCtx,
		int i, int j, // from
		int x, int y, // to
		int dist,
		int search_inter);
void         build_action_list(maze_ctx_t *pCtx,
		robot_direction_t from_dir,
		int from_x, int from_y,
		int to_x, int to_y,
		int search_inter);
action_t     get_next_next_action(maze_ctx_t *pCtx);
void         update_intersection(maze_ctx_t *pCtx);

#endif // _MAZE_INC

// end of maze.h file
