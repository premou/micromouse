#include "stdio.h"
#include "assert.h"

// For unitary test purpose
//#define _MAZE_STANDALONE

// Maze header
#include "controller.h"
#include "serial.h"
#include "WallSensor.h"
#include "maze.h"

#ifndef _MAZE_STANDALONE
// globals
extern HAL_Serial_Handler com;
#endif

// Const
////////
const algo_update_t algo_update_maze_ctx_RUN_1[4] =
{
    // new x, new y,  new dir
    {      0,     1,  DIR_N}, // DIR_N
    {      0,    -1,  DIR_S}, // DIR_S
    {      1,     0,  DIR_E}, // DIR_E
    {     -1,     0,  DIR_W}  // DIR_W
} ;

const algo_update_t algo_update_maze_ctx_TURN_RIGHT[4] =
{
    // new x, new y,  new dir
    {     0,     1,   DIR_E}, // DIR_N
    {     0,    -1,   DIR_W}, // DIR_S
    {     1,     0,   DIR_S}, // DIR_E
    {    -1,     0,   DIR_N}  // DIR_W
} ;

const algo_update_t algo_update_maze_ctx_TURN_LEFT[4] =
{
    // new x, new y,  new dir
    {      0,     1,  DIR_W}, // DIR_N
    {      0,    -1,  DIR_E}, // DIR_S
    {      1,     0,  DIR_N}, // DIR_E
    {     -1,     0,  DIR_S}  // DIR_W
} ;

const algo_update_t algo_update_maze_ctx_U_TURN_RIGHT[4] =
{
    // new x, new y,  new dir
    {      0,     1,  DIR_S}, // DIR_N
    {      0,    -1,  DIR_N}, // DIR_S
    {      1,     0,  DIR_W}, // DIR_E
    {     -1,     0,  DIR_E}  // DIR_W
} ;

// Left hand algo
const algo_next_action_ctx_t algo_next_action_left_hand[4] =
{
    // DIR_N
    {
        CASE_WALL_W, ACTION_TURN_LEFT,
        CASE_WALL_N, ACTION_RUN_1,
        CASE_WALL_E, ACTION_TURN_RIGHT,
        ACTION_U_TURN_RIGHT
    },

    // DIR_S
    {
        CASE_WALL_E, ACTION_TURN_LEFT,
        CASE_WALL_S, ACTION_RUN_1,
        CASE_WALL_W, ACTION_TURN_RIGHT,
        ACTION_U_TURN_RIGHT
    },

    // DIR_E
    {
        CASE_WALL_N, ACTION_TURN_LEFT,
        CASE_WALL_E, ACTION_RUN_1,
        CASE_WALL_S, ACTION_TURN_RIGHT,
        ACTION_U_TURN_RIGHT
    },

    // DIR_W
    {
        CASE_WALL_S, ACTION_TURN_LEFT,
        CASE_WALL_W, ACTION_RUN_1,
        CASE_WALL_N, ACTION_TURN_RIGHT,
        ACTION_U_TURN_RIGHT
    }
};

// Right hand algo
const algo_next_action_ctx_t algo_next_action_right_hand[4] =
{
    // DIR_N
    {
        CASE_WALL_E, ACTION_TURN_RIGHT,
        CASE_WALL_N, ACTION_RUN_1,
        CASE_WALL_W, ACTION_TURN_LEFT,
        ACTION_U_TURN_RIGHT
    },

    // DIR_S
    {
        CASE_WALL_W, ACTION_TURN_RIGHT,
        CASE_WALL_S, ACTION_RUN_1,
        CASE_WALL_E, ACTION_TURN_LEFT,
        ACTION_U_TURN_RIGHT
    },

    // DIR_E
    {
        CASE_WALL_S, ACTION_TURN_RIGHT,
        CASE_WALL_E, ACTION_RUN_1,
        CASE_WALL_N, ACTION_TURN_LEFT,
        ACTION_U_TURN_RIGHT
    },

    // DIR_W
    {
        CASE_WALL_N, ACTION_TURN_RIGHT,
        CASE_WALL_W, ACTION_RUN_1,
        CASE_WALL_S, ACTION_TURN_LEFT,
        ACTION_U_TURN_RIGHT
    }
};

const char *wall_state_txt[16] =
{
    "----", // 0000b
    "---N", // 0001b
    "--S-", // 0010b
    "--SN", // 0011b
    "-E--", // 0100b
    "-E-N", // 0101b
    "-ES-", // 0110b
    "-ESN", // 0111b
    "W---", // 1000b
    "W--N", // 1001b
    "W-S-", // 1010b
    "W-SN", // 1011b
    "WE--", // 1100b
    "WE-N", // 1101b
    "WES-", // 1110b
    "WESN", // 1111b
};

const char* maze_mode_txt[2] =
{
    "LEARN",
    "SOLVE"
};

// These arrays are used to get row and column
// numbers of 4 neighbours of a given cell
const int rowNum[4] = {-1, 0, 0, 1};
const int colNum[4] = {0, -1, 1, 0};

// Local global
static int step = 0;

// Txt function for enum action_t
static inline char *action2txt(action_t a)
{
    static const char *strings[] = {
        "IDLE        ",
        "START       ",
        "RUN_1       ",
        "TURN_RIGHT  ",
        "TURN_LEFT   ",
        "U_TURN_RIGHT",
        "STOP        ",
        "CTR         "
    };
    return ((char *)strings[a]);
}

// Return the printable arrow direction
char *robot_direction_to_txt(robot_direction_t dir)
{
    switch(dir)
    {
        default:
            return("?");
            break;
        case DIR_E:
            return("E");
            break;
        case DIR_N:
            return("N");
            break;
        case DIR_W:
            return("W");
            break;
        case DIR_S:
            return("S");
            break;
    }
}// end of robot_direction_to_txt

// Fill the array maze with case unknown pattern
void maze_ctx_start(maze_ctx_t *pCtx)
{
    // Restart from the beginning
    pCtx->current_direction = DIR_N;
    pCtx->current_x = pCtx->start_x ;
    pCtx->current_y = pCtx->start_y ;
    // Now in SOLVE mode
    pCtx->mode = SOLVE;
}// end of maze_ctx_start

// Fill the array maze with case unknown pattern
void maze_ctx_init(maze_ctx_t *pCtx)
{
    uint32_t x, y;
    for(x=0; x<MAX_MAZE_DEPTH; x++)
    {
        for(y=0; y<MAX_MAZE_DEPTH; y++)
        {
            pCtx->maze_array[x][y]  = CASE_UNKNOWN;
            pCtx->solve_array[x][y] = CASE_UNKNOWN;
        }
    }

    // Start in LEARN mode
    pCtx->mode = LEARN;

    pCtx->current_direction = DIR_N;

    pCtx->start_x = (MAX_MAZE_DEPTH/2)-1;
    pCtx->start_y = 0;

    // End position is unknown
    pCtx->end_x = -1;
    pCtx->end_y = -1;

    pCtx->current_x = pCtx->start_x;
    pCtx->current_y = pCtx->start_y;

    pCtx->maze_array[pCtx->current_x][pCtx->current_y] = CASE_START |
    CASE_VISITED |
    CASE_WALL_W  |
    CASE_WALL_E  |
    CASE_WALL_S  ;

    upadte_connex_case(pCtx);

    pCtx->maze_array[pCtx->current_x][pCtx->current_y] |= (step++) << MAZE_CASE_NUMBER_OFFSET;

}// end of maze_ctx_init


// Return the next action, depending on the wall sensor : left hand first choise
action_t get_next_action(maze_ctx_t *pCtx, maze_case_t wall_sensor, maze_algo_mode_t algo)
{
    action_t                next_action;
    algo_next_action_ctx_t *pAlgo;
    robot_direction_t       dir;

    if(LEFT_HAND == algo)
    {
        pAlgo = (algo_next_action_ctx_t *) algo_next_action_left_hand ;
    }
    else
    {
        pAlgo = (algo_next_action_ctx_t *) algo_next_action_right_hand ;
    }

    dir = pCtx->current_direction ;

    // First test
    if( IS_NOT_SET(wall_sensor, pAlgo[dir].test_1_wall, MAZE_CASE_STATE_MASK) )
    {
        next_action = pAlgo[dir].test1_action;
    }
    else
    {
        // Second test
        if( IS_NOT_SET(wall_sensor, pAlgo[dir].test_2_wall, MAZE_CASE_STATE_MASK) )
        {
            next_action = pAlgo[dir].test_2_action;
        }
        else
        {
            // Third test
            if( IS_NOT_SET(wall_sensor, pAlgo[dir].test_3_wall, MAZE_CASE_STATE_MASK) )
            {
                next_action = pAlgo[dir].test_3_action;
            }
            else
            {
                // Last action
                next_action = pAlgo[dir].test_4_action;
            }
        }
    }
    return(next_action);
}// end of get_next_action

// Update connxex case
void upadte_connex_case(maze_ctx_t *pCtx)
{
    maze_case_t current_case;

    current_case = pCtx->maze_array[pCtx->current_x][pCtx->current_y];

    // Update S case
    if((pCtx->current_y - 1) >= 0)
    {
        if( IS_SET(current_case, CASE_WALL_S, MAZE_CASE_STATE_MASK) )
        {
            pCtx->maze_array[pCtx->current_x][pCtx->current_y - 1] |= CASE_WALL_N ;
        }
    }

    // Upadte N case
    if((pCtx->current_y + 1) < MAX_MAZE_DEPTH)
    {
        if( IS_SET(current_case, CASE_WALL_N, MAZE_CASE_STATE_MASK) )
        {
            pCtx->maze_array[pCtx->current_x][pCtx->current_y + 1] |= CASE_WALL_S;
        }
    }

    // Update W case
    if((pCtx->current_x - 1) >= 0)
    {
        if(IS_SET(current_case, CASE_WALL_W, MAZE_CASE_STATE_MASK))
        {
            pCtx->maze_array[pCtx->current_x - 1][pCtx->current_y] |= CASE_WALL_E;
        }
    }

    // Upadte E case
    if((pCtx->current_x + 1) < MAX_MAZE_DEPTH)
    {
        if(IS_SET(current_case, CASE_WALL_E, MAZE_CASE_STATE_MASK))
        {
            pCtx->maze_array[pCtx->current_x + 1][pCtx->current_y] |= CASE_WALL_W;
        }
    }
}// end of upadte_connex_case

// check if the end pattern is reached
int is_it_the_end(maze_ctx_t *pCtx)
{
    int x, y;

    // End case pattern is a free 2*2. Depending on the current position, 4 possible
    // case pattern may be the end
    x = pCtx->current_x;
    y = pCtx->current_y;

    // Upper W 2x2 pattern
    if( ((x-1)>=0) && ((y+1)<MAX_MAZE_DEPTH))
    {
        // Check if all the 2*2 case has been visited
        if(IS_SET(pCtx->maze_array[x][y],     CASE_VISITED, MAZE_CASE_STATE_MASK) &&
           IS_SET(pCtx->maze_array[x][y+1],   CASE_VISITED, MAZE_CASE_STATE_MASK) &&
           IS_SET(pCtx->maze_array[x-1][y],   CASE_VISITED, MAZE_CASE_STATE_MASK) &&
           IS_SET(pCtx->maze_array[x-1][y+1], CASE_VISITED, MAZE_CASE_STATE_MASK))
        {
            if(IS_NOT_SET(pCtx->maze_array[x][y],     CASE_WALL_W|CASE_WALL_N, MAZE_CASE_STATE_MASK) &&
               IS_NOT_SET(pCtx->maze_array[x][y+1],   CASE_WALL_W|CASE_WALL_S, MAZE_CASE_STATE_MASK) &&
               IS_NOT_SET(pCtx->maze_array[x-1][y],   CASE_WALL_E|CASE_WALL_N, MAZE_CASE_STATE_MASK) &&
               IS_NOT_SET(pCtx->maze_array[x-1][y+1], CASE_WALL_E|CASE_WALL_S, MAZE_CASE_STATE_MASK))
            {
                // End reached, quick exit
                pCtx->end_x = x;
                pCtx->end_y = y;
                return(1) ;
            }
        }
    }

    // Upper E 2x2 pattern
    if( ((x+1)<MAX_MAZE_DEPTH) && ((y+1)<MAX_MAZE_DEPTH))
    {
        // Check if all the 2*2 case has been visited
        if(IS_SET(pCtx->maze_array[x][y],     CASE_VISITED, MAZE_CASE_STATE_MASK) &&
           IS_SET(pCtx->maze_array[x][y+1],   CASE_VISITED, MAZE_CASE_STATE_MASK) &&
           IS_SET(pCtx->maze_array[x+1][y],   CASE_VISITED, MAZE_CASE_STATE_MASK) &&
           IS_SET(pCtx->maze_array[x+1][y+1], CASE_VISITED, MAZE_CASE_STATE_MASK))
        {
            if(IS_NOT_SET(pCtx->maze_array[x][y],     CASE_WALL_E|CASE_WALL_N, MAZE_CASE_STATE_MASK) &&
               IS_NOT_SET(pCtx->maze_array[x][y+1],   CASE_WALL_E|CASE_WALL_S, MAZE_CASE_STATE_MASK) &&
               IS_NOT_SET(pCtx->maze_array[x+1][y],   CASE_WALL_W|CASE_WALL_N, MAZE_CASE_STATE_MASK) &&
               IS_NOT_SET(pCtx->maze_array[x+1][y+1], CASE_WALL_W|CASE_WALL_S, MAZE_CASE_STATE_MASK))
            {
                // End reached, quick exit
                pCtx->end_x = x;
                pCtx->end_y = y;
                return(1) ;
            }
        }
    }

    // Lower W 2x2 pattern
    if( ((x-1)>=0) && ((y-1)>=0))
    {
        // Check if all the 2*2 case has been visited
        if(IS_SET(pCtx->maze_array[x][y],     CASE_VISITED, MAZE_CASE_STATE_MASK) &&
           IS_SET(pCtx->maze_array[x][y-1],   CASE_VISITED, MAZE_CASE_STATE_MASK) &&
           IS_SET(pCtx->maze_array[x-1][y],   CASE_VISITED, MAZE_CASE_STATE_MASK) &&
           IS_SET(pCtx->maze_array[x-1][y-1], CASE_VISITED, MAZE_CASE_STATE_MASK))
        {
            if(IS_NOT_SET(pCtx->maze_array[x][y],     CASE_WALL_W|CASE_WALL_S, MAZE_CASE_STATE_MASK) &&
               IS_NOT_SET(pCtx->maze_array[x][y-1],   CASE_WALL_W|CASE_WALL_N, MAZE_CASE_STATE_MASK) &&
               IS_NOT_SET(pCtx->maze_array[x-1][y],   CASE_WALL_E|CASE_WALL_S, MAZE_CASE_STATE_MASK) &&
               IS_NOT_SET(pCtx->maze_array[x-1][y-1], CASE_WALL_E|CASE_WALL_N, MAZE_CASE_STATE_MASK))
            {
                // End reached, quick exit
                pCtx->end_x = x;
                pCtx->end_y = y;
                return(1) ;
            }
        }
    }

    // Lower E 2x2 pattern
    if( ((x+1)<MAX_MAZE_DEPTH) && ((y-1)>=0))
    {
        // Check if all the 2*2 case has been visited
        if(IS_SET(pCtx->maze_array[x][y],     CASE_VISITED, MAZE_CASE_STATE_MASK) &&
           IS_SET(pCtx->maze_array[x][y-1],   CASE_VISITED, MAZE_CASE_STATE_MASK) &&
           IS_SET(pCtx->maze_array[x+1][y],   CASE_VISITED, MAZE_CASE_STATE_MASK) &&
           IS_SET(pCtx->maze_array[x+1][y-1], CASE_VISITED, MAZE_CASE_STATE_MASK))
        {
            if(IS_NOT_SET(pCtx->maze_array[x][y],     CASE_WALL_E|CASE_WALL_S, MAZE_CASE_STATE_MASK) &&
               IS_NOT_SET(pCtx->maze_array[x][y-1],   CASE_WALL_E|CASE_WALL_N, MAZE_CASE_STATE_MASK) &&
               IS_NOT_SET(pCtx->maze_array[x+1][y],   CASE_WALL_W|CASE_WALL_S, MAZE_CASE_STATE_MASK) &&
               IS_NOT_SET(pCtx->maze_array[x+1][y-1], CASE_WALL_W|CASE_WALL_N, MAZE_CASE_STATE_MASK))
            {
                // End reached, quick exit
                pCtx->end_x = x;
                pCtx->end_y = y;
                return(1) ;
            }
        }
    }

    // End not found...
    return(0) ;
}// end of is_it_the_end

// Get the wall state and build the wall map
maze_case_t get_wall_state(robot_direction_t current_direction)
{
	maze_case_t walls ;

	walls = 0;

	if(wall_sensor_wall_left_presence())
	{
		switch(current_direction)
		{
		case DIR_N:
			walls |=  CASE_WALL_W ;
			break;
		case DIR_E:
			walls |=  CASE_WALL_N ;
			break;
		case DIR_S:
			walls |=  CASE_WALL_E ;
			break;
		case DIR_W:
			walls |=  CASE_WALL_S ;
			break;
		}
	}

	if(wall_sensor_wall_right_presence())
	{
		switch(current_direction)
		{
		case DIR_N:
			walls |=  CASE_WALL_E ;
			break;
		case DIR_E:
			walls |=  CASE_WALL_S ;
			break;
		case DIR_S:
			walls |=  CASE_WALL_W ;
			break;
		case DIR_W:
			walls |=  CASE_WALL_N ;
			break;
		}
	}

	if(wall_sensor_left_front_presence())
	{
		switch(current_direction)
		{
		case DIR_N:
			walls |=  CASE_WALL_N ;
			break;
		case DIR_E:
			walls |=  CASE_WALL_E ;
			break;
		case DIR_S:
			walls |=  CASE_WALL_S ;
			break;
		case DIR_W:
			walls |=  CASE_WALL_W ;
			break;
		}
	}

	return(walls);
}// end of get_wall_state

// Upadte the maze context depending on the action and the current direction
action_t update_maze_ctx(maze_ctx_t *pCtx)
{
    int32_t            step_x;
    int32_t            step_y;
    robot_direction_t  new_direction = (robot_direction_t) -1 ;
    action_t           action;
    int                the_end;
    algo_update_t     *pAlgo = NULL;
    maze_case_t        newState;
    maze_case_t        wall_sensor;

    // Get wall state
    wall_sensor =  get_wall_state(pCtx->current_direction);

    HAL_Serial_Print(&com, "[mode:%s] >> [from:(%d,%d) dir:%s walls:%s] >> [sensor:%s]",
    				 (char *)maze_mode_txt[pCtx->mode],
					 pCtx->current_x, pCtx->current_y, robot_direction_to_txt(pCtx->current_direction),
                     (char *)wall_state_txt[GET_WALL_STATE(pCtx->maze_array[pCtx->current_x][pCtx->current_y])],
                     (char *)wall_state_txt[WALL_STATE_MASK & wall_sensor]);

    // Check if we reach the end pattern
    the_end = is_it_the_end(pCtx) ;
    if(the_end)
    {
        action = ACTION_STOP ;
        pCtx->maze_array[pCtx->current_x][pCtx->current_y] |= CASE_END ;
    }
    else
    {
        // Get the next action depending on the sensor information and the left/right hand
        action = get_next_action(pCtx, wall_sensor, LEFT_HAND);
    }

    HAL_Serial_Print(&com, " >> [action:%s]", action2txt(action));

    // Default: don't move
    step_x        = 0;
    step_y        = 0;
    new_direction = pCtx->current_direction;

    // Get the new position/direction depending on the action to perform
    switch(action)
    {
        default:
        case ACTION_START:
        case ACTION_IDLE:
        case ACTION_STOP:
            break;
        case ACTION_RUN_1:
            pAlgo = (algo_update_t *) algo_update_maze_ctx_RUN_1;
            break;
        case ACTION_TURN_RIGHT:
            pAlgo = (algo_update_t *) algo_update_maze_ctx_TURN_RIGHT;
            break;
        case ACTION_TURN_LEFT:
            pAlgo = (algo_update_t *) algo_update_maze_ctx_TURN_LEFT;
            break;
        case ACTION_U_TURN_RIGHT:
            pAlgo = (algo_update_t *) algo_update_maze_ctx_U_TURN_RIGHT;
            break;
    }

    // According to the action, change the position and direction
    if(NULL!=pAlgo)
    {
        step_x        = pAlgo[pCtx->current_direction].x ;
        step_y        = pAlgo[pCtx->current_direction].y ;
        new_direction = pAlgo[pCtx->current_direction].direction;
    }

    // Apply modification
    pCtx->current_x         += step_x;
    pCtx->current_y         += step_y;
    pCtx->current_direction  = new_direction;

    // Update case with walls position, visited flag and case number
    newState = wall_sensor | CASE_VISITED | ((step++) << MAZE_CASE_NUMBER_OFFSET) ;
    pCtx->maze_array[pCtx->current_x][pCtx->current_y] |= newState;

    // Populate the case state for the connex cases
    upadte_connex_case(pCtx);

    // Check if the end is reached
    the_end = is_it_the_end(pCtx) ;
    if(the_end)
    {
        action = ACTION_STOP ;
        pCtx->maze_array[pCtx->current_x][pCtx->current_y] |= CASE_END ;
    }

    HAL_Serial_Print(&com, " >> [to:(%d,%d) dir:%s walls:[%s]] >> [end:%d]\n",
    				 pCtx->current_x, pCtx->current_y, robot_direction_to_txt(pCtx->current_direction),
					 (char *)wall_state_txt[GET_WALL_STATE(pCtx->maze_array[pCtx->current_x][pCtx->current_y])],
					 the_end);

    return(action);
}// end of update_maze_ctx

// Display the maze context
void display_maze_ctx(maze_ctx_t *pCtx)
{
#if 0
    int x;
    int y;
#endif
    HAL_Serial_Print(&com, "\n[mode:%s] start:(%d,%d) / end:(%d,%d)=> current:(%d,%d) direction:%s walls:%s\n",
           (char *)maze_mode_txt[pCtx->mode],
           pCtx->start_x, pCtx->start_y,
           pCtx->end_x, pCtx->end_y,
           pCtx->current_x, pCtx->current_y,
           robot_direction_to_txt(pCtx->current_direction),
           (char *)wall_state_txt[GET_WALL_STATE(pCtx->maze_array[pCtx->current_x][pCtx->current_y])]);

#if 0
    printf("\n");
    for(y=(MAX_MAZE_DEPTH-1); y>=0; y--)
    {
        printf("\t");
        for(x=0; x<MAX_MAZE_DEPTH; x++)
        {
            // case state
            printf("%3d ", GET_WALL_STATE(pCtx->maze_array[x][y])) ;
        }
        printf("\t");
        for(x=0; x<MAX_MAZE_DEPTH; x++)
        {
            // case number
            printf("%3d ", GET_CASE_NUMBER(pCtx->maze_array[x][y])) ;
        }
        printf("\n");
    }
    printf("\n");
#endif
}// end of display_maze_ctx

// end of file maze.c
