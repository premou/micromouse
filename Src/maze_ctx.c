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

	HAL_Serial_Print(&com, "[maze_ctx_start:(%d,%d) mode:%s]\n", pCtx->current_x, pCtx->current_y, maze_mode_txt[pCtx->mode]);

	// In SOLVE mode, the first action is done by START action, so we start to the next case
	if(pCtx->mode == SOLVE)
	{
		pCtx->start_x   += 1;
		pCtx->current_y += 1;
	}

	HAL_Serial_Print(&com, "[maze_ctx_start:(%d,%d) mode:%s]\n", pCtx->current_x, pCtx->current_y, maze_mode_txt[pCtx->mode]);
}// end of maze_ctx_start

// Fill the array maze with case unknown pattern
void maze_ctx_init(maze_ctx_t *pCtx)
{
	uint32_t x, y;
	for(x=0; x<MAX_MAZE_DEPTH; x++)
	{
		for(y=0; y<MAX_MAZE_DEPTH; y++)
		{
			pCtx->maze_array[x][y]     = CASE_UNKNOWN;
			pCtx->solve_array[x][y]    = CASE_UNKNOWN;
			pCtx->shortest_array[x][y] = 0;
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
	if(pCtx->switch_led){
		HAL_GPIO_WritePin(((GPIO_TypeDef *) GPIOD_BASE),((uint16_t)0x0008U),GPIO_PIN_RESET); // droite ON
		HAL_GPIO_WritePin(((GPIO_TypeDef *) GPIOE_BASE),((uint16_t)0x0004U),GPIO_PIN_RESET); // gauche ON
		pCtx->switch_led=0;
	}
	else{
		HAL_GPIO_WritePin(((GPIO_TypeDef *) GPIOD_BASE),((uint16_t)0x0008U),GPIO_PIN_SET); // droite OFF
		HAL_GPIO_WritePin(((GPIO_TypeDef *) GPIOE_BASE),((uint16_t)0x0004U),GPIO_PIN_SET); // gauche OFF
		pCtx->switch_led=1;
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
	int                x;
	int                y;
	int                current_number;
	int                next_number;

	// Default: don't move
	step_x        = 0;
	step_y        = 0;
	new_direction = pCtx->current_direction;
	action        = ACTION_STOP;

	// Get wall state
	wall_sensor =  get_wall_state(pCtx->current_direction);

	HAL_Serial_Print(&com, "[mode:%s] >> [from:(%d,%d) dir:%s walls:%s] >> [sensor:%s]",
			(char *)maze_mode_txt[pCtx->mode],
			pCtx->current_x, pCtx->current_y, robot_direction_to_txt(pCtx->current_direction),
			(char *)wall_state_txt[GET_WALL_STATE(pCtx->maze_array[pCtx->current_x][pCtx->current_y])],
			(char *)wall_state_txt[WALL_STATE_MASK & wall_sensor]);

	if(pCtx->mode == LEARN)
	{
		/////////////
		// LEARN mode
		/////////////

		// Get the next action depending on the sensor information and the left/right hand
		action = get_next_action(pCtx, wall_sensor, LEFT_HAND);

		HAL_Serial_Print(&com, " >> [action:%s]", action2txt(action));

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

			// Change mode to SOLVE
			pCtx->mode = SOLVE;
		}

		HAL_Serial_Print(&com, " >> [to:(%d,%d) dir:%s walls:[%s]] >> [end:%d]\n",
				pCtx->current_x, pCtx->current_y, robot_direction_to_txt(pCtx->current_direction),
				(char *)wall_state_txt[GET_WALL_STATE(pCtx->maze_array[pCtx->current_x][pCtx->current_y])],
				the_end);
	}
	else
	{
		/////////////
		// SOLVE mode
		/////////////

		// Check if the end is reached, make sure the previous action has been completly finish
		the_end = is_it_the_end(pCtx) ;
		if(the_end)
		{
			action = ACTION_STOP ;
		}
		else
		{
			x = pCtx->current_x;
			y = pCtx->current_y;
			current_number = pCtx->shortest_array[x][y] ;
			next_number    = current_number + 1;

			if (isValid(x + 1, y) && (next_number == pCtx->shortest_array[x+1][y]))
			{
				step_x = 1;
				step_y = 0;
				switch(pCtx->current_direction)
				{
				case DIR_N:
					action = ACTION_TURN_RIGHT;
					new_direction = DIR_E;
					break;
				case DIR_E:
					action = ACTION_RUN_1;
					new_direction = DIR_E;
					break;
				case DIR_S:
					action = ACTION_TURN_LEFT;
					new_direction = DIR_E;
					break;
				case DIR_W:
					action = ACTION_U_TURN_RIGHT;
					new_direction = DIR_E;
					break;
				}
			}
			else if (isValid(x, y + 1) && (next_number == pCtx->shortest_array[x][y+1]))
			{
				step_x = 0;
				step_y = 1;
				switch(pCtx->current_direction)
				{
				case DIR_N:
					action = ACTION_RUN_1;
					new_direction = DIR_N;
					break;
				case DIR_E:
					action = ACTION_TURN_LEFT;
					new_direction = DIR_N;
					break;
				case DIR_S:
					action = ACTION_U_TURN_RIGHT;
					new_direction = DIR_N;
					break;
				case DIR_W:
					action = ACTION_TURN_RIGHT;
					new_direction = DIR_N;
					break;
				}
			}
			else if (isValid(x - 1, y) && (next_number == pCtx->shortest_array[x-1][y]))
			{
				step_x = -1;
				step_y = 0;
				switch(pCtx->current_direction)
				{
				case DIR_N:
					action = ACTION_TURN_LEFT;
					new_direction = DIR_W;
					break;
				case DIR_E:
					action = ACTION_U_TURN_RIGHT;
					new_direction = DIR_W;
					break;
				case DIR_S:
					action = ACTION_TURN_RIGHT;
					new_direction = DIR_W;
					break;
				case DIR_W:
					action = ACTION_RUN_1;
					new_direction = DIR_W;
					break;
				}
			}
			else if (isValid(x, y - 1) && (next_number == pCtx->shortest_array[x][y-1]))
			{
				step_x = 0;
				step_y = -1;
				switch(pCtx->current_direction)
				{
				case DIR_N:
					action = ACTION_U_TURN_RIGHT;
					new_direction = DIR_S;
					break;
				case DIR_E:
					action = ACTION_TURN_RIGHT;
					new_direction = DIR_S;
					break;
				case DIR_S:
					action = ACTION_RUN_1;
					new_direction = DIR_S;
					break;
				case DIR_W:
					action = ACTION_TURN_LEFT;
					new_direction = DIR_S;
					break;
				}
			}
		}

		HAL_Serial_Print(&com, " >> [action:%s]", action2txt(action));

		// Apply modification
		pCtx->current_x         += step_x;
		pCtx->current_y         += step_y;
		pCtx->current_direction  = new_direction;

		HAL_Serial_Print(&com, " >> [to:(%d,%d) dir:%s walls:[%s]] >> [end:%d]\n",
				pCtx->current_x, pCtx->current_y, robot_direction_to_txt(pCtx->current_direction),
				(char *)wall_state_txt[GET_WALL_STATE(pCtx->maze_array[pCtx->current_x][pCtx->current_y])],
				the_end);
	}

	return(action);
}// end of update_maze_ctx

int is_no_wall(maze_ctx_t *pCtx, int x, int y, maze_case_t wall)
{
	if( (wall & GET_WALL_STATE(pCtx->maze_array[x][y])) != wall )
		return 1;
	return 0;
}

int isSafe(maze_ctx_t *pCtx, int x, int y)
{
	if (( (CASE_VISITED & pCtx->maze_array[x][y]) == CASE_VISITED ) && (pCtx->solve_array[x][y]==0))
		return 1;
	return 0;
}
int isValid(int x, int y)
{
	if ( (x<MAX_MAZE_DEPTH) && (y<MAX_MAZE_DEPTH) && (x>=0) && (y>=0) )
	{
		return(1);
	}
	else
	{
		return(0);
	}
}
void find_shortest_path(maze_ctx_t *pCtx,
		int i, int j, // current position
		int x, int y, // exit
		int dist)
{
	int xx;
	int yy;

	// if destination is found, update min_dist
	if (i == x && j == y)
	{
		if(dist < pCtx->min_dist)
		{
			for(yy=(MAX_MAZE_DEPTH-1); yy>=0; yy--)
			{
				for(xx=0; xx<MAX_MAZE_DEPTH; xx++)
				{
					pCtx->shortest_array[xx][yy] = pCtx->solve_array[xx][yy];
				}
			}
			pCtx->min_dist = dist;
			pCtx->shortest_array[pCtx->end_x][pCtx->end_y] = pCtx->min_dist + 1 ;
		}
		return;
	}

	// set (i, j) cell as visited
	pCtx->solve_array[i][j] = dist + 1;

	// go to bottom cell
	if (is_no_wall(pCtx, i, j, CASE_WALL_E) && isValid(i + 1, j) && isSafe(pCtx, i + 1, j))
	{
		find_shortest_path(pCtx, i + 1, j, x, y, dist + 1);
	}
	// go to right cell
	if (is_no_wall(pCtx, i, j, CASE_WALL_N) && isValid(i, j + 1) && isSafe(pCtx, i, j + 1))
	{
		find_shortest_path(pCtx, i, j + 1, x, y, dist + 1);
	}
	// go to top cell
	if (is_no_wall(pCtx, i, j, CASE_WALL_W) && isValid(i - 1, j) && isSafe(pCtx, i - 1, j))
	{
		find_shortest_path(pCtx, i - 1, j, x, y, dist + 1);
	}
	// go to left cell
	if ( is_no_wall(pCtx, i, j, CASE_WALL_S) && isValid(i, j - 1) && isSafe(pCtx, i, j - 1))
	{
		find_shortest_path(pCtx, i, j - 1, x, y, dist + 1);
	}

	// RewindRemove (i, j) from visited matrix
	pCtx->solve_array[i][j] = 0;
}

// ugly patch
static int one_display_only = 0;

// Display the maze context
void display_maze_ctx(maze_ctx_t *pCtx)
{
	int xx, yy;

	if(one_display_only != 0)
		return;

	one_display_only = 1;

	for(yy=(MAX_MAZE_DEPTH-1); yy>=0; yy--)
	{
		HAL_Serial_Print(&com,"\t");
		HAL_Delay(10);
		for(xx=0; xx<MAX_MAZE_DEPTH; xx++)
		{
			// case state
			HAL_Serial_Print(&com,"%s ", wall_state_txt[GET_WALL_STATE(pCtx->maze_array[xx][yy])] ) ;
			HAL_Delay(10);
		}
		HAL_Serial_Print(&com,"\t");
		HAL_Delay(10);
		for(xx=0; xx<MAX_MAZE_DEPTH; xx++)
		{
			// case state
			HAL_Serial_Print(&com,"%d ",pCtx->shortest_array[xx][yy]) ;
			HAL_Delay(10);
		}
		HAL_Serial_Print(&com,"\n");
		HAL_Delay(10);
	}
	HAL_Serial_Print(&com,"\n");
	HAL_Delay(10);
}// end of display_maze_ctx

// end of file maze.c
