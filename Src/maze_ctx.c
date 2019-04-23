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

const char* action_txt[8] =
{
    "IDLE        ",
    "START       ",
    "RUN_1       ",
    "TURN_RIGHT  ",
    "TURN_LEFT   ",
    "U_TURN_RIGHT",
    "STOP        ",
    "CTR         "
};

const char *direction_txt[4] =
{
    "N",
    "S",
    "E",
    "W"
};


// Fill the array maze with case unknown pattern
void maze_ctx_start(maze_ctx_t *pCtx)
{
	// Restart from the beginning
	pCtx->current_direction = DIR_N;
	pCtx->current_x = pCtx->start_x ;
	pCtx->current_y = pCtx->start_y ;

	HAL_Serial_Print(&com, "[maze_ctx_start: mode:%s (%d,%d)]\n", maze_mode_txt[pCtx->mode], pCtx->current_x, pCtx->current_y);
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

    // Init action list
    init_action_array(pCtx);

    // Init intersection list
    init_inter_array(pCtx);

	// Start in LEARN mode
	pCtx->mode = LEARN;

	pCtx->current_direction = DIR_N;

	pCtx->start_x = (MAX_MAZE_DEPTH/2)-1;
	pCtx->start_y = (MAX_MAZE_DEPTH/2)-1;

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

	// Default algo is left hand
	pCtx->algo = LEFT_HAND;

	upadte_connex_case(pCtx);

}// end of maze_ctx_init

void init_action_array(maze_ctx_t *pCtx)
{
    int x;

    // Action list
    pCtx->nb_action = 0;
    pCtx->current_action_index = -1;
    for(x=0; x<MAX_ACTION; x++)
    {
        pCtx->action_array[x].action = ACTION_STOP;
        pCtx->action_array[x].x = -1;
        pCtx->action_array[x].y = -1;
        pCtx->action_array[x].dir = -1;
    }
}// end of init_action_array

void init_inter_array(maze_ctx_t *pCtx)
{
    int x;

    // Inter list
    pCtx->nb_inter = 0;
    for(x=0; x<MAX_INTER; x++)
    {
        pCtx->inter_array[x].enable = 0;
        pCtx->inter_array[x].x = -1;
        pCtx->inter_array[x].y = -1;
    }
}// end of init_inter_array

void display_action_list(maze_ctx_t *pCtx)
{
    int x;

    // Action list
    HAL_Serial_Print(&com, "[nb_action:%d current_index:%d]\n",
           pCtx->nb_action, pCtx->current_action_index);
    HAL_Delay(10);
    for(x=0; x < pCtx->nb_action; x++)
    {
    	HAL_Serial_Print(&com, "\taction[%d]:%s (%d,%d) %s\n",
               x,
               (char*)action_txt[pCtx->action_array[x].action],
               pCtx->action_array[x].x,
               pCtx->action_array[x].y,
               (char*)direction_txt[pCtx->action_array[x].dir]);
    	HAL_Delay(10);
    }
}// end of display_action_array

action_t get_next_next_action(maze_ctx_t *pCtx)
{

	if(pCtx->mode == LEARN)
	{
		return(ACTION_IDLE);
	}
	else
	{
		// SOLVE mode: check if we do not exceed the action array
		if(pCtx->current_action_index < pCtx->nb_action)
		{
			return(pCtx->action_array[pCtx->current_action_index].action);
		}
		else
		{
			return(ACTION_IDLE);
		}
	}
}// end of get_next_next_action

// Return the next action, depending on the wall sensor : left hand first choise
action_t get_next_action(maze_ctx_t *pCtx, maze_case_t wall_sensor)
{
	action_t                next_action;
	algo_next_action_ctx_t *pAlgo;
	robot_direction_t       dir;

	if(pCtx->algo == LEFT_HAND)
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

// Add intersection in list
void add_inter(maze_ctx_t *pCtx, int x, int y)
{
    int i;
    int to_add = 1;

    // Already present ?
    for (i=0;i<MAX_INTER;i++)
    {
        if(pCtx->inter_array[i].enable == 1)
        {
            if((pCtx->inter_array[i].x == x) && (pCtx->inter_array[i].y == y))
            {
                to_add = 0;
            }
        }
    }

    if(to_add == 1)
    {
        for (i=0;i<MAX_INTER;i++)
        {
            if(pCtx->inter_array[i].enable == 0)
            {
                pCtx->inter_array[i].x = x;
                pCtx->inter_array[i].y = y;
                pCtx->inter_array[i].enable = 1;

                if(pCtx->nb_inter<MAX_INTER)
                    pCtx->nb_inter++;
                else
                	HAL_Serial_Print(&com,"###BIG MESS### in line: %d\n", __LINE__);
                break;
            }
        }
    }
}

// check depending on the position if a new intresection is present
void upadte_inter(maze_ctx_t *pCtx)
{
    int x,y;

    x = pCtx->current_x;
    y = pCtx->current_y;

    switch(pCtx->current_direction)
    {
        case DIR_N:
        case DIR_S:
            if(IS_NOT_SET(pCtx->maze_array[x][y], CASE_WALL_W, MAZE_CASE_STATE_MASK) &&
               IS_NOT_SET(pCtx->maze_array[x-1][y], CASE_VISITED, MAZE_CASE_STATE_MASK))
            {
                add_inter(pCtx, x-1, y);
            }
            if(IS_NOT_SET(pCtx->maze_array[x][y], CASE_WALL_E, MAZE_CASE_STATE_MASK) &&
               IS_NOT_SET(pCtx->maze_array[x+1][y], CASE_VISITED, MAZE_CASE_STATE_MASK))
            {
                add_inter(pCtx, x+1, y);
            }
            break;
            break;
        case DIR_E:
        case DIR_W:
            if(IS_NOT_SET(pCtx->maze_array[x][y], CASE_WALL_N, MAZE_CASE_STATE_MASK) &&
               IS_NOT_SET(pCtx->maze_array[x][y+1], CASE_VISITED, MAZE_CASE_STATE_MASK))
            {
                add_inter(pCtx, x, y+1);
            }
            if(IS_NOT_SET(pCtx->maze_array[x][y], CASE_WALL_S, MAZE_CASE_STATE_MASK) &&
               IS_NOT_SET(pCtx->maze_array[x][y-1], CASE_VISITED, MAZE_CASE_STATE_MASK))
            {
                add_inter(pCtx, x, y-1);
            }
            break;
    }
}

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

	// Default: don't move
	step_x        = 0;
	step_y        = 0;
	new_direction = pCtx->current_direction;
	action        = ACTION_STOP;

	// Get wall state
	wall_sensor =  get_wall_state(pCtx->current_direction);

	HAL_Serial_Print(&com, "[m:%s] >> [f:(%d,%d) d:%s w:%s] >> [s:%s]",
			(char *)maze_mode_txt[pCtx->mode],
			pCtx->current_x, pCtx->current_y,
			(char *)direction_txt[pCtx->current_direction],
			(char *)wall_state_txt[GET_WALL_STATE(pCtx->maze_array[pCtx->current_x][pCtx->current_y])],
			(char *)wall_state_txt[MAZE_CASE_WALL_MASK & wall_sensor]);

	if(pCtx->mode == LEARN)
	{
		/////////////
		// LEARN mode
		/////////////

		// Get the next action depending on the sensor information and the left/right hand
		action = get_next_action(pCtx, wall_sensor);

		HAL_Serial_Print(&com, " >> [a:%s]", (char *)action_txt[action]);

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
		newState = wall_sensor | CASE_VISITED ;
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

			// Build the action list for the SOLVE mode
			build_action_list(pCtx, DIR_N, pCtx->start_x, pCtx->start_y, pCtx->end_x, pCtx->end_y);
		}

		HAL_Serial_Print(&com, " >> [t:(%d,%d) d:%s w:%s] >> [e:%d]\n",
				pCtx->current_x, pCtx->current_y,
				(char *) direction_txt[pCtx->current_direction],
				(char *) wall_state_txt[GET_WALL_STATE(pCtx->maze_array[pCtx->current_x][pCtx->current_y])],
				the_end);
	}
	else
	{
		/////////////
		// SOLVE mode
		/////////////
        if((pCtx->current_action_index == -1) || (pCtx->current_action_index >= pCtx->nb_action))
        {
            action = ACTION_STOP;
        }
        else
        {
            pCtx->current_x         = pCtx->action_array[pCtx->current_action_index].x;
            pCtx->current_y         = pCtx->action_array[pCtx->current_action_index].y;
            pCtx->current_direction = pCtx->action_array[pCtx->current_action_index].dir;
            action                  = pCtx->action_array[pCtx->current_action_index].action;
            pCtx->current_action_index++;
        }

		HAL_Serial_Print(&com, " >> [a:%s] >> [to:(%d,%d) dir:%s walls:[%s]]\n",
				action_txt[action],
				pCtx->current_x, pCtx->current_y,
				(char *)direction_txt[pCtx->current_direction],
				(char *)wall_state_txt[GET_WALL_STATE(pCtx->maze_array[pCtx->current_x][pCtx->current_y])]);
	}

	return(action);
}// end of update_maze_ctx

// Return 1 if the wall is not present depending of the case and the wall to check
int is_no_wall(maze_ctx_t *pCtx, int x, int y, maze_case_t wall)
{
	if( (wall & GET_WALL_STATE(pCtx->maze_array[x][y])) != wall )
		return 1;
	return 0;
}

// Return 1 if the case has been visited
int is_safe(maze_ctx_t *pCtx, int x, int y)
{
	if (( (CASE_VISITED & pCtx->maze_array[x][y]) == CASE_VISITED ) && (pCtx->solve_array[x][y]==0))
		return 1;
	return 0;
}

// Return 1 if the case is valid, in the usage domaine
int is_valid(int x, int y)
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

// Find the shortest path
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
	if (is_no_wall(pCtx, i, j, CASE_WALL_E) && is_valid(i + 1, j) && is_safe(pCtx, i + 1, j))
	{
		find_shortest_path(pCtx, i + 1, j, x, y, dist + 1);
	}
	// go to right cell
	if (is_no_wall(pCtx, i, j, CASE_WALL_N) && is_valid(i, j + 1) && is_safe(pCtx, i, j + 1))
	{
		find_shortest_path(pCtx, i, j + 1, x, y, dist + 1);
	}
	// go to top cell
	if (is_no_wall(pCtx, i, j, CASE_WALL_W) && is_valid(i - 1, j) && is_safe(pCtx, i - 1, j))
	{
		find_shortest_path(pCtx, i - 1, j, x, y, dist + 1);
	}
	// go to left cell
	if ( is_no_wall(pCtx, i, j, CASE_WALL_S) && is_valid(i, j - 1) && is_safe(pCtx, i, j - 1))
	{
		find_shortest_path(pCtx, i, j - 1, x, y, dist + 1);
	}

	// RewindRemove (i, j) from visited matrix
	pCtx->solve_array[i][j] = 0;
}

// Build action list from a case to another case, path must be the shortest
void build_action_list(maze_ctx_t *pCtx, robot_direction_t from_dir, int from_x, int from_y, int to_x, int to_y)
{
    int a;
    int x;
    int y;
    int current_number;
    int next_number;
    int step_x;
    int step_y;
    action_t action;
    robot_direction_t new_direction;

    init_action_array(pCtx);

    pCtx->min_dist = MAX_INT;
    find_shortest_path(pCtx,
                       from_x, from_y,
                       to_x, to_y,
                       0);

    if(pCtx->min_dist > MAX_ACTION)
    {
    	HAL_Serial_Print(&com,"###BIG MESS### in line: %d\n", __LINE__);
    }

    pCtx->nb_action = pCtx->min_dist ;

    pCtx->action_array[0].x   = from_x;
    pCtx->action_array[0].y   = from_y;
    pCtx->action_array[0].dir = from_dir;

    for(a=0;a<pCtx->nb_action;a++)
    {
    	// Default value
    	action = ACTION_STOP;
    	step_x        = 0;
    	step_y        = 0;
    	new_direction = -1;

        // Get current case
        x = pCtx->action_array[a].x;
        y = pCtx->action_array[a].y;
        current_number = pCtx->shortest_array[x][y] ;

        // Next case to find
        next_number = current_number + 1;

        if (is_valid(x + 1, y) && (next_number == pCtx->shortest_array[x+1][y]))
        {
            step_x = 1;
            step_y = 0;
            switch(pCtx->action_array[a].dir)
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
        else if (is_valid(x, y + 1) && (next_number == pCtx->shortest_array[x][y+1]))
        {
            step_x = 0;
            step_y = 1;
            switch(pCtx->action_array[a].dir)
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
        else if (is_valid(x - 1, y) && (next_number == pCtx->shortest_array[x-1][y]))
        {
            step_x = -1;
            step_y = 0;
            switch(pCtx->action_array[a].dir)
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
        else if (is_valid(x, y - 1) && (next_number == pCtx->shortest_array[x][y-1]))
        {
            step_x = 0;
            step_y = -1;
            switch(pCtx->action_array[a].dir)
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
        else
        {
        	HAL_Serial_Print(&com,"###BIG MESS### in line: %d\n", __LINE__);
        }

        pCtx->action_array[a].action = action;

        pCtx->action_array[a+1].x   = pCtx->action_array[a].x + step_x;
        pCtx->action_array[a+1].y   = pCtx->action_array[a].y + step_y;
        pCtx->action_array[a+1].dir = new_direction;
    }

    pCtx->current_action_index = 1;

    display_action_list(pCtx);
}// end of build_action_list

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

	display_action_list(pCtx);

	HAL_Serial_Print(&com, "Nb inter:%d\n", pCtx->nb_inter);
	HAL_Delay(10);
    for(yy=0;yy<MAX_INTER;yy++)
    {
        if(pCtx->inter_array[yy].enable != 0)
        {
        	HAL_Serial_Print(&com, "inter[%d]:(%d,%d)\n", yy, pCtx->inter_array[yy].x, pCtx->inter_array[yy].y);
        	HAL_Delay(10);
        }
    }

}// end of display_maze_ctx

// end of file maze.c
