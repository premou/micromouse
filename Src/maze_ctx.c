#include "stdio.h"
#include "assert.h"

// For unitary test purpose
//#define _MAZE_STANDALONE

// Maze header
#include "controller.h"
#include "serial.h"
#include "WallSensor.h"
#include "maze.h"

// globals
extern HAL_Serial_Handler com;

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
        CASE_WALL_W,  0,  1, ACTION_TURN_LEFT,
        CASE_WALL_N,  0,  1, ACTION_RUN_1,
        CASE_WALL_E,  0,  1, ACTION_TURN_RIGHT,
        (CASE_WALL_N|CASE_WALL_W|CASE_WALL_E), ACTION_U_TURN_RIGHT
    },

    // DIR_S
    {
        CASE_WALL_E,  0, -1, ACTION_TURN_LEFT,
        CASE_WALL_S,  0, -1, ACTION_RUN_1,
        CASE_WALL_W,  0, -1, ACTION_TURN_RIGHT,
        (CASE_WALL_S|CASE_WALL_E|CASE_WALL_W), ACTION_U_TURN_RIGHT
    },

    // DIR_E
    {
        CASE_WALL_N,   1,  0, ACTION_TURN_LEFT,
        CASE_WALL_E,   1,  0, ACTION_RUN_1,
        CASE_WALL_S,   1,  0, ACTION_TURN_RIGHT,
        (CASE_WALL_E|CASE_WALL_N|CASE_WALL_S), ACTION_U_TURN_RIGHT
    },

    // DIR_W
    {
        CASE_WALL_S,  -1,  0, ACTION_TURN_LEFT,
        CASE_WALL_W,  -1,  0, ACTION_RUN_1,
        CASE_WALL_N,  -1,  0, ACTION_TURN_RIGHT,
        (CASE_WALL_S|CASE_WALL_W|CASE_WALL_N), ACTION_U_TURN_RIGHT
    }
};

// Right hand algo
const algo_next_action_ctx_t algo_next_action_right_hand[4] =
{
    // DIR_N
    {
        CASE_WALL_E,   1,  0, ACTION_TURN_RIGHT,
        CASE_WALL_N,   0,  1, ACTION_RUN_1,
        CASE_WALL_W,  -1,  0, ACTION_TURN_LEFT,
        (CASE_WALL_N|CASE_WALL_W|CASE_WALL_E), ACTION_U_TURN_RIGHT
    },

    // DIR_S
    {
        CASE_WALL_W,  -1,  0, ACTION_TURN_RIGHT,
        CASE_WALL_S,   0, -1, ACTION_RUN_1,
        CASE_WALL_E,   1,  0, ACTION_TURN_LEFT,
        (CASE_WALL_S|CASE_WALL_E|CASE_WALL_W), ACTION_U_TURN_RIGHT
    },

    // DIR_E
    {
        CASE_WALL_S,   0, -1, ACTION_TURN_RIGHT,
        CASE_WALL_E,   1,  0, ACTION_RUN_1,
        CASE_WALL_N,   0,  1, ACTION_TURN_LEFT,
        (CASE_WALL_E|CASE_WALL_N|CASE_WALL_S), ACTION_U_TURN_RIGHT
    },

    // DIR_W
    {
        CASE_WALL_N,   0,  1, ACTION_TURN_RIGHT,
        CASE_WALL_W,  -1,  0, ACTION_RUN_1,
        CASE_WALL_S,   0, -1, ACTION_TURN_LEFT,
        (CASE_WALL_S|CASE_WALL_W|CASE_WALL_N), ACTION_U_TURN_RIGHT
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
		"IDLE      ",
		"START     ",
		"RUN_1     ",
		"TURN_RIGHT",
		"TURN_LEFT ",
		"U_TURN    ",
		"STOP      ",
		"CTR       "
};

const char *direction_txt[4] =
{
		"N",
		"S",
		"E",
		"W"
};


// DISPLAY
//////////

void display_action_list(maze_ctx_t *pCtx)
{
    int x;

    // Action list
    HAL_Serial_Print(&com, "[nb_action:%d from(%d,%d) d:%s to(%d,%d) d:%s]\n",
           pCtx->nb_action,
           pCtx->current_x, pCtx->current_y, direction_txt[pCtx->current_direction],
           pCtx->action_array[pCtx->nb_action - 1].x, pCtx->action_array[pCtx->nb_action - 1].y,
           (char*)direction_txt[pCtx->action_array[pCtx->nb_action - 1].dir]);
    HAL_Delay(10);

    for(x=0; x < pCtx->nb_action ; x++)
    {
    	HAL_Serial_Print(&com, "#>\ta[%d]=%s (%d,%d) %s\n",
               x,
               (char*)action_txt[pCtx->action_array[x].action],
               pCtx->action_array[x].x,
               pCtx->action_array[x].y,
               (char*)direction_txt[pCtx->action_array[x].dir]);
    	HAL_Delay(10);
    }
    HAL_Serial_Print(&com, "\n");
    HAL_Delay(10);
}// end of display_action_array


// INIT and START
/////////////////

void init_action_array(maze_ctx_t *pCtx)
{
	int x;

	// Action list
	pCtx->nb_action = 0;
	pCtx->current_action_index = -1;
	for(x=0; x<MAX_ACTION; x++)
	{
		pCtx->action_array[x].action = ACTION_STOP;
		pCtx->action_array[x].x      = -1;
		pCtx->action_array[x].y      = -1;
		pCtx->action_array[x].dir    = -1;
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

void init_shortest_array(maze_ctx_t *pCtx)
{
    uint32_t x, y;
    for(x=0; x<MAX_MAZE_DEPTH; x++)
    {
        for(y=0; y<MAX_MAZE_DEPTH; y++)
        {
            pCtx->solve_array[x][y]    = CASE_UNKNOWN;
            pCtx->shortest_array[x][y] = 0;
        }
    }
}

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

// Fill the array maze with case unknown pattern
void maze_ctx_start(maze_ctx_t *pCtx)
{
	// Restart from the beginning
	pCtx->current_direction = DIR_N;
	pCtx->current_x = pCtx->start_x ;
	pCtx->current_y = pCtx->start_y ;

	HAL_Serial_Print(&com, "[maze_ctx_start: mode:%s (%d,%d)]\n", maze_mode_txt[pCtx->mode], pCtx->current_x, pCtx->current_y);
}// end of maze_ctx_start


// MISCELLANEOUS
////////////////

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


// Return the next action:
// - depending on the wall sensor
// - left or right hand choice
// - not yet visited case
action_t get_next_action(maze_ctx_t *pCtx, maze_case_t wall_sensor)
{
    algo_next_action_ctx_t *pAlgo;
    robot_direction_t       dir;
    int                     min_dist;
    int                     min_dist_index;
    int                     xx;
    int                     x, y;
    action_t                next_action;

    if(LEFT_HAND == pCtx->algo)
    {
        pAlgo = (algo_next_action_ctx_t *) algo_next_action_left_hand ;
    }
    else
    {
        pAlgo = (algo_next_action_ctx_t *) algo_next_action_right_hand ;
    }

    // Init
    dir         = pCtx->current_direction ;
    x           = pCtx->current_x ;
    y           = pCtx->current_y ;
    next_action = ACTION_STOP;

    // First test: check if left/right is wall free and not visited
    if( IS_NOT_SET(wall_sensor, pAlgo[dir].test_1_wall, MAZE_CASE_STATE_MASK) &&
       ((CASE_VISITED & pCtx->maze_array[x + pAlgo[dir].test_1_x][y + pAlgo[dir].test_1_y]) != CASE_VISITED) )
    {
        next_action = pAlgo[dir].test1_action;
    }
    else if( IS_NOT_SET(wall_sensor, pAlgo[dir].test_2_wall, MAZE_CASE_STATE_MASK) &&
            ((CASE_VISITED & pCtx->maze_array[x + pAlgo[dir].test_2_x][y + pAlgo[dir].test_2_y]) != CASE_VISITED) )
    {
        next_action = pAlgo[dir].test_2_action;
    }
    else if( IS_NOT_SET(wall_sensor, pAlgo[dir].test_3_wall, MAZE_CASE_STATE_MASK) &&
            ((CASE_VISITED & pCtx->maze_array[x + pAlgo[dir].test_3_x][y + pAlgo[dir].test_3_y]) != CASE_VISITED) )
    {
        next_action = pAlgo[dir].test_3_action;
    }
    else if( wall_sensor == pAlgo[dir].test_4_wall )
    {
        next_action = pAlgo[dir].test_4_action;
    }
    else
    {
        next_action = ACTION_STOP;
    }

    if( next_action == ACTION_STOP )
    {
        // No easy solution found, find the nearest intersection
        if(pCtx->nb_inter!=0)
        {
            min_dist = MAX_INT ;
            for(xx=0; xx<MAX_INTER; xx++)
            {
                if(pCtx->inter_array[xx].enable)
                {
                    pCtx->min_dist = MAX_INT;
                    init_shortest_array(pCtx);
                    find_shortest_path(pCtx,
                                       pCtx->current_x, pCtx->current_y,
                                       pCtx->inter_array[xx].x, pCtx->inter_array[xx].y,
                                       0,
                                       0); // do not build solve array
                    if(pCtx->min_dist < min_dist)
                    {
                        min_dist       = pCtx->min_dist ;
                        min_dist_index = xx ;
                    }
                }
            }
            build_action_list(pCtx,
                              pCtx->current_direction,
                              pCtx->current_x,
                              pCtx->current_y,
                              pCtx->inter_array[min_dist_index].x,
                              pCtx->inter_array[min_dist_index].y);

            HAL_Serial_Print(&com, ">[build action f(%d,%d) t(%d,%d)]>",
            		pCtx->current_x, pCtx->current_y,
					pCtx->inter_array[min_dist_index].x, pCtx->inter_array[min_dist_index].y);

            return(ACTION_IDLE);
        }
        else
        {
            // No solution ?
            next_action = ACTION_STOP;
            HAL_Serial_Print(&com, "###> BIG MESS: %d\n", __LINE__);
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

				// Update end pattern
				pCtx->maze_array[x][y]     |= CASE_END;
				pCtx->maze_array[x][y+1]   |= CASE_END;
				pCtx->maze_array[x-1][y]   |= CASE_END;
				pCtx->maze_array[x-1][y+1] |= CASE_END;
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

				// Update end pattern
				pCtx->maze_array[x][y]     |= CASE_END;
				pCtx->maze_array[x][y+1]   |= CASE_END;
				pCtx->maze_array[x+1][y]   |= CASE_END;
				pCtx->maze_array[x+1][y+1] |= CASE_END;
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

				// Update end pattern
				pCtx->maze_array[x][y]     |= CASE_END;
				pCtx->maze_array[x][y-1]   |= CASE_END;
				pCtx->maze_array[x-1][y]   |= CASE_END;
				pCtx->maze_array[x-1][y-1] |= CASE_END;
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

				// Update end pattern
				pCtx->maze_array[x][y]     |= CASE_END;
				pCtx->maze_array[x][y-1]   |= CASE_END;
				pCtx->maze_array[x+1][y]   |= CASE_END;
				pCtx->maze_array[x+1][y-1] |= CASE_END;
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

	if(wall_sensor_is_left_wall_detected())
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

	if(wall_sensor_is_right_wall_detected())
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

	if(wall_sensor_is_front_wall_detected())
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
void add_intersection(maze_ctx_t *pCtx, int x, int y)
{
	int i;

	// Already present ?
	for (i=0;i<MAX_INTER;i++)
	{
		if((pCtx->inter_array[i].enable == 1) &&
				(pCtx->inter_array[i].x == x) && (pCtx->inter_array[i].y == y))
		{
			// Exit now
			return;
		}
	}

	if(pCtx->nb_inter < MAX_INTER)
	{
		for (i=0;i<MAX_INTER;i++)
		{
			if(pCtx->inter_array[i].enable == 0)
			{
				pCtx->inter_array[i].x = x;
				pCtx->inter_array[i].y = y;
				pCtx->inter_array[i].enable = 1;
				pCtx->nb_inter++;
				// Exit now
				return;
			}
		}
	}
}

// check depending on the position if a new intresection is present
void update_intersection(maze_ctx_t *pCtx)
{
	int x,y;
	int i;

	x = pCtx->current_x;
	y = pCtx->current_y;

	// Check if the intersection have been visited
	for (i=0;i<MAX_INTER;i++)
	{
		if( (pCtx->inter_array[i].enable == 1) &&
				((pCtx->maze_array[pCtx->inter_array[i].x][pCtx->inter_array[i].y] & CASE_VISITED) == CASE_VISITED))
		{
			pCtx->inter_array[i].enable = 0;
			pCtx->nb_inter--;
		}
	}

	switch(pCtx->current_direction)
	{
	case DIR_N:
	case DIR_S:
		if(IS_NOT_SET(pCtx->maze_array[x][y], CASE_WALL_W, MAZE_CASE_STATE_MASK) &&
				IS_NOT_SET(pCtx->maze_array[x-1][y], CASE_VISITED, MAZE_CASE_STATE_MASK))
		{
			add_intersection(pCtx, x-1, y);
			pCtx->maze_array[x-1][y] |= CASE_TO_VISIT;
		}
		if(IS_NOT_SET(pCtx->maze_array[x][y], CASE_WALL_E, MAZE_CASE_STATE_MASK) &&
				IS_NOT_SET(pCtx->maze_array[x+1][y], CASE_VISITED, MAZE_CASE_STATE_MASK))
		{
			add_intersection(pCtx, x+1, y);
			pCtx->maze_array[x+1][y] |= CASE_TO_VISIT;
		}
		if(IS_NOT_SET(pCtx->maze_array[x][y], CASE_WALL_N, MAZE_CASE_STATE_MASK) &&
				IS_NOT_SET(pCtx->maze_array[x][y+1], CASE_VISITED, MAZE_CASE_STATE_MASK))
		{
			add_intersection(pCtx, x, y+1);
			pCtx->maze_array[x][y+1] |= CASE_TO_VISIT;
		}
		if(IS_NOT_SET(pCtx->maze_array[x][y], CASE_WALL_S, MAZE_CASE_STATE_MASK) &&
				IS_NOT_SET(pCtx->maze_array[x][y-1], CASE_VISITED, MAZE_CASE_STATE_MASK))
		{
			add_intersection(pCtx, x, y-1);
			pCtx->maze_array[x][y-1] |= CASE_TO_VISIT;
		}
		break;
	case DIR_E:
	case DIR_W:
		if(IS_NOT_SET(pCtx->maze_array[x][y], CASE_WALL_N, MAZE_CASE_STATE_MASK) &&
				IS_NOT_SET(pCtx->maze_array[x][y+1], CASE_VISITED, MAZE_CASE_STATE_MASK))
		{
			add_intersection(pCtx, x, y+1);
			pCtx->maze_array[x][y+1] |= CASE_TO_VISIT;
		}
		if(IS_NOT_SET(pCtx->maze_array[x][y], CASE_WALL_S, MAZE_CASE_STATE_MASK) &&
				IS_NOT_SET(pCtx->maze_array[x][y-1], CASE_VISITED, MAZE_CASE_STATE_MASK))
		{
			add_intersection(pCtx, x, y-1);
			pCtx->maze_array[x][y-1] |= CASE_TO_VISIT;
		}
		if(IS_NOT_SET(pCtx->maze_array[x][y], CASE_WALL_E, MAZE_CASE_STATE_MASK) &&
				IS_NOT_SET(pCtx->maze_array[x+1][y], CASE_VISITED, MAZE_CASE_STATE_MASK))
		{
			add_intersection(pCtx, x+1, y);
			pCtx->maze_array[x+1][y] |= CASE_TO_VISIT;
		}
		if(IS_NOT_SET(pCtx->maze_array[x][y], CASE_WALL_W, MAZE_CASE_STATE_MASK) &&
				IS_NOT_SET(pCtx->maze_array[x-1][y], CASE_VISITED, MAZE_CASE_STATE_MASK))
		{
			add_intersection(pCtx, x-1, y);
			pCtx->maze_array[x-1][y] |= CASE_TO_VISIT;
		}
		break;
	}
}

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
    if (( ((CASE_VISITED  & pCtx->maze_array[x][y]) == CASE_VISITED) ||
         ((CASE_TO_VISIT & pCtx->maze_array[x][y]) == CASE_TO_VISIT) ) && (pCtx->solve_array[x][y]==0))
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
                        int dist,
                        int build_array)
{
    int xx;
    int yy;

    // if destination is found, update min_dist
    if (i == x && j == y)
    {
        if(build_array == 1)
        {
            for(yy=(MAX_MAZE_DEPTH-1); yy>=0; yy--)
            {
                for(xx=0; xx<MAX_MAZE_DEPTH; xx++)
                {
                    pCtx->shortest_array[xx][yy] = pCtx->solve_array[xx][yy];
                }
            }
        }

        if(dist < pCtx->min_dist)
        {
            pCtx->min_dist = dist ;
            pCtx->shortest_array[x][y] = pCtx->min_dist + 1 ;
        }
        return;
    }

    // set (i, j) cell as visited
    pCtx->solve_array[i][j] = dist + 1;

    // go to bottom cell
    if (is_no_wall(pCtx, i, j, CASE_WALL_E) && is_valid(i + 1, j) && is_safe(pCtx, i + 1, j))
    {
        find_shortest_path(pCtx, i + 1, j, x, y, dist + 1, build_array);
    }
    // go to right cell
    if (is_no_wall(pCtx, i, j, CASE_WALL_N) && is_valid(i, j + 1) && is_safe(pCtx, i, j + 1))
    {
        find_shortest_path(pCtx, i, j + 1, x, y, dist + 1, build_array);
    }
    // go to top cell
    if (is_no_wall(pCtx, i, j, CASE_WALL_W) && is_valid(i - 1, j) && is_safe(pCtx, i - 1, j))
    {
        find_shortest_path(pCtx, i - 1, j, x, y, dist + 1, build_array);
    }
    // go to left cell
    if ( is_no_wall(pCtx, i, j, CASE_WALL_S) && is_valid(i, j - 1) && is_safe(pCtx, i, j - 1))
    {
        find_shortest_path(pCtx, i, j - 1, x, y, dist + 1, build_array);
    }

    // RewindRemove (i, j) from visited matrix
    pCtx->solve_array[i][j] = 0;
}

// Build action list from a case to another case, path must be the shortest
void build_action_list(maze_ctx_t *pCtx, robot_direction_t from_dir, int from_x, int from_y, int to_x, int to_y)
{
    int               a, x, y       ;
    int               id_current    ;
    int               id_next       ;
    int               id_next_next  ;
    int               next_next_flag;
    robot_direction_t dir           ;
    robot_direction_t next_dir      ;
    robot_direction_t next_next_dir ;
    int               step_x, step_y;
    action_t          next_action   ;

    // Clean the place
    init_action_array(pCtx);
    init_shortest_array(pCtx);

    // Find the nearest case
    pCtx->min_dist = MAX_INT;
    find_shortest_path(pCtx,
                       from_x, from_y,
                       to_x, to_y,
                       0,
                       1); // build solve array
    if(pCtx->min_dist > MAX_ACTION)
    {
    	HAL_Serial_Print(&com, "###> BIG MESS: %d\n", __LINE__);
    }

    // Set the number of action
    pCtx->nb_action            = pCtx->min_dist ;
    pCtx->current_action_index = 0 ;

    // Init the firt case position
    x           = from_x;
    y           = from_y;
    dir         = from_dir;

    // Warning erraduication
    next_action = ACTION_STOP;
    step_x      = 0;
    step_y      = 0;
    next_dir    = DIR_N;

    // Build the action to follow the shortest path
    for ( a=0 ; a<pCtx->nb_action ; a++)
    {
        // Get current, next and next next id
        id_current   = pCtx->shortest_array[x][y] ;
        id_next      = id_current + 1 ;
        id_next_next = id_next + 1 ;

        ///////////////////
        // Find the next id
        ///////////////////
        if (is_valid(x + 1, y) && (id_next == pCtx->shortest_array[x+1][y]))
        {
            step_x   = 1;
            step_y   = 0;
            next_dir = DIR_E;
            switch(dir)
            {
                case DIR_N:
                    next_action = ACTION_TURN_RIGHT;
                    break;
                case DIR_S:
                    next_action = ACTION_TURN_LEFT;
                    break;
                case DIR_E:
                    next_action = ACTION_RUN_1;
                    break;
                case DIR_W:
                    next_action = ACTION_U_TURN_RIGHT;
                    break;
            }
        }
        else if (is_valid(x, y + 1) && (id_next == pCtx->shortest_array[x][y+1]))
        {
            step_x   = 0;
            step_y   = 1;
            next_dir = DIR_N;
            switch(dir)
            {
                case DIR_N:
                    next_action = ACTION_RUN_1;
                    break;
                case DIR_S:
                    next_action = ACTION_U_TURN_RIGHT;
                    break;
                case DIR_E:
                    next_action = ACTION_TURN_LEFT;
                    break;
                case DIR_W:
                    next_action = ACTION_TURN_RIGHT;
                    break;
            }
        }
        else if (is_valid(x - 1, y) && (id_next == pCtx->shortest_array[x-1][y]))
        {
            step_x   = -1;
            step_y   = 0;
            next_dir = DIR_W;
            switch(dir)
            {
                case DIR_N:
                    next_action = ACTION_TURN_LEFT;
                    break;
                case DIR_S:
                    next_action = ACTION_TURN_RIGHT;
                    break;
                case DIR_E:
                    next_action = ACTION_U_TURN_RIGHT;
                    break;
                case DIR_W:
                    next_action = ACTION_RUN_1;
                    break;
            }
        }
        else if (is_valid(x, y - 1) && (id_next == pCtx->shortest_array[x][y-1]))
        {
            step_x   = 0;
            step_y   = -1;
            next_dir = DIR_S;
            switch(dir)
            {
                case DIR_N:
                    next_action = ACTION_U_TURN_RIGHT;
                    break;
                case DIR_S:
                    next_action = ACTION_RUN_1;
                    break;
                case DIR_E:
                    next_action = ACTION_TURN_RIGHT;
                    break;
                case DIR_W:
                    next_action = ACTION_TURN_LEFT;
                    break;
            }
        }
        else
        {
        	HAL_Serial_Print(&com, "###> BIG MESS: %d\n", __LINE__);
        }

        ////////////////////////
        // Find the next next id
        ////////////////////////
        next_next_flag = 0;
        if (is_valid(x+step_x+1, y+step_y) && (id_next_next == pCtx->shortest_array[x+step_x+1][y+step_y]))
        {
            next_next_dir  = DIR_E;
            next_next_flag = 1 ;
        }
        else if (is_valid(x+step_x, y+step_y+1) && (id_next_next == pCtx->shortest_array[x+step_x][y+step_y+1]))
        {
            next_next_dir  = DIR_N;
            next_next_flag = 1 ;
        }
        else if (is_valid(x+step_x-1, y+step_y) && (id_next_next == pCtx->shortest_array[x+step_x-1][y+step_y]))
        {
            next_next_dir  = DIR_W;
            next_next_flag = 1 ;
        }
        else if (is_valid(x+step_x, y+step_y-1) && (id_next_next == pCtx->shortest_array[x+step_x][y+step_y-1]))
        {
            next_next_dir  = DIR_S;
            next_next_flag = 1 ;
        }

        ///////////////////////////////////////////////////////////////////////
        // Overwrite next_action and next_dir depending on the next next action
        ///////////////////////////////////////////////////////////////////////
        if(next_next_flag == 1)
        {
            switch(dir)
            {
                case DIR_N:
                    switch(next_next_dir)
                {
                    case DIR_N:
                        next_dir = dir;
                        next_action = ACTION_RUN_1;
                        break;
                    case DIR_S:
                    	HAL_Serial_Print(&com, "###> BIG MESS: %d\n", __LINE__);
                        break;
                    case DIR_E:
                        next_dir = DIR_E;
                        next_action = ACTION_TURN_RIGHT;
                        break;
                    case DIR_W:
                        next_dir = DIR_W;
                        next_action = ACTION_TURN_LEFT;
                        break;
                }
                    break;

                case DIR_S:
                    switch(next_next_dir)
                {
                    case DIR_N:
                    	HAL_Serial_Print(&com, "###> BIG MESS: %d\n", __LINE__);
                        break;
                    case DIR_S:
                        next_dir = dir;
                        next_action = ACTION_RUN_1;
                        break;
                    case DIR_E:
                        next_dir = DIR_E;
                        next_action = ACTION_TURN_LEFT;
                        break;
                    case DIR_W:
                        next_dir = DIR_W;
                        next_action = ACTION_TURN_RIGHT;
                        break;
                }
                    break;

                case DIR_E:
                    switch(next_next_dir)
                {
                    case DIR_N:
                        next_dir = DIR_N;
                        next_action = ACTION_TURN_LEFT;
                        break;
                    case DIR_S:
                        next_dir = DIR_S;
                        next_action = ACTION_TURN_RIGHT;
                        break;
                    case DIR_E:
                        next_dir = dir;
                        next_action = ACTION_RUN_1;
                        break;
                    case DIR_W:
                    	HAL_Serial_Print(&com, "###> BIG MESS: %d\n", __LINE__);
                        break;
                }
                    break;

                case DIR_W:
                    switch(next_next_dir)
                {
                    case DIR_N:
                        next_dir = DIR_N;
                        next_action = ACTION_TURN_RIGHT;
                        break;
                    case DIR_S:
                        next_dir = DIR_S;
                        next_action = ACTION_TURN_LEFT;
                        break;
                    case DIR_E:
                    	HAL_Serial_Print(&com, "###> BIG MESS: %d\n", __LINE__);
                        break;
                    case DIR_W:
                        next_dir = dir;
                        next_action = ACTION_RUN_1;
                        break;
                }
                    break;
            }
        }

        // Update the current action list
        pCtx->action_array[a].x      = x + step_x  ;
        pCtx->action_array[a].y      = y + step_y  ;
        pCtx->action_array[a].action = next_action ;
        pCtx->action_array[a].dir    = next_dir    ;

        // Next
        x   = pCtx->action_array[a].x;
        y   = pCtx->action_array[a].y;
        dir = next_dir ;

        // End reached ?
        if((x == to_x) && (y == to_y))
        {
            pCtx->action_array[a].dir = next_dir;
            break;
        }
    }

#if 0
    display_action_list(pCtx);
#endif

}// end of build_action_list

//HAL_Serial_Print(&com,

// Upadte the maze context depending on the action and the current direction
action_t update_maze_ctx(maze_ctx_t *pCtx)
{
    int32_t            step_x;
    int32_t            step_y;
    robot_direction_t  new_direction = (robot_direction_t) -1 ;
    action_t           action = ACTION_STOP;
    int                the_end;
    algo_update_t     *pAlgo = NULL;
    int                action_flag;
    maze_case_t        wall_sensor;

	// Get wall state
	wall_sensor =  get_wall_state(pCtx->current_direction);

	HAL_Serial_Print(&com, "[%s]>[f:(%d,%d) d:%s w:%s]>[s:%s]",
           (char *)maze_mode_txt[pCtx->mode],
           pCtx->current_x, pCtx->current_y,
           (char *)direction_txt[pCtx->current_direction],
           (char *)wall_state_txt[GET_WALL_STATE(pCtx->maze_array[pCtx->current_x][pCtx->current_y])],
           (char *)wall_state_txt[MAZE_CASE_WALL_MASK & wall_sensor]);

    // Fetch end state depending of the current position
    the_end = is_it_the_end(pCtx) ;

    /////////////
    // LEARN mode
    /////////////
    if(pCtx->mode == LEARN)
    {
        action_flag = 0;

        // End reached ?
        if(the_end)
        {
            pCtx->maze_array[pCtx->current_x][pCtx->current_y] |= CASE_END ;
            action = ACTION_STOP;
        }
        else
        {
            // Learn mode but action to perform in order to go to the nearest insersection
            if(pCtx->current_action_index != -1)
            {
            	HAL_Serial_Print(&com, ">[id action:%d/%d]>", pCtx->current_action_index, pCtx->nb_action -1);

                action_flag             = 1;

                pCtx->current_x         = pCtx->action_array[pCtx->current_action_index].x;
                pCtx->current_y         = pCtx->action_array[pCtx->current_action_index].y;
                pCtx->current_direction = pCtx->action_array[pCtx->current_action_index].dir;
                action                  = pCtx->action_array[pCtx->current_action_index].action;

                // Update case state
                pCtx->maze_array[pCtx->current_x][pCtx->current_y] |= CASE_VISITED;

                // Next action, and check if the action list is empty
                pCtx->current_action_index++;
                if(pCtx->current_action_index >= pCtx->nb_action)
                {
                    pCtx->current_action_index = -1 ;
                }
            }
            else
            {
                // Get the next action depending on the sensor information and the left/right hand
                action = get_next_action(pCtx, wall_sensor);
            }
        }

        // Apply action motion :

        // Default
        step_x        = 0;
        step_y        = 0;
        new_direction = pCtx->current_direction;

        // Get the new position/direction depending on the action to perform
        switch(action)
        {
            default:
            case ACTION_START:
            case ACTION_STOP:
                pAlgo = NULL;
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

            case ACTION_IDLE:
                // If action list has to be run, the function get_next_action returns ACTION_IDLE
                pAlgo = NULL;

                printf( ">[id action:%d/%d]>",pCtx->current_action_index, pCtx->nb_action -1);

                action_flag             = 1;

                pCtx->current_x         = pCtx->action_array[pCtx->current_action_index].x;
                pCtx->current_y         = pCtx->action_array[pCtx->current_action_index].y;
                pCtx->current_direction = pCtx->action_array[pCtx->current_action_index].dir;
                action                  = pCtx->action_array[pCtx->current_action_index].action;

                pCtx->maze_array[pCtx->current_x][pCtx->current_y] |= CASE_VISITED;

                // Next action and check if the action list is empty
                pCtx->current_action_index++;
                if(pCtx->current_action_index >= pCtx->nb_action)
                {
                    pCtx->current_action_index = -1 ;
                }
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
        if(1 != action_flag)
        {
            pCtx->current_x         += step_x;
            pCtx->current_y         += step_y;
            pCtx->current_direction  = new_direction;
            pCtx->maze_array[pCtx->current_x][pCtx->current_y] |= wall_sensor | CASE_VISITED;
        }

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
    }
    else
    {
        /////////////
        // SOLVE mode
        /////////////

        // End reached ?
        if( the_end || (pCtx->current_action_index == -1) )
        {
            action = ACTION_STOP ;
            pCtx->maze_array[pCtx->current_x][pCtx->current_y] |= CASE_END ;
        }
        else
        {
            printf( ">[id action:%d/%d]>",pCtx->current_action_index, pCtx->nb_action -1);

            pCtx->current_x         = pCtx->action_array[pCtx->current_action_index].x;
            pCtx->current_y         = pCtx->action_array[pCtx->current_action_index].y;
            pCtx->current_direction = pCtx->action_array[pCtx->current_action_index].dir;
            action                  = pCtx->action_array[pCtx->current_action_index].action;

            the_end = is_it_the_end(pCtx) ;
            if( the_end )
            {
            	action = ACTION_STOP ;
            	pCtx->maze_array[pCtx->current_x][pCtx->current_y] |= CASE_END ;
            }
            else
            {
            	// Next action and check if the action list is empty
            	pCtx->current_action_index++;
            	if(pCtx->current_action_index >= pCtx->nb_action)
            	{
            		pCtx->current_action_index = -1 ;
            	}
            }
        }
    }

    // Populate the case state for the connex cases
    upadte_connex_case(pCtx);

    // find case to check
    update_intersection(pCtx);

    HAL_Serial_Print(&com, " >> [a:%s] >> [t:(%d,%d) d:%s w:%s] >> [e:%d]\n",
           (char *)action_txt[action],
           pCtx->current_x, pCtx->current_y,
           (char *) direction_txt[pCtx->current_direction],
           (char *) wall_state_txt[GET_WALL_STATE(pCtx->maze_array[pCtx->current_x][pCtx->current_y])],
           the_end);

    return(action);
}// end of update_maze_ctx

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

	HAL_Serial_Print(&com, "Nb inter:%d ", pCtx->nb_inter);
	HAL_Delay(10);
	for(yy=0;yy<MAX_INTER;yy++)
	{
		if(pCtx->inter_array[yy].enable != 0)
		{
			HAL_Serial_Print(&com, "[%d](%d,%d) ", yy, pCtx->inter_array[yy].x, pCtx->inter_array[yy].y);
			HAL_Delay(10);
		}
	}
	HAL_Serial_Print(&com, "\n");
	HAL_Delay(10);

}// end of display_maze_ctx

// end of file maze.c
