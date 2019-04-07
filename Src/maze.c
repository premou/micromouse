#include <stdio.h>
#include <robot_math.h>
#include "controller.h"

#define DEFINE_VARIABLES
#include "maze.h"

#include "serial.h"
#include "datalogger.h"
#include <math.h>

// globals
extern HAL_Serial_Handler com;

// Return the printable arrow direction
char *robot_direction_to_txt(robot_direction_t dir)
{
    switch(dir)
    {
    default:
        return("?");
        break;
    case RIGHT:
        return("\u2192");
        break;
    case UP:
        return("\u2191");
        break;
    case LEFT:
        return("\u2190");
        break;
    case DOWN:
        return("\u2193");
        break;
    }
}// end of robot_direction_to_txt

// Fill the array maze with case unknown pattern
void maze_ctx_init(maze_ctx_t *pCtx)
{
	uint32_t x, y;
	for(x=0; x<MAX_MAZE_DEPTH; x++)
	{
		for(y=0; y<MAX_MAZE_DEPTH; y++)
		{
            pCtx->maze_array[x][y] = CASE_UNKNOWN;
        }
	}

	pCtx->current_direction = RIGHT;

    pCtx->start_x = 0;
    pCtx->start_y = (MAX_MAZE_DEPTH/2)-1;

    pCtx->end_x = 0;
    pCtx->end_y = 0;

    pCtx->current_x = pCtx->start_x;
    pCtx->current_y = pCtx->start_y;

    pCtx->maze_array[pCtx->current_x][pCtx->current_y] = CASE_START;

}// end of maze_ctx_init

// Return the next action, depending on the wall sensor
action_t get_next_action(maze_case_t wall_sensor)
{
    action_t next_action;

    // No wall on the left ?
    if( (wall_sensor & CASE_WALL_LEFT) != CASE_WALL_LEFT )
    {
        // Go left
        next_action = ACTION_TURN_LEFT;
    }
    else
    {
        // Wall present on the left, wall in front ?
        if( (wall_sensor & CASE_WALL_FRONT) != CASE_WALL_FRONT )
        {
            // Go straight away
            next_action = ACTION_RUN_1;
        }
        else
        {
            // Wall on the left and front, wall on the rigth ?
            if( (wall_sensor & CASE_WALL_RIGHT) != CASE_WALL_RIGHT )
            {
                // Go rigth
                next_action = ACTION_TURN_RIGHT;
            }
            else
            {
                // Wall left, front and rigth, turn around
                next_action = ACTION_U_TURN_RIGHT;
            }
        }
    }
    return(next_action);
}// end of get_next_action

// Upadte the maze context depending on the action and the current direction
void update_maze_ctx(maze_ctx_t *pCtx, maze_case_t wall_sensor)
{
	int32_t           step_x = 0;
	int32_t           step_y = 0;
	robot_direction_t new_direction = (robot_direction_t) -1 ;
    action_t          action;

    // Upadte the wall state
    pCtx->maze_array[pCtx->current_x][pCtx->current_y] |= wall_sensor;

    // get the next action
    action = get_next_action(wall_sensor);

	switch(action)
	{
	default:
	case ACTION_START:
	case ACTION_IDLE:
	case ACTION_STOP:
		// do nothing
		break;
	case ACTION_RUN_1:
		switch(pCtx->current_direction)
		{
		case RIGHT:
			step_x = 1;
			step_y = 0;
			// direction is not impacted
			break;
		case UP:
			step_x = 0;
			step_y = 1;
			// direction is not impacted
			break;
		case LEFT:
			step_x = -1;
			step_y = 0;
			// direction is not impacted
			break;
		case DOWN:
			step_x = 0;
			step_y = -1;
			// direction is not impacted
			break;
		}
		break;
	case ACTION_TURN_RIGHT:
		switch(pCtx->current_direction)
		{
		case RIGHT:
			step_x = 0;
			step_y = -1;
			new_direction = DOWN;
			break;
		case UP:
			step_x = 1;
			step_y = 0;
			new_direction = RIGHT;
			break;
		case LEFT:
			step_x = 0;
			step_y = 1;
			new_direction = UP;
			break;
		case DOWN:
			step_x = -1;
			step_y = 0;
			new_direction = LEFT;
			break;
		}
		break;
	case ACTION_TURN_LEFT:
		switch(pCtx->current_direction)
		{
		case RIGHT:
			step_x = 0;
			step_y = 1;
			new_direction = UP;
			break;
		case UP:
			step_x = -1;
			step_y = 0;
			new_direction = LEFT;
			break;
		case LEFT:
			step_x = 0;
			step_y = -1;
			new_direction = DOWN;
			break;
		case DOWN:
			step_x = -1;
			step_y = 0;
			new_direction = RIGHT;
			break;
		}
		break;
	case ACTION_U_TURN_RIGHT:
		switch(pCtx->current_direction)
		{
		case RIGHT:
			// x/y position is not impacted
			new_direction = LEFT;
			break;
		case UP:
			// x/y position is not impacted
			new_direction = DOWN;
			break;
		case LEFT:
			// x/y position is not impacted
			new_direction = RIGHT;
			break;
		case DOWN:
			// x/y position is not impacted
			new_direction = UP;
			break;
		}
		break;
	}

	// Align GPS
	if(new_direction != (robot_direction_t) -1)
	{
		pCtx->current_direction = new_direction;
	}

    // Update position x
	if((pCtx->current_x + step_x) < MAX_MAZE_DEPTH)
	{
		pCtx->current_x += step_x;
	}
    else
    {
    	HAL_Serial_Print(&com, "#ERROR");
    }

    // Update position y
    if((pCtx->current_y + step_y) < MAX_MAZE_DEPTH)
	{
		pCtx->current_y += step_y;
	}
    else
    {
    	HAL_Serial_Print(&com, "#ERROR");
    }
    HAL_Serial_Print(&com, "Update case (x,y): (%d,%d)\n",
    		         (int)pCtx->current_x, (int)pCtx->current_y);

    // Update case state
    if((pCtx->maze_array[pCtx->current_x][pCtx->current_y] & CASE_VISITED) == 0)
    {
        pCtx->maze_array[pCtx->current_x][pCtx->current_y] |= CASE_VISITED;
    }
    else
    {
        pCtx->maze_array[pCtx->current_x][pCtx->current_y] |= CASE_RE_VISITED;
    }

}// end of update_maze_ctx

// Display the maze context
void display_maze_ctx(maze_ctx_t *pCtx)
{
    int         x;
    int         y;
    maze_case_t case_state;

    HAL_Serial_Print(&com,"\tStart (x,y): (%d,%d)\n",
    		(int)pCtx->start_x, (int)pCtx->start_y);

    HAL_Serial_Print(&com,"\tCurrent_direction: %s (%d)\n",
           robot_direction_to_txt(pCtx->current_direction),
		   (int)pCtx->current_direction);

    HAL_Serial_Print(&com,"\tCurrent (x,y): (%d,%d)\n",
    		(int)pCtx->current_x, (int)pCtx->current_y);

    case_state = pCtx->maze_array[pCtx->current_x][pCtx->current_y];

    if(case_state == CASE_UNKNOWN)
    	HAL_Serial_Print(&com,"\tCASE UNKNOWN\n");
    if((case_state & CASE_START) == CASE_START)
    	HAL_Serial_Print(&com,"\tCASE START\n");
    if((case_state & CASE_END) == CASE_END)
    	HAL_Serial_Print(&com,"\tCASE END\n");
    if((case_state & CASE_VISITED) == CASE_VISITED)
    	HAL_Serial_Print(&com,"\tCASE VISITED\n");
    if((case_state & CASE_RE_VISITED) == CASE_RE_VISITED)
    	HAL_Serial_Print(&com,"\tCASE RE VISITED\n");
    if((case_state & CASE_WALL_LEFT) == CASE_WALL_LEFT)
    	HAL_Serial_Print(&com,"\tCASE WALL_LEFT\n");
    if((case_state & CASE_WALL_RIGHT) == CASE_WALL_RIGHT)
    	HAL_Serial_Print(&com,"\tCASE WALL_RIGTH\n");
    if((case_state & CASE_WALL_FRONT) == CASE_WALL_FRONT)
    	HAL_Serial_Print(&com,"\tCASE WALL_FRONT\n");

    HAL_Serial_Print(&com,"\n");
    for(y=(MAX_MAZE_DEPTH-1); y>=0; y--)
    {
    	HAL_Serial_Print(&com,"\t");
        for(x=0; x<MAX_MAZE_DEPTH; x++)
        {
        	HAL_Serial_Print(&com,"%3d ", pCtx->maze_array[x][y]) ;
        }
        HAL_Serial_Print(&com,"\n");
    }
    HAL_Serial_Print(&com,"\n");

}// end of display_maze_ctx

#ifdef __MAZE_TEST__
// Main test
int maze_test(void)
{
    maze_case_t wall_sensor ;

    maze_ctx_init(&the_maze_ctx);

    // Manage start

    wall_sensor = CASE_WALL_LEFT | CASE_WALL_RIGHT ;
    update_maze_ctx(&the_maze_ctx, wall_sensor);
    display_maze_ctx(&the_maze_ctx);

    wall_sensor = CASE_WALL_LEFT | CASE_WALL_RIGHT ;
    update_maze_ctx(&the_maze_ctx, wall_sensor);
    display_maze_ctx(&the_maze_ctx);

    wall_sensor = CASE_WALL_RIGHT | CASE_WALL_FRONT ;
    update_maze_ctx(&the_maze_ctx, wall_sensor);
    display_maze_ctx(&the_maze_ctx);

    wall_sensor = CASE_WALL_LEFT | CASE_WALL_RIGHT ;
    update_maze_ctx(&the_maze_ctx, wall_sensor);
    display_maze_ctx(&the_maze_ctx);

    wall_sensor = CASE_WALL_LEFT | CASE_WALL_RIGHT ;
    update_maze_ctx(&the_maze_ctx, wall_sensor);
    display_maze_ctx(&the_maze_ctx);

    wall_sensor = CASE_WALL_LEFT | CASE_WALL_FRONT ;
    update_maze_ctx(&the_maze_ctx, wall_sensor);
    display_maze_ctx(&the_maze_ctx);

    wall_sensor = CASE_WALL_LEFT | CASE_WALL_RIGHT | CASE_WALL_FRONT ;
    update_maze_ctx(&the_maze_ctx, wall_sensor);
    display_maze_ctx(&the_maze_ctx);

    return(0);
}// End of maze_test
#endif // __MAZE_TEST__

// End of file
