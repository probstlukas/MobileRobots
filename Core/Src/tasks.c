/**
 * @brief  All tasks needed to complete the parkour.
 *
 * @author Lukas Probst
 */

#include <stdio.h>

#include "main.h"
#include "gpio.h"
#include "sensors.h"
#include "utility.h"
#include "driving.h"

/* Enables conversion from encoder ticks to millimetres */
#define TICKS_TO_MM 0.19

/* Enables conversion from encoder ticks to degree (but it still depends on the driving speed) */
#define TICKS_TO_DEGREE 0.13

/* Properties of the sections of the yellow trajectory in the parkour */
#define FIRST_STRAIGHT_LENGTH  470
#define RIGHT_CURVE_DEGREE     150
#define SECOND_STRAIGHT_LENGTH 355
#define LEFT_CURVE_DEGREE      90
#define THIRD_STRAIGHT_LENGTH  320

/* Properties to check perimeter for line */
#define HALF_PERIMETER_DEGREE 100
#define NEXT_PERIMETER_LENGTH 80

/* Time until the robot eventually stops */
#define FINISH_LINE_SPURT 100

typedef enum {FIRST_STRAIGHT, SECOND_STRAIGHT, THIRD_STRAIGHT, RIGHT_CURVE, LEFT_CURVE, FINISHED} YellowTrajectory;
YellowTrajectory yellow_trajectory_state = FIRST_STRAIGHT;

typedef enum {LEFT, RIGHT, CENTER, DRIVE_FORWARD} SearchLine;
SearchLine search_line_state = LEFT;

typedef enum {REVERSE, TURN, CIRCUIT} AvoidObstacle;
AvoidObstacle avoid_obstacle_state = REVERSE;

uint8_t perimeter_checked = 0;
int8_t obstacle_passed = 0;

/**
 * @brief  The robot follows a fixed trajectory.
 *
 * @return None
 */
void task_followTrajectory()
{
	switch (yellow_trajectory_state)
	{
		case FIRST_STRAIGHT:
			if (encoder_left_cnt <= FIRST_STRAIGHT_LENGTH * TICKS_TO_MM)
			{
				driveForward();
			}
			else
			{
				resetEncoderCnt();
				yellow_trajectory_state = RIGHT_CURVE;
			}
			break;
		case RIGHT_CURVE:
			if (encoder_left_cnt <= RIGHT_CURVE_DEGREE * TICKS_TO_DEGREE)
			{
				drive(0.5, -0.5);
			}
			else
			{
				resetEncoderCnt();
				yellow_trajectory_state = SECOND_STRAIGHT;
			}
			break;
		case SECOND_STRAIGHT:
			if (encoder_left_cnt <= SECOND_STRAIGHT_LENGTH * TICKS_TO_MM)
			{
				setNormalSpeed();
				driveForward();
			}
			else
			{
				resetEncoderCnt();
				yellow_trajectory_state = LEFT_CURVE;
			}
			break;
		case LEFT_CURVE:
			if (encoder_left_cnt <= LEFT_CURVE_DEGREE * TICKS_TO_DEGREE)
			{
				drive(-0.5, 0.5);
			}
			else
			{
				resetEncoderCnt();
				yellow_trajectory_state = THIRD_STRAIGHT;
			}
			break;
		case THIRD_STRAIGHT:
			if (encoder_left_cnt <= THIRD_STRAIGHT_LENGTH * TICKS_TO_MM)
			{
				setNormalSpeed();
				driveForward();
			}
			else
			{
				resetEncoderCnt();
				drive(0, 0);
				yellow_trajectory_state = FINISHED;
			}
			break;
		/* Trajectory completed */
		case FINISHED:
			resetEncoderCnt();
			current_state = FOLLOW_LINE;
			break;
	}
}

/**
 * @brief  P-controller for line following.
 *
 * @return None
 */
void task_followLine()
{
	float proportional_gain = 0.00015f;
	int32_t error = LINESENSOR_LEFT - LINESENSOR_RIGHT;

	drive(speed_left - error * proportional_gain, speed_right + error * proportional_gain);

	/* Check if robot is on the last part of the parkour to prepare for finish line  */
	if (obstacle_passed == 1 && encoder_left_cnt > 75)
	{
		/* Here greyish/white indicates that the finish line was reached */
		if (middle_linesensor_state == WHITE)
		{
			resetEncoderCnt();
			setMaxSpeed();
			driveForward();
			current_state = FINISH_LINE;
		}
	}
	else
	{
		/* Line lost and robot must first search for the line again */
		if (left_linesensor_state == WHITE && middle_linesensor_state == WHITE && right_linesensor_state == WHITE)
		{
			drive(0, 0);
			resetEncoderCnt();
			search_line_state = LEFT;
			current_state = SEARCH_LINE;
		}

		/* Obstacle detected */
		if (HAL_GPIO_ReadPin(GPIOA, switch_middle_Pin) == 0)
		{
			resetEncoderCnt();
			current_state = AVOID_OBSTACLE;
		}
	}
}

/**
 * @brief  The line is searched for.
 *
 * First, the left wheel is turned until the robot has turned about 100 degrees or the line is found again.
 * Once the line has been detected, the robot continues with the follow line task.
 * If no line is detected, the left wheel is turned backwards until the robot is back in the starting position.
 * Repeat the steps above, but with the right wheel. If no line is detected on the left or right side,
 * the robot returns to the starting position and begins to overcome a potential gap.
 *
 * @return None
 */
void task_searchLine()
{
	switch (search_line_state)
	{
		/* Check left perimeter */
		case LEFT:
			drive(-0.5, 0.5);
			if (encoder_left_cnt > HALF_PERIMETER_DEGREE * TICKS_TO_DEGREE)
			{
				encoder_left_cnt = 0;
				resetEncoderCnt();
				search_line_state = CENTER;
			}
			break;
		/* Check right perimeter */
		case RIGHT:
			drive(0.5, -0.5);
			if (encoder_left_cnt > HALF_PERIMETER_DEGREE * TICKS_TO_DEGREE)
			{
				resetEncoderCnt();
				perimeter_checked = 1;
				search_line_state = CENTER;
			}
			break;
		case CENTER:
			if (perimeter_checked == 1)
			{
				/* Turn back to initial position from right */
				drive(-0.5, 0.5);
				if (encoder_left_cnt > HALF_PERIMETER_DEGREE * TICKS_TO_DEGREE)
				{
					resetEncoderCnt();
					search_line_state = DRIVE_FORWARD;
				}
			}
			else
			{
				drive(0.5, -0.5);
				/* Turn back to initial position from left */
				if (encoder_left_cnt > HALF_PERIMETER_DEGREE * TICKS_TO_DEGREE)
				{
					resetEncoderCnt();
					search_line_state = RIGHT;
				}
			}
			break;
		/* Drive forward to check next perimeter for line */
		case DRIVE_FORWARD:
			drive(0.5, 0.5);
			if (encoder_left_cnt > NEXT_PERIMETER_LENGTH * TICKS_TO_MM)
			{
				perimeter_checked = 0;
				resetEncoderCnt();
				search_line_state = LEFT;
			}
			break;
	}

	/* Constantly check whether the line has been found again */
	if (left_linesensor_state == BLACK || middle_linesensor_state == BLACK || right_linesensor_state == BLACK)
	{
		drive(0,0);
		setNormalSpeed();
		search_line_state = LEFT;
		current_state = FOLLOW_LINE;
	}
}

/**
 * @brief  Circumnavigates the obstacle on the line.
 *
 * @return None
 */
void task_avoidObstacle()
{
	switch (avoid_obstacle_state)
	{
		case REVERSE:
			drive(-0.5, -0.5);
			if (encoder_left_cnt > 5)
			{
				resetEncoderCnt();
				avoid_obstacle_state = TURN;
			}
			break;
		case TURN:
			drive(0.5, -0.5);
			if (encoder_left_cnt > 7)
			{
				resetEncoderCnt();
				avoid_obstacle_state = CIRCUIT;
			}
			break;
		case CIRCUIT:
			drive(0.3, 0.55);
			if (middle_linesensor_state == BLACK)
			{
				resetEncoderCnt();
				setNormalSpeed();
				avoid_obstacle_state = REVERSE;
				obstacle_passed = 1;
				current_state = FOLLOW_LINE;
			}
			break;
	}
}

/**
 * @brief  The finish line is reached.
 *
 * @return None
 */
void task_finishLine()
{
	/* Time until the final spurt is over and the robot comes to a standstill */
	if (encoder_left_cnt > FINISH_LINE_SPURT * TICKS_TO_MM)
	{
		drive(0, 0);
		blinkAllLEDs();
	}
}
