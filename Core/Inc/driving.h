/**
 * @brief  Header file for driving.c.
 *
 * @author Lukas Probst
 */

#ifndef __DRIVING_H__
#define __DRIVING_H__

extern double speed_left;
extern double speed_right;

void drive(double speed_left, double speed_right);
void driveForward();

#endif /* __DRIVING_H__ */
