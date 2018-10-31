/*
 * File      : kalman.h
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-10-26     weety    first version.
 */

#ifndef __KALMAN_H__
#define __KALMAN_H__

void kalman_filter(float Accel, float Gyro, float dt);

#endif

