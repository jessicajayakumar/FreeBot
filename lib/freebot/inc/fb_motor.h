/**
 * @file fb_motor.h
 * @author Mengyao Liu <mengyao.liu@kuleuven.be>
 * @author Lowie Deferme <lowie.deferme@kuleuven.be>
 * @brief FreeBot motor drivers
 * @version 2.0
 * @date 2024-03-26
 *
 * @copyright Copyright (c) 2024
 *
 */

/* Quick overview:
 *                               ^
 *                               |
 *                        fb_straight_forw
 *                               |
 *
 *           ^          +----------------+          ^
 *            \     +---+                +---+     /
 *    fb_side_d315  |   |                |   |  fb_side_d45
 *              \   |   |                |   |   /
 *                  |   |                |   |
 *                  |   |                |   |
 *                  +---+                +---+
 *                      |                |
 *       fb_side_left   |                |   fb_side_right
 *       <-----------   |    fb_stop     |   ------------>
 *                      |                |
 *                      |                |
 *                  +---+                +---+
 *                  |   |                |   |
 *                  |   |                |   |
 *              /   |   |                |   |   \
 *    fb_side_d225  |   |                |   |  fb_side_d135
 *            /     +---+                +---+     \
 *           v          +----------------+          v
 *
 *                               |
 *                        fb_straight_back
 *                               |
 *                               v
 */

#ifndef FB_MOTOR_H
#define FB_MOTOR_H

/**
 * @brief Initialize FreeBot's motors
 * @details v1.0 equivalent: Motors_init(void)
 */
int fb_motor_init(void);

/**
 * @brief Stop FreeBot
 * @details v1.0 equivalent: FreeBotStop(void)
 */
void fb_stop(void);

/**
 * @brief Move forward
 * @details v1.0 equivalent: FreeBotStraight_Forward(bool MotorSpeed)
 */
void fb_straight_forw(void);

/**
 * @brief Move backward
 * @details v1.0 equivalent: FreeBotStraight_Backward(bool MotorSpeed)
 */
void fb_straight_back(void);

/**
 * @brief Move right
 * @details v1.0 equivalent: FreeBotSide_Right(bool MotorSpeed)
 */
void fb_side_right(void);

/**
 * @brief Move left
 * @details v1.0 equivalent: FreeBotSide_Left(bool MotorSpeed)
 */
void fb_side_left(void);

/**
 * @brief Move 45° to the right
 * @details v1.0 equivalent: FreeBotSide_DIAGONAL45(bool MotorSpeed)
 */
void fb_side_d45(void);

/**
 * @brief Move 135° to the right
 * @details v1.0 equivalent: FreeBotSide_DIAGONAL135(bool MotorSpeed)
 */
void fb_side_d135(void);

/**
 * @brief Move 135° to the left
 * @details v1.0 equivalent: FreeBotSide_DIAGONAL225(bool MotorSpeed)
 */
void fb_side_d225(void);

/**
 * @brief Move 45° to the left
 * @details v1.0 equivalent: FreeBotSide_DIAGONAL315(bool MotorSpeed)
 */
void fb_side_d315(void);

/**
 * @brief Rotate clockwise
 * @details v1.0 equivalent: FreeBotRotate_CLOCKWISE(bool MotorSpeed)
 */
void fb_rotate_cw(void);

/**
 * @brief Rotate counterclockwise
 * @details v1.0 equivalent: FreeBotRotate_COUNTERCLOCKWISE(bool MotorSpeed)
 */
void fb_rotate_ccw(void);

/** @brief Motor speeds (rpm) */
typedef struct fb_motor_speed_s
{
    /** @brief Front left motor speed (rpm)*/
    int front_left;
    /** @brief Front right motor speed (rpm)*/
    int front_right;
    /** @brief Back left motor speed (rpm)*/
    int back_left;
    /** @brief Back right motor speed (rpm)*/
    int back_right;
} fb_motor_speed_t;

/** @brief Motor angles (degrees) */
typedef struct fb_motor_angle_s
{
    /** @brief Front left motor angle (degrees)*/
    int front_left;
    /** @brief Front right motor angle (degrees)*/
    int front_right;
    /** @brief Back left motor angle (degrees)*/
    int back_left;
    /** @brief Back right motor angle (degrees)*/
    int back_right;
} fb_motor_angle_t;

/**
 * @brief Get motor speeds
 * @param speeds struct to populate with the new speeds
 */
void fb_get_motor_speed(fb_motor_speed_t *speeds);

/**
 * @brief Get motor angles
 * @param angles struct to populate with the new angles
 */
void fb_get_motor_angle(fb_motor_angle_t *angles);

#endif /* FB_MOTOR_H */
