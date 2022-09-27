/*******************************************************************************
* mb_odometry.c
*
* TODO: Implement these functions to add odometry and dead rekoning 
*
*******************************************************************************/

#include "../mobilebot/mobilebot.h"
#include "mb_defs.h"
#include <math.h>

#define PI 3.14159265358979323846

/*******************************************************************************
* mb_initialize_odometry() 
*
* TODO: initialize odometry
* NOTE: you should initialize from Optitrack data if available
*
*******************************************************************************/
void mb_initialize_odometry(mb_odometry_t* mb_odometry, float x, float y, float theta){

    mb_odometry-> x = 0.0;
    mb_odometry-> y = 0.0;
    mb_odometry-> theta = 0.0;

}

/*******************************************************************************
* mb_update_odometry()
*
* TODO: calculate odometry from internal variables
*       publish new odometry to lcm ODOMETRY_CHANNEL
*
*******************************************************************************/
static float prev_mb_odometry_theta = 0.0;

void mb_update_odometry(mb_odometry_t* mb_odometry, mb_state_t* mb_state){

// convert encoders to meters
    float enc2meters = (WHEEL_DIAMETER * 3.14159) / (GEAR_RATIO * ENCODER_RES);

//Summary of the Odometry Equations
    float deltaThetaOdo = enc2meters*(mb_state->left_encoder_delta - mb_state->right_encoder_delta) / WHEEL_BASE;
    float deltaDis = enc2meters*(mb_state->left_encoder_delta + mb_state->right_encoder_delta) / 2.0;

    float deltaX = deltaDis * cos(mb_odometry->theta + deltaThetaOdo / 2.0);
    float deltaY = deltaDis * sin(mb_odometry->theta + deltaThetaOdo / 2.0);

//Calculate Gyrodometry
    float threshold = 0.001;
    float oldTheta = prev_mb_odometry_theta; //might be problem area
    float deltaThetaGyro = mb_angle_diff_radians(mb_state->last_yaw, mb_state->tb_angles[2]);
    float deltaGyroOdo = mb_angle_diff_radians(deltaThetaGyro , deltaThetaOdo);

    float NewTheta = 0.0;

    if (fabs(deltaGyroOdo) > threshold){
        NewTheta = mb_clamp_radians(oldTheta + deltaThetaGyro);
    }
    else{
        NewTheta = mb_clamp_radians(oldTheta + deltaThetaOdo);
    }

    //NewTheta = mb_clamp_radians(oldTheta + deltaThetaOdo);
    mb_odometry->x = mb_odometry->x + deltaX;
    mb_odometry->y = mb_odometry->y +  deltaY;
    mb_odometry->theta = NewTheta;

    prev_mb_odometry_theta = mb_odometry->theta;
}

/*******************************************************************************
* mb_clamp_radians() 
* clamp an angle from -PI to PI
*******************************************************************************/
float mb_clamp_radians(float angle){

    if(angle < -PI)
    {
        for(; angle < -PI; angle += 2.0*PI);
    }
    else if(angle > PI)
    {
        for(; angle > PI; angle -= 2.0*PI);
    }

    return angle;
}


/*******************************************************************************
* mb_angle_diff_radians() 
* computes difference between 2 angles and wraps from -PI to PI
*******************************************************************************/
float mb_angle_diff_radians(float angle1, float angle2){
    float diff = angle2 - angle1;
    while(diff < -PI) diff+=2.0*PI;
    while(diff > PI) diff-=2.0*PI;
    return diff;
}
