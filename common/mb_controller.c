#include "../mobilebot/mobilebot.h"

/*******************************************************************************
* int mb_initialize()
*
* this initializes all the PID controllers from the configuration file
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/

int mb_initialize_controller(){
    mb_load_controller_config();
    //Temporary till we load our own config file

    left_wheel_velocity_pid_PD = rc_filter_empty();
    right_wheel_velocity_pid_PD = rc_filter_empty();
    left_wheel_velocity_pid_I = rc_filter_empty();
    right_wheel_velocity_pid_I = rc_filter_empty();
    left_velocity_ma=rc_filter_empty();
    right_velocity_ma=rc_filter_empty();
//    left_encoder_velocity_lpf= rc_filter_empty();
//    right_encoder_velocity_lpf= rc_filter_empty();
    left_cmd_lpf= rc_filter_empty();
    right_cmd_lpf= rc_filter_empty();

//    rc_filter_first_order_lowpass(&left_encoder_velocity_lpf,DT,50);
//    rc_filter_first_order_lowpass(&right_encoder_velocity_lpf,DT,50);
    rc_filter_moving_average(&left_velocity_ma,30,DT);
    rc_filter_moving_average(&right_velocity_ma,30,DT);
    rc_filter_first_order_lowpass(&left_cmd_lpf,DT,5*DT);
    rc_filter_first_order_lowpass(&right_cmd_lpf,DT,5*DT);

    rc_filter_pid(&left_wheel_velocity_pid_PD, left_wheel.kp, 0.0, left_wheel.kd, left_wheel.dFilterHz, DT);
    rc_filter_pid(&right_wheel_velocity_pid_PD, right_wheel.kp, 0.0, right_wheel.kd, right_wheel.dFilterHz, DT);
    rc_filter_pid(&left_wheel_velocity_pid_I, 0.0, left_wheel.ki, 0.0, left_wheel.dFilterHz, DT);
    rc_filter_pid(&right_wheel_velocity_pid_I, 0.0, right_wheel.ki, 0.0, right_wheel.dFilterHz, DT);

    rc_filter_enable_saturation(&left_wheel_velocity_pid_I, -0.3, 0.3);
    rc_filter_enable_saturation(&right_wheel_velocity_pid_I, -0.3, 0.3);
    rc_filter_enable_saturation(&left_wheel_velocity_pid_PD, -0.7, 0.7);
    rc_filter_enable_saturation(&right_wheel_velocity_pid_PD, -0.7, 0.7);
    return 0;
}

/*******************************************************************************
* int mb_load_controller_config()
*
* this provides a basic configuration load routine
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/


int mb_load_controller_config(){
    FILE* file = fopen("MBCFG.txt", "r");
    if (file == NULL){
        printf("Error opening pid.cfg\n");
    }
    fscanf(file, "%f %f %f %f %f %f %f %f",
                   &(left_wheel.kp),
                   &(left_wheel.ki),
                   &(left_wheel.kd),
                   &(left_wheel.dFilterHz),

                   &(right_wheel.kp),
                   &(right_wheel.ki),
                   &(right_wheel.kd),
                   &(right_wheel.dFilterHz)
    );



/******
*
*   Example of loading a line from .cfg file:
*
*    fscanf(file, "%f %f %f %f", 
*        &pid_params.kp,
*        &pid_params.ki,
*        &pid_params.kd,
*        &pid_params.dFilterHz
*        );
*
******/

    fclose(file);
    return 0;
}

/*******************************************************************************
* int mb_controller_update()
* 
* TODO: Write your PID controller here
* take inputs from the global mb_state
* write outputs to the global mb_state
*
* return 0 on success
*
*******************************************************************************/

int mb_controller_update(mb_state_t* mb_state, mb_setpoints_t* mb_setpoints){

    mb_setpoints->left_velocity=mb_setpoints->fwd_velocity-mb_setpoints->turn_velocity*WHEEL_BASE/2;
    mb_setpoints->right_velocity=mb_setpoints->fwd_velocity+mb_setpoints->turn_velocity*WHEEL_BASE/2;


    float left_setpoint=/* rc_filter_march(&left_cmd_lpf, */mb_setpoints->left_velocity;//);
    float right_setpoint=/* rc_filter_march(&right_cmd_lpf,*/ mb_setpoints->right_velocity;//);

//    float left_velocity= rc_filter_march(&left_encoder_velocity_lpf, mb_state->left_velocity);
//    float right_velocity= rc_filter_march(&right_encoder_velocity_lpf, mb_state->right_velocity);

    mb_state->left_velocity = rc_filter_march(&left_velocity_ma,mb_state->left_velocity);
    mb_state->right_velocity = rc_filter_march(&right_velocity_ma,mb_state->right_velocity);

    float left_error = left_setpoint - mb_state->left_velocity;
    float right_error = right_setpoint - mb_state->right_velocity;
    float open_loop_left;
    float open_loop_right;
    if(left_setpoint==0 && right_setpoint==0)
    {
        open_loop_left = 0;
        open_loop_right = 0;
    }
    else
    {
        open_loop_left = OPEN_LOOP_LEFT_Y_INT + OPEN_LOOP_LEFT_Y_SLOPE*(left_setpoint);
        open_loop_right = OPEN_LOOP_RIGHT_Y_INT + OPEN_LOOP_RIGHT_Y_SLOPE*(right_setpoint);
    }


    mb_state -> left_cmd = rc_filter_march(&left_wheel_velocity_pid_PD, left_error)+ rc_filter_march(&left_wheel_velocity_pid_I, left_error) + open_loop_left;
    mb_state -> right_cmd = rc_filter_march(&right_wheel_velocity_pid_PD, right_error)+ rc_filter_march(&right_wheel_velocity_pid_I, right_error) + open_loop_right;

    mb_state->left_cmd=rc_filter_march(&left_cmd_lpf,mb_state->left_cmd);
    mb_state->right_cmd=rc_filter_march(&right_cmd_lpf,mb_state->right_cmd);
    return 0;
}


/*******************************************************************************
* int mb_destroy_controller()
* 
* TODO: Free all resources associated with your controller
*
* return 0 on success
*
*******************************************************************************/

int mb_destroy_controller(){
    rc_filter_free(&left_wheel_velocity_pid_PD);
    rc_filter_free(&right_wheel_velocity_pid_PD);
    rc_filter_free(&left_wheel_velocity_pid_I);
    rc_filter_free(&right_wheel_velocity_pid_I);
    rc_filter_free(&left_velocity_ma);
    rc_filter_free(&right_velocity_ma);
    rc_filter_free(&left_cmd_lpf);
    rc_filter_free(&right_cmd_lpf);
    return 0;
}
