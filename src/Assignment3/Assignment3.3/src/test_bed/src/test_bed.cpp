#include "test_bed.hpp"

Test_Bed::Test_Bed(uint write_decimator_freq, uint monitor_freq) :
    XenoFrt20Sim( write_decimator_freq, monitor_freq, file, &data_to_be_logged),
    file(1,"/home/pi/workspace/template","bin")
{
     printf("%s: Constructing rampio\n", __FUNCTION__);
    // Add variables to logger to be logged, has to be done before you can log data
    logger.addVariable("this_is_a_int", integer);
    logger.addVariable("this_is_a_double", double_);
    logger.addVariable("this_is_a_float", float_);
    logger.addVariable("this_is_a_char", character);
    logger.addVariable("this_is_a_bool", boolean);
    
    // To infinite run the controller, uncomment line below
    //controller.SetFinishTime(0.0);
}

Test_Bed::~Test_Bed()
{
    
}

int Test_Bed::initialising()
{
    // Set physical and cyber system up for use in a 
    // Return 1 to go to initialised state

    evl_printf("Hello from initialising\n");      // Do something

    // The logger has to be initialised at only once
    // logger.initialise();
    // The FPGA has to be initialised at least once
    ico_io.init();

    return 1;
}

int Test_Bed::initialised()
{
    // Keep the physical syste in a state to be used in the run state
    // Call start() or return 1 to go to run state

    evl_printf("Hello from initialised\n");       // Do something

    return 1;
}

int Test_Bed::run()
{
    // Do what you need to do
    // Return 1 to go to stopping state

    // Start logger
    // logger.start();                             
    monitor.printf("Hello from run\n");  
    //  Change some data for logger            
    data_to_be_logged.this_is_a_bool = !data_to_be_logged.this_is_a_bool;
    data_to_be_logged.this_is_a_int++;
    if(data_to_be_logged.this_is_a_char == 'R')
        data_to_be_logged.this_is_a_char = 'A';
    else if (data_to_be_logged.this_is_a_char == 'A')
        data_to_be_logged.this_is_a_char = 'M';
    else
        data_to_be_logged.this_is_a_char = 'R';
    data_to_be_logged.this_is_a_float = data_to_be_logged.this_is_a_float/2;
    data_to_be_logged.this_is_a_double = data_to_be_logged.this_is_a_double/4; 

        
    /* From Loop Controller
        u[0];		 PosLeft 
        u[1];		 PosRight 
        u[2];		 SetVelLeft 
        u[3];		 SetVelRight 
        y[0]         SteerLeft 
        y[1] 		 SteerRight 
        channel 1 is left pos
        channel 2 is right pos
        pwm1 is left motor input
        pwm2 is right motor input
    */

    // update current ticks
    position.current_tick_left = sample_data.channel1;
    position.current_tick_right = sample_data.channel2;

    // Calculate angle difference
    position.difference_left = position.current_tick_left-position.previous_tick_left;
    position.difference_right = position.current_tick_right-position.previous_tick_right;    
    //account for overflow
    if (position.difference_left > max_encoder_ticks/2)
    {
        position.difference_left -= (max_encoder_ticks+1);
    }
    else if(position.difference_left < -max_encoder_ticks/2)
    {
        position.difference_left += (max_encoder_ticks+1);
    }

    
    if (position.difference_right > max_encoder_ticks/2)
    {
        position.difference_right -= (max_encoder_ticks+1);
    }
    else if(position.difference_right < -max_encoder_ticks/2)
    {
        position.difference_right += (max_encoder_ticks+1);
    }
    // update previous ticks
    position.previous_tick_left = position.current_tick_left;
    position.previous_tick_right = position.current_tick_right;
    // Calculate displacement
    position.current_pos_left += position.difference_left/(encoder_rev_count*gear_ratio)*(wheel_r*rad_full_rev_conv);
    position.current_pos_right += position.difference_right/(encoder_rev_count*gear_ratio)*(wheel_r*rad_full_rev_conv);

    u[0] = position.current_pos_left; // Pos left
    u[1] = position.current_pos_right; //Pos right
    u[2] = ros_data.left_motor_setpoint_vel; /* SetVelLeft */
    u[3] = ros_data.right_motor_setpoint_vel; /* SetVelright */
    controller.Calculate(u, y);
    xeno_data.current_pos_left  = position.current_pos_left;
    xeno_data.current_pos_right = position.current_pos_right;
    xeno_data.difference_left   = position.difference_left;
    xeno_data.difference_right  = position.difference_right;
    xeno_data.left_motor_pwm    = ros_data.left_motor_setpoint_vel;
    xeno_data.right_motor_pwm   = ros_data.right_motor_setpoint_vel;
    actuate_data.pwm1 =  ros_data.left_motor_setpoint_vel;          // left motor y[0]-->/* Steer Left */
    actuate_data.pwm2 =  ros_data.right_motor_setpoint_vel;          // right motor y[1]-->/* Steer right */
    monitor.printf("Right set_vel value : %f\n",ros_data.right_motor_setpoint_vel);
    monitor.printf("Left set_vel value : %f\n",ros_data.left_motor_setpoint_vel);

    return 0;
}

int Test_Bed::stopping()
{
    // Bring the physical system to a stop and set it in a state that the system can be deactivated
    // Return 1 to go to stopped state
    // logger.stop();                                // Stop logger
    evl_printf("Hello from stopping\n");          // Do something

    return 1;
}

int Test_Bed::stopped()
{
    // A steady state in which the system can be deactivated whitout harming the physical system

    monitor.printf("Hello from stopping\n");          // Do something

    return 0;
}

int Test_Bed::pausing()
{
    // Bring the physical system to a stop as fast as possible without causing harm to the physical system

    evl_printf("Hello from pausing\n");           // Do something
    return 1 ;
}

int Test_Bed::paused()
{
    // Keep the physical system in the current physical state

    monitor.printf("Hello from paused\n");            // Do something
    return 0;
}

int Test_Bed::error()
{
    // Error detected in the system 
    // Can go to error if the previous state returns 1 from every other state function but initialising 

    monitor.printf("Hello from error\n");             // Do something

    return 0;
}
