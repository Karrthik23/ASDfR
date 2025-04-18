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
    actuate_data.pwm1 = 0;
    actuate_data.pwm2 = 0;
    u[0] =0;// Pos left
    u[1] =0;///Pos right
    u[2] =0;/* SetVelLeft */
    u[3] = 0; /* SetVelright */
    pos_left.previous_pos = sample_data.channel2;
    pos_right.previous_pos = sample_data.channel1;
    return 1;
}

int Test_Bed::run()
{
    // Do what you need to do
    // Return 1 to go to stopping state

    // Start logger
    // logger.start();                             
    // monitor.printf("Hello from run\n");  
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
    
    double current_tick_left = sample_data.channel2;
    pos_left.diff = current_tick_left - pos_left.previous_pos;
   //account for overflow
    if (pos_left.diff > max_encoder_ticks/2){
        pos_left.diff -= max_encoder_ticks;
    }
    else if(pos_left.diff < -max_encoder_ticks/2){
        pos_left.diff += max_encoder_ticks;
    }
    pos_left.previous_pos = current_tick_left;

    double current_tick_right = sample_data.channel1;
    pos_right.diff = -(current_tick_right - pos_right.previous_pos);
    if (pos_right.diff > max_encoder_ticks/2)    {
        pos_right.diff -= max_encoder_ticks;
    }
    else if(pos_right.diff < -max_encoder_ticks/2)    {
        pos_right.diff += max_encoder_ticks;
    }
    pos_right.previous_pos = current_tick_right;
    // update previous ticks
    
    // Calculate displacement
    pos_left.current_pos += pos_left.diff/(encoder_rev_count*gear_ratio)*(rad_full_rev_conv);
    pos_right.current_pos += pos_right.diff/(encoder_rev_count*gear_ratio)*(rad_full_rev_conv);

    u[0] = pos_left.current_pos; // Pos left
    u[1] = pos_right.current_pos; //Pos right
    u[2] = ros_data.left_motor_setpoint_vel; /* SetVelLeft */
    u[3] = ros_data.right_motor_setpoint_vel; /* SetVelright */

    controller.Calculate(u, y);

    xeno_data.current_pos_left  = pos_left.current_pos;
    xeno_data.current_pos_right = pos_right.current_pos;
    xeno_data.difference_left   = pos_left.diff;
    xeno_data.difference_right  = pos_right.diff;
    

    output_left =  y[0];            // y[0]-->/* Steer Left, so left motor turns */
    output_right =  y[1];           // y[1]-->/* Steer Right, so right motor turns */


    output_right = std::clamp(output_right,-2047.0,2047.0);
    output_left = std::clamp(output_left,-2047.0,2047.0);
    // std::clamp(output_right,-2047,2047);
    // right motor y[1]-->/* Steer right */
    actuate_data.pwm1 = -output_right;
    actuate_data.pwm2 = output_left;
    // monitor.printf("Right steer value : %f\n",output_left);
    // monitor.printf("Left steer value : %f\n",output_right);
    xeno_data.left_motor_pwm    = output_left;
    xeno_data.right_motor_pwm   = -output_right;
    // if(controller.IsFinished())
    //     return 1;
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
