#ifndef TEMPLATE20SIM_HPP
#define TEMPLATE20SIM_HPP

#include "XenoFrt20Sim.hpp"
#include "LoopController.h"
#include <stdio.h>
#include <math.h>
#pragma pack (1)    //https://carlosvin.github.io/langs/en/posts/cpp-pragma-pack/#_performance_test
struct ThisIsAStruct
{
    int this_is_a_int = 0;
    double this_is_a_double = 100.0;
    float this_is_a_float = 10.0;
    char this_is_a_char = 'R';
    bool this_is_a_bool = false;
};

struct Pos
{
    double current_pos_left = 0;
    double current_pos_right = 0;
    double previous_tick_left = 0;
    double previous_tick_right = 0;
    double current_tick_left = 0;
    double current_tick_right = 0;
    double difference_left = 0;
    double difference_right = 0; 
};
#pragma pack(0)

class Test_Bed : public XenoFrt20Sim
{
public:
Test_Bed(uint write_decimator_freq, uint monitor_freq);
    ~Test_Bed();
private:
    XenoFileHandler file;
    struct ThisIsAStruct data_to_be_logged;
    LoopController controller;
    struct Pos position;
    double u[4];
    double y[2];
    float gear_ratio = 15.58;
    float encoder_rev_count = 4096;
    float rad_full_rev_conv = 2*M_PI;
    float wheel_r = 0.101;
    float max_encoder_ticks = 16383;
    int print1 = 0;
    
protected:
    //Functions
    int initialising() override;
    int initialised() override;
    int run() override;
    int stopping() override;
    int stopped() override;
    int pausing() override;
    int paused() override;
    int error() override;

    // current error
    int current_error = 0;
};

#endif // TEMPLATE20SIM_HPP