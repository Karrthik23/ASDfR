#include <cstring>
#include <stdio.h>
#include <stdlib.h>
#include <evl/thread.h>
#include <evl/clock.h>
#include <evl/timer.h>
#include <evl/sched.h>
#include <evl/mutex.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <sched.h>
#include <pthread.h>

#define PERIOD_NS 1000000    // 1 ms (1,000,000 nanoseconds)
#define RUN_TIME 5       // run time = 5s
#define CPU_CORE 1       // thread pinned to core 1
#define FILE_NAME "ss_timing_log_with_load_evl_bash.csv"
static struct evl_mutex rt_mutex; // Creating a struct for mutex

// buffer to hold exec time logs
static double exec_times[RUN_TIME * 1000];

void *real_time_task(void *arg) {
    struct timespec next_time, start_time, end_time; //for recoding timing 

    clock_gettime(CLOCK_MONOTONIC, &next_time);  // grab current time

    // attach thread to evl
    if (evl_attach_thread(EVL_CLONE_PRIVATE, "rt-task") < 0) {
        perror("Failed to attach real-time thread");
        return NULL;
    }

    printf("Real-time periodic task started on CPU core %d\n", CPU_CORE);
    //printing to know when the loop starts

    for (int i = 0; i < (RUN_TIME * 1000); i++) {
        clock_gettime(CLOCK_MONOTONIC, &start_time);   // start timestamp

        // some cpu load (dummy loop)
        volatile int product = 0;
        for (int j = 0; j < 100000; j++) product += j*j;

        clock_gettime(CLOCK_MONOTONIC, &end_time);     // end timestamp

        // calc duration in ms
        exec_times[i] = (end_time.tv_sec - start_time.tv_sec) * 1e3 +
                        (end_time.tv_nsec - start_time.tv_nsec) / 1e6;

        // add period to next_time
        next_time.tv_nsec += PERIOD_NS;
        while (next_time.tv_nsec >= 1000000000) {
            next_time.tv_sec++;
            next_time.tv_nsec -= 1000000000;
        }

        evl_usleep(PERIOD_NS / 1000);   // sleep 1ms using evl
    }

    return NULL;
}