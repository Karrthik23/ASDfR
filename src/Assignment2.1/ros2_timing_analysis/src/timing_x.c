#include <stdio.h>
#include <stdlib.h>
#include <evl/thread.h>
#include <evl/clock.h>
#include <evl/timer.h>
#include <evl/sched.h>
#include <time.h>

#define PERIOD_NS 1000000  // 1 ms (1,000,000 nanoseconds)
#define RUN_TIME 5  // Run for 5 seconds (5000 iterations)

void real_time_task(void *arg) {
    struct timespec next_time, start_time, end_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);

    printf("Real-time periodic task started.\n");

    for (int i = 0; i < (RUN_TIME * 1000); i++) {
        // Record start time
        clock_gettime(CLOCK_MONOTONIC, &start_time);

        // Simulated workload
        volatile int sum = 0;
        for (int j = 0; j < 100000; j++) sum += j;

        // Record end time
        clock_gettime(CLOCK_MONOTONIC, &end_time);

        // Compute execution time in milliseconds
        double exec_time = (end_time.tv_sec - start_time.tv_sec) * 1e3 +
                           (end_time.tv_nsec - start_time.tv_nsec) / 1e6;

        printf("Iteration %d - Execution Time: %.3f ms\n", i, exec_time);

        // Sleep until next period using EVL timer
        next_time.tv_nsec += PERIOD_NS;
        while (next_time.tv_nsec >= 1000000000) {
            next_time.tv_sec++;
            next_time.tv_nsec -= 1000000000;
        }
        evl_usleep(PERIOD_NS / 1000); // EVL real-time sleep function
    }
}

int main() {
    int ret;
    struct evl_sched_attrs attr;

    // Set real-time scheduling attributes
    attr.sched_policy = SCHED_FIFO;
    attr.sched_priority = 80; // High priority

    // Create EVL real-time thread
    ret = evl_attach_self("rt-thread");
    if (ret) {
        perror("Failed to attach to Xenomai EVL");
        return -1;
    }

    printf("Starting real-time task...\n");
    real_time_task(NULL);

    return 0;
}
