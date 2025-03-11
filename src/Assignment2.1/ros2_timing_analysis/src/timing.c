#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>

#define PERIOD_NS 1000000  // 1 ms (1,000,000 nanoseconds)
#define RUN_TIME 5  // Run for 5 seconds

void *periodic_task(void *arg) {
    struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);  // Get current time

    for (int i = 0; i < (RUN_TIME * 1000); i++) {
        // Record start time
        struct timespec start_time, end_time;
        clock_gettime(CLOCK_MONOTONIC, &start_time);

        // Perform some computation (simulated workload)
        volatile int sum = 0;
        for (int j = 0; j < 100000; j++) sum += j;

        // Record end time
        clock_gettime(CLOCK_MONOTONIC, &end_time);

        // Print timing information
        double exec_time = (end_time.tv_sec - start_time.tv_sec) * 1e3 +
                           (end_time.tv_nsec - start_time.tv_nsec) / 1e6;
        printf("Iteration %d - Execution Time: %.3f ms\n", i, exec_time);

        // Sleep until next period
        next_time.tv_nsec += PERIOD_NS;
        while (next_time.tv_nsec >= 1000000000) {
            next_time.tv_sec++;
            next_time.tv_nsec -= 1000000000;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
    }
    return NULL;
}

int main() {
    pthread_t thread;
    pthread_create(&thread, NULL, periodic_task, NULL);
    pthread_join(thread, NULL);
    return 0;
}
