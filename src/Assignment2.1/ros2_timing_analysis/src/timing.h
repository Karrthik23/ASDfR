#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>

#define PERIOD_NS 1000000  // 1 ms (1,000,000 nanoseconds)
#define RUN_TIME 5  // Run for 5 seconds
pthread_mutex_t exec_time_mutex;
double exec_time = 0;
void *periodic_task(void *arg) {
    struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);  // Get current time

    for (int i = 0; i < (RUN_TIME * 1000); i++) {
        // Record start time
        struct timespec start_time, end_time;
        clock_gettime(CLOCK_MONOTONIC, &start_time);

        // Perform some computation (simulated workload)
        volatile int sum = 0;
        for (int j = 0; j < 100000; j++) sum += j*j;

        // Record end time
        clock_gettime(CLOCK_MONOTONIC, &end_time);

        // Calculate execution time
        double exec_time_local = (end_time.tv_sec - start_time.tv_sec) * 1e3 +
                                (end_time.tv_nsec - start_time.tv_nsec) / 1e6;

        // Store execution time in globally accesible variable
        pthread_mutex_lock(&exec_time_mutex);
        exec_time = exec_time_local;
        pthread_mutex_unlock(&exec_time_mutex);

        // Calculates new absolute time based on period
        // Compensates for second overflow to ensure correct operation
        next_time.tv_nsec += PERIOD_NS;
        while (next_time.tv_nsec >= 1000000000) {
            next_time.tv_sec++;
            next_time.tv_nsec -= 1000000000;
        }
        // Sleep until next period
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
    }
    return NULL;
}


void *log_task(void *arg){
    struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);  // Get current time

    // Create new file
    FILE *log_file = fopen("timing_log.csv", "w");
    if (log_file == NULL) {
        perror("Failed to create log file");
        return NULL;
    }
    fclose(log_file);  // Close the new file

    // Open the log file to append measured data
    log_file = fopen("timing_log.csv", "a");
    if (log_file == NULL) {
        perror("Failed to open log file to append data to");
        return NULL;
    }

    // Ensures that this thread waits 1 period before it starts logging data from other thread
    next_time.tv_nsec += PERIOD_NS;
    while (next_time.tv_nsec >= 1000000000) {
        next_time.tv_sec++;
        next_time.tv_nsec -= 1000000000;
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);

    for (int i = 0; i < (RUN_TIME * 1000); i++) {
        // Record start time
        struct timespec start_time, end_time;
        clock_gettime(CLOCK_MONOTONIC, &start_time);

        // Read execution time from global variable
        pthread_mutex_lock(&exec_time_mutex);
        double exec_time_local = exec_time;
        pthread_mutex_unlock(&exec_time_mutex);
        
        // Log execution time to file
        fprintf(log_file, "Iteration %d - Execution Time: %.3f ms\n", i, exec_time);

        // Sleep until next period
        next_time.tv_nsec += PERIOD_NS;
        while (next_time.tv_nsec >= 1000000000) {
            next_time.tv_sec++;
            next_time.tv_nsec -= 1000000000;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
    }

    // Close the log file
    fclose(log_file);
    return NULL;
}