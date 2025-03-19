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

#define PERIOD_NS 1000000  // 1 ms (1,000,000 nanoseconds)
#define RUN_TIME 5         // Run for 5 seconds (5000 iterations)
#define CPU_CORE 1         // Assign thread to CPU core 1
#define FILE_NAME "ss_timing_log_with_load_evl_bash.csv"
static struct evl_mutex rt_mutex;

// Array to store execution time log
static double exec_times[RUN_TIME * 1000];

void *real_time_task(void *arg) {
    struct timespec next_time, start_time, end_time;

    // Get current time for the periodic loop
    clock_gettime(CLOCK_MONOTONIC, &next_time);

    // Attach as a real-time thread
    if (evl_attach_thread(EVL_CLONE_PRIVATE, "rt-task") < 0) {
        perror("Failed to attach real-time thread");
        return NULL;
    }

    printf("Real-time periodic task started on CPU core %d\n", CPU_CORE);

    for (int i = 0; i < (RUN_TIME * 1000); i++) {
        // Record start time
        clock_gettime(CLOCK_MONOTONIC, &start_time);

        // Simulated workload
        volatile int sum = 0;
        for (int j = 0; j < 100000; j++) sum += j;

        // Record end time
        clock_gettime(CLOCK_MONOTONIC, &end_time);

        // Compute execution time in milliseconds
        exec_times[i] = (end_time.tv_sec - start_time.tv_sec) * 1e3 +
                        (end_time.tv_nsec - start_time.tv_nsec) / 1e6;

        // Sleep until the next period using EVL real-time timer
        next_time.tv_nsec += PERIOD_NS;
        while (next_time.tv_nsec >= 1000000000) {
            next_time.tv_sec++;
            next_time.tv_nsec -= 1000000000;
        }
        evl_usleep(PERIOD_NS / 1000); // EVL real-time sleep function
    }

    return NULL;
}

int main() {
    int ret;
    pthread_t rt_thread;
    struct evl_sched_attrs attr;
    cpu_set_t cpu_set;

    // Set real-time scheduling attributes
    attr.sched_policy = SCHED_FIFO;
    attr.sched_priority = 80; // High priority

    // Attach main process to EVL
    ret = evl_attach_self("rt-main");
    if (ret < 0) {
        perror("Failed to attach to Xenomai EVL");
        return -1;
    }

    // Set CPU affinity to force thread to run on a single core
    CPU_ZERO(&cpu_set);
    CPU_SET(CPU_CORE, &cpu_set);

    pthread_attr_t attr_thread;
    pthread_attr_init(&attr_thread);
    pthread_attr_setaffinity_np(&attr_thread, sizeof(cpu_set_t), &cpu_set);

    printf("Starting real-time task on CPU core %d...\n", CPU_CORE);

    // Create and start real-time thread
    pthread_create(&rt_thread, &attr_thread, real_time_task, NULL);
    pthread_join(rt_thread, NULL);

    // Open log file after execution completes
    int log_fd = open(FILE_NAME, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (log_fd < 0) {
        perror("Failed to open log file");
        return -1;
    }

    // Write execution times to the log file
    char log_buffer[256];
    for (int i = 0; i < (RUN_TIME * 1000); i++) {
        snprintf(log_buffer, sizeof(log_buffer), "Iteration %d - Execution Time: %.3f ms\n", i, exec_times[i]);
        write(log_fd, log_buffer, strlen(log_buffer));
    }

    // Close log file
    close(log_fd);

    return 0;
}

