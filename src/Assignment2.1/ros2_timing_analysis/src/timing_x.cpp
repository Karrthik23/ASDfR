

#include <stdio.h>
#include <stdlib.h>
#include <evl/thread.h>
#include <evl/clock.h>
#include <evl/timer.h>
#include <evl/sched.h>
#include <evl/mutex.h>
#include <evl/cond.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>

// 🔹 Real-time task: Measures execution time & stores it in buffer
void *real_time_task(void *arg) {
    struct timespec next_time, start_time, end_time;
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
        double exec_time = (end_time.tv_sec - start_time.tv_sec) * 1e3 +
                           (end_time.tv_nsec - start_time.tv_nsec) / 1e6;

        // Compute jitter
        double jitter = fabs(exec_time - 1.0);

        // 🔹 Store data in buffer (thread-safe using EVL mutex)
        evl_lock_mutex(&buffer_mutex);
        if (buffer_index < BUFFER_SIZE) {
            timing_buffer[buffer_index].exec_time = exec_time;
            timing_buffer[buffer_index].jitter = jitter;
            buffer_index++;
        }
        evl_signal_cond(&buffer_cond);  // Notify logging thread
        evl_unlock_mutex(&buffer_mutex);

        // Sleep until next period using EVL timer
        next_time.tv_nsec += PERIOD_NS;
        while (next_time.tv_nsec >= 1000000000) {
            next_time.tv_sec++;
            next_time.tv_nsec -= 1000000000;
        }
        evl_usleep(PERIOD_NS / 1000);  // EVL real-time sleep function
    }

    // Notify logging thread that real-time task is done
    evl_lock_mutex(&buffer_mutex);
    logging_done = 1;
    evl_signal_cond(&buffer_cond);
    evl_unlock_mutex(&buffer_mutex);

    return NULL;
}

// 🔹 Logging thread: Writes execution times to a file
void *logging_task(void *arg) {
    char log_buffer[256];

    while (1) {
        evl_lock_mutex(&buffer_mutex);
        
        // Wait until new data is available or real-time thread is done
        while (buffer_index == 0 && !logging_done) {
            evl_wait_cond(&buffer_cond, &buffer_mutex);
        }

        // Process buffer
        for (int i = 0; i < buffer_index; i++) {
            snprintf(log_buffer, sizeof(log_buffer),
                     "Iteration %d - Execution Time: %.3f ms - Jitter: %.3f ms\n",
                     i, timing_buffer[i].exec_time, timing_buffer[i].jitter);
            write(log_fd, log_buffer, strlen(log_buffer));
        }

        // Reset buffer
        buffer_index = 0;

        // Exit if real-time task has completed
        if (logging_done) {
            evl_unlock_mutex(&buffer_mutex);
            break;
        }

        evl_unlock_mutex(&buffer_mutex);
    }

    return NULL;
}


#define PERIOD_NS 1000000  // 1 ms (1,000,000 nanoseconds)
#define RUN_TIME 5  // Run for 5 seconds (5000 iterations)
#define CPU_CORE 1  // Assign real-time thread to CPU core 1
#define BUFFER_SIZE 1000  // Buffer to store timing data

typedef struct {
    double exec_time;
    double jitter;
} TimingData;

// Shared buffer and control variables
static TimingData timing_buffer[BUFFER_SIZE];
static int buffer_index = 0;
static int logging_done = 0;
static struct evl_mutex buffer_mutex;
static struct evl_cond buffer_cond;

// File descriptor for logging thread
static int log_fd;

int main() {
    int ret;
    pthread_t rt_thread, log_thread;
    struct evl_sched_attrs attr;
    cpu_set_t cpu_set;

    // Open log file
    log_fd = open("timing_log.txt", O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (log_fd < 0) {
        perror("Failed to open log file");
        return -1;
    }

    // Initialize EVL mutex and condition variable
    ret = evl_new_mutex(&buffer_mutex);
    if (ret < 0) {
        perror("Failed to create EVL mutex");
        return -1;
    }

    ret = evl_new_cond(&buffer_cond);
    if (ret < 0) {
        perror("Failed to create EVL condition variable");
        return -1;
    }

    // Attach main process to EVL
    ret = evl_attach_self("rt-main");
    if (ret < 0) {
        perror("Failed to attach to Xenomai EVL");
        return -1;
    }

    // Set CPU affinity for real-time thread
    CPU_ZERO(&cpu_set);
    CPU_SET(CPU_CORE, &cpu_set);

    pthread_attr_t attr_thread;
    pthread_attr_init(&attr_thread);
    pthread_attr_setaffinity_np(&attr_thread, sizeof(cpu_set_t), &cpu_set);

    printf("Starting real-time task on CPU core %d...\n", CPU_CORE);

    // Create real-time and logging threads
    pthread_create(&rt_thread, &attr_thread, real_time_task, NULL);
    pthread_create(&log_thread, NULL, logging_task, NULL);

    // Wait for both threads to finish
    pthread_join(rt_thread, NULL);
    pthread_join(log_thread, NULL);

    // Close log file
    close(log_fd);

    return 0;
}

