#include "timing_x.h"
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
