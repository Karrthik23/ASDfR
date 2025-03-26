#include "timing_x.h"

int main() {
    int ret;
    pthread_t rt_thread;
    struct evl_sched_attrs attr;
    cpu_set_t cpu_set;

    // set scheduler config
    attr.sched_policy = SCHED_FIFO;
    attr.sched_priority = 80;    // fairly high prio

    // try attaching main thread to evl
    ret = evl_attach_self("rt-main");
    if (ret < 0) {
        perror("failed to attach to xenomai evl");
        return -1;
    }

    // limit to specific cpu core
    CPU_ZERO(&cpu_set);
    CPU_SET(CPU_CORE, &cpu_set);

    pthread_attr_t attr_thread;                      //thread attr object
    pthread_attr_init(&attr_thread);                //init with default values
    pthread_attr_setaffinity_np(&attr_thread, sizeof(cpu_set_t), &cpu_set);   // Set CPU  affinity
    

    printf("Starting real-time task on CPU core %d...\n", CPU_CORE);  // notify

    // real time tthread
    pthread_create(&rt_thread, &attr_thread, real_time_task, NULL);
    pthread_join(rt_thread, NULL);    // wait for it to end

    //open output file
    int log_fd = open(FILE_NAME, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (log_fd < 0) {
        perror("couldn't open log file");
        return -1;
    }

    //log all data to csv
    char log_buffer[256];
    for (int i = 0; i < (RUN_TIME * 1000); i++) {
        snprintf(log_buffer, sizeof(log_buffer), "Iteration %d - Execution Time: %.3f ms\n", i, exec_times[i]);
        write(log_fd, log_buffer, strlen(log_buffer));  // write line
    }

    //Close file
    close(log_fd);

    return 0;
}