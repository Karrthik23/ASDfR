#define _GNU_SOURCE
#include <stdio.h>
#include <pthread.h>
#include <time.h>
#include <stdint.h>
#include <math.h>
#define PERIOD_NS 1000000 
#define RUNTIME_SEC 5    

void *periodic_task(void *arg) {
    struct timespec next_time, start_time, end_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);
    FILE *log_file = fopen("timing_log.csv", "w");
    fprintf(log_file, "Iteration,Execution Time (ns)\n");
    for (int i = 0; i < (RUNTIME_SEC * 1000); i++) {
        clock_gettime(CLOCK_MONOTONIC, &start_time);
        next_time.tv_nsec += PERIOD_NS;
        if (next_time.tv_nsec >= 1000000000) {
            next_time.tv_sec++;
            next_time.tv_nsec -= 1000000000;
        }
        double sum = 0;
        for (int j = 0; j < 1000; j++) {
            sum += sqrt(j);
        }
        clock_gettime(CLOCK_MONOTONIC, &end_time);
        uint64_t exec_time_ns = (end_time.tv_sec - start_time.tv_sec) * 1000000000ULL + (end_time.tv_nsec - start_time.tv_nsec);
        fprintf(log_file, "%d,%lu\n", i, exec_time_ns);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
    }
    fclose(log_file);
    return NULL;
}

int main() {
    pthread_t thread;
    struct sched_param param;
    pthread_attr_t attr;

    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    param.sched_priority = 90;
    pthread_attr_setschedparam(&attr, &param);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    
    pthread_create(&thread, &attr, periodic_task, NULL);
    pthread_join(thread, NULL);

    return 0;
}
