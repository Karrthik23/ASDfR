#include "timing.h"





int main() {
    // Initialize the mutex 
    pthread_mutex_init(&exec_time_mutex, NULL);

    // Declares 2 POSIX threads
    pthread_t thread; 
    pthread_t log_thread;

    // Creates 1 thread for measuring the data and 1 thread for logging data
    pthread_create(&thread, NULL, periodic_task, NULL);
    pthread_create(&log_thread, NULL, log_task, NULL);

    // Tells main to wait until both threads have been terminated
    pthread_join(log_thread, NULL);
    pthread_join(thread, NULL);

    // Destroy the mutex
    pthread_mutex_destroy(&exec_time_mutex);

    return 0;
}
//Create new thread, which logs the variables