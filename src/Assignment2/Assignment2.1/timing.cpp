#include "timing.h"
#include <cerrno>
#include <cstring>
#include <iostream>

int main() {
    // Initialize the mutex 
    pthread_mutex_init(&exec_time_mutex, NULL);

    // Declares 2 POSIX threads
    pthread_t thread; 
    pthread_t log_thread;

    // Creates 1 thread for measuring the data and 1 thread for logging data
    // Throws an error if either thread failes to be created
    if(pthread_create(&thread, NULL, loop_time_measurement_function, NULL) != 0){
        std::cerr << "Error: Failed to create timing thread\n";
        return EXIT_FAILURE;
    }
    if(pthread_create(&log_thread, NULL, logger_thread, NULL) != 0){
        std::cerr << "Error: Failed to create logging thread\n";
        return EXIT_FAILURE;
    }
    

    // Tries to join the thread used for logging
    int ret_log = pthread_join(log_thread, NULL);
    if (ret_log != 0) {
        std::cerr << "Error: log_thread join failed (" << strerror(ret_log) << ")\n";
        return EXIT_FAILURE;
    }

    
    // Tries to join the thread for timing
    int ret_time = pthread_join(thread, NULL);
    if (ret_time != 0) {
        std::cerr << "Error: timing_thread join failed (" << strerror(ret_time) << ")\n";
        return EXIT_FAILURE;
    }
    // Destroy the mutex
    pthread_mutex_destroy(&exec_time_mutex);

    return 0;
}