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
    

    // Tells main to wait until both threads have been terminated
    int ret = pthread_join(log_thread, NULL);
    if (ret != 0) {
        std::cerr << "Error: log_thread join failed (" << strerror(ret) << ")\n";
        return EXIT_FAILURE;
    }

    
    int ret = pthread_join(thread, NULL);
    if (ret != 0) {
        std::cerr << "Error: timing_thread join failed (" << strerror(ret) << ")\n";
        return EXIT_FAILURE;
    }
    // Destroy the mutex
    pthread_mutex_destroy(&exec_time_mutex);

    return 0;
}