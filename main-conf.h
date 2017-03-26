/* Include this file to create project-specific macros. Try to keep their use
in the application code, not inside libraries. */

/* keep this port under 255 because the application code is not ready to adapt
the new uint16_t port number */
#define THREAD1_PORT    170 
#define MAIN_THR_PORT   165
#define NULL_PKT_TYPE   0xFF 