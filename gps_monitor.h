#define GPS_TASK_STACK_SIZE (2048 * 4)
#define GPS_TASK_NAME       "GPS Task"

#define TRACE 1 // Log to HST programmer UART
#define DEBUG 2 // Log to debug.log file
#define UART  4 // Log to serial

#define Output(...)                                                            \
  do {                                                                         \
    snprintf(logbuf, 1020, __VA_ARGS__);                                       \
    if (config.loglevel & TRACE) Trace(1, logbuf);                             \
    strcat(logbuf, "\r\n");                                                    \
    if (config.loglevel & DEBUG) {                                             \
      int32_t logfile = API_FS_Open(GPS_LOG_FILE, FS_O_RDWR | FS_O_APPEND, 0); \
      API_FS_Write(logfile, (uint8_t*)logbuf, strlen(logbuf));                 \
      API_FS_Close(logfile);                                                   \
    }                                                                          \
    if (config.loglevel & UART) UART_Write(UART1, logbuf, strlen(logbuf));     \
  } while (0)
