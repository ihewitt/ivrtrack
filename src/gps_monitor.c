/*
 * IVRTrac GPS Tracker software for A9G board
 * © Ivor Hewitt 2020, GPL2
 *
 * Features:
 * sends tk102 "style" location updates.
 * requires either a custom server (https://github.com/ihewitt/gps-server) or a modified GPS
 * format string to use generic server.
 * Uses an SSD1306 OLED, power circuit "patch", and preferably
 * power source board. see (https://blog.ivor.org/2020/10/tracking-running-part-2.html)
 */

/*
 * Update SERVER_ADDRESS and SERVER_PORT as suitable
 */

// TODO check processor speed options for battery life and stability.
//      gps/gprs seemed unstable over several hours (5+) with speed changes.
// TODO restructure. started
// TODO add FOTA upgrade logic

#include <api_hal_pm.h>
#include <api_hal_uart.h>
#include <api_hal_watchdog.h>

#include <api_event.h>
#include <api_fota.h>
#include <api_fs.h>
#include <api_info.h>
#include <api_key.h>
#include <api_network.h>
#include <api_os.h>
#include <api_sms.h>
#include <api_socket.h>

#include <stdlib.h>

//#include "buffer.h" //just use a single chunk for now
#include "gps.h"
#include "gps_parse.h"
#include "ntp.h"

#include "fsutil.h"
#include "ledutil.h"
#include "oled.h"

#include "gps_monitor.h"

// some intersting looking functions not exposed in the csdk library
//#define hal_SysRestart CSDK_FUNC(hal_SysRestart)
//#define hal_SysSoftReset CSDK_FUNC(hal_SysSoftReset)
//#define hal_SysGetFreq CSDK_FUNC(hal_SysGetFreq)
//#define boot_SysGetFreq CSDK_FUNC(boot_SysGetFreq)
//#define hal_SysUsbHostEnable CSDK_FUNC(hal_SysUsbHostEnable)

// Whether to attempt fastfix based on last known position?
#define GPS_FASTFIX
//#define GPS_AGPSFIX

#undef VERBOSE // Excessive logging

#define CONFIG_FILE_NAME  "/t/config.txt"
#define GPS_LOG_FILE_PATH "/t/gps-current.log"
#define GPS_LOG_FILE      "/t/debug.log"

#define ROLL_SIZE            1024 * 256 // 256k logfiles
#define NMEA_INTERVAL        10
#define MAIN_TASK_STACK_SIZE (2048 * 2)
#define MAIN_TASK_PRIORITY   0
#define MAIN_TASK_NAME       "Main Task"

#define SERVER_ADDRESS   ""
#define SERVER_PORT      8181
#define SOFT_VERSION     "V0.5.3"
#define FOTA_HTTP_SERVER "https://" SERVER_ADDRESS "/fota/%stonew.pack"

const char timeServer[] = "time.nist.gov"; // time.nist.gov NTP server

HANDLE mainTaskHandle = NULL;

typedef struct {
  char apn[PDP_APN_MAX_LENGTH];            // Network info
  char apnuser[PDP_USER_NAME_MAX_LENGTH];  //
  char apnpwd[PDP_USER_PASSWD_MAX_LENGTH]; //
  int  gps;                                // Store GPS every
  int  upload;                             // Upload data every
  char server[128];                        // Tracking server
  char server_ip[16];                      //
  int  port;                               //
  int  loglevel;                           // Log to debug,file or uart
  int  screentime;                         // Turn off screen time
} config_t;

// global config with basic defaults
//"everywhere","eesecure","secure",
config_t config = {
    .apn        = "pp.vodafone.co.uk",
    .apnuser    = "wap",
    .apnpwd     = "wap",                // apn
    .gps        = 30,                   // gps store every 30 seconds
    .upload     = 300,                  // data upload every 5 minutes
    .server     = SERVER_ADDRESS,       // tracking server
    .port       = SERVER_PORT,          // server data port
    .loglevel   = DEBUG | TRACE | UART, // boot on full logging
    .screentime = 60                    // screen off time
};

// Store last known state
typedef struct {
  float      latitude;
  float      longitude;
  float      altitude;
  RTC_Time_t time;
} state_t;
state_t state;

bool nettime = false; // Have we got a nettime?

bool fota_on     = false;
bool gpsReady    = false; // Global flag to prevent gps logging until all setup
bool initialised = false; // Global flag to pause run until chipset initialised
bool netAttach   = false; // have we attached to the mobile network

bool dat_on  = false; // have we got gprs
bool gps_on  = false; // have we got lock (add sat num)
bool mob_on  = false; // have we registered at all?
bool dsk_on  = false; // data to write
int  gps_num = 0;

// Save verbose diagnostics
uint8_t logbuf[1024]; // TODO tidy up!

bool CreateLog() {
  int32_t logfile = API_FS_Open(GPS_LOG_FILE, FS_O_RDWR | FS_O_APPEND, 0);
  API_FS_Close(logfile);
  return (logfile > 0);
}

// TODO change to a callback helper iterator
bool ListDirsRoot(char* path) {
  Output("SDCard contents for %s", path);
  char buff[64];

  Dir_t* dir = API_FS_OpenDir(path);
  if (!dir || dir->fs_index < 0) {
    Output("Open fail %s", path);
    return false;
  }
  Dirent_t* dirent = NULL;
  while ((dirent = API_FS_ReadDir(dir))) {
    snprintf(buff, sizeof(buff), "%s/%s", path, dirent->d_name);

    int32_t fd  = API_FS_Open(buff, FS_O_RDONLY, 0);
    int32_t len = API_FS_GetFileSize(fd);
    API_FS_Close(fd);
    Output("file: %s length %d", buff, len);
  }
  API_FS_CloseDir(dir);
  return true;
}

#define BUFFER_SIZE 1024 // What's a sane "buffer"? 870bytes is ~5min
char* sdbuffer;

#define SMS_STORE SMS_STORAGE_SIM_CARD

uint8_t imei[32];
void    ImeiRead() {
  memset(imei, 0, sizeof(imei));
  while (!INFO_GetIMEI(imei)) OS_Sleep(100);

  Output("IMEI: %s", imei);
}

// Used to hold screen current state message
#define MSG_RUN "Running"

char stateMsg[32] = {0}; // TODO tidy

void updateScreen(char* msg) {
  OLED_clear();

  if (gps_on) drawIcon(2, 0, ICON_GPS);
  if (dat_on) drawIcon(5, 0, ICON_DATA);
  if (mob_on) drawIcon(8, 0, ICON_SIGNAL);
  if (dsk_on) drawIcon(0, 6, ICON_DISK);

  uint8_t tmp[32];

  sprintf(tmp, "%2d", gps_num);
  drawString(0, 0, tmp);

  TIME_System_t time;
  TIME_GetLocalTime(&time);
  sprintf(tmp, "%02d:%02d", time.hour, time.minute);
  drawString(11, 0, tmp);

  uint8_t  percent;
  uint16_t v = PM_Voltage(&percent);
  sprintf(tmp, "%dmV %d%%", v, percent);
  drawString(4, 6, tmp);

  if (percent > 80) drawIcon(14, 6, ICON_BAT_H);
  else if (percent > 50)
    drawIcon(14, 6, ICON_BAT_M);
  else
    drawIcon(14, 6, ICON_BAT_L);

  drawString(0, 2, msg);

  OLED_show();
}

void refreshScreen() { updateScreen(stateMsg); }

// Turn off oled before shutdown
bool StoreCache(uint8_t*);
void PowerOff() {
  OLED_off();
  StoreCache(sdbuffer); // Try to save any non uploaded data
  PM_ShutDown();
}

// TODO rework this mess
uint8_t* GetPair(uint8_t** key, uint8_t** val, uint8_t* line) {
  uint8_t* eol = strchr(line, '\n');
  uint8_t* sep = strchr(line, ':');
  if (eol) *eol = 0;
  if (sep) {
    *sep = 0;                             // terminate
    *key = line;                          // key is start
    while (**key == ' ') *key = *key + 1; // skip any lead
    *val = sep + 1;
    while (**val == ' ') *val = *val + 1; // skip space
  }

  //    if (eol) eol++; //will either be start of next line or null
  return eol;
}

// TODO rework this mess
bool ReadConfig() {
  int32_t  fd;
  int32_t  ret;
  uint8_t* path = CONFIG_FILE_NAME;
  int32_t  len;

  fd = API_FS_Open(path, FS_O_RDONLY, 0);
  if (fd < 0) {
    Output("[ReadCfg] Open file failed:%s, %d", path, fd);
    return false;
  }

  len = API_FS_GetFileSize(fd);
  Output("File size %s, %d", path, len);

  uint8_t* text = OS_Malloc(len + 2);
  memset(text, 0, len + 2);
  uint32_t lenread = API_FS_Read(fd, text, len);
  strcat(text, "\n");
  API_FS_Close(fd);

  if (lenread < 1) {
    Output("Unable to read %s, %d", path, lenread);
    ret = false;
  } else {

    char* line = text;
    do {
      uint8_t* key;
      uint8_t* val;
      line = GetPair(&key, &val, line); // get key value
      if (!line || !key) break;

      if (strcmp(key, "apn") == 0) strncpy(config.apn, val, PDP_APN_MAX_LENGTH);
      else if (strcmp(key, "apnuser") == 0)
        strncpy(config.apnuser, val, PDP_USER_NAME_MAX_LENGTH);
      else if (strcmp(key, "apnpwd") == 0)
        strncpy(config.apnpwd, val, PDP_USER_PASSWD_MAX_LENGTH);
      else if (strcmp(key, "gps") == 0)
        config.gps = strtol(val, 0, 0);
      else if (strcmp(key, "upload") == 0)
        config.upload = strtol(val, 0, 0);
      else if (strcmp(key, "server") == 0)
        strncpy(config.server, val, 128);
      else if (strcmp(key, "serverip") == 0)
        strncpy(config.server_ip, val, 16);
      else if (strcmp(key, "port") == 0)
        config.port = strtol(val, 0, 0);
      else if (strcmp(key, "log") == 0)
        config.loglevel = strtol(val, 0, 0);
      else if (strcmp(key, "screentime") == 0)
        config.screentime = strtol(val, 0, 0);

    } while (line++);

    Output("[Config] Server:%s APN:%s User:%s Pass:%s GPS frq:%d, Upload:frq"
           "%d, log: %d",
           config.server, config.apn, config.apnuser, config.apnpwd, config.gps, config.upload, config.loglevel);

    ret = true;
  }
  OS_Free(text);
  return ret;
}

bool WriteConfig() {
  int32_t  fd;
  int32_t  ret  = 0;
  uint8_t* path = CONFIG_FILE_NAME;

  Output("Update config file %s", path);

  uint8_t* text = OS_Malloc(512); // more than enough
  if (!text) {
    Output("WriteConfig: Malloc failure.");
    return false;
  }

  snprintf(text, 512,
           "apn: %s\n"
           "apnuser: %s\n"
           "apnpwd: %s\n"
           "gps: %d\n"
           "upload: %d\n"
           "server: %s\n"
           "serverip: %s\n"
           "port: %d\n"
           "log: %d\n"
           "screentime: %d\n",
           config.apn, config.apnuser, config.apnpwd, config.gps, config.upload, config.server, config.server_ip, config.port,
           config.loglevel, config.screentime);

  fd = API_FS_Open(path, FS_O_RDWR | FS_O_CREAT | FS_O_TRUNC, 0);
  if (fd < 0) {
    Output("Write file failed:%s, %d", path, fd);
    OS_Free(text);
    return false;
  }
  ret = API_FS_Write(fd, (uint8_t*)text, strlen(text));
  API_FS_Flush(fd);
  API_FS_Close(fd);
  OS_Free(text);

  if (ret <= 0) return false;
  return true;
}

void RollLog() {
  RTC_Time_t time;
  uint8_t    newpath[128];
  uint8_t*   path = (uint8_t*)GPS_LOG_FILE_PATH;

  TIME_GetRtcTime(&time);
  sprintf(newpath, "%s/gps-%04d%02d%02d-%02d%02d%02d.log", FS_TFLASH_ROOT, time.year, time.month, time.day, time.hour, time.minute,
          time.second);
  API_FS_Rename(path, newpath);
  Output("GPS logfile rolled to %s", newpath);
}

// Flush any cache to SD
bool SaveToSDLog() {
  int32_t  fd;
  uint8_t* path = (uint8_t*)GPS_LOG_FILE_PATH;
  bool     ret  = true;

  fd = API_FS_Open(path, FS_O_RDWR | FS_O_APPEND, 0);

  if (fd < 0) ret = false;
  else if (API_FS_Write(fd, (uint8_t*)sdbuffer, strlen(sdbuffer)) < 0)
    ret = false;

  API_FS_Flush(fd);
  int64_t logsize = API_FS_GetFileSize(fd);
  API_FS_Close(fd);

  if (!ret) { Output("SaveToSDLog: Open gps file %s failed:%d", path, fd); }

  if (logsize > ROLL_SIZE) RollLog();

  return ret;
}

// Remove logfile, although not essential with a large SD card.
// gpsdata - remove gps-* logs, state and cache
bool ClearSD(bool gpsdata) {
  API_FS_Delete(GPS_LOG_FILE); // debug logfile

  if (gpsdata) {
    char buff[64];

    API_FS_Delete("/t/state");        // Last known state
    API_FS_Delete("/t/cache");        // Cached GPS data
    API_FS_Delete(GPS_LOG_FILE_PATH); // Current GPS log

    // Now remove all rolled logfiles
    Dir_t* dir = API_FS_OpenDir("/t");
    if (!dir || dir->fs_index < 0) {
      Output("Open fail /t");
      return false;
    }
    Dirent_t* dirent = NULL;
    while ((dirent = API_FS_ReadDir(dir))) {
      snprintf(buff, sizeof(buff), "/t/%s", dirent->d_name);

      if (strncmp(dirent->d_name, "gps", 3) == 0) {
        API_FS_Delete(buff);
        Output("file: %s deleted", buff);
      }
    }
    API_FS_CloseDir(dir);
    // Since state data removed, need to reboot
    OLED_on();
    updateScreen("Wiped, rebooting\nin 6 seconds...");
    OS_Sleep(6000);
    PM_Restart();
  }
  return true;
}

//
// Cache gps, (e.g. out of mobile signal for a period, so store to SD cache.)
//
bool StoreCache(uint8_t* buffer) {
  bool ret;

  int len = strlen(buffer);
  if (len == 0) // nothing to do
    return true;

  int32_t fs = API_FS_Open("/t/cache", FS_O_RDWR | FS_O_APPEND | FS_O_CREAT, 0);
  if (fs <= 0) {
    ret = false;
  } else {
    if (API_FS_Write(fs, (uint8_t*)buffer, len) > 0) {
      dsk_on = true; // caching to sd icon
      ret    = true;
    } else
      ret = false;
    API_FS_Close(fs);
  }
  return ret;
}

bool CacheGPS(char* str) {

  if (strlen(sdbuffer) + strlen(str) > BUFFER_SIZE) {
    // Full unflushed buffer, try to backup
    if (!StoreCache(sdbuffer)) Output("Unable to cache, discarded");
    else
      Output("Cached unsent to SD");

    sdbuffer[0] = 0;
  }
  strcat(sdbuffer, str);

  // update last known state
  int32_t fs = API_FS_Open("/t/state", FS_O_RDWR | FS_O_CREAT | FS_O_TRUNC, 0);
  API_FS_Write(fs, (uint8_t*)&state, sizeof(state_t));
  API_FS_Close(fs);

  return true;
}

// Does registering with known network speed up the initial link?
/*void Register()
{
    // MMI 01 : Current: 02.03.04.01.05.0f mode 1
    // UK 234, 15 Vodafone
    Trace(2, "register on preferred network");
    Network_Register((uint8_t[]){2, 3, 4, 1, 5, 0xf}, NETWORK_REGISTER_MODE_MANUAL_AUTO);
}*/

int nofixcount = 0;
int fixcount   = 0;

void HandleGps() {
  static uint8_t gpstimer = 0;
  static int     boottry  = 0;

  GPS_Info_t* gpsInfo = Gps_GetInfo();

  // show fix info
  uint8_t isFixed = 0;
  for (int i = 0; i < GPS_PARSE_MAX_GSA_NUMBER; ++i) {
    if (gpsInfo->gsa[i].fix_type > isFixed) { isFixed = gpsInfo->gsa[i].fix_type; }
  }

  int num;
  if (gps_on) {
    num = gpsInfo->gga.satellites_tracked;
  } else {
    num = gpsInfo->gsv[0].total_sats;
  }

  bool last_state = gps_on;
  if (isFixed > 1) {
    gps_on     = true;
    nofixcount = 0;
    fixcount++;
    boottry = 0;
  } else {
    gps_on = false;
    if (gpsInfo->gga.satellites_tracked == 0) { nofixcount++; }
  }

  // convert unit ddmm.mmmm to degree(°)
  int   temp = (int)(gpsInfo->rmc.latitude.value / gpsInfo->rmc.latitude.scale / 100);
  float latitude =
      temp + (float)(gpsInfo->rmc.latitude.value - temp * gpsInfo->rmc.latitude.scale * 100) / gpsInfo->rmc.latitude.scale / 60.0;
  temp            = (int)(gpsInfo->rmc.longitude.value / gpsInfo->rmc.longitude.scale / 100);
  float longitude = temp + (float)(gpsInfo->rmc.longitude.value - temp * gpsInfo->rmc.longitude.scale * 100) /
                               gpsInfo->rmc.longitude.scale / 60.0;

  float altitude = (float)gpsInfo->gga.altitude.value / gpsInfo->gga.altitude.scale;

  // mph
  float speed = (gpsInfo->vtg.speed_kph.value / gpsInfo->vtg.speed_kph.scale / 100) / 1.609;
  // min/mile
  float pace = speed ? 60 / speed : 0;

  // Keep state cached.
  if (isFixed > 1) {
    state.latitude  = latitude;
    state.longitude = longitude;
    state.altitude  = altitude;
    // Store last lock time, use to decide if to discard?
    state.time.year   = gpsInfo->rmc.date.year + 2000; // gps returns 2 char?
    state.time.month  = gpsInfo->rmc.date.month;
    state.time.day    = gpsInfo->rmc.date.day;
    state.time.hour   = gpsInfo->rmc.time.hours;
    state.time.minute = gpsInfo->rmc.time.minutes;
    state.time.second = gpsInfo->rmc.time.seconds;
  }

  if ((last_state != gps_on) || (gps_num != num)) {
    gps_num = num;
    refreshScreen();
  }

  gpstimer += NMEA_INTERVAL; // nmea is every 10 seconds.
  // But, only save every x seconds else discard.
  if (gpstimer < config.gps) return;
  gpstimer = 0;

  // Now start formatting
  char* isFixedStr = 0;
  if (isFixed == 2) isFixedStr = "2D fix";
  else if (isFixed == 3) {
    if (gpsInfo->gga.fix_quality == 2) isFixedStr = "3D/DGPS fix";
    else // if (gpsInfo->gga.fix_quality == 1)
      isFixedStr = "3D fix";
  } else {
    isFixedStr = "no fix";
  }

  char datestr[14];
  sprintf(datestr, "%02d%02d%02d%02d%02d%02d", gpsInfo->rmc.date.year, gpsInfo->rmc.date.month, gpsInfo->rmc.date.day,
          gpsInfo->rmc.time.hours, gpsInfo->rmc.time.minutes, gpsInfo->rmc.time.seconds);

  uint8_t  percent;
  uint16_t v = PM_Voltage(&percent);

#ifdef VERBOSE
  char satstr[128];
  char locstr[64];
  char batstr[12];

  snprintf(batstr, sizeof(batstr), "%dmV, %d%%", v, percent);
  snprintf(satstr, sizeof(satstr),
           "gsa: %d,%d "
           "qu: %d trk: %d tot: %d fix: %s %d",
           gpsInfo->gsa[0].fix_type, gpsInfo->gsa[1].fix_type, gpsInfo->gga.fix_quality, gpsInfo->gga.satellites_tracked,
           gpsInfo->gsv[0].total_sats, isFixedStr, isFixed);
  snprintf(locstr, sizeof(locstr), "Lat:%3.5f, Lon:%3.5f, alt:%5.2f", latitude, longitude, altitude);

  Output("GPS: [%s] %s %s %s", datestr, satstr, locstr, batstr);
#endif

  // TODO rework logic, can we get a "we have good enough information fix?"
  char fix = (isFixed > 1) ? 'A' : 'V'; // Use the format lots of trackers use

  char message[128]; // more than enough, usually ~78
  snprintf(message, sizeof(message),
           "*IVR,%s,"
           "%s," // YYMMDDMMHHSS
           "%3.5f,%3.5f,"
           "%c,%0.2f,"
           "%5.2f,%s,%d#\n",
           imei, datestr, latitude, longitude, fix, pace, altitude, isFixedStr, percent);

  CacheGPS(message);
}

// TODO make this smarter/slicker.
// but. this will do for now.
// TODO add file management
bool handleCommand(bool  verbose, // verbose - sms or uart
                   char* command, // will be changed/modified by tokeniser
                   char* response // buffer for short reply
) {
  // get state. gprs, battery, gps.
  if (strnicmp(command, "help", 4) == 0) {
    sprintf(response, "Commands: info, poweroff, reboot, log, clear, apn <s> <u> "
                      "<p>, frq <gps> <up>\n");
  } else if (strnicmp(command, "info", 4) == 0) {
    uint8_t  percent;
    uint8_t  status;
    uint16_t v = PM_Voltage(&percent);
    Network_GetActiveStatus(&status);
    GPS_Info_t* gpsInfo = Gps_GetInfo();

    sprintf(response,
            "GPRS %d, Power %dmV %d%%, "
            "FIX %d, "
            "GPS %ds, UP %ds",
            status, v, percent, gpsInfo->gga.satellites_tracked, config.gps, config.upload);

  } else if (strnicmp(command, "poweroff", 8) == 0) // shutdown
  {
    // Callback to shutdown so event removed from queue
    strcpy(response, "Poweroff in 5s");
    OS_StartCallbackTimer(mainTaskHandle, 5000, PowerOff, NULL);
  } else if (strnicmp(command, "reboot", 5) == 0) // Reboot
  {
    strcpy(response, "Reboot in 5s");
    OS_StartCallbackTimer(mainTaskHandle, 5000, PM_Restart, NULL);
  } else if (strnicmp(command, "apn ", 3) == 0) // iupdate apn info
  {
    char* ptr    = &command[3];
    char* server = strsep(&ptr, " ,\n");
    char* user   = strsep(&ptr, " ,\n");
    char* pwd    = strsep(&ptr, " ,\n");
    if (server && user && pwd) {
      strcpy(config.apn, server);
      strcpy(config.apnuser, user);
      strcpy(config.apnpwd, pwd);
    }
    WriteConfig();

    Network_PDP_Context_t context;
    strcpy(context.apn, config.apn);
    strcpy(context.userName, config.apnuser);
    strcpy(context.userPasswd, config.apnpwd);
    Network_StartActive(context);
    sprintf(response, "APN updated: %s", config.apn);
  } else if (strnicmp(command, "frq ", 3) == 0) // Change save/upload times
  {
    char* ptr     = &command[3];
    char* gps     = strsep(&ptr, " ,\n");
    char* up      = strsep(&ptr, " ,\n");
    config.gps    = atol(gps);
    config.upload = atol(up);
    WriteConfig();

    sprintf(response, "Times updated: %d %d", config.gps, config.upload);
  } else if (strnicmp(command, "clear", 5) == 0) // Reset logfiles.
  {
    sprintf(response, "logs cleared");
    ClearSD(true); // full wipe and reboot
  } else if (strnicmp(command, "loglevel ", 9) == 0) {
    char* ptr       = &command[9];
    char* lev       = strsep(&ptr, " ,\n");
    config.loglevel = atol(lev);
    WriteConfig();
    sprintf(response, "Log level %d", config.loglevel);
  } else if (strnicmp(command, "log", 3) == 0) // read debug log and dump
  {
    char buffer[1024];

    int32_t logfile = API_FS_Open(GPS_LOG_FILE, FS_O_RDONLY, 0);

    while (!API_FS_IsEndOfFile(logfile)) {
      int32_t len = API_FS_Read(logfile, buffer, 1024);
      if (len > 0) {
        UART_Write(UART1, buffer, len);
        WatchDog_KeepAlive();
      } else {
        break;
      }
    }
    API_FS_Close(logfile);
  } else
    return false; // not handled

  return true; // handled
}

static bool button_down = false;
void        KeyHandler() {
  static uint32_t button_time = 0;
  if (button_time > 20) { // seems to be a button up on poweron
    button_time = 0;
    return;
  }
  button_time++;
  int sec = (button_time / 2);

  if (button_down) {
    // Power off
    if (sec > 5) // shutdown
    {
      OLED_on();
      updateScreen("Power off!");
      OS_Sleep(3000); // Quickly display message
      PowerOff();
    } else if (sec > 1) // Just warn
    {
      OLED_on();
      char saveMsg[32];
      sprintf(saveMsg, "Power off in %d", 6 - sec);
      updateScreen(saveMsg);
    }
    OS_StartCallbackTimer(mainTaskHandle, 500, KeyHandler, NULL); // Keep ticking
  } else {
    {
      button_time = 0;
      bool oled   = OLED_state();
      if (oled) {
        OLED_off();
      } else {
        OLED_on();
        refreshScreen();
      }
    }
  }
}

void EventNetwork(API_Event_t* pEvent) {
  switch (pEvent->id) {
      // TODO move network logic into network handler
    case API_EVENT_ID_NETWORK_REGISTER_SEARCHING: //
      Output("network register searching");
      break;

    case API_EVENT_ID_NETWORK_REGISTER_DENIED: // Is this happening when changing towers/networks?
      Output("network register denied");       // any way to resolve this?
    case API_EVENT_ID_NETWORK_REGISTER_NO:
      Output("network register no");
      mob_on = false;
      refreshScreen();
      break;

    case API_EVENT_ID_NETWORK_REGISTERED_HOME:
    case API_EVENT_ID_NETWORK_REGISTERED_ROAMING: {
      Output("Registered"); // now attach

      Network_Register_Mode_t mode;
      uint8_t                 operatorId[6];
      Network_GetCurrentOperator(operatorId, &mode);
      Output("Current operator: %02x.%02x.%02x.%02x.%02x.%02x", //
             operatorId[0], operatorId[1], operatorId[2], operatorId[3], operatorId[4], operatorId[5]);

      mob_on = true;
      refreshScreen();

      uint8_t status = 0; // do we need to attach or reactivate?
      if (Network_GetAttachStatus(&status)) Output("GetAttach %d vs attachflag %d", status, netAttach);
      if (status == 0) {
        if (!Network_StartAttach()) Trace(1, "network attach fail");

        break;
      }
      // drop through
      // break;
    }
    case API_EVENT_ID_NETWORK_ATTACHED: {
      if (pEvent->id == API_EVENT_ID_NETWORK_ATTACHED) Output("Attached");
      netAttach = true;

      Output("Activate %s %s %s", config.apn, config.apnuser, config.apnpwd);
      Network_PDP_Context_t context;
      strcpy(context.apn, config.apn);
      strcpy(context.userName, config.apnuser);
      strcpy(context.userPasswd, config.apnpwd);

      if (!Network_StartActive(context)) Output("false from StartActive");
      break;
    }
    case API_EVENT_ID_NETWORK_DEACTIVED:
    case API_EVENT_ID_NETWORK_ACTIVATE_FAILED:
      Output("Activated off");
      dat_on = false;
      refreshScreen();
      break;

    case API_EVENT_ID_NETWORK_DETACHED:
      netAttach = false;
      Output("Detached"); // reattach?
      //    Network_StartAttach(); //?is this automatic?
      break;

    case API_EVENT_ID_NETWORK_ATTACH_FAILED:
      netAttach = false;
      Output("Attach failed."); // reattach?
      // Network_StartDetach(); // ?is this automatic?
      break;

    case API_EVENT_ID_NETWORK_ACTIVATED: // go! we have gprs connection
      dat_on = true;                     // data connection up
      Output("network activate success, connect");
      refreshScreen();

      // Lookup server ip once if we need it.
      if (strlen(config.server_ip) == 0) {
        memset(config.server_ip, 0, sizeof(config.server_ip));
        if (DNS_GetHostByName2(config.server, config.server_ip) != 0) {
          Output("Get Host fail");
          break;
        }
      }
      break;

    case API_EVENT_ID_NETWORK_CELL_INFO: {
      uint8_t             number   = pEvent->param1;
      Network_Location_t* location = (Network_Location_t*)pEvent->pParam1;
      break;
    }

    default: break;
  }
}

void EventDispatch(API_Event_t* pEvent) {
  switch (pEvent->id) {

    case API_EVENT_ID_SYSTEM_READY:
      Output("system initialize complete");
      initialised = true;
      break;

    case API_EVENT_ID_NO_SIMCARD:
      Output("!!NO SIM CARD%d!!!!", pEvent->param1);
      sprintf(stateMsg, "NO SIM CARD!");
      refreshScreen();
      break;

    case API_EVENT_ID_SIGNAL_QUALITY:
      // param1: SQ(0~31,99(unknown)), param2:RXQUAL(0~7,99(unknown))  (RSSI = SQ*2-113)
      break;

    case API_EVENT_ID_POWER_INFO:
      ////param1: (PM_Charger_State_t<<16|charge_level(%)) , param2:
      ///(PM_Battery_State_t<<16|battery_voltage(mV))
      break;

    case API_EVENT_ID_NETWORK_GOT_TIME: // Do we need to tell rtc/gps?
      nettime = true;
      RTC_Time_t time;
      TIME_GetRtcTime(&time);
      Output("GSM Time: %04d%02d%02d-%02d%02d%02d", //
             time.year, time.month, time.day, time.hour, time.minute, time.second);
      break;

    case API_EVENT_ID_KEY_DOWN: // Start a keypress handler
      if (pEvent->param1 == KEY_POWER && !button_down) {
        button_down = true;
        OS_StartCallbackTimer(mainTaskHandle, 500, KeyHandler, NULL);
      }
      break;

    case API_EVENT_ID_KEY_UP: // Toggle screen on and off
      if (pEvent->param1 == KEY_POWER) { button_down = false; }
      break;

    case API_EVENT_ID_NETWORK_AVAILABEL_OPERATOR: break;

    case API_EVENT_ID_NETWORK_REGISTER_SEARCHING:
    case API_EVENT_ID_NETWORK_REGISTER_DENIED:
    case API_EVENT_ID_NETWORK_REGISTER_NO:
    case API_EVENT_ID_NETWORK_REGISTERED_HOME:
    case API_EVENT_ID_NETWORK_REGISTERED_ROAMING:
    case API_EVENT_ID_NETWORK_ATTACHED:
    case API_EVENT_ID_NETWORK_DEACTIVED:
    case API_EVENT_ID_NETWORK_ACTIVATE_FAILED:
    case API_EVENT_ID_NETWORK_DETACHED:
    case API_EVENT_ID_NETWORK_ATTACH_FAILED:
    case API_EVENT_ID_NETWORK_ACTIVATED: EventNetwork(pEvent); break;

    case API_EVENT_ID_SMS_SENT: Output("Send Message Success"); break;

    case API_EVENT_ID_SMS_RECEIVED:
      Output("received message");
      SMS_Encode_Type_t encodeType = pEvent->param1;
      // uint32_t contentLength = pEvent->param2;
      uint8_t* header  = pEvent->pParam1;
      uint8_t* content = pEvent->pParam2;

      Output("message header:%s", header);
      char number[64]; // Ugly as hell, rework this!
      memset(number, 0, sizeof(number));
      strncpy(number, &header[1], strcspn(&header[1], "\""));

      if (encodeType == SMS_ENCODE_TYPE_ASCII) {
        char reply[256];
        reply[0] = 0;
        Output("message content:%s from %s", content, number);

        if (handleCommand(false, content, reply)) {
          if (strlen(reply)) SMS_SendMessage(number, reply, strlen(reply), SIM0);
        }
      }
      break;

    case API_EVENT_ID_SMS_LIST_MESSAGE: {
      SMS_Message_Info_t* messageInfo = (SMS_Message_Info_t*)pEvent->pParam1;
      Output("message header index:%d,status:%d,number "
             "type:%d,number:%s,time:\"%u/%02u/%02u,%02u:%02u:%02u+%02d\"",
             messageInfo->index, messageInfo->status, messageInfo->phoneNumberType, messageInfo->phoneNumber,
             messageInfo->time.year, messageInfo->time.month, messageInfo->time.day, messageInfo->time.hour,
             messageInfo->time.minute, messageInfo->time.second, messageInfo->time.timeZone);

      if (messageInfo->data) {
        char message[256];
        memset(message, 0, 256);
        strncpy(message, messageInfo->data, messageInfo->dataLen);
        Output("message content len:%d,data:%s\n", messageInfo->dataLen, message);
        // need to free data here
        OS_Free(messageInfo->data);
      }
      // Is it just buggy and needs kick?
      for (int i = 0; i < 5; i++) {
        if (!SMS_DeleteMessage(messageInfo->index, SMS_STATUS_ALL, SMS_STORE)) {
          Output("Cant delete SMS retry %d", i);
          OS_Sleep(1000);
        } else {
          Output("Deleted.");
          break;
        }
      }
      break;
    }

    case API_EVENT_ID_GPS_UART_RECEIVED:
      // Trace(1, "received GPS data,length:%d, data:%s", pEvent->param1, pEvent->pParam1);
      GPS_Update(pEvent->pParam1, pEvent->param1);
      if (gpsReady) // Ignore until we're ready
        HandleGps();
      break;

    case API_EVENT_ID_UART_RECEIVED:
      if (pEvent->param1 == UART1) {
        // TODO rework
        Output("uart received data, length:%d", pEvent->param2);
        if (pEvent->param2 && pEvent->pParam1) {
          uint8_t data[pEvent->param2 + 1];
          data[pEvent->param2] = 0;
          memcpy(data, pEvent->pParam1, pEvent->param2);
          char reply[256];
          reply[0] = 0;
          if (handleCommand(true, data, reply)) { UART_Write(UART1, reply, strlen(reply)); }
        }
      }
      break;
    default: break;
  }
}

/*
 * Connect and send data, synchronous connection code
 */
bool UploadToServer(char* data) {
  bool ret = true;
  int  len = strlen(data);

  // Connect
  int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (fd < 0) {
    Output("Create socket fail"); // not sure how this could ever fail?
    return false;                 // seems it can
  }

  LED_data(true);

  struct sockaddr_in sockaddr;
  memset(&sockaddr, 0, sizeof(sockaddr));
  sockaddr.sin_family = AF_INET;
  sockaddr.sin_port   = htons(config.port);
  inet_pton(AF_INET, config.server_ip, &sockaddr.sin_addr);

  int retval = connect(fd, (struct sockaddr*)&sockaddr, sizeof(struct sockaddr_in));

  if (retval < 0) {
    Output("Socket connect fail %d ip:%s, port:%d", retval, config.server_ip, config.port);
    ret = false;
  } else {
    // Tokenise and write each line
    char* curs = data;
    char* line;
    int   llen;
    while ((line = strsep(&curs, "\n")) && (llen = strlen(line))) {
#ifdef VERBOSE
      Output("Writing %d bytes. '%s'", llen, line);
#endif
      retval = send(fd, line, llen, 0);
      if (retval < 0) {
        Output("socket write fail:%d", retval);
        line[llen] = '\n'; // detokenize this line
        curs       = line; // set cursor back to line start
        ret        = false;
        break;
      }
    }
    if (!ret && curs != data) // If failed but still copied some, remove from buffer
    {
      memcpy(data, curs, len - (curs - data));
    }
    close(fd);
  }
  LED_data(false);
  return ret;
}

/*
 * initialise gps params and start logging activity
 */
void InitialiseGPS() {
  Output("Start GPS task");
  gpsReady = false;

  GPS_Close();
  GPS_Init(); // just setups a buffer.

  GPS_Info_t* gpsInfo          = Gps_GetInfo();
  gpsInfo->rmc.latitude.value  = 0;
  gpsInfo->rmc.longitude.value = 0;

  GPS_Open(NULL); // Enable GPS UART

  // Do we need this?
  // wait for gps start up, or gps will not respond
  Output("Wait for GPS start");
  while (gpsInfo->rmc.latitude.value == 0) OS_Sleep(1000);

  bool    ret = false;
  uint8_t retries;

  // Supposed to support SBAS so lets try that.
  if (!GPS_SetSBASEnable(true)) Output("enable sbas fail");

  char version[64]; // scratch space
  if (!GPS_GetVersion(version, 64)) Output("get gps firmware version fail");
  else
    Output("gps firmware version:%s", version);

  // GPS_FIX_MODE_NORMAL     - normal
  // GPS_FIX_MODE_ELEVATION  - balloon mode
  // GPS_FIX_MODE_HIGH_SPEED - aviation mode
  // GPS_FIX_MODE_LOW_SPEED  - fitness mode
  if (!GPS_SetFixMode(GPS_FIX_MODE_LOW_SPEED)) Output("set fix mode fail");

  // GPS_STANDBY_MODE_STOP  = 0,
  // GPS_STANDBY_MODE_SLEEP = 1,
  // GPS_SetStandbyMode() //is this any use?

  // Does this make any difference unless sleep (above) is called to activate?
  // GPS_LP_MODE_NORMAL:0 (// 4 direct low power?)
  // GPS_LP_MODE_LP:8
  // GPS_LP_MODE_SUPPER_LP:9 (sic)
  if (!GPS_SetLpMode(GPS_LP_MODE_SUPPER_LP)) Output("set gps lp mode fail");

  // why is this done 5 times? does every gps call need this??
  for (retries = 0; retries < 5; ++retries) {
    ret = GPS_SetSearchMode(true, true, false, true); // gps, glonass, galileo?
    // ret = GPS_SetSearchMode(true, false, false, false); // just enable gps
    if (ret) break;
    else
      OS_Sleep(1000);
  }
  Output("Set search mode %s (%d tries)", ret ? "ok" : "fail", retries);

  // Try fast start from cached last known location if we know one
  if (state.latitude != 0 && state.longitude != 0) {
    updateScreen("GPS hot start");

    RTC_Time_t now;
    TIME_GetRtcTime(&now);
    GPS_SetRtcTime(&now);

    // test this behaviour, also check rtc date is vaguely sane first
#ifdef GPS_AGPSFIX
    Output("Set AGPS");
    if (GPS_AGPS(state.latitude, state.longitude, state.altitude, true)) { Output("Got AGPS"); }
#endif
#ifdef GPS_FASTFIX
    Output("Fast start RTC: %d/%d/%d %02d:%02d:%02d", now.year, now.month, now.day, now.hour, now.minute, now.second);

    for (retries = 0; retries < 5; ++retries) {
      ret = GPS_SetLocationTime(state.latitude, state.longitude, state.altitude, &now);
      if (ret) break;
      OS_Sleep(1000);
    }
    Output("Set fastfix %s (%d tries)", ret ? "ok" : "fail", retries);
#endif
  } else {
    updateScreen("GPS cold start");
  }
  // set gps output interval
  for (retries = 0; retries < 5; ++retries) { // why is this done 5 times?
    ret = GPS_SetOutputInterval(NMEA_INTERVAL * 1000);
    if (ret) break;
    OS_Sleep(1000);
  }
  Output("Set NMEA %s (%d retry)", ret ? "ok" : "fail", retries);

  Output("GPS init ok");
  gpsReady = true; // Prevent GPS handler until we've completed configuration
}

bool GetNTP(RTC_Time_t* now) {
  time_t timeNTP = 0;

  if (NTP_Update(timeServer, 5, &timeNTP, true) != 0) timeNTP = 0;
  if (timeNTP > 0) {
    Trace(1, "NTP: %d Time: %s", timeNTP, ctime((const time_t*)&timeNTP));
    TIME_GetRtcTime(now);
    return true;
  }
  return false;
}

static void processFotaUpgradeData(const unsigned char* data, int len) {
  Output("FOTA run %d", len);

  if (len) {
    MEMBLOCK_Trace(1, (uint8_t*)data, (uint16_t)len, 16);
    Trace(1, "fota total len:%d data:%s", len, data);
    if (!API_FotaInit(len)) goto fail;
    if (API_FotaReceiveData((unsigned char*)data, (int)len) == 0)
      ;
  } else { // error
    Trace(1, "fota total len:%d data:%s", len, data);
    goto fail;
  }
  return;

fail:
  Trace(1, "server fota false");
  API_FotaClean();
  // then reboot? or does it automatically?
}

/*
 * Initialise data connection and start uploading
 */
void gprs_Task(void* pData) {
  LED_Blink(pData); // Set the LED timer going, 1sec blink to start

  strcpy(stateMsg, "Wait for init");
  refreshScreen();

  uint8_t status;
  while (!initialised) OS_Sleep(1000);

  ImeiRead(); // Populate global imei

  // 60 seconds might be too quick, try 3 min?
  // If we can't get GPRS in this time, start over
  WatchDog_Open(WATCHDOG_SECOND_TO_TICK(60 * 3));

  strcpy(stateMsg, "GPRS Connect"); //
  refreshScreen();
  Output("Wait for GPRS activation");

  while (!dat_on) OS_Sleep(1000);
  Output("GPRS connected");
  strcpy(stateMsg, "Connected");
  refreshScreen();

  // If no network time, get from NTP
  RTC_Time_t time;
  if (!nettime) {
    if (GetNTP(&time)) {
      nettime = true;
      Output("NTP time synchronised.");
    }
  }

  // OTA upgrade check, leave this off by default
  if (fota_on) {
    char url[256];
    memset(url, 0, sizeof(url));
    sprintf(url, FOTA_HTTP_SERVER, SOFT_VERSION);
    Trace(1, "FOTA network activate success url %s", url);
    if (API_FotaByServer(url, processFotaUpgradeData) == 0) Trace(1, "FOTA wait");
    else
      Trace(1, "FOTA no");
  }

  TIME_GetRtcTime(&time);
  Output("Time: %02d/%02d/%04d %02d:%02d:%02d", time.day, time.month, time.year, time.hour, time.minute, time.second);

  strcpy(stateMsg, "GPS Start"); // Start GPS ASAP, power issue seems ok now with power capacitors
  refreshScreen();
  InitialiseGPS();

  SetFlash(6, 1, 1000);      // All good, nice steady blink
  strcpy(stateMsg, MSG_RUN); // TODO Consolidate the display messages.
  refreshScreen();

  // Activate screen timeout
  OS_StartCallbackTimer(mainTaskHandle, 1000 * config.screentime, OLED_off,
                        NULL); // Will this work?

  char msg[64];
  sprintf(msg, "*IVR:%s#", imei);
  UploadToServer(msg); // Confirm we're starting a track

  // Is it worth adding "show text message" feature?
  // SMS_Storage_Info_t storageInfo;
  // SMS_GetStorageInfo(&storageInfo, SMS_STORE);
  // Output("sms storage sim card info, used:%d,total:%d", storageInfo.used,
  //        storageInfo.total);
  // SMS_ListMessageRequst(SMS_STATUS_ALL, SMS_STORE); // Read and delete?

  bool ret = true;
  char reason[16]; // TODO cleanup.
  while (1) {
    if (gps_on && dat_on && mob_on) strcpy(stateMsg, "OK.");

#if 1 // do we want this?
      // Looks like GPS boot needs to be on main thread not sure why.
      // If the GPS isn't getting anywhere try rebooting it?
      // what's a sane time to wait to fix? 4/5min?
    if (nofixcount > (10 * 60 / NMEA_INTERVAL)) {
      OLED_on();
      sprintf(stateMsg, "Reboot GPS...");
      Output(stateMsg);
      refreshScreen();

      // If we never locked, assume moved too far and full cold start
      if (fixcount == 0) {
        Output("Never fix, COLD GPS reboot");
        state.latitude  = 0; // Start from scratch
        state.longitude = 0;
        GPS_Reboot(GPS_REBOOT_MODE_COLD);
        InitialiseGPS(); // Is re-initialise needed?
      }
      // else just stick to a warm reset.
      else {
        Output("Previous fixed, WARM GPS reboot");
        GPS_Reboot(GPS_REBOOT_MODE_WARM); // check if we need reinitialise now?
      }
    }
#endif

    // Do we need a GPRS check/restart?
    Network_GetActiveStatus(&status); // Is this reliable?
    if (mob_on && status) {           // Registered and activated
      // Have connection, so do uploading.
      // are we reconnecting with an SD cache?
      // flush that first
      int32_t fc = API_FS_Open("/t/cache", FS_O_RDONLY, 0);
      if (fc > 0) {
#ifdef VERBOSE
        Output("Uploading cached");
#endif
        int64_t  cachelen = API_FS_GetFileSize(fc);
        uint8_t* cache    = OS_Malloc(cachelen);
        if (cache) {
          memset(cache, 0, cachelen);
          int32_t lenread = API_FS_Read(fc, (uint8_t*)cache, cachelen);
          API_FS_Close(fc);
          if (lenread) {
            if (UploadToServer(cache)) {
              API_FS_Delete("/t/cache"); // Only delete if all done
              dsk_on = false;
            } else { // we'll end up resending some
            }
          }
          OS_Free(cache);
        }
      }

      // now upload our memory buffer (~10mins size)
      // if we uploaded the SD cache
      if (!dsk_on && strlen(sdbuffer)) {
#ifdef VERBOSE
        Output("Uploading RAM");
#endif
        ret = UploadToServer(sdbuffer);
        if (!ret) {
          strcpy(reason, "upload");
          Output("Unable to upload from RAM");
        } else {
          if (SaveToSDLog(sdbuffer)) // add to logfile
            sdbuffer[0] = 0;         // Truncate
        }
      }
    } else {
      strcpy(reason, "register");
      ret = false; // force 1min retry
    }

    // investigate deregister/register
    if (!ret) // Check and retry upload again every minute.
    {
      Output("failed, retry in 1min");
      sprintf(stateMsg, "Fail retry\n(%s)", reason);
      refreshScreen();

      for (int pause = 60; pause > 0; pause--) {
        WatchDog_KeepAlive();
        OS_Sleep(1000); // retry each minute
      }
      ret = true;
    } else {
      strcpy(stateMsg, MSG_RUN);
      refreshScreen();

      // slow down, chill and wait. seems to cause chaos. :(
      // Changing speed seems to cause issues
      PM_SetSysMinFreq(PM_SYS_FREQ_32K);
#ifdef VERBOSE
      Output("Sleep (slow) for %dm", config.upload / 60);
#endif
      for (int pause = config.upload; pause > 0; pause--) {
        WatchDog_KeepAlive();
        OS_Sleep(1000); // check max sleep for 5min upload
      }
#ifdef VERBOSE
      Output("Wakeup (fast)");
#endif
      PM_SetSysMinFreq(PM_SYS_FREQ_178M);
      OS_Sleep(1000); // short pause seems necessary if freq changed
    }
  }
}

void SMSInit() {
  if (!SMS_SetFormat(SMS_FORMAT_TEXT, SIM0)) {
    Output("sms set format error");
    return;
  }
  SMS_Parameter_t smsParam = {
      .fo  = 17,
      .vp  = 167,
      .pid = 0,
      .dcs = 0, // 0:English 7bit, 4:English 8 bit, 8:Unicode 2 Bytes
  };
  if (!SMS_SetParameter(&smsParam, SIM0)) {
    Output("sms set parameter error");
    return;
  }

  if (!SMS_SetNewMessageStorage(SMS_STORE)) {
    Output("sms set message storage fail");
    return;
  }
}

// Decide if we're going to dump/talk over uart
void UARTInit() {
  UART_Config_t uart_config = {                                     // Lets run a bit fast
                               .baudRate   = UART_BAUD_RATE_921600, // UART_BAUD_RATE_115200,
                               .dataBits   = UART_DATA_BITS_8,
                               .stopBits   = UART_STOP_BITS_1,
                               .parity     = UART_PARITY_NONE,
                               .rxCallback = NULL,
                               .useEvent   = true};
  UART_Init(UART1, uart_config);
}

void InitConfig() {
  Output("Init config");
  config.loglevel = DEBUG | TRACE; // Default on, before loading config.

  // Sanity check SD card on powerup and dump contents for info/diags
  Output("Check SD");
  if (ListDirsRoot("/") && // Show what's on TF
      ListDirsRoot("/t"))  // Show what's on SD
    Output("SD OK");

  if (!FileExists(GPS_LOG_FILE_PATH)) {
    int32_t fl = API_FS_Open(GPS_LOG_FILE_PATH, FS_O_RDWR | FS_O_CREAT, 0);
    if (fl <= 0) Output("Unable to create empty log");
    else
      API_FS_Close(fl);
  }

  if (!ReadConfig() && !WriteConfig()) // Create default config if missing.
  {
    updateScreen("SD failure.\nPower off!!!");
    // Fatal so stuck and poweroff
    Flash(100);
    Flash(100);
    Flash(100);
    Flash(100); // Flicker light so we know its not running yet
    OS_Sleep(10000);
    PowerOff(); // Bye bye
  }
  if (config.loglevel & DEBUG) CreateLog(); // Empty logfile

  // preload last good state.
  // show if we have good state?
  int32_t fd = API_FS_Open("/t/state", FS_O_RDONLY, 0);
  if (fd > 0) {
    API_FS_Read(fd, (uint8_t*)&state, sizeof(state_t));
    Output("Loaded last state.");
    API_FS_Close(fd);
  } else {
    memset(&state, 0, sizeof(state_t));
  }

  // Show if we have some unsent data?
  if (FileExists("/t/cache")) { dsk_on = true; }
}

void appMainTask(void* pData) {
  LED_init();

  // Twinkle to show alive and ready
  for (int i = 0; i < 5; ++i) Flash(100);

  // PM_PowerEnable(POWER_TYPE_VPAD, true);//Always on
  PM_PowerEnable(POWER_TYPE_MMC, false);
  PM_PowerEnable(POWER_TYPE_LCD, false);
  PM_PowerEnable(POWER_TYPE_CAM, false);

  // Changing clock frequencies seems to cause GPRS instability/chaos.
  //   PM_SetSysMinFreq(PM_SYS_FREQ_312M); // Full speed startup any help?
  PM_SetSysMinFreq(PM_SYS_FREQ_178M);

  API_Event_t* event = NULL;

  if (!OLED_init()) Output("Unable to allocate screen.");

  drawString(0, 0, "Booting IvrTrac");
  drawString(0, 2, SOFT_VERSION);
  drawString(0, 4, "               ");
  drawString(0, 6, "#ONElove       ");
  OLED_show();
  OS_Sleep(1000);

  UARTInit(); // Logging option
  SMSInit();  // Listen for SMS messages

  sdbuffer = OS_Malloc(BUFFER_SIZE);

  if (!sdbuffer) {
    Output("Cant create sdbuffer");
    PM_ShutDown();
  }

  memset(sdbuffer, 0, BUFFER_SIZE);

  updateScreen("SD init");
  Output("GPS Monitor " SOFT_VERSION " running");
  InitConfig(); // Need defaults for handlers to start running

  // Does this help power issues?
  if (strcmp(config.apn, "everywhere") == 0) {
    Output("Limit to 1800 & 1900 bands");
    Network_SetFrequencyBand(NETWORK_FREQ_BAND_DCS_1800 | NETWORK_FREQ_BAND_PCS_1900);
  } else {
    Output("Use all bands");
    Network_SetFrequencyBand(NETWORK_FREQ_BAND_DCS_1800 | NETWORK_FREQ_BAND_PCS_1900 | NETWORK_FREQ_BAND_GSM_900E |
                             NETWORK_FREQ_BAND_GSM_900P | NETWORK_FREQ_BAND_GSM_850);
  }

  TIME_SetIsAutoUpdateRtcTime(true);

  updateScreen("Boot..");

  Output("Starting tasks");
  OS_CreateTask(gprs_Task, NULL, NULL, GPS_TASK_STACK_SIZE, MAIN_TASK_PRIORITY + 2, 0, 0, "GPRS Task");

  // Wait event
  while (1) {
    if (OS_WaitEvent(mainTaskHandle, (void**)&event, OS_TIME_OUT_WAIT_FOREVER)) {
      EventDispatch(event);
      OS_Free(event->pParam1);
      OS_Free(event->pParam2);
      OS_Free(event);
    }
  }
}

void gps_monitor_Main(void) {
  mainTaskHandle = OS_CreateTask(appMainTask, NULL, NULL, MAIN_TASK_STACK_SIZE, MAIN_TASK_PRIORITY, 0, 0, MAIN_TASK_NAME);
  OS_SetUserMainHandle(&mainTaskHandle);
}
