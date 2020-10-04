#include <api_hal_gpio.h>
#include <api_os.h>

#include "ledutil.h"

// Slow 'alive' background blinking
// Global "current blink state flags"
int  ledoff   = 1;
int  ledon    = 1;
int  leddelay = 1000;
bool active   = true;

void LEDActive(bool pause) { active = pause; }

void LED_data(bool state) { GPIO_Set(UPLOAD_DATA_LED, state ? 1 : 0); }

void LED_init() {
  GPIO_config_t gpioLedBlue   = {.mode         = GPIO_MODE_OUTPUT,
                               .pin          = SYSTEM_STATUS_LED, // Use for activity
                               .defaultLevel = GPIO_LEVEL_LOW};
  GPIO_config_t gpioLedUpload = {.mode         = GPIO_MODE_OUTPUT,
                                 .pin          = UPLOAD_DATA_LED, // Use for upload
                                 .defaultLevel = GPIO_LEVEL_LOW};

  GPIO_Init(gpioLedBlue);
  GPIO_Init(gpioLedUpload);
}

void SetFlash(int off, int on, int delay) {
  ledoff   = off;
  ledon    = on;
  leddelay = delay;
}

void LED_Blink(void* param) {
  static int count = 0;
  if (++count == ledoff) {
    if (active) // Dont flash if we aren't registered
      GPIO_Set(SYSTEM_STATUS_LED, GPIO_LEVEL_HIGH);
  } else if (count >= ledoff + ledon) {
    GPIO_Set(SYSTEM_STATUS_LED, GPIO_LEVEL_LOW);
    count = 0;
  }
  OS_StartCallbackTimer(OS_GetUserMainHandle(), leddelay, LED_Blink, NULL);
}

// Twinkle LED, used to highlight fatal issues/states
void Flash(int time) {
  GPIO_Set(SYSTEM_STATUS_LED, GPIO_LEVEL_HIGH);
  OS_Sleep(time);
  GPIO_Set(UPLOAD_DATA_LED, GPIO_LEVEL_HIGH);
  OS_Sleep(time);
  GPIO_Set(SYSTEM_STATUS_LED, GPIO_LEVEL_LOW);
  OS_Sleep(time);
  GPIO_Set(UPLOAD_DATA_LED, GPIO_LEVEL_LOW);
  OS_Sleep(time);
}
