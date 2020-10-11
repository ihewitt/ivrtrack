#define SYSTEM_STATUS_LED GPIO_PIN27
#define UPLOAD_DATA_LED   GPIO_PIN28

void LED_init();
void LED_Blink(void* param);
void Flash(int);
void SetFlash(int off, int on, int delay);
void LED_data(bool state);
