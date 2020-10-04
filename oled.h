/*
 * OLED drawing utilities
 */
bool OLED_state();
void OLED_off();
void OLED_on();
void OLED_clear();
void OLED_invert(bool invert);
void OLED_init(void);
void OLED_show();

void drawIcon(int x, int y, int id);
void drawLetter(int x, int y, char chr);
void drawString(int x, int y, const char* msg);

#define ICON_DISK   0
#define ICON_DATA   1 * 16
#define ICON_SIGNAL 2 * 16
#define ICON_GPS    3 * 16
#define ICON_BAT_L  4 * 16
#define ICON_BAT_M  5 * 16
#define ICON_BAT_H  6 * 16
