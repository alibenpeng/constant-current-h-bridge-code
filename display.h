#ifndef DISPLAY_H
#define DISPLAY_H

extern PGM_P menu_strings[] PROGMEM;
//extern PGM_P fullauto_strings[] PROGMEM;
//extern PGM_P autocurrent_strings[] PROGMEM;
//extern PGM_P manual_strings[] PROGMEM;
//extern PGM_P dc_strings[] PROGMEM;
extern PGM_P settings_strings[] PROGMEM;
//extern uint16_t pulse_settings[];

void init_display(void);
void display_menu(uint8_t choice);
void display_settings(uint8_t array_size, uint8_t choice, Settings_t *settings);
void display_cancel_menu(uint8_t choice);
void display_plating_params(void);

#endif // DISPLAY_H
