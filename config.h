#ifndef CONFIG_H
#define CONFIG_H

#define MAX_CURRENT 15000
#define MIN_CURRENT -15000

#define MAX_TRANSITIONS 2
#define MAX_MENU_ITEMS 5

#define BUTTON_DEBOUNCE_MS 50
#define BUTTON_CLICK_MS 500
#define EDIT_ACCEL_FACTOR_8 10
#define EDIT_ACCEL_FACTOR_16 100

#define EEPROM_SETTINGS 0

typedef enum
{
  U8,
  I8,
  U16,
  I16
} Types;

typedef struct
{
  PGM_P *menu_string;
  Types type;
  void *value;
} Settings_t;

typedef struct
{
  int8_t pulse_current_ratio;
  uint16_t pulse_Adm;
  uint16_t dc_Adm;
  uint8_t board_x;
  uint8_t board_y;
  int16_t pulse_currents[MAX_TRANSITIONS];
  uint16_t pulse_durations[MAX_TRANSITIONS];
} Global_Settings_t;

extern Global_Settings_t global_settings;

#define ENC_PIN_A PIND6
#define ENC_PIN_B PIND5
#define ENC_PIN_BUTTON PIND7

#define ENC_CTL DDRD  //encoder port control
#define ENC_WR  PORTD //encoder port write  
#define ENC_RD  PIND  //encoder port read
#define ENCODER_vect PCINT2_vect

#endif // CONFIG_H
