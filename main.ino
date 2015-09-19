#include <avr/pgmspace.h>
#include <avr/eeprom.h>

#include "config.h"
#include "misc.h"
#include "display.h"

Global_Settings_t global_settings;
static int16_t enc_counter;
static uint8_t button_state = 1;
static uint8_t last_button_state = 1;


#if 0
ISR(ENCODER_vect)
{
  static uint8_t old_AB = 0; //lookup table index and initial state
  uint8_t encport; //encoder port copy
  uint8_t direction;
  static const int8_t enc_states [] PROGMEM =
  {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; //encoder lookup table

  old_AB <<=2; //remember previous state
  encport = (ENC_RD >> 3) & 0x03; //read encoder
  old_AB |= encport; //create index

  direction = pgm_read_byte(&(enc_states[( old_AB & 0x0f )])); //get direction
  if( direction && ( encport == 3 )) { //check if at detent and transition is valid
    /* post "Navigation forward/reverse" event */
    if( direction == 1 ) {
      enc_counter++;
      //QF::publish(Q_NEW(QEvent, NAV_FWD_SIG));
    } else {
      enc_counter--;
      //QF::publish(Q_NEW(QEvent, NAV_REV_SIG));
    }
  }//if( direction && encport == 3...

  // button
  button_state = ((ENC_RD >> 5) & 0x04);
}

#else

ISR(ENCODER_vect)
{
  static uint8_t old_AB = 0;  //lookup table index
  static int8_t encval = 0;   //encoder value  
  static const int8_t enc_states [] PROGMEM = 
    {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};  //encoder lookup table

  old_AB <<=2;  //remember pREVERSEious state
  old_AB |= ( (ENC_RD >>5) & 0x03 );

  encval += pgm_read_byte(&(enc_states[( old_AB & 0x0f )]));

  if (encval > 3) {
    encval = 0;
    enc_counter++;
  }
  else if (encval < -3) {
    encval = 0;
    enc_counter--;
  }

  // button
  button_state = ((ENC_RD >> 5) & 0x04);
    
}
#endif

void setup() {

  // Setup encoder pins as inputs
  ENC_CTL  &= ~((1 << ENC_PIN_B) | (1 << ENC_PIN_A) | (1 << ENC_PIN_BUTTON));
  // Bias pins
  ENC_WR |=  ((1 << ENC_PIN_B) | (1 << ENC_PIN_A) | (1 << ENC_PIN_BUTTON));

  // portable up until here.
  PCMSK2 = ((1 << PCINT21) | (1 << PCINT22) | (1 << PCINT23)); // Pin Change Mask Register 2
  PCICR = (1 << PCIE2); // Pin Change Interrupt Enable 2

  // initialize display
  init_display();

  // read config from Eeprom

  Serial.begin(57600);
  Serial.print("\n[memCheck]\r\nBytes free: ");
  Serial.println(freeRam());
  eeprom_read_block((Global_Settings_t*)(&(global_settings)), EEPROM_SETTINGS, sizeof(Global_Settings_t));

  // setup PWM pins
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM12) | _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(CS10);
  DDRB |= _BV(PINB1) | _BV(PINB2);
  PINB &= ~((1<<PINB1) | (1<<PINB2));


}

void set_pwm(int16_t c_in) {

  int16_t current = map(constrain(c_in, MIN_CURRENT, MAX_CURRENT), MIN_CURRENT, MAX_CURRENT, -1023, 1023);
  if (c_in < 0) current *= -1;
  uint8_t lowbyte = current;
  uint8_t highbyte = current >> 8;
#ifdef SERIAL_DEBUG
  Serial.print("\nc_in: ");
  Serial.println(c_in);
  Serial.print("current: ");
  Serial.println(current);
  Serial.print("highbyte: ");
  Serial.println(highbyte, HEX);
  Serial.print("lowbyte: ");
  Serial.println(lowbyte, HEX);
#endif
  if (c_in == 0)  {
    OCR1AH = 0;
    OCR1AL = 0;
    OCR1BH = 0;
    OCR1BL = 0;
  } else if (c_in > 0)  {
    OCR1AH = 0;
    OCR1AL = 0;
    OCR1BH = highbyte;
    OCR1BL = lowbyte;
  } else {
    OCR1AH = highbyte;
    OCR1AL = lowbyte;
    OCR1BH = 0;
    OCR1BL = 0;
  }
}

void pulse_loop() {
  static uint8_t pulse_counter, last_counter = 0;
  uint32_t now = micros();
  static uint32_t last_time = now;


  if (now - last_time >= 1000*global_settings.pulse_durations[last_counter]) {
    #ifdef SERIAL_DEBUG
    Serial.print("pulse: ");
    Serial.println(now - last_time);
    Serial.print("t: ");
    Serial.println(global_settings.pulse_durations[pulse_counter]);
    Serial.print("c: ");
    Serial.println(global_settings.pulse_currents[pulse_counter]);
    #endif
    set_pwm(global_settings.pulse_currents[pulse_counter]);
    last_time = now;
    last_counter = pulse_counter;
    pulse_counter++;
    if (pulse_counter >= MAX_TRANSITIONS) pulse_counter = 0;
  }
}

void calc_settings(void)
{
  float pulse_ratio = 10/(float)global_settings.pulse_current_ratio;
  uint32_t board_area = global_settings.board_x * global_settings.board_y;
  uint32_t plating_current = board_area * global_settings.pulse_Adm / 10000;

  Serial.print("Calculating settings for current ratio ");
  Serial.print(pulse_ratio);
  Serial.print(", pulse_Adm: ");
  Serial.print(global_settings.pulse_Adm);
  Serial.print(", and plating current ");
  Serial.print(plating_current);
  Serial.println("mA");
  Serial.print("Board Area: ");
  Serial.print(board_area);
  Serial.print("dm^2\r\n");

  global_settings.pulse_currents[0] = ((global_settings.pulse_durations[0] + global_settings.pulse_durations[1]) * (plating_current * pulse_ratio))
                                      / (global_settings.pulse_durations[0] * pulse_ratio + global_settings.pulse_durations[1]);

  global_settings.pulse_currents[1] = global_settings.pulse_currents[0] / pulse_ratio;

  Serial.print("t0: ");
  Serial.print(global_settings.pulse_durations[0]);
  Serial.print("ms, c0: ");
  Serial.print(global_settings.pulse_currents[0]);
  Serial.print("mA\r\nt1: ");
  Serial.print(global_settings.pulse_durations[1]);
  Serial.print("ms, c1: ");
  Serial.print(global_settings.pulse_currents[1]);
  Serial.print("mA\r\nFree RAM: ");
  Serial.print(freeRam());
  Serial.print("b\r\n");
}

typedef enum {
  MAIN_MENU,
  FULL_AUTO_MENU,
  AUTO_CURRENT_MENU,
  MANUAL_MENU,
  DC_MENU,
  SETTINGS_MENU,
  CANCEL_MENU,
  PLATING
} Global_State_t;
Global_State_t global_state;


void loop() {
  static int16_t last_enc_counter = 0;
  static uint8_t last_menu_choice, menu_choice = 0;
  static uint8_t menu_items = 0;
  static Settings_t menu_settings[MAX_MENU_ITEMS];
  static uint8_t edit_mode = 0;
  static uint8_t accel_factor = 1;

  //global_state = DC_MENU;

/*
  if (enc_counter != last_enc_counter) {
    //Serial.println(enc_counter, DEC);   
    last_enc_counter = enc_counter;
  }
*/
  switch(global_state) {
    case MAIN_MENU:
      menu_items = 5;
      menu_choice = enc_counter % menu_items;
      display_menu(menu_choice);
      break;
    case FULL_AUTO_MENU:
      menu_items = 2;
      if (!edit_mode) menu_choice = enc_counter % (menu_items + 2);

      menu_settings[0].menu_string = &settings_strings[0];
      menu_settings[0].type = U8;
      menu_settings[0].value = &global_settings.board_x;

      menu_settings[1].menu_string = &settings_strings[1];
      menu_settings[1].type = U8;
      menu_settings[1].value = &global_settings.board_y;

      menu_settings[2].menu_string = &settings_strings[6];
      menu_settings[2].type = U16;
      menu_settings[2].value = &global_settings.pulse_Adm;

      display_settings(menu_items, menu_choice, menu_settings);
      break;
    case AUTO_CURRENT_MENU:
      menu_items = 4;
      if (!edit_mode) menu_choice = enc_counter % (menu_items + 2);

      menu_settings[0].menu_string = &settings_strings[0];
      menu_settings[0].type = U8;
      menu_settings[0].value = &global_settings.board_x;

      menu_settings[1].menu_string = &settings_strings[1];
      menu_settings[1].type = U8;
      menu_settings[1].value = &global_settings.board_y;

      menu_settings[2].menu_string = &settings_strings[2];
      menu_settings[2].type = U16;
      menu_settings[2].value = &global_settings.pulse_durations[0];

      menu_settings[3].menu_string = &settings_strings[3];
      menu_settings[3].type = U16;
      menu_settings[3].value = &global_settings.pulse_durations[1];

      display_settings(menu_items, menu_choice, menu_settings);
      break;
    case MANUAL_MENU:
      menu_items = 4;
      if (!edit_mode) menu_choice = enc_counter % (menu_items + 2);

      menu_settings[0].menu_string = &settings_strings[4];
      menu_settings[0].type = I16;
      menu_settings[0].value = &global_settings.pulse_currents[0];

      menu_settings[1].menu_string = &settings_strings[5];
      menu_settings[1].type = I16;
      menu_settings[1].value = &global_settings.pulse_currents[1];

      menu_settings[2].menu_string = &settings_strings[2];
      menu_settings[2].type = U16;
      menu_settings[2].value = &global_settings.pulse_durations[0];

      menu_settings[3].menu_string = &settings_strings[3];
      menu_settings[3].type = U16;
      menu_settings[3].value = &global_settings.pulse_durations[1];

      display_settings(menu_items, menu_choice, menu_settings);
      break;

    case DC_MENU:
      menu_items = 3;
      if (!edit_mode) menu_choice = enc_counter % (menu_items + 2);

      menu_settings[0].menu_string = &settings_strings[0];
      menu_settings[0].type = U8;
      menu_settings[0].value = &global_settings.board_x;

      menu_settings[1].menu_string = &settings_strings[1];
      menu_settings[1].type = U8;
      menu_settings[1].value = &global_settings.board_y;

      menu_settings[2].menu_string = &settings_strings[6];
      menu_settings[2].type = U16;
      menu_settings[2].value = &global_settings.dc_Adm;

      display_settings(menu_items, menu_choice, menu_settings);
      break;

    case SETTINGS_MENU:
      menu_items = 4;
      if (!edit_mode) menu_choice = enc_counter % (menu_items + 2);

      menu_settings[0].menu_string = &settings_strings[6];
      menu_settings[0].type = U16;
      menu_settings[0].value = &global_settings.pulse_Adm;

      menu_settings[1].menu_string = &settings_strings[7];
      menu_settings[1].type = I8;
      menu_settings[1].value = &global_settings.pulse_current_ratio;

      menu_settings[2].menu_string = &settings_strings[2];
      menu_settings[2].type = U16;
      menu_settings[2].value = &global_settings.pulse_durations[0];

      menu_settings[3].menu_string = &settings_strings[3];
      menu_settings[3].type = U16;
      menu_settings[3].value = &global_settings.pulse_durations[1];

      display_settings(menu_items, menu_choice, menu_settings);
      break;
    case CANCEL_MENU:
      menu_items = 0;
      menu_choice = enc_counter % (menu_items + 2);
      if (enc_counter != last_enc_counter) {
        display_cancel_menu(menu_choice);
        last_enc_counter = enc_counter;
      }
      pulse_loop();
      break;
    case PLATING:
      pulse_loop();
      break;
  }

  static uint32_t button_t = millis();
  static uint32_t last_button_t = millis();
  if (button_state != last_button_state) {
    button_t = millis();
    if (button_t > last_button_t + BUTTON_DEBOUNCE_MS) {
      if (!button_state) { // Button pressed
        Serial.println("button pressed!");
        if (edit_mode) {
          switch (menu_settings[menu_choice].type) {
            case U8:
            case I8:
              accel_factor = EDIT_ACCEL_FACTOR_8;
              break;

            case U16:
            case I16:
              accel_factor = EDIT_ACCEL_FACTOR_16;
              break;
          }
        }
      } else { // Button released
        Serial.print("button released: ");
        Serial.println(button_t - last_button_t);
        if (button_t < last_button_t + BUTTON_CLICK_MS) { // Button clicked
          if (edit_mode) {
            edit_mode = 0;
            accel_factor = 1;
            enc_counter = last_menu_choice;
          } else if (global_state == MAIN_MENU) {
            last_menu_choice = menu_choice;
            global_state = (Global_State_t)(menu_choice + 1); // 0 is MAIN_MENU
            menu_choice = 0;
          } else if (global_state == PLATING) {
            display_cancel_menu(0);
            global_state = CANCEL_MENU;
          } else {
            if (menu_choice < menu_items) {
              last_enc_counter = enc_counter = 0;
              last_menu_choice = menu_choice;
              edit_mode = 1;
            } else if (menu_choice == menu_items) { // "OK"
              Serial.println("OK!");
              Serial.println("Saving!");
              eeprom_write_block(&(global_settings), EEPROM_SETTINGS, sizeof(Global_Settings_t));
              if (global_state == SETTINGS_MENU) {
                calc_settings();
                global_state = MAIN_MENU;
              } else if (global_state == FULL_AUTO_MENU) {
                calc_settings();
                display_plating_params();
                global_state = PLATING;
              } else {
                display_plating_params();
                global_state = PLATING;
              }
            } else if (menu_choice == menu_items + 1) { // "cancel"
              Serial.println("cancel!");
              set_pwm(0);
              menu_choice = 0;
              global_state = MAIN_MENU;
            }
          }
        } else { // long click release
          accel_factor = 1;
        }
      }

      last_button_t = button_t;
      last_button_state = button_state;
    }
  }

  if (edit_mode) {
    if (last_enc_counter != enc_counter) {
      Serial.print("enc_counter: ");
      Serial.println(enc_counter);
      Serial.print("last_enc_counter: ");
      Serial.println(last_enc_counter);
      switch (menu_settings[menu_choice].type) {
        case U8:
          *(uint8_t*) menu_settings[menu_choice].value += (enc_counter - last_enc_counter) * accel_factor;
          break;

        case I8:
          *(int8_t*) menu_settings[menu_choice].value += (enc_counter - last_enc_counter) * accel_factor;
          break;

        case U16:
          *(uint16_t*) menu_settings[menu_choice].value += (enc_counter - last_enc_counter) * accel_factor;
          break;

        case I16:
          *(int16_t*) menu_settings[menu_choice].value += (enc_counter - last_enc_counter) * accel_factor;
          break;
      }
      last_enc_counter = enc_counter;
    }
  } else {
    if (enc_counter < 0) {
      enc_counter = menu_items + 1;
      if (global_state == MAIN_MENU) enc_counter -=2;
    }
  }
}


// vim: expandtab sw=2 ts=2
