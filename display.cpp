#include <avr/pgmspace.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

#include "config.h"
#include "misc.h"
#include "display.h"

// Software SPI (slower updates, more flexible pin options):
// pin 7 - Serial clock out (SCLK)
// pin 6 - Serial data out (DIN)
// pin 5 - Data/Command select (D/C)
// pin 4 - LCD chip select (CS)
// pin 3 - LCD reset (RST)
Adafruit_PCD8544 display = Adafruit_PCD8544(11, 13, A0, A1, A2);

// Hardware SPI (faster, but must use certain hardware pins):
// SCK is LCD serial clock (SCLK) - this is pin 13 on Arduino Uno
// MOSI is LCD DIN - this is pin 11 on an Arduino Uno
// pin 5 - Data/Command select (D/C)
// pin 4 - LCD chip select (CS)
// pin 3 - LCD reset (RST)
//Adafruit_PCD8544 display = Adafruit_PCD8544(A2, A1, A0);
// Note with hardware SPI MISO and SS pins aren't used but will still be read
// and written to during SPI transfer.  Be careful sharing these pins!

// Main Menu
static const char menu0[] PROGMEM = "Full auto PRC ";
static const char menu1[] PROGMEM = "Auto curr. PRC";
static const char menu2[] PROGMEM = "Manual PRC    ";
static const char menu3[] PROGMEM = "DC            ";
static const char menu4[] PROGMEM = "Auto Settings ";

PGM_P menu_strings[] PROGMEM = {
  menu0,
  menu1,
  menu2,
  menu3,
  menu4
};


// Settings Strings
static const char settings0[] PROGMEM = "Board X: %3dmm";
static const char settings1[] PROGMEM = "Board Y: %3dmm";
static const char settings2[] PROGMEM = "T+:    %5dms";
static const char settings3[] PROGMEM = "T-:    %5dms";
static const char settings4[] PROGMEM = "C+:    %5dmA";
static const char settings5[] PROGMEM = "C-:    %5dmA";
static const char settings6[] PROGMEM = "Adm:   %5dmA";
static const char settings7[] PROGMEM = "+/-Adm: 10/%3d";

PGM_P settings_strings[] PROGMEM = {
  settings0,
  settings1,
  settings2,
  settings3,
  settings4,
  settings5,
  settings6,
  settings7,
};

static const char misc_string0[] PROGMEM = "   OK  ";
static const char misc_string1[] PROGMEM = " cancel";
static const char misc_string2[] PROGMEM = "Cancel?";
static const char misc_string3[] PROGMEM = "  Yes  ";
static const char misc_string4[] PROGMEM = "   No  ";
static const char misc_string5[] PROGMEM = "Plating";
static const char misc_string6[] PROGMEM = "%4dms%5dmA";

PGM_P misc_strings[] PROGMEM = {
  misc_string0,
  misc_string1,
  misc_string2,
  misc_string3,
  misc_string4,
  misc_string5,
  misc_string6,
};

void init_display(void)
{
  // Setup Display
  display.begin();
  // you can change the contrast around to adapt the display
  // for the best viewing!
  display.setContrast(50);
  display.clearDisplay();   // clears the screen and buffer
  display.setRotation(0);   // 1=rotate 90 degrees counter clockwise, can also use values of 2 and 3 to go further.
}

void print_ram(void) {
  // Display free RAM
  display.setTextSize(1);
  display.setCursor(0,5*8);
  display.setTextColor(WHITE, BLACK); // 'inverted' text
  display.print(freeRam());
  display.println("Bytes free");
  display.setTextColor(BLACK);
}

// Display a Menu w/o adjustable settings
void display_menu(uint8_t choice) {
  display.clearDisplay();
  display.setTextSize(1);
  for (int i=0; i<5; i++) {
    char buffer[14];
    strcpy_P(buffer, (PGM_P)pgm_read_word(&(menu_strings[i])));
    if (i==choice)
      display.setTextColor(WHITE, BLACK); // 'inverted' text
    else if (i==choice+1)
      display.setTextColor(BLACK);
    display.setCursor(0,i*8);
    display.println(buffer);
  }
  print_ram();
  display.display();
}

void display_settings(uint8_t array_size, uint8_t choice, Settings_t *settings) {
  display.clearDisplay();
  display.setTextSize(1);
  char buffer[14];
  for (int i=0;i<array_size;i++) {

    switch (settings[i].type) {
      case U8:
        sprintf_P(buffer, (PGM_P)pgm_read_word(settings[i].menu_string), *(uint8_t*) settings[i].value);
        break;

      case I8:
        sprintf_P(buffer, (PGM_P)pgm_read_word(settings[i].menu_string), *(int8_t*) settings[i].value);
        break;

      case U16:
        sprintf_P(buffer, (PGM_P)pgm_read_word(settings[i].menu_string), *(uint16_t*) settings[i].value);
        break;

      case I16:
        sprintf_P(buffer, (PGM_P)pgm_read_word(settings[i].menu_string), *(int16_t*) settings[i].value);
        break;
    }

    if (i==choice)
      display.setTextColor(WHITE, BLACK); // 'inverted' text
    else if (i==choice+1)
      display.setTextColor(BLACK);

    display.setCursor(0,i*8);
    display.println(buffer);
  }

  display.setTextColor(BLACK);
  display.setCursor(0,4*8);
  if (choice == array_size) display.setTextColor(WHITE, BLACK);
  strcpy_P(buffer, (PGM_P)pgm_read_word(&(misc_strings[0])));
  display.print(buffer);
  if (choice == array_size) display.setTextColor(BLACK);
  if (choice == array_size + 1) display.setTextColor(WHITE, BLACK);
  strcpy_P(buffer, (PGM_P)pgm_read_word(&(misc_strings[1])));
  display.print(buffer);
  display.setTextColor(BLACK);

  // Display free RAM
  print_ram();
  display.display();
}

void display_cancel_menu(uint8_t choice)
{
  char buffer[14];
  display.clearDisplay();
  display.setTextSize(2);
  strcpy_P(buffer, (PGM_P)pgm_read_word(&(misc_strings[2])));
  display.print(buffer);
  display.setTextSize(1);
  display.setCursor(0,4*8);
  if (choice == 0) display.setTextColor(WHITE, BLACK);
  strcpy_P(buffer, (PGM_P)pgm_read_word(&(misc_strings[4])));
  display.print(buffer);
  if (choice == 0) display.setTextColor(BLACK);
  if (choice == 1) display.setTextColor(WHITE, BLACK);
  strcpy_P(buffer, (PGM_P)pgm_read_word(&(misc_strings[3])));
  display.print(buffer);
  display.setTextColor(BLACK);

  print_ram();
  display.display();
  
}

void display_plating_params(void)
{
  char buffer[14];
  display.clearDisplay();
  display.setTextSize(2);
  strcpy_P(buffer, (PGM_P)pgm_read_word(&(misc_strings[5])));
  display.print(buffer);
  display.setTextSize(1);
  for (int i=0; i<2; i++) {
    sprintf_P(buffer, (PGM_P)pgm_read_word(&(misc_strings[6])), global_settings.pulse_durations[i], global_settings.pulse_currents[i]);
    display.print(buffer);
  }

  print_ram();
  display.display();
}
