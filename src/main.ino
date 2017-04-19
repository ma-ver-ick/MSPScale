#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>

#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include "HX711.h"
#include "ADCTouch.h"
#include "Sleep_n0m1.h"

// # Configuration

// ## Display
#define OLED_MOSI                         9
#define OLED_CLK                          10
#define OLED_DC                           11
#define OLED_CS                           12
#define OLED_RESET                        8

// ## Buttons
#define BUTTON_LEFT                       A3
#define BUTTON_RIGHT                      A2
#define BUTTON_TOUCH_SAMPLES              100
#define BUTTON_TOLERANCE                  10

// ## MEASURING
#define SCALE_PIN0                        A1
#define SCALE_PIN1                        A0
#define SCALE_SPEED_PIN                   7
#define SCALE_WINDOW_MAX                  55
#define SCALE_WINDOW_MIN                  10
#define SCALE_CALIBRATION_SAMPLES_MIN     10
#define SCALE_CALIBRATION_SAMPLES_MAX     1000
#define SCALE_CALIBRATION_REPORT_AT       10

#define MAX_WEIGHT_GRAMS                  1000.0f
#define MAX_WEIGHT_KG                     5.0f
#define MAX_WEIGHT_MG                     1000.0f

#define PWR_DOWN_TIME_MIN                 10
#define PWR_DOWN_TIME_MAX                 210

#define CALIBRATION_FACTOR_MIN            10
#define CALIBRATION_FACTOR_MAX            500
// # /Configuration

// # State machine
#define STATE_MEASURE                     0
#define STATE_PWR_DOWN                    1
#define STATE_MENU                        2
byte _stateMachine_currentState = STATE_MEASURE;
byte _stateMachine_menuItem = 0;
// #/State machine

// # Global variables
HX711 _scale = HX711();
Adafruit_SSD1306 _display = Adafruit_SSD1306(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
Sleep _sleep;

long _measure_window[SCALE_WINDOW_MAX];
byte _measure_window_idx = 0;
long _measure_offset = 0;

unsigned long _lastReportingTime_Measure;
unsigned long _timeEnteredState;

int _ref_buttonLeft;
int _ref_buttonRight;

byte _buttonRightPressedLastIteration             = 0;
// #/Global variables

// # Menu
#define MENU_ITEM_PWR_DOWN_TIME                   0
#define MENU_ITEM_SCALE_CALIBRATION_SAMPLES       1
#define MENU_ITEM_SCALE_MEASURE_SAMPLES           2
#define MENU_ITEM_SCALE_SUB_GRAMM                 3
#define MENU_ITEM_HX711_HIGH_SPEED                4
#define MENU_ITEM_CALIBRATION_MODE                5
#define MENU_ITEM_CALIBRATION_FACTOR              6
#define MENU_ITEM_EXIT                            7
#define MENU_ITEM_RESET                           8

#define EEPROM_TRUE                               2
#define EEPROM_FALSE                              1

#define MENU_FAST_UPDATE_TOLERANCE                5
// #/Menu

// # Saved Settings
#define DEFAULT_CONFIG_PWR_DOWN_TIME              30
#define DEFAULT_CONFIG_SCALE_CALIBRATION_SAMPLES  100
#define DEFAULT_CONFIG_SCALE_MEASURE_SAMPLES      50
#define DEFAULT_CONFIG_SCALE_SUB_GRAMM            true
#define DEFAULT_CONFIG_HX711_HIGH_SPEED           HIGH
#define DEFAULT_CONFIG_CALIBRATION_MODE           false
#define DEFAULT_CONFIG_CALIBRATION_FACTOR         248.9487f

byte _config_pwr_down_time              = DEFAULT_CONFIG_PWR_DOWN_TIME; // seconds
byte _config_scale_calibration_samples  = DEFAULT_CONFIG_SCALE_CALIBRATION_SAMPLES;
byte _config_scale_measure_samples      = DEFAULT_CONFIG_SCALE_MEASURE_SAMPLES;
bool _config_scale_sub_gramm            = DEFAULT_CONFIG_SCALE_SUB_GRAMM;
bool _config_hx711_high_speed           = DEFAULT_CONFIG_HX711_HIGH_SPEED;
bool _config_calibration_mode           = DEFAULT_CONFIG_CALIBRATION_MODE;
float _config_calibration_factor        = DEFAULT_CONFIG_CALIBRATION_FACTOR;
// # /Saved Settings

// # PROGMEM
// # /PROGMEM

void setup() {
  // init scale
  _scale.begin(SCALE_PIN0, SCALE_PIN1);
  pinMode(SCALE_SPEED_PIN, OUTPUT);
  digitalWrite(SCALE_SPEED_PIN, HIGH);

  // init display, clear adafruit logo
  _display.begin(SSD1306_SWITCHCAPVCC);
  _display.clearDisplay();
  _display.display();
  _display.setTextColor(WHITE);

  loadFromEEPROM();

  switchToStateMeasure();
}

void loop() {
  if(_stateMachine_currentState == STATE_MENU) {
    loop_stateMenu();
  } else if(_stateMachine_currentState == STATE_MEASURE) {
    loop_stateMeasure();
  } else if(_stateMachine_currentState == STATE_PWR_DOWN) {
    loop_statePwrDown();
  }
}

void loop_stateMenu() {
  _display.clearDisplay();
  drawMenu();
  _display.display();

  if(readButtonLeft()) {
    _stateMachine_menuItem = (_stateMachine_menuItem + 1) % 9;
    delay(100);
  }
  if(readButtonRight()) {
    executeCurrentMenuItem();
    if(_buttonRightPressedLastIteration >= MENU_FAST_UPDATE_TOLERANCE) {
      delay(20);
    } else {
      delay(100);
      _buttonRightPressedLastIteration++;
    }
  } else {
    _buttonRightPressedLastIteration = 0;
  }
}

void loop_stateMeasure() {
  _measure_window_idx = (_measure_window_idx + 1) % _config_scale_measure_samples;
  _measure_window[_measure_window_idx] = _scale.read();

  int temp = (millis() - (unsigned long)_lastReportingTime_Measure);
  if(temp >= 250) {

    float v = 0;
    for(int i = 0; i < _config_scale_measure_samples; i++) {
      v += (_measure_window[i] - _measure_offset);
    }
    v = v / (_config_scale_measure_samples * 1.0f);

    _display.clearDisplay();
    if(_config_calibration_mode) {
      _display.setFont(&FreeSansBold12pt7b);
      _display.setTextSize(1);
      drawTextCentered(String(v, 0));
    } else {
      v = v / _config_calibration_factor;
      drawWeight(v);
    }
    _display.display();

    _lastReportingTime_Measure = millis();
  }

  if(readButtonRight()) {
    calibrateTare();
  }
  if(readButtonLeft()) {
    switchToStateMenu();
  }

  if( (millis() - _timeEnteredState) > (unsigned long)(_config_pwr_down_time * 1000L)) {
    switchToStatePwrDown();
  }

  // 80 SPS from HX711 would mean a sample every 12.5ms, so this fast running loop could be throttled to safe power.
  _sleep.sleepDelay(10);
}

void calibrateTare() {
  _display.clearDisplay();
  drawTextCentered(F("Tare"));
  _display.display();
  delay(300);

  // Calculate the offset
  float sum = 0;
  for(int i = 0; i < _config_scale_measure_samples; i++) {
    sum += _measure_window[i];
  }

  _measure_offset = sum / _config_scale_measure_samples;
}

void loop_statePwrDown() {
  _sleep.sleepDelay(1000);

  if(readButtonRight()) {
    switchToStateMeasure();
  }
}

void switchToStateMeasure() {
  _stateMachine_currentState = STATE_MEASURE;
  _display.ssd1306_command(SSD1306_DISPLAYON);
  _display.clearDisplay();

  digitalWrite(SCALE_SPEED_PIN, _config_hx711_high_speed);
  _buttonRightPressedLastIteration = 0;

  _scale.power_up();

  calibrate();

  _sleep.idleMode();

  _lastReportingTime_Measure = millis();
  _timeEnteredState = millis();
}

void switchToStatePwrDown() {
  _stateMachine_currentState = STATE_PWR_DOWN;
  _display.clearDisplay();
  _display.display();

  _display.ssd1306_command(SSD1306_DISPLAYOFF);

  _sleep.pwrDownMode();
  _scale.power_down();
}

void switchToStateMenu() {
  _stateMachine_currentState = STATE_MENU;
  _stateMachine_menuItem = 0;
}

// draws the current weight, assumes that the weight is given as grams
void drawWeight(float weight) {
  _display.setTextSize(1);
  _display.setFont(&FreeSansBold12pt7b);

  byte decimalPlaces = _config_scale_sub_gramm ? 1 : 0;

  byte progress = 0;
  float progressMax = 0;
  String text;
  boolean neg = weight < 0;
  weight = abs(weight);

  if(neg) {
    text = F("-");
  } else {
    text = "";
  }

  text.reserve(10);

  if(weight > MAX_WEIGHT_KG * 1000.0f) {
    text = String(F("MAX"));

    progressMax = 1000.0f;
    progress = 0;
  } else if(weight > 1000.0f) {
    weight = weight / 1000.0f;
    text += String(weight, 3) + F(" kg");

    progressMax = MAX_WEIGHT_KG;
    progress = (SSD1306_LCDWIDTH - 10) / progressMax * weight;
  } else if(weight > 1) {
    text += String(weight, decimalPlaces) + F(" g");

    progressMax = MAX_WEIGHT_GRAMS;
    progress = (SSD1306_LCDWIDTH - 10) / progressMax * weight;
  } else if(_config_scale_sub_gramm) {
    weight = weight * 1000.0f;
    text += String(weight, 0) + F(" mg");

    progressMax = MAX_WEIGHT_MG;
    progress = (SSD1306_LCDWIDTH - 10) / progressMax * weight;
  } else {
    progressMax = MAX_WEIGHT_GRAMS;
    text = F("0 g");
    progress = 0;
  }

  int16_t  x1, y1;
  uint16_t w, h;

  _display.getTextBounds(text.c_str(), (int16_t)0, (int16_t)0, &x1, &y1, &w, &h);

  byte x = SSD1306_LCDWIDTH - w;
  x = x / 2;

  byte y = SSD1306_LCDHEIGHT - h - 10; // we are positioning the bottom of the baseline

  _display.setCursor(x, y);
  _display.println(text);

  if(neg) {
    _display.fillRect(5 + progress, 50, (SSD1306_LCDWIDTH - 10) - progress, 5, 1);
  } else {
    _display.fillRect(5, 50, progress, 5, 1);
  }
}

// Draws a text horizontally and vertically centered
void drawTextCentered(String text) {
  int16_t  x1, y1;
  uint16_t w, h;

  _display.getTextBounds(text.c_str(), (int16_t)0, (int16_t)0, &x1, &y1, &w, &h);

  byte x = SSD1306_LCDWIDTH - w;
  x = x / 2;

  byte y = (SSD1306_LCDHEIGHT/2.0) + (h/2.0); // we are positioning the bottom of the baseline
  _display.setCursor(x, y);
  _display.println(text);
}

// Draws a single menu entry
void drawSingleMenuEntry(String line1, String line2, String value) {
  _display.setFont();
  _display.setTextSize(1);

  _display.setCursor(0, 15);
  _display.println(line1);

  _display.setCursor(_display.getCursorX(), _display.getCursorY() + 5);
  _display.println(line2);

  int16_t  x1, y1;
  uint16_t w, h;

  _display.setTextSize(2);
  _display.getTextBounds(value.c_str(), (int16_t)0, (int16_t)0, &x1, &y1, &w, &h);

  byte x = SSD1306_LCDWIDTH - w;
  x = x - 10;

  byte y = SSD1306_LCDHEIGHT - h - 10; // we are positioning the bottom of the baseline
  _display.setCursor(x, y);
  _display.println(value);
}

bool readButtonRight() {
  int value0 = ADCTouch.read(BUTTON_RIGHT, BUTTON_TOUCH_SAMPLES);
  return value0 - _ref_buttonRight >= BUTTON_TOLERANCE;
}

bool readButtonLeft() {
  int value0 = ADCTouch.read(BUTTON_LEFT, BUTTON_TOUCH_SAMPLES);
  return value0 - _ref_buttonLeft >= BUTTON_TOLERANCE;
}

void calibrate() {
  _display.setTextSize(1);
  _display.setFont(&FreeSansBold12pt7b);
  _display.setTextColor(WHITE);
  _display.setCursor(0, 30);

  _display.println(F("Calibrating"));
  _display.display();
  delay(200); // let the user remove his finger!

  float progressBarStep = (SSD1306_LCDWIDTH - 10) / (_config_scale_calibration_samples * 1.0f);
  byte progressBar = 0;

  // Calculate the offset
  float sum = 0;
  for(int i = 0; i < _config_scale_calibration_samples; i++) {
    long temp = _scale.read();

    _measure_window[i % SCALE_WINDOW_MAX] = temp;
    sum += temp;
    if(i % 10 == 0) {
      progressBar = progressBarStep * i;
      _display.fillRect(5, 50, progressBar, 5, 1);
      _display.display();
    }
  }

  _display.fillRect(5, 50, SSD1306_LCDWIDTH - 10, 5, 1);
  _display.display();

  _measure_offset = sum / (_config_scale_calibration_samples * 1.0);
  // Serial.println(_measure_offset);

  _ref_buttonLeft = ADCTouch.read(A3, 1000);
  _ref_buttonRight = ADCTouch.read(A2, 1000);
}

byte eeprom_read(byte address, byte defaultValue) {
  byte ret = EEPROM.read(address);
  if(ret == 0 || ret == 255) {
    return defaultValue;
  }
  return ret;
}

void drawMenu() {
  const String EMPTY_STRING = F("");
  const String YES = F("Yes?");
  if(_stateMachine_menuItem == MENU_ITEM_PWR_DOWN_TIME) {
    drawSingleMenuEntry(F("Power down time"), EMPTY_STRING, String(_config_pwr_down_time));
  } else if (_stateMachine_menuItem == MENU_ITEM_SCALE_CALIBRATION_SAMPLES) {
    drawSingleMenuEntry(F("Amount of samples for"), F("calibration"), String(_config_scale_calibration_samples));
  } else if (_stateMachine_menuItem == MENU_ITEM_SCALE_MEASURE_SAMPLES) {
    drawSingleMenuEntry(F("Amount of samples for"), F("measurement"), String(_config_scale_measure_samples));
  } else if (_stateMachine_menuItem == MENU_ITEM_SCALE_SUB_GRAMM) {
    drawSingleMenuEntry(F("Show sub gramm?"), EMPTY_STRING, _config_scale_sub_gramm ? F("On") : F("Off"));
  } else if (_stateMachine_menuItem == MENU_ITEM_HX711_HIGH_SPEED) {
    drawSingleMenuEntry(F("HX711 update rate"), EMPTY_STRING, _config_hx711_high_speed ? F("80 Hz") : F("10 Hz"));
  } else if (_stateMachine_menuItem == MENU_ITEM_CALIBRATION_MODE) {
    drawSingleMenuEntry(F("Calibration mode?"), EMPTY_STRING, _config_calibration_mode ? F("On") : F("Off"));
  } else if (_stateMachine_menuItem == MENU_ITEM_RESET) {
    drawSingleMenuEntry(F("Reset to defaults?"), EMPTY_STRING, YES);
  } else if (_stateMachine_menuItem == MENU_ITEM_EXIT) {
    drawSingleMenuEntry(F("Exit?"), EMPTY_STRING, YES);
  } else if(_stateMachine_menuItem == MENU_ITEM_CALIBRATION_FACTOR) {
    drawSingleMenuEntry(F("Calibration factor"), EMPTY_STRING, String(_config_calibration_factor, 4));
  }
}

void executeCurrentMenuItem() {
  if(_stateMachine_menuItem == MENU_ITEM_PWR_DOWN_TIME) {
    _config_pwr_down_time = (_config_pwr_down_time + 10) % PWR_DOWN_TIME_MAX;
    _config_pwr_down_time = constrain(_config_pwr_down_time, PWR_DOWN_TIME_MIN, PWR_DOWN_TIME_MAX);
  } else if (_stateMachine_menuItem == MENU_ITEM_SCALE_CALIBRATION_SAMPLES) {
    _config_scale_calibration_samples = (_config_scale_calibration_samples + 5) % (SCALE_CALIBRATION_SAMPLES_MAX);
    _config_scale_calibration_samples = constrain(_config_scale_calibration_samples, SCALE_CALIBRATION_SAMPLES_MIN, SCALE_CALIBRATION_SAMPLES_MAX);
  } else if (_stateMachine_menuItem == MENU_ITEM_SCALE_MEASURE_SAMPLES) {
    _config_scale_measure_samples = (_config_scale_measure_samples + 5) % (SCALE_WINDOW_MAX);
    _config_scale_measure_samples = constrain(_config_scale_measure_samples, SCALE_WINDOW_MIN, SCALE_WINDOW_MAX);
  } else if (_stateMachine_menuItem == MENU_ITEM_SCALE_SUB_GRAMM) {
    _config_scale_sub_gramm = !_config_scale_sub_gramm;
  } else if (_stateMachine_menuItem == MENU_ITEM_HX711_HIGH_SPEED) {
    _config_hx711_high_speed = _config_hx711_high_speed == HIGH ? LOW : HIGH;
  } else if (_stateMachine_menuItem == MENU_ITEM_CALIBRATION_MODE) {
    _config_calibration_mode = !_config_calibration_mode;
  } else if (_stateMachine_menuItem == MENU_ITEM_RESET) {
    resetToDefaults();
  } else if (_stateMachine_menuItem == MENU_ITEM_EXIT) {
    switchToStateMeasure();
    return; // prevent store to eeprom
  } else if (_stateMachine_menuItem == MENU_ITEM_CALIBRATION_FACTOR) {
    if(_buttonRightPressedLastIteration >= MENU_FAST_UPDATE_TOLERANCE) {
      _config_calibration_factor = _config_calibration_factor + 0.5f;
    } else {
      _config_calibration_factor = _config_calibration_factor + 0.0001f;
    }
    if(_config_calibration_factor > CALIBRATION_FACTOR_MAX) {
      _config_calibration_factor = _config_calibration_factor - CALIBRATION_FACTOR_MAX;
    }
    _config_calibration_factor = constrain(_config_calibration_factor, CALIBRATION_FACTOR_MIN, CALIBRATION_FACTOR_MAX);
  }
  storeToEEPROM();
}

void resetToDefaults() {
  _config_pwr_down_time               = DEFAULT_CONFIG_PWR_DOWN_TIME;
  _config_scale_calibration_samples   = DEFAULT_CONFIG_SCALE_CALIBRATION_SAMPLES;
  _config_scale_measure_samples       = DEFAULT_CONFIG_SCALE_MEASURE_SAMPLES;
  _config_scale_sub_gramm             = DEFAULT_CONFIG_SCALE_SUB_GRAMM;
  _config_hx711_high_speed            = DEFAULT_CONFIG_HX711_HIGH_SPEED;
  _config_calibration_mode            = DEFAULT_CONFIG_CALIBRATION_MODE;
  _config_calibration_factor          = DEFAULT_CONFIG_CALIBRATION_FACTOR;

  _display.clearDisplay();
  _display.setTextSize(1);
  _display.setFont(&FreeSansBold12pt7b);
  drawTextCentered(F("Reset"));
  _display.display();
  delay(500);
}

void loadFromEEPROM() {
  _config_pwr_down_time               = eeprom_read(0, DEFAULT_CONFIG_PWR_DOWN_TIME);
  _config_scale_calibration_samples   = eeprom_read(1, DEFAULT_CONFIG_SCALE_CALIBRATION_SAMPLES);
  _config_scale_measure_samples       = eeprom_read(2, DEFAULT_CONFIG_SCALE_MEASURE_SAMPLES);
  _config_scale_sub_gramm             = eeprom_read(3, DEFAULT_CONFIG_SCALE_SUB_GRAMM ? EEPROM_TRUE : EEPROM_FALSE) == EEPROM_TRUE ? true : false;
  _config_hx711_high_speed            = eeprom_read(4, DEFAULT_CONFIG_HX711_HIGH_SPEED == HIGH ? EEPROM_TRUE : EEPROM_FALSE) == EEPROM_TRUE ? HIGH : LOW;
  _config_calibration_mode            = eeprom_read(5, DEFAULT_CONFIG_CALIBRATION_MODE ? EEPROM_TRUE : EEPROM_FALSE) == EEPROM_TRUE ? true : false;
  EEPROM.get(6, _config_calibration_factor);
}

void storeToEEPROM() {
  EEPROM.update(0, _config_pwr_down_time);
  EEPROM.update(1, _config_scale_calibration_samples);
  EEPROM.update(2, _config_scale_measure_samples);
  EEPROM.update(3, _config_scale_sub_gramm ? EEPROM_TRUE : EEPROM_FALSE);
  EEPROM.update(4, _config_hx711_high_speed == HIGH ? EEPROM_TRUE : EEPROM_FALSE);
  EEPROM.update(5, _config_calibration_mode ? EEPROM_TRUE : EEPROM_FALSE);
  EEPROM.put(6, _config_calibration_factor);
}



// # Error checking
#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif
// # /Error Checking
