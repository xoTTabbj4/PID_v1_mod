#define INIT_ADDR 1023  // номер резервной ячейки
#define INIT_KEY 88     // ключ первого запуска. 0-254, на выбор
#define BTN_UP_PIN 3    // пин кнопки вверх
#define BTN_DOWN_PIN 4  // пин кнопки вниз
#define LED_PIN 5       // пин светодиода
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include "PID_v1.h"
double Setpoint, Output;
double Kp = 550, Ki = 1, Kd = 300;
unsigned long windowStartTime;
double temp = 0;
double celsius = 0;
double filtered = 0;
PID heater(&filtered, &Output, &temp, Kp, Ki, Kd, DIRECT);
int WindowSize = 10000;
int counter = 0;
int k = 0;
int buttonState = 0;
byte i;
byte type_s;
byte data[12];
byte addr[8];
float average = 0;
float sqrdev = 0.03;
float varProcess = 0.001;
float Pc = 0.0;
float G = 0.0;
float P = 1.0;
float Xp = 0.0;
float Zp = 0.0;
float Xe = 0.0;
float draindelta = 0.2;
OneWire  ds(8);
LiquidCrystal_I2C lcd(0x3f, 16, 2);

void setup(void) {
  if (EEPROM.read(INIT_ADDR) != INIT_KEY) { // первый запуск
    EEPROM.write(INIT_ADDR, INIT_KEY);    // записали ключ

    for (int i = 0 ; i < (EEPROM.length() - 1) ; i++) {
      EEPROM.write(i, 0);
    }
    //put the coefs;
  }
  Serial.begin(9600);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  lcd.init();
  lcd.backlight();
  digitalWrite(2, HIGH);
  digitalWrite(4, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  windowStartTime = millis();
  heater.SetOutputLimits(0, WindowSize);
  heater.SetSampleTime(50);
  heater.SetMode(AUTOMATIC);
  digitalWrite(3, LOW);//overheat and red LED
}

void checkbutton() {
  buttonState = !digitalRead(5);
  if (buttonState == 1) {
    counter = 1;
  }
  else {
    counter = 0;
  }
}
void checktemp() {
  if (counter == 0) {
    temp = 37.7;
  }
  else {
    temp = 38.7;
  }
}
void preparesensor() {
  if ( !ds.search(addr))
  {
    ds.reset_search();
    return;
  }
  if (OneWire::crc8(addr, 7) != addr[7]) {
    return;
  }
  switch (addr[0])
  {
    case 0x10:
      type_s = 1;
      break;
    case 0x28:
      type_s = 0;
      break;
    case 0x22:
      type_s = 0;
      break;
    default:
      return;
  }
  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);
  ds.reset();
  ds.select(addr);
  ds.write(0xBE);
  for ( i = 0; i < 9; i++) {
    data[i] = ds.read();
  }
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3;
    if (data[7] == 0x10) {
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    if (cfg == 0x00) raw = raw & ~7;
    else if (cfg == 0x20) raw = raw & ~3;
    else if (cfg == 0x40) raw = raw & ~1;
  }
  celsius =  (double)raw / 16.0;

}
void filtered_temp() {
  Pc = P + varProcess;
  G = Pc / (Pc + sqrdev);
  P = (1 - G) * Pc;
  Xp = Xe;
  Zp = Xp;
  Xe = G * (celsius - Zp) + Xp; // "фильтрованное" значение
  filtered = Xe;

  Serial.print(celsius * 100);
  Serial.print(" ");
  Serial.print(filtered * 100);
  Serial.print(" ");
  Serial.print(Output);
  Serial.println("");
}

void pid_heater() {
  heater.Compute();
  if (millis() - windowStartTime > WindowSize)
  {
    windowStartTime += WindowSize;
  }
  if (Output < millis() - windowStartTime) digitalWrite(2, HIGH);
  else digitalWrite(2, LOW);
}
void logical_op() {
  if (celsius > (temp + draindelta)) {
    lcd.setCursor(14, 0);
    lcd.print("OH");
    digitalWrite(4, HIGH);//overheat and red LED
    digitalWrite(2, HIGH);
  }
  if (filtered == temp) {
    digitalWrite(7, HIGH);
    digitalWrite(6, LOW);
    digitalWrite(4, LOW);
    lcd.setCursor(14, 0);
    lcd.print("NL");
  }
  else if (filtered < temp) {
    digitalWrite(6, HIGH);
    digitalWrite(7, LOW);
    digitalWrite(4, LOW);
    lcd.setCursor(14, 0);
    lcd.print("AB");
  }

}
void LCD_draw() {
  lcd.setCursor(0, 0);
  lcd.print(celsius);
  lcd.setCursor(6, 0);
  lcd.print(filtered);
  lcd.setCursor(0, 1);
  lcd.print(Output);
  lcd.setCursor(14, 1);
  lcd.print("M");
  lcd.setCursor(15, 1);
  lcd.print((counter + 1));

}
void loop(void) {
  checkbutton();
  checktemp();
  preparesensor();
  filtered_temp();
  pid_heater();
  logical_op();
  LCD_draw();
}
