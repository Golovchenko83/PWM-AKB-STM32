#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <GyverTimer.h>
#include <LiquidCrystal_I2C_Hangul.h>
#include <OneWire.h>
#include <EEPROM.h>

#define ENC_A PB11 // пин энкодера
#define ENC_B PB10 // пин энкодера

GTimer analog(US, 500);
GTimer lcd_up(MS, 250);
GTimer eeprom_w(MS, 5000);
int h, h_s, p = 8, p_set;
int An0, Anal_raw, Aset_raw, V_raz, pwm_output, An_tik, An0_lcd_up;
HardwareTimer pwm1 TIM1;
LiquidCrystal_I2C_Hangul lcd(0x27, 20, 4);
volatile float encCounter, encCounter_lcd_up;
volatile byte state0, lastState, turnFlag, write_eep;

void EEPROM_float_write(int addr, float val)
{ /* запись в ЕЕПРОМ*/
  byte *x = (byte *)&val;
  for (byte i = 0; i < 4; i++)
    EEPROM.write(i + addr, x[i]);
}

float EEPROM_float_read(int addr)
{ /* чтение из ЕЕПРОМ*/
  byte x[4];
  for (byte i = 0; i < 4; i++)
    x[i] = EEPROM.read(i + addr);
  float *y = (float *)&x;
  return y[0];
}

void ENC()
{
  state0 = digitalRead(ENC_A);

  if (state0 != lastState)
  {
    encCounter += (digitalRead(ENC_B) != lastState) ? 0.5 : -0.5;
    lastState = state0;
    Aset_raw = map(encCounter, 0, 2355, 0, 1022);
    write_eep = 1;
  }
}

void PWM()
{
  V_raz = Anal_raw - Aset_raw;

  if (V_raz > 0 && pwm_output > 0)
  {
    pwm_output--;
  }
  else if (V_raz < 0 && pwm_output < 400)
  {
    pwm_output++;
  }
  pwm1.setCaptureCompare(2, pwm_output);
}

void loop()
{
  if (analog.isReady())
  {
    Anal_raw = analogRead(0);
    PWM();
    An0 = An0 + Anal_raw;
    An_tik++;
  }

  ENC();

  if (lcd_up.isReady())
  {
    An0 = An0 / An_tik;

    if (An0_lcd_up != An0 || encCounter_lcd_up != encCounter)
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("AKB_Volt:");
      float v_A0 = map(An0, 0, 1022, 0, 2355);
      float V1 = v_A0 / 100;
      lcd.print(V1, 2);
      lcd.setCursor(0, 1);
      lcd.print("SET_Volt:");
      lcd.print(encCounter / 100);
      An0_lcd_up = An0;
      encCounter_lcd_up = encCounter;
      if (V1 < 12)
      {
        digitalWrite(PB5, HIGH);
      }
      else if (V1 > 12.5)
      {
        digitalWrite(PB5, LOW);
      }
    }
    An0 = 0;
    An_tik = 0;
  }

  if (eeprom_w.isReady() && write_eep == 1)
  {
    write_eep = 0;
    if (EEPROM_float_read(4) != encCounter)
    {
      EEPROM_float_write(4, encCounter);
    }
  }
}

void setup()
{
  Wire.setSCL(PB6);
  Wire.setSDA(PB7);
  pinMode(PB11, INPUT);
  pinMode(PB10, INPUT);
  pinMode(PB5, OUTPUT);
  lcd.init();
  lcd.backlight();
  Wire.setClock(400000);
  pwm1.setMode(2, TIMER_OUTPUT_COMPARE_PWM1, PA9);
  // pwm1.attachInterrupt(2, imstop);
  pwm1.setPrescaleFactor(6);
  pwm1.setOverflow(400);
  pwm1.resumeChannel(2);
  pwm1.setCaptureCompare(2, 0);
  EEPROM_float_write(4, 1420);
  encCounter = EEPROM_float_read(4);
}