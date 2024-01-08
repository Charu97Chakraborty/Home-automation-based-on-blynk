/*
Title         :   Home automation based on blynk
Submitted By  :   Charushila Chakraborty
Date          :   15.04.2023
Description   :   To control light's brigntness with ldr sensor , monitor temperature , monitor water level in the tank through blynk app
Pheripherals  :   Arduino UNO , Temperature system, LED, LDR module, Serial Tank, Blynk cloud, Blynk App.
 */

// Define Blynk project details and authentication token
#define BLYNK_TEMPLATE_ID "TMPL3JD_aym0Q"
#define BLYNK_TEMPLATE_NAME "Home Automation"
#define BLYNK_AUTH_TOKEN "eu7oxtWOe-RSrvYciuYlX5pvcKCndeZC"

// Define constants for ON and OFF states
#define ON    1
#define OFF   0

// Define Blynk virtual pins for widgets
#define TEMPERATURE_GAUGE       V1
#define COOLER_V_PIN            V0
#define HEATER_V_PIN            V2
#define WATER_VOL_GAUGE         V3
#define INLET_V_PIN             V4
#define OUTLET_V_PIN            V5
#define BLYNK_TERMINAL_V_PIN    V6

// Define pinouts for LDR and Garden Light
#define LDR_SENSOR       A1
#define GARDEN_LIGHT     3

void init_ldr(void);
void brightness_control(void);

// Define commands and states for Serial Tank
#define INLET_VALVE  0x00
#define OUTLET_VALVE 0x01
#define HIGH_FLOAT 0x10
#define LOW_FLOAT  0x11
#define VOLUME 0x30
#define   ENABLE  0x01
#define   DISABLE 0x00

void init_serial_tank(void);
void enable_inlet(void);
void enable_outlet(void);
void disable_inlet(void);
void disable_outlet(void);
unsigned int volume(void);

// Define pins for Heater and Cooler
#define HEATER                5
#define COOLER                4
#define TEMPERATURE_SENSOR    A0

float read_temperature(void);
void init_temperature_system(void);
void cooler_control(bool control);
void heater_control(bool control);

// Include necessary libraries
#include <SPI.h>
#include <Ethernet.h>
#include <BlynkSimpleEthernet.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Initialize variables and objects
LiquidCrystal_I2C lcd(0x27, 16, 2);

char auth[] = BLYNK_AUTH_TOKEN;
bool heater_sw, inlet_sw, outlet_sw;
unsigned int tank_volume;
BlynkTimer timer;

// Function to initialize LDR
void init_ldr(void) 
{
  pinMode(GARDEN_LIGHT, OUTPUT);
}
// Function to control brightness based on LDR input
unsigned int input;

void brightness_control(void) 
{
  input= analogRead(LDR_SENSOR);
  input=(1023-input)/4;
  analogWrite(GARDEN_LIGHT,input);
  delay(100);
}
// Function to initialize Serial Tank communication
void init_serial_tank(void) 
{
  Serial.begin(19200);
  Serial.write(0xFF);
  Serial.write(0xFF);
  Serial.write(0xFF);   
}
// Function to read water volume from Serial Tank
unsigned int volume_value;
unsigned char valueh, valuel;

unsigned int volume(void) 
{
  Serial.write(VOLUME);
  while(!Serial.available());
  valueh=Serial.read();
  while(!Serial.available());
  valuel=Serial.read();
  volume_value=(valueh<<8)|valuel;
  return volume_value;
}
// Functions to control inlet and outlet valves of Serial Tank
void enable_inlet(void) 
{
  Serial.write(INLET_VALVE);
  Serial.write(ENABLE);    
}  

void disable_inlet(void) 
{
  Serial.write(INLET_VALVE);
  Serial.write(DISABLE);
}  

void enable_outlet(void) 
{  
  Serial.write(OUTLET_VALVE);
  Serial.write(ENABLE);
}

void disable_outlet(void) 
{  
  Serial.write(OUTLET_VALVE);
  Serial.write(DISABLE);
}
// Function to initialize temperature control system
void init_temperature_system(void) 
{
  pinMode(HEATER, OUTPUT);
  pinMode(COOLER, OUTPUT);
  digitalWrite(HEATER, LOW);
  digitalWrite(COOLER, LOW);
}
// Function to read temperature from sensor
float read_temperature(void) 
{
  float temperature;
  temperature=(((analogRead(A0)*(float)5/1024))/(float)0.01);
  return temperature;
}
// Functions to control cooler and heater
void cooler_control(bool control) 
{
  digitalWrite(COOLER,control);
}

void heater_control(bool control) 
{
  digitalWrite(HEATER,control);
}
// Function to initialize the LCD display
void init_lcd() 
{
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.home();
}
// Functions to update LCD with temperature, volume, and status
void update_lcd_temperature(float temperature) 
{
  lcd.setCursor(0, 0);
  lcd.print("T=");
  lcd.print(temperature);
}

void update_lcd_tank_volume(unsigned int volume) 
{
  lcd.setCursor(0, 1);
  lcd.print("V=");
  lcd.print(volume);
}

void update_lcd_cooler_status(bool isOn) 
{
  lcd.setCursor(7, 0);
  lcd.print(isOn ? "CO_LR_ON" : "CO_LR_OFF");
}

void update_lcd_heater_status(bool isOn) 
{
  lcd.setCursor(7, 0);
  lcd.print(isOn ? "HT_TR_ON " : "HE_TR_OFF");
}

void update_lcd_inlet_status(bool isOn) 
{
  lcd.setCursor(7, 1);
  lcd.print(isOn ? "IN_FL_ON " : "IN_FL_OFF");
}

void update_lcd_outlet_status(bool isOn) 
{
  lcd.setCursor(7, 1);
  lcd.print(isOn ? "OT_FL_ON " : "OT_FL_OFF");
}

void setup(void) 
{
  Blynk.begin(auth);
  init_ldr();
  init_temperature_system();
  init_serial_tank();  
  init_lcd();  // Initialize the LCD display
  timer.setInterval(500L,update_temperature_reading);
}

void loop(void) 
{
  brightness_control();
  tank_volume=volume();
  handle_temp();
  handle_tank();
  Blynk.run();
  timer.run();
}

BLYNK_WRITE(COOLER_V_PIN) 
{
  int value=param.asInt();
  if(value)  
  {
    cooler_control(ON);
    update_lcd_cooler_status(true);
  }
  else {
    cooler_control(OFF);
    update_lcd_cooler_status(false);
  }  
}

BLYNK_WRITE(HEATER_V_PIN ) 
{
  int heater_sw=param.asInt();
  if(heater_sw)  
  {
    heater_control(ON);
    update_lcd_heater_status(true);
  }
  else 
  {
    heater_control(OFF);
    update_lcd_heater_status(false);
  }
}

BLYNK_WRITE(INLET_V_PIN) 
{
  inlet_sw=param.asInt();
  if(inlet_sw) 
  {
    enable_inlet();
    update_lcd_inlet_status(true);
  }
  else 
  {
    disable_inlet();
    update_lcd_inlet_status(false);
  }
}

BLYNK_WRITE(OUTLET_V_PIN) 
{
  outlet_sw=param.asInt();
  if(outlet_sw) 
  {
    enable_outlet();
    update_lcd_outlet_status(true);
  }
  else 
  {
    disable_outlet();
    update_lcd_outlet_status(false);
  }
}

void update_temperature_reading() 
{
  float temperature = read_temperature();
  Blynk.virtualWrite(TEMPERATURE_GAUGE, temperature);
  update_lcd_temperature(temperature);
  Blynk.virtualWrite(WATER_VOL_GAUGE, volume());
  update_lcd_tank_volume(volume());
}

void handle_temp(void) 
{
  if((read_temperature()>float(30))) 
  {
    heater_sw =0;
    heater_control(OFF);
    Blynk.virtualWrite(BLYNK_TERMINAL_V_PIN,"Temperature is above 30 degree Celcius\n, so turning OFF the heater\n");
    Blynk.virtualWrite(HEATER_V_PIN,OFF);
    update_lcd_heater_status(false);
  }  
}

void handle_tank(void) 
{
  if((tank_volume< 1500)&&(inlet_sw==OFF)) 
  {
    enable_inlet();
    inlet_sw=ON;
    Blynk.virtualWrite(INLET_V_PIN, ON);
    update_lcd_inlet_status(true);
    Blynk.virtualWrite(BLYNK_TERMINAL_V_PIN,"Water volume is less than 1500\n");
    Blynk.virtualWrite(BLYNK_TERMINAL_V_PIN,"Turning ON the inlet valve\n");
  }

  if((tank_volume == 3000)&&(inlet_sw==ON)) 
  {
    disable_inlet();
    inlet_sw=OFF;
    Blynk.virtualWrite(INLET_V_PIN, OFF);
    update_lcd_inlet_status(false);
    Blynk.virtualWrite(BLYNK_TERMINAL_V_PIN,"Water level is Full\n");
    Blynk.virtualWrite(BLYNK_TERMINAL_V_PIN,"Turning OFF the inlet valve\n");
  }
}
