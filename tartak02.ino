//  #include <LiquidCrystal_I2C.h>
#include "DFRobot_GP8403.h"
#include "EmonLib.h"
#include <WiFi.h>

const char* ssid = "jasio";
const char* password = "Polinezyjska7/6";

EnergyMonitor emon1;
EnergyMonitor emon2;
EnergyMonitor emon3;
EnergyMonitor emon4;

//  LiquidCrystal_I2C lcd(0x27, 16, 2);
DFRobot_GP8403 dac(&Wire,0x5F);

int VOLTS;

#define BUTTON_PIN15 15
#define BUTTON_PIN5 5
#define BUTTON_PIN14 14

int lastState = HIGH;
int currentState15;
int currentState5;
int currentState14;

int EIGHT = 8000;


void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }

}


void setup()
{

  initWiFi();

//  lcd.init();
//  lcd.backlight();

  Serial.begin(115200);

  emon1.current(36, 7.1);             // Current: input pin, calibration.
  emon2.current(39, 7.1);             // Current: input pin, calibration.
  emon3.current(34, 7.1);             // Current: input pin, calibration.
  emon4.current(35, 7.1);             // Current: input pin, calibration.

  while(dac.begin()!=0){
  Serial.println("init error");
  delay(1000);

  pinMode(BUTTON_PIN15, INPUT_PULLUP);
  pinMode(BUTTON_PIN5, INPUT_PULLUP);
  pinMode(BUTTON_PIN14, INPUT_PULLUP);
  
   }

  Serial.println("init succeed");
  dac.setDACOutRange(dac.eOutputRange10V);//Set the output range as 0-10V
  dac.setDACOutVoltage(VOLTS,0);//The DAC value for 3.5V output in OUT0 channel
  delay(1000);
  dac.store(); //Save the set 3.5V voltage inside the chip

   
}

void loop()
{
  // read the state of the switch/button:
  currentState15 = digitalRead(BUTTON_PIN15);
  currentState5 = digitalRead(BUTTON_PIN5);
  currentState14 = digitalRead(BUTTON_PIN14);

  Serial.print("RRSI: ");
  Serial.println(WiFi.RSSI());
  Serial.println(WiFi.localIP());

  if(currentState5 == LOW) {
    Serial.println("PRESSED 5");
    VOLTS = 4000;
    Serial.println(VOLTS);
    dac.setDACOutVoltage(VOLTS,0);
  }

  else if (currentState15 == LOW) {
    Serial.println("PRESSED 15");
    VOLTS = 6000;
    Serial.println(VOLTS);
    dac.setDACOutVoltage(VOLTS,0);
  }

  else if (currentState14 == LOW) {
    Serial.println("PRESSED 14");
    VOLTS = 8000;
    Serial.println(VOLTS);
    dac.setDACOutVoltage(VOLTS,0);
  }
  
  else {
    Serial.println("NOT PRESSED");
    VOLTS = 2000;
    Serial.println(VOLTS);
    dac.setDACOutVoltage(VOLTS,0);
  }


  
double Irms1 = emon1.calcIrms(3333);  // Calculate Irms only
double Irms2 = emon2.calcIrms(3333);  // Calculate Irms only
double Irms3 = emon3.calcIrms(3333);  // Calculate Irms only
double Irms4 = emon4.calcIrms(3333);  // Calculate Irms only
//  Serial.print("WATTS1: ");
//  Serial.print(Irms1*230.0);         // Apparent power

  Serial.print(" AMPS1: ");
  Serial.println(Irms1);          // Irms
  
//  Serial.print("WATTS2: ");
//  Serial.print(Irms2*230.0);         // Apparent power
  Serial.print(" AMPS2: ");
  Serial.println(Irms2);          // Irms

    Serial.print(" AMPS3: ");
  Serial.println(Irms3);          // Irms

  Serial.print(" AMPS4: ");
  Serial.println(Irms4);          // Irms

//  lcd.setCursor(0, 0);
//  lcd.print("1");
//  lcd.setCursor(2, 0);
//  lcd.print(Irms1);
//  lcd.setCursor(8, 0);
//  lcd.print("2");
//  lcd.setCursor(10, 0);
//  lcd.print(Irms2);
//  lcd.setCursor(0, 1);
//  lcd.print("POSUW");
//  lcd.setCursor(8, 1);
//  lcd.print(VOLTS/100);
//  lcd.setCursor(11, 1);
//  lcd.print("%");


  // save the last state
//  lastState = currentState;

    // read the state of the switch/button:
//  int buttonState = digitalRead(BUTTON_PIN);

  // print out the button's state
//  Serial.println(buttonState);
  
}
