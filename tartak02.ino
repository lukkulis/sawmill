//  #include <LiquidCrystal_I2C.h>
#include "DFRobot_GP8403.h"
#include "EmonLib.h"
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <HTTPClient.h>
HTTPClient https;
const char* ssid = "tartak01";
const char* password = "tartak01";

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

// Variables to save date and time
String formattedDate;
String dayStamp;
String timeStamp;


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
#define RELAY 32

int lastState = HIGH;
int currentState15;
int currentState5;
int currentState14;

int EIGHT = 8000;

long previousMillis;
long currentMillis;
long interval=100;
float total1=0;
float total2=0;
float total3=0;
float total4=0;
float adjusted_sample1;
float adjusted_sample2;
float adjusted_sample3;
float adjusted_sample4;

int counter=0;
String TRYB;

float sample=70;

// Google script ID and required credentials
String GOOGLE_SCRIPT_ID = "AKfycbyQpgWMu66H0sZIotdExgS-oW5_roxzVpXQSyRm5OpQnw5wDQn_cRXi31ysDyqdLCYLbA";

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
//    Serial.print('.');
    delay(1000);
  }

}

void setup()
{
  pinMode(RELAY,OUTPUT);
  initWiFi();

  Serial.begin(115200);

  emon1.current(36, 7.1);             // Current: input pin, calibration.
  emon2.current(39, 7.1);             // Current: input pin, calibration.
  emon3.current(34, 7.1);             // Current: input pin, calibration.
  emon4.current(35, 7.1);             // Current: input pin, calibration.

  while(dac.begin()!=0){
  Serial.println("init error");
  delay(1000);

  pinMode(BUTTON_PIN15, INPUT_PULLUP);

  pinMode(BUTTON_PIN5, OUTPUT);
  pinMode(BUTTON_PIN14, INPUT_PULLUP);
  
   }

  Serial.println("init succeed");
  dac.setDACOutRange(dac.eOutputRange10V);//Set the output range as 0-10V
  dac.setDACOutVoltage(VOLTS,0);//The DAC value for 3.5V output in OUT0 channel
  delay(1000);
  dac.store(); //Save the set 3.5V voltage inside the chip

  timeClient.begin();
  https.setReuse(true);
}

void loop()
{
  // read the state of the switch/button:
  currentState15 = digitalRead(BUTTON_PIN15);
  currentState5 = digitalRead(BUTTON_PIN5);
  currentState14 = digitalRead(BUTTON_PIN14);

  Serial.print(" ");
  Serial.print("RRSI: ");
  Serial.println(WiFi.RSSI());
  Serial.println(WiFi.localIP());

  if(currentState5 == LOW) {
    //Serial.println("PRESSED 5");
    //VOLTS = 4000;
    //Serial.println(VOLTS);
    //dac.setDACOutVoltage(VOLTS,0);
    //digitalWrite(RELAY, LOW);
  }

  else if (currentState14 == LOW) {
    //Serial.println("PRESSED 14");
    //VOLTS = 8000;
    //Serial.println(VOLTS);
    //dac.setDACOutVoltage(VOLTS,0);
  }
  
  else {
    // Serial.println("NOT PRESSED");
    //VOLTS = 2000;
    // Serial.println(VOLTS);
    //dac.setDACOutVoltage(VOLTS,0);
    //digitalWrite(RELAY, HIGH);
  }

float Irms1 = emon1.calcIrms(268);  // Calculate Irms only
float Irms2 = emon2.calcIrms(268);  // Calculate Irms only
float Irms3 = emon3.calcIrms(268);  // Calculate Irms only
float Irms4 = emon4.calcIrms(268);  // Calculate Irms only
//  Serial.print("WATTS1: ");
//  Serial.print(Irms1*230.0);         // Apparent power

//  Serial.print(" AMPS1: ");
//  Serial.println(Irms1);          // Irms
  
//  Serial.print("WATTS2: ");
//  Serial.print(Irms2*230.0);         // Apparent power
//  Serial.print(" AMPS2: ");
//  Serial.println(Irms2);          // Irms

//  Serial.print(" AMPS3: ");
//  Serial.println(Irms3);          // Irms

//  Serial.print(" AMPS4: ");
//  Serial.println(Irms4);          // Irms



  // save the last state
//  lastState = currentState;

    // read the state of the switch/button:
//  int buttonState = digitalRead(BUTTON_PIN);

  // print out the button's state
//  Serial.println(buttonState);


// timeClient.update(); 
// Serial.print(timeClient.getFormattedTime());
// Serial.print('\n'); 

//delay(10);

currentMillis = millis();
  if(currentMillis > 5000) {
    total1 = total1 + Irms1;
    total2 = total2 + Irms2;
    total3 = total3 + Irms3;
    total4 = total4 + Irms4;
counter++;
if (counter==20) {


  adjusted_sample1 = (total1/20);
  adjusted_sample2 = (total2/20);
  adjusted_sample3 = (total3/20);
  adjusted_sample4 = (total4/20);



  Serial.print("AVERAGE 1: ");
  Serial.println (adjusted_sample1);
  Serial.print("AVERAGE 2: ");
  Serial.println (adjusted_sample2);
  Serial.print("AVERAGE 3: ");
  Serial.println (adjusted_sample3);
  Serial.print("AVERAGE 4: ");
  Serial.println (adjusted_sample4);
  
   total1=0;
   total2=0;
   total3=0;
   total4=0;
  counter=0;
  
  
  if(currentState5 == LOW && adjusted_sample1 < 30 && adjusted_sample2 < 30 && adjusted_sample3 < 30 && adjusted_sample4 < 30) {
    Serial.println("PRESSED 5");
    VOLTS = 1000;
    TRYB = "Automatyczny";
    Serial.println(VOLTS);
    dac.setDACOutVoltage(VOLTS,0);
    digitalWrite(RELAY, LOW);
  } 
    else if (currentState15 == HIGH) {
    Serial.println("NOT PRESSED");
    VOLTS = 9000;
    TRYB = "Manualny";
    Serial.println(VOLTS);
    dac.setDACOutVoltage(VOLTS,0);
    digitalWrite(RELAY, HIGH);
    }
  else {
    Serial.println("NOT PRESSED");
    VOLTS = 9000;
    TRYB = "Manualny";
    Serial.println(VOLTS);
    dac.setDACOutVoltage(VOLTS,0);
    digitalWrite(RELAY, HIGH);
  }
  
    String urlFinal = "https://script.google.com/macros/s/"+GOOGLE_SCRIPT_ID+"/exec?"+"Status=" + String(TRYB) + "&" + "Faza1=" + int(adjusted_sample1) + "&" + "Faza2=" + int(adjusted_sample2) + "&" + "Faza3=" + int(adjusted_sample3) + "&" + "Faza4=" + int(adjusted_sample4) + "&" + "Posuw=" + String(VOLTS/1000);
    Serial.print("POST data to spreadsheet:");
    Serial.println(urlFinal);
//    HTTPClient http;
    https.begin(urlFinal.c_str());
    https.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    int httpCode = https.GET(); 
//    Serial.print("HTTP Status Code: ");
//    Serial.println(httpCode);
    //---------------------------------------------------------------------
    //getting response from google sheet
//    String payload;
//    if (httpCode > 0) {
//        payload = https.getString();
 //       Serial.println("Payload: "+payload);    
//    }
    //---------------------------------------------------------------------
    https.end();

  
  }  
       previousMillis = currentMillis;
Serial.print("COUNTER: ");
 Serial.print(counter);
// Serial.print ('\n');
// Serial.print("AMPS 1: ");
// Serial.print(Irms1);
// Serial.print ('\n');
// Serial.print("AMPS 2: ");
// Serial.print(Irms2);
// Serial.print ('\n');
// Serial.print("AMPS 3: ");
// Serial.print(Irms3);
// Serial.print ('\n');
// Serial.print("AMPS 4: ");
// Serial.print(Irms4);
 Serial.print ('\n');

// Serial.print("SUM OF PROBES 1: ");
// Serial.print (total1);
// Serial.print ('\n');
// Serial.print("SUM OF PROBES 2: ");
// Serial.print (total2);
// Serial.print ('\n');
// Serial.print("SUM OF PROBES 3: ");
// Serial.print (total3); 
// Serial.print ('\n');
// Serial.print("SUM OF PROBES 4: ");
// Serial.print (total4);
 Serial.print ('\n');
 
// Serial.print("AVERAGE 1: ");
// Serial.println (adjusted_sample1);
// Serial.print("AVERAGE 2: ");
// Serial.println (adjusted_sample2);
// Serial.print("AVERAGE 3: ");
// Serial.println (adjusted_sample3);
// Serial.print("AVERAGE 4: ");
// Serial.println (adjusted_sample4);

  }

}
