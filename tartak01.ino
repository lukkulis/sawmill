#include <LiquidCrystal_I2C.h>
#include <Wire.h>
const int ACPin = 36;         //set arduino signal read pin
#define ACTectionRange 20;    //set Non-invasive AC Current Sensor tection range (5A,10A,20A)
LiquidCrystal_I2C lcd (0x27, 16,2);

// VREF: Analog reference
// For Arduino UNO, Leonardo and mega2560, etc. change VREF to 5
// For Arduino Zero, Due, MKR Family, ESP32, etc. 3V3 controllers, change VREF to 3.3
#define VREF 1.0

float readACCurrentValue()
{
  float ACCurrtntValue = 0;
  float peakVoltage = 0;  
  float voltageVirtualValue = 0;  //Vrms
  for (int i = 0; i < 5; i++)
  {
    peakVoltage += analogRead(ACPin);   //read peak voltage
    delay(1);
  }
  peakVoltage = peakVoltage / 5;   
  voltageVirtualValue = peakVoltage * 0.737;    //change the peak voltage to the Virtual Value of voltage

  /*The circuit is amplified by 2 times, so it is divided by 2.*/
  voltageVirtualValue = (voltageVirtualValue / 1024 * VREF ) / 2;  

  ACCurrtntValue = voltageVirtualValue * ACTectionRange;

  return ACCurrtntValue;
}

void setup() 
{
  Wire.begin();
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  while (!Serial);
  Serial.println("\nI2C address Scanner CircuitSchools.com");

  lcd. begin ();
  
  // Turn on the backlight on LCD. 
  lcd. backlight ();
  
  // print the Message on the LCD. 
  //  lcd. print ( "I1:" );

}

void loop() 
{
  float ACCurrentValue = readACCurrentValue(); //read AC Current Value
 // Serial.print(ACCurrentValue);
 // Serial.println(" A");
 // digitalWrite(13, HIGH);
 // delay(500);
 // digitalWrite(13, LOW);
 // delay(500);
  lcd.setCursor(0, 0); 
  lcd.print(String("I1 = ") + String(ACCurrentValue));
 // lcd. print (ACCurrentValue);
  lcd. setCursor (0, 1);
   // We write the number of seconds elapsed 
  lcd. print ( millis () / 1000);
  lcd. print ( "SECONDS" );
  delay (100);


{
  byte error, address; //variable for error and I2C address
  int devicecount;
 


  Serial.println("Scanning...");
 
  devicecount = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      devicecount++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (devicecount == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
 
  delay(5000); // wait 5 seconds for the next I2C scan
}

}
//dupa123