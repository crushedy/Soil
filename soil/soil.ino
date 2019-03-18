#include <M41T62.h>
#include <Wire.h>
#include <LowPower.h>

#define RELAY D3 //the relay is on D3
#define SOIL_HUMIDITY A2 //soil humidity sensor is on A2
#define TEMPERATURE A1 // temperature sensor is on A1
#define ILLUMINANCE A0 // illuzminance sensor is on A0
#define FLOW_PIN D2 //water flow for sensor

RTC_M41T62 rtc;
int counter = 0;
int interrupted = 0;
DateTime now;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(RELAY, OUTPUT); 
  digitalWrite(RELAY, LOW);
  counter = 0;
  pinMode(FLOW_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), flow_event, RISING);
  Wire.begin();
  //rtc section
  rtc.begin();
  rtc.checkFlags(); // clear any rtc interrupt flags
  pinMode(TUINO_RTC_INTERRUPT , INPUT);
  attachInterrupt(digitalPinToInterrupt(TUINO_RTC_INTERRUPT ), RTC_event, FALLING);
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  DateTime alarmTime (rtc.now()+5);
  Serial.print("Time Now:");
  printTime(rtc.now());
  rtc.alarmSet(alarmTime);
  Serial.print("Alarm Set to: ");
  printTime(alarmTime);

  delay(200);
  
  rtc.alarmEnable(1); // enable the alarm
  
  Serial.println("Starting:");
}


void loop() {

 float temperature = 0;
 int illuminance = 0;
 int humidity    = 0;
 
 
 

 
 if(interrupted)
 {
  
 temperature = TemperatureCalculation();
 illuminance = IlluminanceCalculation();
 humidity    = SoilHumidityCalculation();
 
 Serial.print("Temperature = ");
 Serial.print(temperature);
 Serial.println(" deg C");

 Serial.print("Illuminance = ");
 Serial.print(illuminance);
 Serial.println("%");

 Serial.print("Soil Humidity = ");
 Serial.print(humidity);
  Serial.println("%");

 Serial.print("Flow Counter = ");
 Serial.print(counter);
 Serial.println(" pulses counted");

 Serial.print("Debit = ");
 Serial.println(debit());

 counter=0;
 
 if(rtc.checkFlags())
 {
    DateTime alarmTime (rtc.now()+5);
    rtc.alarmSet(alarmTime);
    Serial.print("Next Alarm Set to: ");
    printTime(alarmTime);
    rtc.alarmEnable(1);
 }
 
 interrupted=0;
 Serial.flush();
 attachInterrupt(digitalPinToInterrupt(TUINO_RTC_INTERRUPT ), RTC_event, FALLING);
 LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
 detachInterrupt(digitalPinToInterrupt(TUINO_RTC_INTERRUPT));
}
}

//gets the temperature
float TemperatureCalculation()
{
  int TemperatureValue = analogRead(TEMPERATURE);
  float R = 1023.0/((float)TemperatureValue)-1.0;
  R = 100000.0*R;
  //convert to temperature via datasheet
  float temperature=1.0/(log(R/100000.0)/4275+1/298.15)-273.15;
  return temperature;  
}


// gets the value given by the sensor and maps it from 0 to 800 to another range 0 - 100
int IlluminanceCalculation()
{
  int IlluminanceValue = analogRead(ILLUMINANCE);
  return map(IlluminanceValue, 0, 800, 0, 100);
  
}

// gets the value given by the sensor and maps it from 10 to 550 to another range 0 - 100
int SoilHumidityCalculation()
{
  int SoilHumidityValue = analogRead(SOIL_HUMIDITY);
  return map(SoilHumidityValue, 1024, 0,0,100);
  //return SoilHumidityValue;
}

void flow_event()
{
  counter++;
}

float debit()
{
  return (float)counter/24;
}

void RTC_event()
{
    interrupted = 1;
}

void printTime(DateTime myTime){
  Serial.print(myTime.year(), DEC);
  Serial.print('/');
  Serial.print(myTime.month(), DEC);
  Serial.print('/');
  Serial.print(myTime.day(), DEC);
  Serial.print(' ');
  Serial.print(myTime.hour(), DEC);
  Serial.print(':');
  Serial.print(myTime.minute(), DEC);
  Serial.print(':');
  Serial.print(myTime.second(), DEC);
  Serial.println();
}


