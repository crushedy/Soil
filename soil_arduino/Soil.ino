#include <M41T62.h>
#include <Wire.h>
#include <LowPower.h>
#include "gmx_lr.h"
#include "Regexp.h"
#include "SeeedOLED.h"
#include "display_utils.h"
#include "MAX17048.h"


#define RELAY D7 //the relay is on D7
#define SOIL_HUMIDITY A2 //soil humidity sensor is on A2
#define TEMPERATURE A1 // temperature sensor is on A1
#define ILLUMINANCE A0 // illuzminance sensor is on A0
#define FLOW_PIN D4 //water flow for sensor

//State Machine Definitions
#define POWER_ON_STATE 0
#define RTC_WAKED_UP 1
#define GET_SENSOR_DATA 2
#define TRANSMIT_SENSOR_DATA 3
#define RECEIVE_INSTRUCTIONS 4
#define GET_TIME 5
#define SEND_TIME_COMMAND 6
#define UNEXPECTED_FLOW 7
#define LOW_BATTERY 8
#define SET_ALARM_AND_SLEEP 9
#define WATERING 10



String rx_data;
int port;
int watering_duration = 5;
bool action = false; //false for waking up without watering, true for watering


int state = POWER_ON_STATE;

RTC_M41T62 rtc;
MAX17048 batteryMonitor;

int counter = 0;
int interrupted = 0;
DateTime time;
DateTime next_alarm;
int battery_voltage = 0;
bool data_received = false;

void loraRx() {
	data_received = true;
}

void setup() {
	char string[64];
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
	pinMode(TUINO_RTC_INTERRUPT, INPUT);
	attachInterrupt(digitalPinToInterrupt(TUINO_RTC_INTERRUPT), RTC_event, FALLING);
	rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
	next_alarm = rtc.now() + 5;
	Serial.print("Time Now:");
	printTime(rtc.now());
	rtc.alarmSet(next_alarm);
	Serial.print("Alarm Set to: ");
	printTime(next_alarm);
	delay(200);
	rtc.alarmEnable(1); // enable the alarm
	Serial.println("Connecting to Lora:");
	String DevEui, NewDevEui;
	String AppEui, NewAppEui;
	String AppKey, NewAppKey;
	String loraClass, LoRaWANClass;
	String adr, dcs, dxrate;
	byte join_status;
	int join_wait;
	Wire.begin();
	Serial.begin(9600);
	Serial.println("Starting");
	batteryMonitor.attatch(Wire);
	delay(1000);

	// Init Oled
	SeeedOled.init();  //initialze SEEED OLED display
	SeeedOled.clearDisplay();          //clear the screen and set start position to top left corner
	SeeedOled.setNormalDisplay();      //Set display to normal mode (i.e non-inverse mode)
	SeeedOled.setHorizontalMode();           //Set addressing mode to Page Mode

#pragma region LoraInit
							 // GMX-LR init pass callback function
	gmxLR_init(&loraRx);
	LoRaWANClass = "A";
	gmxLR_getAppEui(AppEui);
	if (NewAppEui.length() > 0)
	{
		AppEui.trim();

		Serial.println("**** UPDATING AppEUI ****");
		if (!AppEui.equals(NewAppEui))
		{
			Serial.println("Setting AppEui:" + NewAppEui);
			gmxLR_setAppEui(NewAppEui);
		}
		else
		{
			Serial.println("AppEui is already:" + AppEui);
		}
	}

	gmxLR_getAppKey(AppKey);
	if (NewAppKey.length() > 0)
	{
		AppKey.trim();

		Serial.println("**** UPDATING AppKey ****");
		if (!AppKey.equals(NewAppKey))
		{
			Serial.println("Setting AppKey:" + NewAppKey);
			gmxLR_setAppKey(NewAppKey);
		}
		else
		{
			Serial.println("AppKey is already:" + AppKey);
		}
	}

	// Disable Duty Cycle  ONLY FOR DEBUG!
	gmxLR_setDutyCycle("0");

	// Set LoRaWAN Class
	gmxLR_setClass(LoRaWANClass);


	// Show Splash Screen on OLED
	splashScreen();
	delay(2000);

	// Show LoRaWAN Params on OLED
	gmxLR_getDevEui(DevEui);
	gmxLR_getAppKey(AppKey);
	gmxLR_getAppEui(AppEui);
	displayLoraWanParams(DevEui, AppEui, AppKey);

	delay(2000);

	SeeedOled.clearDisplay();
	SeeedOled.setTextXY(0, 0);
	SeeedOled.putString("Joining...");

	Serial.println("Joining...");
	join_wait = 0;
	while ((join_status = gmxLR_isNetworkJoined()) != LORA_NETWORK_JOINED) {


		if (join_wait == 0)
		{
			Serial.println("LoRaWAN Params:");
			gmxLR_getDevEui(DevEui);
			Serial.println("DevEui:" + DevEui);
			gmxLR_getAppEui(AppEui);
			Serial.println("AppEui:" + AppEui);
			gmxLR_getAppKey(AppKey);
			Serial.println("AppKey:" + AppKey);
			gmxLR_getClass(loraClass);
			Serial.println("Class:" + loraClass);
			adr = String(gmxLR_getADR());
			Serial.println("ADR:" + adr);
			dcs = String(gmxLR_getDutyCycle());
			Serial.println("DCS:" + dcs);
			gmxLR_getRX2DataRate(dxrate);
			Serial.println("RX2 DataRate:" + dxrate);

			gmxLR_Join();
		}

		SeeedOled.setTextXY(1, 0);
		sprintf(string, "Attempt: %d", join_wait);
		SeeedOled.putString(string);
		Serial.println("Joined...");
		join_wait++;

		if (!(join_wait % 100)) {
			gmxLR_Reset();
			join_wait = 0;
		}

		delay(5000);

	};
#pragma endregion

	SeeedOled.setTextXY(2, 0);
	SeeedOled.putString("Joined!");

	delay(2000);
	SeeedOled.clearDisplay();
}


void loop() {

	char lora_payload[51];
	String tx_data;

	char lora_data[128];
	char lora_rx[51];
	char lora_payload_length;
	char lora_rx_length;
	byte rx_buf[128];
	int buf_len;

	float temperature = 0;
	int illuminance = 0;
	int humidity = 0;


	switch (state)
	{

	case POWER_ON_STATE:
		state = SEND_TIME_COMMAND;
		break;


	case GET_SENSOR_DATA:
		temperature = TemperatureCalculation();
		illuminance = IlluminanceCalculation();
		humidity = SoilHumidityCalculation();
		battery_voltage = Battery_Voltage();

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

		Serial.print("Battery Voltage = ");
		Serial.println(battery_voltage);

		state = TRANSMIT_SENSOR_DATA;
		break;

	case TRANSMIT_SENSOR_DATA:
		lora_payload[0] = ((int)(temperature * 100)>>8)&(0xFF);
		lora_payload[1] = (int)(temperature * 100)&(0xFF);
		lora_payload[2] = (illuminance)&(0xFF);
		lora_payload[3] = (humidity * 100)&(0xFF);
		lora_payload[4] = ((counter) >> 8)&(0xFF);
		lora_payload[5] = (counter)&(0xFF);
		lora_payload[6] = ((int)(debit() * 100) >> 8)&(0xFF);
		lora_payload[7] = (int)(debit() * 100)&(0xFF);
		lora_payload[8] = ((int)(battery_voltage >> 8))&(0xFF);
		lora_payload[9] = (int)(battery_voltage)&(0xFF);

		sprintf(lora_data, "%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X", lora_payload[0], lora_payload[1], lora_payload[2], lora_payload[3], lora_payload[4], lora_payload[5], lora_payload[6], lora_payload[7], lora_payload[8], lora_payload[9]);
		tx_data = String(lora_data);
		gmxLR_TXData(tx_data);
		counter = 0;
		time = rtc.now();
		state = RECEIVE_INSTRUCTIONS;
		break;

	
	case RECEIVE_INSTRUCTIONS:
		if ((rtc.now().secondstime() - time.secondstime()) > 10)
		{
			Serial.println("Timeout Fetching Instructions");
			state = TRANSMIT_SENSOR_DATA;
		}

		if (data_received)
		{
				gmxLR_RXData(rx_data, &port);
				gmxLR_StringToHex(rx_data, (char*)rx_buf, &buf_len);
				Serial.println("LORA RX DATA:" + rx_data);
				Serial.print("LORA RX LEN:");
				Serial.println(buf_len);
				Serial.print("LORA RX PORT:");
				Serial.println(port);
				int32_t next_event = (rx_data[0] << 24) | (rx_data[1] << 16) | (rx_data[2] << 8) | rx_data[3];
				watering_duration = rx_data[4] + rx_data[5] * 8;
				if(rx_data[6] == 1) action = true; else action = false;
				next_alarm = DateTime(next_event);	//server is sending time in unix format
				data_received = false;
				state = SET_ALARM_AND_SLEEP;
		}
		break;

	case WATERING:
		digitalWrite(RELAY, HIGH);	//relay_on
		DateTime watering_time = rtc.now();
		while ((rtc.now().unixtime() - watering_time.unixtime()) < watering_duration)
		{

		}
		digitalWrite(RELAY, LOW); //relay_off
		state = GET_SENSOR_DATA;
		break;

	case SET_ALARM_AND_SLEEP:
		rtc.alarmSet(next_alarm);
		rtc.alarmEnable(1);
		Serial.flush();
		attachInterrupt(digitalPinToInterrupt(TUINO_RTC_INTERRUPT), RTC_event, FALLING);
		LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
		detachInterrupt(digitalPinToInterrupt(TUINO_RTC_INTERRUPT));
		if (action == true) 
			state = WATERING; 
		else 
			state = GET_SENSOR_DATA;
		break;


	case SEND_TIME_COMMAND:
		lora_payload[0] = 't';
		lora_payload_length = 1;
		tx_data = "t";
		Serial.println("sending time request command");
		time = rtc.now();
		gmxLR_TXData(tx_data);
		state = GET_TIME;
		break;

	case GET_TIME:
		//set an alarm to 10s and wait for something to happen, such as lora to receive something.
		//wait for LoRa to receive unix time from server
		// if timer < 10s, send again the command

		if ((rtc.now().secondstime() - time.secondstime()) > 10)
		{
			Serial.println("timeout fetching time");
			state = SEND_TIME_COMMAND;
		}
		if (data_received)
		{
			gmxLR_RXData(rx_data, &port);
			gmxLR_StringToHex(rx_data, (char*)rx_buf, &buf_len);

			Serial.println("LORA RX DATA:" + rx_data);
			Serial.print("LORA RX LEN:");
			Serial.println(buf_len);
			Serial.print("LORA RX PORT:");
			Serial.println(port);
			
			int32_t set_time = (rx_data[0] << 24) | (rx_data[1] << 16) | (rx_data[2] << 8) | rx_data[3];	//server is sending time in unix format

			//write the RTC with the actual time
			rtc.adjust(set_time);

			state = GET_SENSOR_DATA;
			data_received = false;
		}
		break;

	case UNEXPECTED_FLOW:
		Serial.println("Unexpected Flow");
		tx_data = "U";
		gmxLR_TXData(tx_data);
		break;

	case LOW_BATTERY:
		Serial.println("Low Battery");
		tx_data = "B";
		gmxLR_TXData(tx_data);
		break;


	default:
		state = POWER_ON_STATE;
		break;
	}

}

//gets the temperature
float TemperatureCalculation()
{
	int TemperatureValue = analogRead(TEMPERATURE);
	float R = 1023.0 / ((float)TemperatureValue) - 1.0;
	R = 100000.0*R;
	//convert to temperature via datasheet
	float temperature = 1.0 / (log(R / 100000.0) / 4275 + 1 / 298.15) - 273.15;
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
	return map(SoilHumidityValue, 1024, 0, 0, 100);
	//return SoilHumidityValue;
}

int Battery_Voltage()
{
	int bat_vol;
	float cellVoltage = batteryMonitor.voltage();
	Serial.print("Voltage:\t\t");
	Serial.print(cellVoltage, 4);
	Serial.println("V");
	bat_vol = (int)(cellVoltage * 1000);
	return bat_vol;
}

void flow_event()
{
	counter++;
	if (state == SET_ALARM_AND_SLEEP)
	{
		state = UNEXPECTED_FLOW;
	}
}

float debit()
{
	return (float)counter / 24;
}

void RTC_event()
{
	if (state == SET_ALARM_AND_SLEEP)
	{
		state = GET_SENSOR_DATA;
	}

	
	interrupted = 1;
}

void printTime(DateTime myTime) {
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


