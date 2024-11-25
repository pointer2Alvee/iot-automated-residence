//FINAL - - - - - 
//**************************************DEFINITIONS-&-MACROS***************************************/
//BLYNK CRED
#define BLYNK_TEMPLATE_ID "TMPL6ci9W6Gxz"
#define BLYNK_TEMPLATE_NAME "IoT Based Automated Smart Home"
#define BLYNK_AUTH_TOKEN "2Nw3bxRNb1M40bNR6ll6EUW-DoRkzfHy"
#define BLYNK_PRINT Serial


////////////////////ROOM LIGHTS////////////////////////
//LIGHT-01
#define Light01_virtualPin  V0
#define Light01_pin  32 //pwm pin

//LIGHT-02
#define Light02_virtualPin  V1
#define Light02_pin  33 //pwm pin

////////////////////ROOM FANS////////////////////////
//FAN-01
#define Fan01_virtualPin  V2
//L298N MOTOR DRIVER VARIABBLES
#define ENA 13
#define IN1 2
#define IN2 15

//FAN-02
#define Fan02_virtualPin  V3
//L298N MOTOR DRIVER VARIABBLES
#define ENB 12
#define IN3 5
#define IN4 18
////////////////////GAS SENSORS////////////////////////
#define MQ2Sensor_virtualPin  V4
#define MQ2SensorLED_virtualPin  V8
#define MQ2SensorPin 35

////////////////////DHT11//////////////////////////////
#define DHTPIN 14
#define DHTTYPE DHT11   // DHT 11
#define DHT11Temp_virtualPin  V5
#define DHT11Humid_virtualPin  V6

////////////////////FLAME SENSOR//////////////////////////////
#define FlameSensor_virtualPin  V7
//**************************************LIBRARIES**************************************************/
//WIFI
#include <WiFi.h>
#include <WiFiClient.h>
//BLYNK
#include <BlynkSimpleEsp32.h>

//DHT11
#include "DHT.h"



//**************************************VARIABlES**************************************************/


//WIFI ANDD BLYNK
//BLYNK
char auth[] = BLYNK_AUTH_TOKEN;

//HOTSPOT
char ssid[] = "Hotspot40";
char pass[] = "abcd_123";


//APP BUTTON PRESSE OR NOT STATUS
uint8_t fromAppButtonstate = NULL;

//VALUE CONTROL PWM ANALOG PROPERTIES
const uint16_t frequency = 5000;

//ROOM LIGHTS
const int led01_PwmChannel = 0;
const int led02_PwmChannel = 1;
const int ledResolution = 12; //12-bit

uint8_t light01_Status = NULL;
uint8_t light02_Status = NULL;


//ROOM FAN SPEED CONTROL PWM ANALOG PROPERTIES
const uint8_t fanResolution = 8; //8-bit
const uint8_t fan01_PwmChannel = 2;
const uint8_t fan02_PwmChannel = 3;


uint8_t fan01_Status = NULL;
uint8_t fan02_Status = NULL;



//LDR VARIABLES
uint16_t ldrVal = 0;     /*Variable to store photoresistor value*/
uint8_t ldrSensorPin = 34;    /*Analogue Input for photoresistor pwm*/
uint8_t ldrPwmChannel = 4;

//MQ2 SENSOR VARIABLES
uint8_t MQ2SensorValue = NULL;

//FLAME-SENSOR VARIABLES
uint8_t pin_FlameSensor = 22;
uint8_t flameSensorData = 0;

//IR SENSOR
uint8_t irSensorPin = 21;


//ALARM LED_BUZZER PIN
uint8_t ledBuzzerPin = 23;
//**************************************OBJECTS****************************************************/
//BLYNK
BlynkTimer timer;
//DHT11
DHT dht(DHTPIN, DHTTYPE);
//**************************************FUNCTIONS DEFINITIONS**************************************/

//**************************************FUNCTIONS**************************************************/

//ALARM SYSTEM FUCNTION

void alarmSystem(boolean state) {

  if (state == true) {
    for (int i = 0; i < 10; i++) {
      digitalWrite(ledBuzzerPin, HIGH);
      delay(40);
      digitalWrite(ledBuzzerPin, LOW);
      delay(40);
    }
  } else if (state == false) {
    digitalWrite(ledBuzzerPin, LOW);

  }
  /*
    while (state) {
    digitalWrite(ledBuzzerPin, HIGH);
    delay(80);
    digitalWrite(ledBuzzerPin, LOW);
    delay(80);
    break;
    } */
}
//DHT 11

float tempHumidRead() {
  delay(5);

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return 0.0;
  }

  Blynk.virtualWrite(DHT11Temp_virtualPin, t);
  Blynk.virtualWrite(DHT11Humid_virtualPin, h);

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.print(F("°C "));
  Serial.print(f);
  Serial.print(F("°F  Heat index: "));
  Serial.print(hic);
  Serial.print(F("°C "));
  Serial.print(hif);
  Serial.println(F("°F"));

  return t;
}

//IR sensor Function
void movementDetection() {

  if (digitalRead(irSensorPin) == 0) {
    alarmSystem(true);
  } else if (digitalRead(irSensorPin) == 1) {
    alarmSystem(false);
  }
}

//Flame Detection Function
void FlameDetection() {
  flameSensorData = digitalRead(pin_FlameSensor);
  if (flameSensorData == 0) {

    Blynk.virtualWrite(FlameSensor_virtualPin, 1);
    alarmSystem(true);
  } else if (flameSensorData == 1) {
    Blynk.virtualWrite(FlameSensor_virtualPin, 0);
    alarmSystem(false);
  }
}

//MQ2 SENSOR FUNCTIONS
void mq2SensorData() {

  MQ2SensorValue = analogRead(MQ2SensorPin);
  Blynk.virtualWrite(MQ2Sensor_virtualPin, MQ2SensorValue);
  Serial.println(MQ2SensorValue);
  Serial.print("digital val ");
  Serial.println(digitalRead(MQ2SensorPin));
  if (MQ2SensorValue > 240)
  {
    //    Blynk.notify("Gas Detected!");

Blynk.virtualWrite(MQ2SensorLED_virtualPin, 1);
    alarmSystem(true);

  }

  else if (MQ2SensorValue < 240)
  {
    Blynk.virtualWrite(MQ2SensorLED_virtualPin, 0);
    alarmSystem(false);
  }
}
//LDR FUNCTIONS
uint16_t ldrValue() {
  ldrVal = analogRead(ldrSensorPin);   /*Analog read LDR value*/
  // ldrVal = ledcRead(ldrPwmChannel);   /*Analog read LDR value*/
  Serial.print("LDR Output Value: ");
  Serial.println(ldrVal);   /*Display LDR Output Val on serial monitor*/
  if (ldrVal > 2000) {      /*If light intensity is HIGH*/
    return ldrVal;
    Serial.println(" High intensity ");
  }
  else {
    /*Else if Light intensity is LOW LED will Remain ON*/
    return ldrVal;
    Serial.println("LOW Intensity ");
  }
  delay(2);     /*Reads value after every 1 sec*/
}

void lightIntensityControl() {
  if (light01_Status == 1) {
    uint16_t currLdrValue = ldrValue();
    ledcWrite(led01_PwmChannel, 4095 - currLdrValue);
    delay(2);
  } else if (light01_Status == 0) {
    ledcWrite(led01_PwmChannel, 0);
  }
  if (light02_Status == 1) {
    uint16_t currLdrValue = ldrValue();
    ledcWrite(led02_PwmChannel, 4095 - currLdrValue);
    delay(2);
  } else if (light02_Status == 0) {
    ledcWrite(led02_PwmChannel, 0);
  }

}
//MOTOR-FAN FUCNTIONS
void fanSpeedControl() {

  if (fan01_Status == 1) {
    uint8_t currTemp = tempHumidRead();
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    delay(5);

    if (currTemp >= 18 && currTemp <= 25) {
      ledcWrite(fan01_PwmChannel, 140);
    } else if (currTemp >= 26 && currTemp <= 30) {
      ledcWrite(fan01_PwmChannel, 190);
    } else if (currTemp >= 31 && currTemp <= 40) {
      ledcWrite(fan01_PwmChannel, 235);
    }
  } else if (fan01_Status == 0) {
    ledcWrite(fan01_PwmChannel, 50);
  }

  if (fan02_Status == 1) {
    uint8_t currTemp = tempHumidRead();
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    delay(5);

    if (currTemp >= 18 && currTemp <= 25) {
      ledcWrite(fan02_PwmChannel, 140);
    } else if (currTemp >= 26 && currTemp <= 30) {
      ledcWrite(fan02_PwmChannel, 190);
    } else if (currTemp >= 31 && currTemp <= 40) {
      ledcWrite(fan02_PwmChannel, 235);
    }
  } else if (fan02_Status == 0) {
    ledcWrite(fan02_PwmChannel, 0);

  }
}




//BLYNK LIGHTS
/*This function is called every time the device is connected to the Blynk.Cloud
  Request the latest state from the server */
BLYNK_CONNECTED() {
  Blynk.syncVirtual(Light01_virtualPin);
  Blynk.syncVirtual(Light02_virtualPin);
  Blynk.syncVirtual(Fan01_virtualPin);
  Blynk.syncVirtual(Fan02_virtualPin);

}
/* This function is called every time the Virtual Pin state change
  i.e when web push switch from Blynk App or Web Dashboard */

BLYNK_WRITE(Light01_virtualPin) {
  fromAppButtonstate = param.asInt(); //Reads app button state 0/1, pressed or not
  if (fromAppButtonstate == 1) {

    light01_Status = 1;



  }
  else if (fromAppButtonstate == 0) {
    light01_Status = 0;
    //ledcWrite(led01_PwmChannel, 0);

  }

}


BLYNK_WRITE(Light02_virtualPin) {
  fromAppButtonstate = param.asInt();
  if (fromAppButtonstate == 1) {
    light02_Status = 1;
    //ldrValue();
    /*
      for (int i = 0; i < 4096; i += 204) {
      ledcWrite(led02_PwmChannel, 4095);
      delay(150);
      if (param.asInt() != 1) {
        ledcWrite(led02_PwmChannel, 0);
        break;
      }

      }*/
  }
  else if (fromAppButtonstate == 0) {
    light02_Status = 0;
    // ledcWrite(led02_PwmChannel, 0);
  }
}


BLYNK_WRITE(Fan01_virtualPin) {
  fromAppButtonstate = param.asInt();
  if (fromAppButtonstate == 1) {
    // setDirection();
    //delay(1000);
    // changeSpeed();
    // delay(1000);
    fan01_Status = 1;

    /*
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        delay(20);
        ledcWrite(fan01_PwmChannel, 200);
    */

  }
  else if (fromAppButtonstate == 0) {
    fan01_Status = 0;
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    ledcWrite(fan01_PwmChannel, 0);

  }

}

BLYNK_WRITE(Fan02_virtualPin) {
  fromAppButtonstate = param.asInt();
  if (fromAppButtonstate == 1) {
    // setDirection();
    //delay(1000);
    // changeSpeed();
    // delay(1000);
    fan02_Status = 1;

    /*
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      delay(20);
      ledcWrite(fan02_PwmChannel, 200); */

  }
  else if (fromAppButtonstate == 0) {
    fan02_Status = 0;
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    ledcWrite(fan02_PwmChannel, 0);

  }

}

///////////////////////////////SETUP-LOOP DRIVER CODES//////////////////////////////////////////////

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  //DHT
  dht.begin();

  //LDR
  // pinMode(ldrSensorPin, INPUT);
  // pinMode(ldrSensorPin, INPUT_PULLUP);
  // ledcSetup(ldrPwmChannel, frequency, ledResolution);
  //ledcAttachPin(ldrSensorPin, ldrPwmChannel);
  //ldrValue();
  //L298N MOTOR DRIVER VARIABBLES
  //FAN-01
  pinMode(ENA, INPUT_PULLUP);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  ledcSetup(fan01_PwmChannel, frequency, fanResolution);
  ledcAttachPin(ENA, fan01_PwmChannel);

  //FAN-02
  pinMode(ENB, INPUT_PULLUP);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcSetup(fan02_PwmChannel, frequency, fanResolution);
  ledcAttachPin(ENB, fan02_PwmChannel);

  //ROOM LIGHTS
  //LIGHT-01
  pinMode(Light01_pin, INPUT_PULLUP);
  ledcSetup(led01_PwmChannel, frequency, ledResolution);
  ledcAttachPin(Light01_pin, led01_PwmChannel);  // attach the channel to the GPIO to be controlled
  //LIGHT-02
  pinMode(Light02_pin, INPUT_PULLUP);
  ledcSetup(led02_PwmChannel, frequency, ledResolution);
  ledcAttachPin(Light02_pin, led02_PwmChannel);// attach the channel to the GPIO to be controlled

  //MQ2
  pinMode(MQ2SensorPin, INPUT);
  //ALARM SYSTEM LED BUZZER PIN
  pinMode(ledBuzzerPin, OUTPUT);

  //FIRE SENSOR
  pinMode(pin_FlameSensor, INPUT);


  //IR SENSOR
  pinMode(irSensorPin, INPUT);

  //WIFI & BLYNK
  Blynk.begin(auth, ssid, pass);
  delay(2000);
  //pinMode(2,OUTPUT);
  // STUDDY THIS
  //  timer.setInterval(1000L, sendUptime);
}

void loop() {
  // put your main code here, to run repeatedly:
  tempHumidRead();
  movementDetection();
  fanSpeedControl();
  FlameDetection();
  lightIntensityControl() ;
  mq2SensorData();
  Blynk.run();
  timer.run();
  //digitalWrite(2,HIGH);
  //delay(1000);
  //digitalWrite(2,LOW);
  //delay(1000);

}
