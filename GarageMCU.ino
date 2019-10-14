// **********************************************************************************************************
// GarageMCU garage door controller sketch that works with NodeMCU
// Inspired by and some code copied from:
// http://www.LowPowerLab.com/GarageMote
// **********************************************************************************
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHTesp.h>

// DEFINE DHT TEMPERATURE SENSOR
#define WEATHERSENDDELAY  3000    // send WeatherShield data every so often (ms)
#define DHTPIN D2
DHTesp dht;

// DEFINE GPIO PINS
#define RELAYPIN D1
#define REEDSENSOR1 D6
#define REEDSENSOR2 D7

#define RELAY_PULSE_MS      250  //just enough that the opener will pick it up

#define DOOR_MOVEMENT_TIME 14000 // this has to be at least as long as the max between [door opening time, door closing time]
                                 // my door opens and closes in about 12s
#define STATUS_CHANGE_MIN  1500  // this has to be at least as long as the delay 
                                 // between a opener button press and door movement start
                                 // most garage doors will start moving immediately (within half a second)
//*****************************************************************************************************************************
#define HALLSENSOR_OPENSIDE   0
#define HALLSENSOR_CLOSEDSIDE 1

#define STATUS_CLOSED        0
#define STATUS_CLOSING       1
#define STATUS_OPENING       2
#define STATUS_OPEN          3
#define STATUS_UNKNOWN       4

// #define LED                  9   //pin connected to onboard LED
#define LED_PULSE_PERIOD  5000   //5s seems good value for pulsing/blinking (not too fast/slow)
#define SERIAL_BAUD     115200
#define SERIAL_EN                //comment out if you don't want any serial output

#ifdef SERIAL_EN
  #define DEBUG(input)   {Serial.print(input); delay(1);}
  #define DEBUGln(input) {Serial.println(input); delay(1);}
#else
  #define DEBUG(input);
  #define DEBUGln(input);
#endif

//function prototypes
void setStatus(byte newSTATUS, boolean reportStatus=true);
void reportStatus();
boolean hallSensorRead(byte which);
void pulseRelay();

//global program variables
byte GARAGE_STATUS;
unsigned long lastStatusTimestamp=0;
unsigned long ledPulseTimestamp=0;
unsigned long lastWeatherSent=0;
int ledPulseValue=0;
boolean ledPulseDirection=false; //false=down, true=up
char Pstr[10];
char Fstr[10];
char Hstr[10];
double F,P,H;
char sendBuf[30];

const char* ssid = "NETWORKNAME";
const char* password = "PASSWORD";
const char* mqtt_server = "192.168.1.20";
const char* mqtt_user = "mqttuser";
const char* mqtt_password = "mqttpassword";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  byte newStatus=GARAGE_STATUS;
  boolean reportStatusRequest=false;
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if (strcmp(topic, "switch/garage/set") == 0) {
    Serial.println("Got something...");
    payload[length] = '\0';
    String s = String((char*)payload);
    if (strcmp(s.c_str(), "status") == 0) {
        Serial.println("Got status request, reporting status now...");
//        reportStatus();
        reportStatusRequest = true;
    } else if (strcmp(s.c_str(), "open") == 0) {
        Serial.println("Opening garage door.");
        if (millis()-(lastStatusTimestamp) > STATUS_CHANGE_MIN && (GARAGE_STATUS == STATUS_CLOSED || GARAGE_STATUS == STATUS_CLOSING || GARAGE_STATUS == STATUS_UNKNOWN))
          newStatus = STATUS_OPENING;
        // TODO: relay
    } else if (strcmp(s.c_str(), "close") == 0) {
        Serial.println("Closing garage door.");
        if (millis()-(lastStatusTimestamp) > STATUS_CHANGE_MIN && (GARAGE_STATUS == STATUS_OPEN || GARAGE_STATUS == STATUS_OPENING || GARAGE_STATUS == STATUS_UNKNOWN))
          newStatus = STATUS_CLOSING;
    }

    if (GARAGE_STATUS != newStatus)
    {
      pulseRelay();
      setStatus(newStatus);
    }
    if (reportStatusRequest)
    {
      reportStatus();
    }
    
  }

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "GarageMCU-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
//      client.publish("switch/garage", GARAGE_STATUS);
      // ... and resubscribe
      client.subscribe("switch/garage/set");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


void setup(void)
{
#ifdef SERIAL_EN
  Serial.begin(SERIAL_BAUD);
#endif
  dht.setup(D2, DHTesp::DHT22);
  pinMode(REEDSENSOR1, INPUT_PULLUP);
  pinMode(REEDSENSOR2, INPUT_PULLUP);
  pinMode(RELAYPIN, OUTPUT);
  pinMode(BUILTIN_LED, OUTPUT);
  
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  char buff[50];
  sprintf(buff, "GarageMCU v0.1");
  DEBUGln(buff);

  if (hallSensorRead(HALLSENSOR_OPENSIDE)==true)
    setStatus(STATUS_OPEN);
  if (hallSensorRead(HALLSENSOR_CLOSEDSIDE)==true)
    setStatus(STATUS_CLOSED);
  else setStatus(STATUS_UNKNOWN);

}

unsigned long doorPulseCount = 0;
char input=0;

void loop()
{
#ifdef SERIAL_EN
  if (Serial.available())
    input = Serial.read();
#endif

  if (input=='r')
  {
    DEBUGln("Relay test...");
    pulseRelay();
    input = 0;
  }

  if (!client.connected()) {
    reconnect();
  }
  client.loop();
    
  // UNKNOWN => OPEN/CLOSED
  if (GARAGE_STATUS == STATUS_UNKNOWN && millis()-(lastStatusTimestamp)>STATUS_CHANGE_MIN)
  {
    if (hallSensorRead(HALLSENSOR_OPENSIDE)==true)
      setStatus(STATUS_OPEN);
    if (hallSensorRead(HALLSENSOR_CLOSEDSIDE)==true)
      setStatus(STATUS_CLOSED);
  }

  // OPEN => CLOSING
  if (GARAGE_STATUS == STATUS_OPEN && millis()-(lastStatusTimestamp)>STATUS_CHANGE_MIN)
  {
    if (hallSensorRead(HALLSENSOR_OPENSIDE)==false)
      setStatus(STATUS_CLOSING);
  }

  // CLOSED => OPENING  
  if (GARAGE_STATUS == STATUS_CLOSED && millis()-(lastStatusTimestamp)>STATUS_CHANGE_MIN)
  {
    if (hallSensorRead(HALLSENSOR_CLOSEDSIDE)==false)
      setStatus(STATUS_OPENING);
  }

  // OPENING/CLOSING => OPEN (when door returns to open due to obstacle or toggle action)
  //                 => CLOSED (when door closes normally from OPEN)
  //                 => UNKNOWN (when more time passes than normally would for a door up/down movement)
  if ((GARAGE_STATUS == STATUS_OPENING || GARAGE_STATUS == STATUS_CLOSING) && millis()-(lastStatusTimestamp)>STATUS_CHANGE_MIN)
  {
    if (hallSensorRead(HALLSENSOR_OPENSIDE)==true)
      setStatus(STATUS_OPEN);
    else if (hallSensorRead(HALLSENSOR_CLOSEDSIDE)==true)
      setStatus(STATUS_CLOSED);
    else if (millis()-(lastStatusTimestamp)>DOOR_MOVEMENT_TIME)
      setStatus(STATUS_UNKNOWN);
  }

  
  //use LED to visually indicate STATUS
  if (GARAGE_STATUS == STATUS_OPEN || GARAGE_STATUS == STATUS_CLOSED) //solid ON/OFF
  {
    digitalWrite(BUILTIN_LED, GARAGE_STATUS == STATUS_OPEN ? LOW : HIGH);
  }
  if (GARAGE_STATUS == STATUS_OPENING || GARAGE_STATUS == STATUS_CLOSING) //pulse
  {
    if (millis()-(ledPulseTimestamp) > LED_PULSE_PERIOD/256)
    {
      ledPulseValue = ledPulseDirection ? ledPulseValue + LED_PULSE_PERIOD/256 : ledPulseValue - LED_PULSE_PERIOD/256;

      if (ledPulseDirection && ledPulseValue > 255)
      {
        ledPulseDirection=false;
        ledPulseValue = 255;
      }
      else if (!ledPulseDirection && ledPulseValue < 0)
      {
        ledPulseDirection=true;
        ledPulseValue = 0;
      }
      
      analogWrite(BUILTIN_LED, ledPulseValue);
      ledPulseTimestamp = millis();
    }
  }
  if (GARAGE_STATUS == STATUS_UNKNOWN) //blink
  {
    if (millis()-(ledPulseTimestamp) > LED_PULSE_PERIOD/20)
    {
      ledPulseDirection = !ledPulseDirection;
      digitalWrite(BUILTIN_LED, ledPulseDirection ? HIGH : LOW);
      ledPulseTimestamp = millis();
    }
  }


  // Report temperature/humidity every 2 seconds.
  if (millis()-lastWeatherSent > WEATHERSENDDELAY) {
    lastWeatherSent = millis();

  float humidity = dht.getHumidity();
  float temperature = dht.getTemperature();
  char humidityStr[8];
  char temperatureStr[8];
  dtostrf(humidity, 6, 2, humidityStr);
  dtostrf(dht.toFahrenheit(temperature), 6, 2, temperatureStr);

//  Serial.print(dht.getStatusString());
  Serial.print("\tH: ");
  Serial.print(humidityStr);
  Serial.print(" %\t\tT: ");
//  Serial.print(temperature, 1);
//  Serial.print(" *C\t\t");
  Serial.print(temperatureStr);
  Serial.println(" *F\t\t");
//  Serial.print(dht.computeHeatIndex(temperature, humidity, false), 1);
//  Serial.print("\t\t");
//  Serial.println(dht.computeHeatIndex(dht.toFahrenheit(temperature), humidity, true), 1);
  client.publish("sensor/garage/temperature", temperatureStr);
  client.publish("sensor/garage/humidity", humidityStr);

//  byte read1 = digitalRead(REEDSENSOR1);
//  Serial.print("reed1 (open side)  : ");
//  Serial.println(read1 == HIGH ? "high" : "low");
//  byte read2 = digitalRead(REEDSENSOR2);
//  Serial.print("reed2 (closed side): ");
//  Serial.println(read2 == HIGH ? "high" : "low");
  }
}

//returns TRUE if magnet is next to sensor, FALSE if magnet is away
boolean hallSensorRead(byte which)
{
  //while(millis()-lastStatusTimestamp<STATUS_CHANGE_MIN);
//   digitalWrite(which ? HALLSENSOR2_EN : HALLSENSOR1_EN, HIGH); //turn sensor ON
//   delay(1); //wait a little
  byte reading = digitalRead(which ? REEDSENSOR2 : REEDSENSOR1);
//   digitalWrite(which ? HALLSENSOR2_EN : HALLSENSOR1_EN, LOW); //turn sensor OFF
  return reading==0;
}

void setStatus(byte newSTATUS, boolean reportIt)
{
  if (GARAGE_STATUS != newSTATUS) lastStatusTimestamp = millis();
  GARAGE_STATUS = newSTATUS;
  DEBUGln(GARAGE_STATUS==STATUS_CLOSED ? "closed" : GARAGE_STATUS==STATUS_CLOSING ? "closing" : GARAGE_STATUS==STATUS_OPENING ? "opening" : GARAGE_STATUS==STATUS_OPEN ? "open" : "unknown");
  if (reportIt)
    reportStatus();
}

void reportStatus(void)
{
  Serial.print("reporting status...");
  Serial.println(GARAGE_STATUS==STATUS_CLOSED ? "closed" : GARAGE_STATUS==STATUS_CLOSING ? "closing" : GARAGE_STATUS==STATUS_OPENING ? "opening" : GARAGE_STATUS==STATUS_OPEN ? "open" : "unknown");
  client.publish("switch/garage", GARAGE_STATUS==STATUS_CLOSED ? "closed" : GARAGE_STATUS==STATUS_CLOSING ? "closing" : GARAGE_STATUS==STATUS_OPENING ? "opening" : GARAGE_STATUS==STATUS_OPEN ? "open" : "unknown");
}

void pulseRelay()
{
  digitalWrite(RELAYPIN, HIGH);
  delay(RELAY_PULSE_MS);
  digitalWrite(RELAYPIN, LOW);
}

void Blink(byte PIN, byte DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}
