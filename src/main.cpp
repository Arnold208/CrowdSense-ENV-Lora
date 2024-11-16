#include <Arduino.h>

#include <ArduinoJson.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_INA219.h>
#include <Adafruit_MPU6050.h>
#include "Adafruit_LTR390.h"
#include "Adafruit_VEML7700.h"


#define SLEEP_PIN 13
#define INA219_I2C_ADDRESS 0x41
#define but 25
#define led 32


bool bmeState = false;
bool ina219State = false;
bool mpuState = false;
bool uvState = false;
bool vemlState = false;


SoftwareSerial SerialS(16, 17);
SoftwareSerial pmsSerial(35, 36);
Adafruit_BME280 bme;
Adafruit_INA219 ina219(INA219_I2C_ADDRESS);
Adafruit_MPU6050 mpu;
Adafruit_LTR390 ltr = Adafruit_LTR390();
Adafruit_VEML7700 veml = Adafruit_VEML7700();


// Anti-theft Detection
const int ledPin = 2;
const float movementThreshold = 0.5;
float prevX = 0, prevY = 0, prevZ = 0;
int count1;


String err = "";
static char recv_buf[512];
static bool is_exist = false;
static bool is_join = false;


void butt_check() {
  if (digitalRead(but) == HIGH) {
    digitalWrite(led, HIGH);
    delay(250);
    digitalWrite(led, LOW);
    delay(250);
    digitalWrite(led, HIGH);
    delay(250);
    digitalWrite(led, LOW);
    delay(250);
  }
}

static int at_send_check_response(char *p_ack, int timeout_ms, char *p_cmd, ...)
{
  int ch;
  int num = 0;
  int index = 0;
  int startMillis = 0;
  va_list args;
  memset(recv_buf, 0, sizeof(recv_buf));
  va_start(args, p_cmd);
  SerialS.printf(p_cmd, args);
  Serial.printf(p_cmd, args);
  va_end(args);
  delay(200);
  startMillis = millis();

  if (p_ack == NULL)
    return 0;

  do
  {
    while (SerialS.available() > 0)
    {
      ch = SerialS.read();
      recv_buf[index++] = ch;
      Serial.print((char)ch);
      delay(2);
    }

    if (strstr(recv_buf, p_ack) != NULL)
      return 1;

  } while (millis() - startMillis < timeout_ms);
  return 0;
}

static void recv_prase(char *p_msg)
{
  if (p_msg == NULL)
  {
    Serial.println("Received null");
    return;
  }
  char *p_start = NULL;
  char data[128];       // To hold the received bytes as characters

  int bytes_len = 0;
  p_start = strstr(p_msg, "RX");
  if (p_start && (1 == sscanf(p_start, "RX: \"%s", &data)))
  {
    for (int i = 0; i < sizeof(data); i++) {
      if (int(data[i + 1]) == 0) {
        bytes_len = i;
        break;
      }
    }

    // Convert the characters to a byteArray
    int message_len = bytes_len / 2 + 1;
    byte out[message_len];
    auto getNum = [](char c) {
      return c > '9' ? c - 'A' + 10 : c - '0';
    };
    for (int x = 0, y = 0; x < bytes_len; ++x, ++y)
      out[y] = (getNum(data[x++]) << 4) + getNum(data[x]);
    out[message_len] = '\0';

    // Create a JSON document of specified capacity <100> and load the byteArray to it
    JsonDocument doc;
    deserializeJson(doc, out);

    // Print the received JSON message by serializing it
    serializeJson(doc, Serial);
    Serial.println();

    // Access a specific key value from the JSON formatted message
    Serial.println((const char *)doc["name"]);
  }
}

double round2(double value) {
  return (int)(value * 100 + 0.5) / 100.0;
}

void blinkLed()
{
  for (int i = 0; i < 5; i++)
  {
    digitalWrite(led, HIGH);
    delay(200);
    digitalWrite(led, LOW);
    delay(200);
  }
}

struct pms5003data
{
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};

struct pms5003data data;

boolean readPMSdata(Stream *s)
{
  if (!s->available())
  {
    return false;
  }

  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42)
  {
    s->read();
    return false;
  }

  // Now read all 32 bytes
  if (s->available() < 32)
  {
    return false;
  }

  uint8_t buffer[32];
  uint16_t sum = 0;
  s->readBytes(buffer, 32);

  // get checksum ready
  for (uint8_t i = 0; i < 30; i++)
  {
    sum += buffer[i];
  }

  /* debugging
    for (uint8_t i=2; i<32; i++) {
    Serial.print("0x"); Serial.print(buffer[i], HEX); Serial.print(", ");
    }
    Serial.println();
  */

  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i = 0; i < 15; i++)
  {
    buffer_u16[i] = buffer[2 + i * 2 + 1];
    buffer_u16[i] += (buffer[2 + i * 2] << 8);
  }

  // put it into a nice struct :)
  memcpy((void *)&data, (void *)buffer_u16, 30);

  if (sum != data.checksum)
  {
    Serial.println("Checksum failure");
    return false;
  }
  // success!
  return true;
}

// Task to detect theft by monitoring the MPU6050 sensor
void theftDetectionTask(void *parameters)
{
  while (1)
  {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Calculate the difference between current and previous acceleration values
    float diffX = abs(a.acceleration.x - prevX);
    float diffY = abs(a.acceleration.y - prevY);
    float diffZ = abs(a.acceleration.z - prevZ);

    // If any difference exceeds the threshold, consider it as movement
    if (diffX > movementThreshold || diffY > movementThreshold || diffZ > movementThreshold)
    {
      Serial.println("Movement detected! Triggering anti-theft alert...");
      blinkLed(); // Blink LED as an alert
    }

    // Update the previous values
    prevX = a.acceleration.x;
    prevY = a.acceleration.y;
    prevZ = a.acceleration.z;

    vTaskDelay(500 / portTICK_PERIOD_MS); // Run the task every 500ms
  }
}

void timeoutChecker(void *parameters)
{
  for (;;)
  {
    Serial.print("Timeout Checker:");
    Serial.println(count1++);
    if (count1 == 240)
    {
      Serial.println("Shutting Down Now!!");
      digitalWrite(SLEEP_PIN, HIGH);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setup(void)
{
  delay(2000);
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  pinMode(led, OUTPUT);
  pinMode(but, INPUT);
  delay(500);
  butt_check();
  Serial.begin(9600);
  SerialS.begin(9600);
  pmsSerial.begin(9600);


  pinMode(SLEEP_PIN, OUTPUT);
  //  digitalWrite(PWR, LOW);

  bmeState = bme.begin(0x76);
  if (!bmeState) {
    Serial.println("BME280 sensor not working");
  }

  ina219State = ina219.begin();
  if (!ina219State) {
    Serial.println("INA219 sensor not working");
  }
  else {
    ina219.setCalibration_16V_400mA();
  }

  mpuState = mpu.begin(0x69);
  if (!mpuState) {
    Serial.println("Failed to find MPU6050 chip");
  }
  else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  // Create task for anti-theft detection
  xTaskCreate(
    theftDetectionTask, // Task function
    "Theft Detection",  // Task name
    2000,               // Stack size (in words)
    NULL,               // Task parameters
    1,                  // Task priority
    NULL                // Task handle
  );


  uvState = ltr.begin();
  if (!uvState) {
    Serial.println("UV sensor not working");
  }
  else {
    ltr.setMode(LTR390_MODE_UVS);
    ltr.setGain(LTR390_GAIN_3);
    ltr.setResolution(LTR390_RESOLUTION_16BIT);
  }

  vemlState = veml.begin();
  if (!vemlState) {
    Serial.println("Sensor not found!");
  }
  else {
    veml.setGain(VEML7700_GAIN_1_8);
    veml.setIntegrationTime(VEML7700_IT_100MS);
  }

  xTaskCreate(
    timeoutChecker,    // Task function
    "TimeOut Checker", // Task name
    2000,              // Stack size (in words)
    NULL,              // Task parameters
    1,                 // Task priority
    NULL               // Task handle
  );
  Serial.print("E5 LORAWAN TEST\r\n");
  if (at_send_check_response("+AT: OK", 100, "AT\r\n"))
  {
    is_exist = true;
    at_send_check_response("+ID: AppEui", 1000, "AT+ID\r\n");
    at_send_check_response("+MODE: LWOTAA", 1000, "AT+MODE=LWOTAA\r\n");
    at_send_check_response("+DR: US915", 1000, "AT+DR=US915\r\n");
    at_send_check_response("+DR: DR2", 1000, "AT+DR=DR2\r\n"); // Set data rate to DR1
    at_send_check_response("+CH: NUM", 1000, "AT+CH=NUM,8-15\r\n");
    at_send_check_response("+KEY: APPKEY", 1000, "AT+KEY=APPKEY,\"AA16EE08BEC2BE9885EC404C977A14A9\"\r\n");
    at_send_check_response("+CLASS: C", 1000, "AT+CLASS=A\r\n");
    at_send_check_response("+PORT: 8", 1000, "AT+PORT=8\r\n");
    delay(1000);
    is_join = true;
  }
  else
  {
    is_exist = false;
    Serial.print("No E5 module found.\r\n");
  }

  delay(30000);

}

void loop(void)
{
  if (is_exist)
  {
    int ret = 0;
    if (is_join)
    {
      ret = at_send_check_response("+JOIN: Network joined", 12000, "AT+JOIN\r\n");
      if (ret)
        is_join = false;
      else
      {
        at_send_check_response("+ID: AppEui", 1000, "AT+ID\r\n");
        Serial.print("JOIN failed!\r\n\r\n");
        delay(5000);
      }
    }
    else
    {
      int sendCount = 0;  // Initialize counter for transmissions
      while (sendCount < 2)  // Loop to send data twice
      {
        JsonDocument json;
        delay(1000);
        if (bmeState) {
          json["state"]["t"] = round2(bme.readTemperature());
          json["state"]["p"] = round2(bme.readPressure() / 100.0F);
          json["state"]["h"] = round2(bme.readHumidity());
          err += "0";
        }
        else {
          json["state"]["t"] = 0;
          json["state"]["h"] = 0;
          json["state"]["p"] = 0;
          err += "1";
        }
        if (ina219State) {
          delay(1000);
          json["state"]["b"] = round2(ina219.getBusVoltage_V());
          json["state"]["i"] = round2(ina219.getCurrent_mA());
          err += "0";

        }
        else {
          json["state"]["b"] = 0;
          json["state"]["i"] = 0;
          err += "1";

        }

        if (uvState) {
          delay(1000);

          if (ltr.newDataAvailable())
          {
            json["state"]["u"] = round2(ltr.readUVS());
            err += "0";

          }
          else {
            json["state"]["u"] = 0;
          }
        }
        else {
          json["state"]["u"] = 0;
          err += "1";

        }

        if (vemlState) {
          delay(1000);

          json["state"]["l"] = round2(veml.readLux());
          err += "0";

        }
        else {
          json["state"]["l"] = 0;
          err += "1";

        }

        if (readPMSdata(&pmsSerial))
        {
          // reading data was successful!
          Serial.println();
          Serial.println("---------------------------------------");
          Serial.println("Concentration Units (standard)");
          Serial.print("PM 1.0: ");
          Serial.print(data.pm10_standard);
          Serial.print("\t\tPM 2.5: ");
          Serial.print(data.pm25_standard);
          Serial.print("\t\tPM 10: ");
          Serial.println(data.pm100_standard);
          Serial.println("---------------------------------------");
          Serial.println("Concentration Units (environmental)");
          Serial.print("PM 1.0: ");
          Serial.print(data.pm10_env);
          Serial.print("\t\tPM 2.5: ");
          Serial.print(data.pm25_env);
          Serial.print("\t\tPM 10: ");
          Serial.println(data.pm100_env);
          Serial.println("---------------------------------------");
          Serial.print("Particles > 0.3um / 0.1L air:");
          Serial.println(data.particles_03um);
          Serial.print("Particles > 0.5um / 0.1L air:");
          Serial.println(data.particles_05um);
          Serial.print("Particles > 1.0um / 0.1L air:");
          Serial.println(data.particles_10um);
          Serial.print("Particles > 2.5um / 0.1L air:");
          Serial.println(data.particles_25um);
          Serial.print("Particles > 5.0um / 0.1L air:");
          Serial.println(data.particles_50um);
          Serial.print("Particles > 10.0 um / 0.1L air:");
          Serial.println(data.particles_100um);
          Serial.println("---------------------------------------");

          json["state"]["p0"] = round2(data.pm10_env);
          json["state"]["p1"] = round2(data.pm25_env);
          json["state"]["p2"] = round2(data.pm100_env);
          err += "0";

          pmsSerial.flush();
        }
        else {
          json["state"]["p0"] = 0;
          json["state"]["p1"] = 0;
          json["state"]["p2"] = 0;
          err += "1";

        }
        json["state"]["s"] = 0;

        json["state"]["e"] = err;

        char charArray[400];
        int len = serializeJson(json, charArray);
        serializeJson(json, Serial);
        Serial.println("");
        char buildBuffer[2] = {0};
        char compositionBuffer[len * 3 + 1] = {0}; // this will hold a string we build

        for (int i = 0; i < len; i++) {
          sprintf( buildBuffer, "%02X ", (uint8_t)charArray[i]);
          strcat( compositionBuffer, buildBuffer);
        }

        char cmd[712];
        sprintf(cmd, "AT+CMSGHEX=\"");
        strcat(cmd, compositionBuffer);
        strcat(cmd, "\"\r\n");
        ret = at_send_check_response("Done", 5000, cmd);
        err = "";

        if (ret) {
          recv_prase(recv_buf);
          sendCount++;  // Increment transmission counter
          Serial.printf("Transmission #%d successful.\n", sendCount);
          err = "";

        }
        else {
          Serial.print("Send failed!\r\n\r\n");
        }

        // Delay between transmissions if necessary
        delay(1500);
      }

      // Power down after sending data twice
      Serial.print("Powering Down!!");
      err = "";
      delay(500);
      digitalWrite(SLEEP_PIN, HIGH);
    }
  }
  else {
    delay(1000);
  }
}