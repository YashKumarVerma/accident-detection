// Yash Kumar Verma

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Firebase_ESP_Client.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Adafruit_ADXL345_U.h>
#include <SimpleDHT.h>

// Needed for token generation info
#include "addons/TokenHelper.h"
// Needed for firebase realtime database operations
#include "addons/RTDBHelper.h"

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// Ultra Sonic SONAR
const int trigPin = 12;
const int echoPin = 14;

// initialize Temperature and Humidity Sensor
SimpleDHT11 dht11(13);

// define sound velocity in cm/uS
#define SOUND_VELOCITY 0.034
#define CM_TO_INCH 0.393701

long duration;
float distanceCm;

// Insert your network credentials
#define WIFI_SSID "yash_verma"
#define WIFI_PASSWORD "sudo1sudo5"

// Insert Firebase project API Key
#define API_KEY "redacted"

// Insert Authorized Email and Corresponding Password
#define USER_EMAIL "redacted"
#define USER_PASSWORD "redacted"

// Insert RTDB URLefine the RTDB URL
#define DATABASE_URL "https://redacted.firebaseio.com/"

// Define Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Variable to save USER UID
String uid;

// Database main path (to be updated in setup with the user UID)
String databasePath;

// Structure of data to be stored in the database
// geolocation latitude
String lati = "/lat";
// geolocation longitude
String lon = "/lon";
// type of event
String type = "/type";
// temperature reading
String temp = "/temp";
// humidity reading
String hum = "/hum";
// timestamp when data was collected;
String timePath = "/timestamp";
// accelerometer x-axis reading
String xAcc = "/xAcc";
// accelerometer y-axis reading
String yAcc = "/yAcc";
// ultra sonic distance in cm
String dist = "/dist";
int x, y, a, b;

String parentPath;
FirebaseJson json;

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

// Variable to save current epoch time
int timestamp;

// BME280 sensor
// Adafruit_BME280 bme; // I2C
float temperature;
float humidity;
float pressure;

// Timer variables (send new readings every three minutes)
unsigned long sendDataPrevMillis = 0;
unsigned long timerDelay = 180000;

// Initialize WiFi
void initializeWifi()
{
    Serial.print("Attempting to connect to WiFi: ");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print('.');
        delay(1000);
    }
    Serial.println(WiFi.localIP());
    Serial.print("Wifi Connected.");
}

// Function to fetch current unix timestamp
unsigned long getTime()
{
    timeClient.update();
    unsigned long now = timeClient.getEpochTime();
    return now;
}

void setup()
{
    pinMode(0, OUTPUT);
    Serial.begin(9600);

    // Initialize Sonic Sensor pins
    pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
    pinMode(echoPin, INPUT);  // Sets the echoPin as an Input

    // Initialize WiFi
    initializeWifi();

    // Initialize Gyroscope
    Serial.println("[gyroscope] testing connection:");
    if (!accel.begin())
    {
        Serial.println("[gyroscope] error getting reading");
    }

    // read gyroscope data
    sensors_event_t event;
    accel.getEvent(&event);
    a = event.acceleration.x;
    b = event.acceleration.y;

    timeClient.begin();

    // Firebase Operations -- to send data to realtime cloud
    // Assign the api key (required)
    config.api_key = API_KEY;

    // Assign the user sign in credentials
    auth.user.email = USER_EMAIL;
    auth.user.password = USER_PASSWORD;

    // Assign the RTDB URL (required)
    config.database_url = DATABASE_URL;

    Firebase.reconnectWiFi(true);
    fbdo.setResponseSize(4096);

    // Assign the callback function for the long running token generation task */
    config.token_status_callback = tokenStatusCallback; // see addons/TokenHelper.h

    // Assign the maximum retry of token generation
    config.max_token_generation_retry = 5;

    // Initialize the library with the Firebase authen and config
    Firebase.begin(&config, &auth);

    // Getting the user UID might take a few seconds
    Serial.println("[firebase] getting user uid: ");
    while ((auth.token.uid) == "")
    {
        Serial.print('.');
        delay(1000);
    }
    // Print user UID
    uid = auth.token.uid.c_str();
    Serial.print("[firebase] user uid: ");
    Serial.println(uid);

    // Update database path
    databasePath = "/UsersData/" + uid + "/readings";
}

// get distance from ultrasonic sensor
float getDistanceInCMFromUltraSonic()
{
    // set pin to low for 2 microseconds
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    // Emit a 10 microsecond pulse
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);

    // calculate the distance using speed = distance/time formula
    return duration * SOUND_VELOCITY / 2;
}

void loop()
{
    byte temperature = 0;
    byte humidity = 0;
    float latitude = 12.9701;
    float longitude = 79.164;

    // Read temperature and humidity
    int err = SimpleDHTErrSuccess;
    if ((err = dht11.read(&temperature, &humidity, NULL)) != SimpleDHTErrSuccess)
    {
        Serial.println("[dht11] error reading : ");
        Serial.print("[dht11] :");
        Serial.println(SimpleDHTErrCode(err));
        Serial.print("[dht11] :");
        Serial.println(SimpleDHTErrDuration(err));
    }
    else
    {
        Serial.println("[dht11] reading successful : ");
        Serial.print("[dht11] temperature in *C : ");
        Serial.println((int)temperature);

        Serial.print("[dht11] humidity : ");
        Serial.print((int)humidity);
        Serial.println(" H");
    }

    // Read accelerometer data
    sensors_event_t event;
    accel.getEvent(&event);
    x = event.acceleration.x;
    y = event.acceleration.y;

    Serial.print("[gyro] x : ");
    Serial.println(x);
    Serial.print("[gyro] y : ");
    Serial.println(y);

    // figure out if the accelerometer is moving
    if (x - a > 2)
    {
        type = "left";
        Serial.println("left");
    }
    if (a - x > 2)
    {
        type = "right";
        Serial.println("right");
    }
    if (b - y > 2)
    {
        type = "rear";
        Serial.println("rear");
    }
    if (y - b > 2)
    {
        type = "front";
        Serial.println("front");
    }

    // Reading Sensor Data -- UltraSonic
    int cm = getDistanceInCMFromUltraSonic();
    Serial.print("[ultrasonic] distance in cm : ");
    Serial.println(cm);

    // set threshold for distance
    if (cm <= 15)
    {
        // turn on the buzzer
        digitalWrite(0, HIGH);

        Serial.println("[ultrasonic] danger");
    }
    else
    {
        // turn off the buzzer
        digitalWrite(0, LOW);
        Serial.println("[ultrasonic] safe");
    }

    // Send new readings to database
    if (Firebase.ready() && (millis() - sendDataPrevMillis > timerDelay || sendDataPrevMillis == 0))
    {
        sendDataPrevMillis = millis();

        // Get current timestamp
        timestamp = getTime();
        Serial.print("time: ");
        Serial.println(timestamp);

        parentPath = databasePath + "/" + String(timestamp);
        json.set(lati.c_str(), String(latitude));
        json.set(lon.c_str(), String(longitude));
        json.set(type.c_str(), type);
        json.set(timePath.c_str(), String(timestamp));
        json.set(dist.c_str(), String(cm));
        if ((err = dht11.read(&temperature, &humidity, NULL)) == SimpleDHTErrSuccess)
        {
            json.set(temp.c_str(), String(temperature) + " *c");
            json.set(hum.c_str(), String(humidity) + " H");
        }
        json.set(xAcc.c_str(), String(x));
        json.set(yAcc.c_str(), String(y));
        Serial.printf("Set json... %s\n", Firebase.RTDB.setJSON(&fbdo, parentPath.c_str(), &json) ? "ok" : fbdo.errorReason().c_str());
    }
    delay(1500);
}
