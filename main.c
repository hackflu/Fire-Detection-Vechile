#include <ESP32Servo.h>
#include <NewPing.h>
#include <TinyGPS++.h>
#include <WiFi.h>
#include <HTTPClient.h>

// Pin definitions
#define TRIG_PIN 12
#define ECHO_PIN 13
#define MAX_DISTANCE 200
#define MIN_DISTANCE 40
#define ENA 14
#define ENB 15
#define IN1 32
#define IN2 33
#define IN3 26
#define IN4 25
#define SERVO_PIN 21
#define FLAME_SENSOR_LEFT 36
#define FLAME_SENSOR_FORWARD 39
#define FLAME_SENSOR_RIGHT 34
#define RELAY_PIN 18
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define FLAME_THRESHOLD 700
#define SPRAY_POSITION 180
#define INITIAL_POSITION 90
#define STEP 45

Servo myServo;
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);
TinyGPSPlus gps;

HardwareSerial gpsSerial(1);
int distance = 0;
int distances[181]; // Array to store distances at every angle

const char *ssid = "hello";
const char *password = "hisatyamhere";
const char *twilioAccountSID = "";
const char *twilioAuthToken = "";
const char *twilioNumber = "+";
const char *yourNumber = "+91";
bool flameTaskRunning = false;

// Task Handles
TaskHandle_t gpsTaskHandle = NULL;

void setup()
{
    Serial.begin(115200);
    gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    myServo.attach(SERVO_PIN);
    myServo.write(90);

    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(ENA, HIGH);
    digitalWrite(ENB, HIGH);
    digitalWrite(RELAY_PIN, LOW);

    connectWiFi();
    Serial.println("Setup complete.");
}

void loop()
{
    while (gpsSerial.available() > 0)
    {
        gps.encode(gpsSerial.read());
    }

    static unsigned long lastWiFiCheck = 0;
    unsigned long currentMillis = millis();

    if (currentMillis - lastWiFiCheck >= 5000)
    {
        lastWiFiCheck = currentMillis;
        if (WiFi.status() != WL_CONNECTED)
        {
            Serial.println("WiFi not connected. Attempting to reconnect...");
            connectWiFi();
        }
    }

    static unsigned long lastFlameCheck = 0;

    if (currentMillis - lastFlameCheck >= 1000)
    {
        lastFlameCheck = currentMillis;
        int flameDirection = checkFlameSensors();

        if (flameDirection >= 0)
        {
            handleFlame(flameDirection);
        }
        else
        {
            scanUltrasonic();
        }
    }
}

void connectWiFi()
{
    Serial.println("Connecting to WiFi...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("WiFi connected.");
}

int checkFlameSensors()
{
    int flameReadings[3] = {
        analogRead(FLAME_SENSOR_LEFT),
        analogRead(FLAME_SENSOR_FORWARD),
        analogRead(FLAME_SENSOR_RIGHT)};
    Serial.printf("Flame sensor readings - Left: %d, Forward: %d, Right: %d\n",
                  flameReadings[0], flameReadings[1], flameReadings[2]);
    for (int i = 0; i < 3; i++)
    {
        if (flameReadings[i] < FLAME_THRESHOLD)
        {
            return i;
        }
    }
    return -1;
}

int getDistance()
{
    int dist = sonar.ping_cm();
    if (dist < 3 || dist > MAX_DISTANCE)
    {
        return -1; // No valid reading
    }
    return dist;
}

void handleFlame(int direction)
{
    if (!flameTaskRunning)
    {
        flameTaskRunning = true;

        moveInDirectionTask((void *)(uintptr_t)direction);
        xTaskCreatePinnedToCore(fetchLocationAndSendMessageTask, "Fetch Location and Send Message", 8192, NULL, 1, &gpsTaskHandle, 0);

        flameTaskRunning = false; // Reset the flag after completion
    }
}

void moveInDirectionTask(void *parameter)
{
    int direction = (int)(uintptr_t)parameter;

    // Stop the motors immediately
    stopMotors();

    // Handle the flame detection based on direction
    switch (direction)
    {
    case 0:
        Serial.println("Flame detected on the left! Turning left...");
        turnLeft();
        break;
    case 1:
        Serial.println("Flame detected forward! Moving forward...");
        moveForward();
        break;
    case 2:
        Serial.println("Flame detected on the right! Turning right...");
        turnRight();
        break;
    }

    // Wait for a short duration to allow the movement to take effect
    delay(2000);
    stopMotors(); // Stop the motors again after the movement

    // Activate the relay to extinguish the flame
    digitalWrite(RELAY_PIN, HIGH);
    Serial.println("Relay activated to extinguish flame!");

    // Wait for the specified duration to spray water
    sprayWater(5000);

    // Deactivate the relay after spraying
    digitalWrite(RELAY_PIN, LOW);
}

void sprayWater(unsigned long duration)
{
    Serial.println("Spraying water...");
    delay(duration);
    Serial.println("Stopped spraying water.");
}

void fetchLocationAndSendMessageTask(void *parameter)
{
    unsigned long startTime = millis();
    const unsigned long timeout = 20000;

    while (millis() - startTime < timeout)
    {
        if (gps.location.isValid())
        {
            float latitude = gps.location.lat();
            float longitude = gps.location.lng();
            // Determine the direction for latitude
            String latDirection = (latitude >= 0) ? "N" : "S";
            // Determine the direction for longitude
            String longDirection = (longitude >= 0) ? "E" : "W";
            // Create the location message with direction
            String locationMessage = String("Flame detected! Location:\n") +
                                     "Latitude: " + String(abs(latitude), 6) + " " + latDirection + "\n" +
                                     "Longitude: " + String(abs(longitude), 6) + " " + longDirection;
            // Create a Google Maps link
            String mapLink = "https://www.google.com/maps?q=" + String(latitude, 6) + "," + String(longitude, 6);
            locationMessage += "\nOpen map: " + mapLink;
            sendMessage(locationMessage);
            break;
        }
        delay(500);
    }

    if (!gps.location.isValid())
    {
        Serial.println("Failed to fetch GPS location within timeout.");
    }
    vTaskDelete(NULL);
}

void sendMessage(String message)
{
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("WiFi not connected. Attempting to reconnect...");
        connectWiFi();
    }
    if (WiFi.status() == WL_CONNECTED)
    {
        HTTPClient http;
        String url = "https://api.twilio.com/2010-04-01/Accounts/" + String(twilioAccountSID) + "/Messages.json";
        String payload = "To=" + String(yourNumber) +
                         "&From=" + String(twilioNumber) +
                         "&Body=" + message;
        http.begin(url);
        http.setAuthorization(twilioAccountSID, twilioAuthToken);
        http.addHeader("Content-Type", "application/x-www-form-urlencoded");

        int httpResponseCode = http.POST(payload);
        if (httpResponseCode > 0)
        {
            Serial.println("Message sent successfully!");
        }
        else
        {
            Serial.printf("Error sending message. Code: %d\n", httpResponseCode);
        }
        http.end();
    }
    else
    {
        Serial.println("Unable to send message. WiFi not connected.");
    }
}

void scanUltrasonic()
{
    distance = getDistance();
    if (distance <= 0 || distance > MIN_DISTANCE)
    {
        Serial.println("No obstacle detected. Moving forward...");
        moveForward();
    }
    else
    {
        Serial.println("Obstacle detected! Scanning...");
        stopMotors();
        for (int angle = 0; angle <= 180; angle += 10)
        { // Scanning with 10-degree increments
            myServo.write(angle);
            delay(300);
            distances[angle] = getDistance();
            Serial.printf("Distance at angle %d: %d cm\n", angle, distances[angle]);
        }

        // Find the clearest path
        int maxDistance = -1;
        int bestAngle = 90;
        for (int angle = 0; angle <= 180; angle++)
        {
            if (distances[angle] > MIN_DISTANCE && distances[angle] > maxDistance)
            {
                maxDistance = distances[angle];
                bestAngle = angle;
            }
        }

        Serial.printf("Best angle: %d with distance: %d cm\n", bestAngle, maxDistance);

        if (maxDistance > MIN_DISTANCE)
        {
            if (bestAngle < 90)
            {
                Serial.println("Turning left toward clear path...");
                turnLeft();
                delay((90 - bestAngle) * 8);
            }
            else if (bestAngle > 90)
            {
                Serial.println("Turning right toward clear path...");
                turnRight();
                delay((bestAngle - 90) * 8);
            }
            else
            {
                Serial.println("Path ahead is clear. Moving forward...");
            }
            moveForward();
        }
        else
        {
            Serial.println("No clear path found. Avoiding obstacle...");
            avoidObstacle();
        }

        myServo.write(90); // Reset servo position
        delay(300);
    }
    delay(100);
}

void setMotorSpeed(int speed)
{
    // speed should be between 0 and 255
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
}

void stopMotors()
{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

void moveForward()
{
    setMotorSpeed(120); // Adjust speed here
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void turnLeft()
{
    setMotorSpeed(120); // Adjust speed here
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void turnRight()
{
    setMotorSpeed(120); // Adjust speed here
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}
void avoidObstacle()
{
    Serial.println("Obstacle detected! Avoiding...");
    stopMotors();
    delay(500);
    turnRight();
    delay(2000);
    stopMotors();
}
