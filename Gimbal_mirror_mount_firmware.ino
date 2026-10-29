#include <WiFi.h>
#include <math.h>

const char* WIFI_SSID = "";
const char* WIFI_PASSWORD = "";

WiFiServer server(80);
HardwareSerial ServoSerial(2);


#define SERVO_RX_PIN
#define SERVO_TX_PIN

#define SERVO_ADDRESS 0xE0
#define SERVO_BAUD 38400



#define MOTOR_STEPS_PER_REV 200
#define MICROSTEPS 16

// 4:1 gear reduction
#define GEAR_RATIO 4.0


// PID

float Kp = 1.8;
float Ki = 3.6;
float Kd = 0.0;

float targetAngle = 0.0;
float actualAngle = 0.0;

float error = 0.0;
float previousError = 0.0;
float integral = 0.0;

float pidOutput = 0.0;

unsigned long previousTime = 0;


#define PID_MAX 12.0
#define PID_MIN -12.0

// CURRENT AXIS


// 0 = none
// 1 = horizontal
// 2 = vertical

int selectedAxis = 0;

// CHECKSUM

uint8_t checksum(uint8_t *data, uint8_t length)
{
    uint8_t sum = 0;

    for (uint8_t i = 0; i < length; i++)
    {
        sum += data[i];
    }

    return sum;
}

// MOVE SERVO42C

void moveMotor(float motorAngle)
{
    if (fabs(motorAngle) < 0.001)
        return;


    // Convert motor angle to pulses

    float pulsesFloat =
        (fabs(motorAngle) / 360.0) *
        MOTOR_STEPS_PER_REV *
        MICROSTEPS;


    uint32_t pulses =
        (uint32_t)(pulsesFloat + 0.5);


    if (pulses == 0)
        return;


    // Motor speed

    uint8_t speed = 10;


    // Direction

    if (motorAngle < 0)
    {
        speed |= 0x80;
    }


    // Create packet

    uint8_t packet[8];


    packet[0] = SERVO_ADDRESS;
    packet[1] = 0xFD;
    packet[2] = speed;


    packet[3] =
        (pulses >> 24) & 0xFF;

    packet[4] =
        (pulses >> 16) & 0xFF;

    packet[5] =
        (pulses >> 8) & 0xFF;

    packet[6] =
        pulses & 0xFF;


    packet[7] =
        checksum(packet, 7);


    // Send to SERVO42C

    ServoSerial.write(packet, 8);
}

// READ ENCODER

float readMotorAngle()
{
    /*
       SERVO42C encoder response goes here.

       The function will return the actual
       motor-shaft angle once the V1.1 response
       bytes are decoded.
    */

    return 0.0;
}

// PID FUNCTION

void runPID()
{
    unsigned long currentTime = millis();


    float dt =
        (currentTime - previousTime)
        / 1000.0;


    if (dt <= 0)
        return;


    previousTime = currentTime;

    // ERROR
   
    error =
        targetAngle -
        actualAngle;

    // INTEGRAL

    integral =
        integral +
        error * dt;

    // DERIVATIVE
    
    float derivative =
        (error - previousError)
        / dt;

    // PID CALCULATION
   
    pidOutput =
        (Kp * error) +
        (Ki * integral) +
        (Kd * derivative);

    if (pidOutput > PID_MAX)
        pidOutput = PID_MAX;


    if (pidOutput < PID_MIN)
        pidOutput = PID_MIN;


    previousError = error;


    float motorCorrection =
        pidOutput * GEAR_RATIO;


    moveMotor(motorCorrection);
}

// RESET PID

void resetPID()
{
    integral = 0.0;

    previousError = 0.0;

    pidOutput = 0.0;
}

// STOP MOTOR

void stopMotor()
{
    uint8_t packet[3];


    packet[0] =
        SERVO_ADDRESS;


    packet[1] =
        0xF7;


    packet[2] =
        checksum(packet, 2);


    ServoSerial.write(
        packet,
        3
    );


    Serial.println(
        "Motor stopped"
    );
}

// SETUP

void setup()
{

    Serial.begin(115200);

    Serial.println();

    Serial.println(
        "ESP32 Mirror Controller"
    );

    // SERVO42C UART
  

    ServoSerial.begin(
        SERVO_BAUD,
        SERIAL_8N1,
        SERVO_RX_PIN,
        SERVO_TX_PIN
    );

    WiFi.begin(
        WIFI_SSID,
        WIFI_PASSWORD
    );

    Serial.print(
        "Connecting to Wi-Fi"
    );

    while (
        WiFi.status() != WL_CONNECTED
    )
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println();

    Serial.println(
        "Wi-Fi connected"
    );

    Serial.print(
        "ESP32 IP address: "
    );

    Serial.println(
        WiFi.localIP()
    );

    // START SERVER

    server.begin();
    Serial.println(
        "Server started"
    );

    Serial.println(
        "Waiting for Python GUI..."
    );

    previousTime =
        millis();
}

void loop()
{
    // CHECK FOR PYTHON GUI
    
    WiFiClient client =
        server.available();

    if (client)
    {
        String request = "";

        // READ REQUEST

        while (client.connected())
        {
            if (client.available())
            {
                char c =
                    client.read();
                if (c == '\n')
                    break;

                request += c;
            }
        }
        Serial.print(
            "Received: "
        );

        Serial.println(
            request
        );

        // HORIZONTAL SELECTED
    

        if (
            request.indexOf("H:")
            >= 0
        )
        {
            int index =
                request.indexOf("H:");

            String value =
                request.substring(
                    index + 2
                );

            int space =
                value.indexOf(' ');


            if (space >= 0)
            {
                value =
                    value.substring(
                        0,
                        space
                    );
            }

            targetAngle =
                value.toFloat();

            selectedAxis =
                1;

            resetPID();

            Serial.print(
                "Horizontal selected"
            );

            Serial.print(
                " | Target = "
            );
            Serial.println(
                targetAngle
            );

            client.println(
                "HTTP/1.1 200 OK"
            );

            client.println(
                "Content-Type: text/plain"
            );

            client.println();

            client.println(
                "Horizontal selected"
            );
        }

        // VERTICAL SELECTED

        else if (
            request.indexOf("V:")
            >= 0
        )
        {
            int index =
                request.indexOf("V:");

            String value =
                request.substring(
                    index + 2
                );

            int space =
                value.indexOf(' ');

            if (space >= 0)
            {
                value =
                    value.substring(
                        0,
                        space
                    );
            }

            targetAngle =
                value.toFloat();

            selectedAxis =
                2;

            resetPID();

            Serial.print(
                "Vertical selected"
            );

            Serial.print(
                " | Target = "
            );

            Serial.println(
                targetAngle
            );

            client.println(
                "HTTP/1.1 200 OK"
            );

            client.println(
                "Content-Type: text/plain"
            );

            client.println();

            client.println(
                "Vertical selected"
            );
        }


        else if (
            request.indexOf("STOP")
            >= 0
        )
        {
            stopMotor();

            resetPID();

            selectedAxis = 0;


            client.println(
                "HTTP/1.1 200 OK"
            );

            client.println(
                "Content-Type: text/plain"
            );

            client.println();

            client.println(
                "Motor stopped"
            );
        }

        // STATUS

        else if (
            request.indexOf("STATUS")
            >= 0
        )
        {
            client.println(
                "HTTP/1.1 200 OK"
            );

            client.println(
                "Content-Type: text/plain"
            );

            client.println();


            client.print(
                "Axis: "
            );

            if (selectedAxis == 1)
                client.println(
                    "Horizontal"
                );

            else if (selectedAxis == 2)
                client.println(
                    "Vertical"
                );

            else
                client.println(
                    "None"
                );

            client.print(
                "Target: "
            );

            client.println(
                targetAngle
            );

            client.print(
                "Actual: "
            );

            client.println(
                actualAngle
            );

            client.print(
                "Error: "
            );

            client.println(
                error
            );

            client.print(
                "PID Output: "
            );

            client.println(
                pidOutput
            );
        }

        client.stop();
    }


    // READ ENCODER
   
    float motorAngle =
        readMotorAngle();


    // Convert motor angle to mirror angle

    actualAngle =
        motorAngle /
        GEAR_RATIO;


    if (selectedAxis != 0)
    {
        runPID();
    }

    // SERIAL MONITOR

    static unsigned long lastPrint = 0;

    if (
        millis() - lastPrint > 500
    )
    {
        lastPrint =
            millis();


        Serial.print(
            "Target = "
        );

        Serial.print(
            targetAngle
        );

        Serial.print(
            " | Actual = "
        );

        Serial.print(
            actualAngle
        );

        Serial.print(
            " | Error = "
        );

        Serial.print(
            error
        );

        Serial.print(
            " | PID = "
        );

        Serial.println(
            pidOutput
        );
    }
}
