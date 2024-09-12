#include <Servo.h>

//
// PORT NUMBERS
//
#define SRV_0_PORT 6 // front (left) servo facing +x
#define SRV_1_PORT 9 // back (right) servo facing -x

// 
// SERVO PARAMETERS
//
#define SRV_0_OPEN 170 // 172 might just touch the frame, 170 bit below
#define SRV_0_CLOSE 82 // 82
const int SRV_0_RANGE = SRV_0_OPEN - SRV_0_CLOSE;
const float SRV_0_FACTOR = float(SRV_0_RANGE) / 90.0;

#define SRV_1_OPEN 6 // 4 might just touch the frame, 6 bit below
#define SRV_1_CLOSE 94 // 94
const int SRV_1_RANGE = SRV_1_OPEN - SRV_1_CLOSE;
const float SRV_1_FACTOR = float(SRV_1_RANGE) / 90.0;

// Declare Servo Objects
Servo Servo_0; // front (left)
Servo Servo_1; // back (right)

// buffer for Serial receiving (RX)
const int RX_BUFFER_SIZE = 2;
uint8_t rx_buffer[RX_BUFFER_SIZE];


void setup() {
  // initialize Servos
  Servo_0.attach(SRV_0_PORT);
  Servo_1.attach(SRV_1_PORT);

  // initial values
  Servo_0.write(SRV_0_CLOSE + SRV_0_FACTOR * float(70)); // front (left)
  Servo_1.write(SRV_1_OPEN);

  // DEV
  // Servo_0.write(SRV_0_CLOSE);
  // Servo_1.write(SRV_1_CLOSE);

  // initialize Serial
  Serial.begin(115200);
}


void loop() {
  /* SERIAL CONNECTION */
  if (Serial.available()) {
    // read in command
    Serial.readBytes(rx_buffer, RX_BUFFER_SIZE);

    // check command
    if (rx_buffer[0] < 0 || rx_buffer[0] > 90 ||
        rx_buffer[1] < 0 || rx_buffer[1] > 90) {
      // Serial.println("Gripper angles must be between 0 and 90 degrees.");
      return;
    }

    // write command to servo
    Servo_0.write(SRV_0_CLOSE + SRV_0_FACTOR * float(rx_buffer[0])); // front (left)
    Servo_1.write(SRV_1_CLOSE + SRV_1_FACTOR * float(rx_buffer[1])); // back (right)
  }

  
  /* SERVO TEST */
  // Servo_0.write(SRV_0_OPEN);
  // Servo_1.write(SRV_1_OPEN);
  // delay(1000);
  // Servo_0.write(SRV_0_CLOSE);
  // Servo_1.write(SRV_1_CLOSE);
  // delay(1000);
}
