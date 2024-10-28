const int IN1 = 5; // Motor 1, direction pin
const int IN2 = 6; // Motor 1, PWM speed pin
const int IN3 = 9; // Motor 2, direction pin
const int IN4 = 10; // Motor 2, PWM speed pin

void setup() {
  // Initialize motor control pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Set all outputs to low initially
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void loop() {
  // Test Motor 1 forward at full speed
  digitalWrite(IN1, HIGH);   // Set Motor 1 forward direction
  analogWrite(IN2, 255);     // Set full speed for Motor 1

  // Test Motor 2 backward at half speed
  digitalWrite(IN3, LOW);    // Set Motor 2 backward direction
  analogWrite(IN4, 128);     // Set half speed for Motor 2

  delay(2000); // Run for 2 seconds

  // Stop both motors
  analogWrite(IN2, 0);       // Stop Motor 1
  analogWrite(IN4, 0);       // Stop Motor 2

  delay(2000); // Pause for 2 seconds
}