#define MOTOR_DIR_L 13
#define MOTOR_PWN_L 12
#define MOTOR_DIR_R 14
#define MOTOR_PWN_R 27

const int freq = 10000;       // 10 kHz frequency
const int pwmChannelL = 0;    // PWM channel for left motor
const int pwmChannelR = 1;    // PWM channel for right motor
const int resolution = 8;     // 8-bit resolution (0-255)

void setup() {
    // Initialize motor direction pins as outputs
    pinMode(MOTOR_DIR_L, OUTPUT);
    pinMode(MOTOR_DIR_R, OUTPUT);

    // Configure PWM channels
    ledcSetup(pwmChannelL, freq, resolution);
    ledcSetup(pwmChannelR, freq, resolution);

    // Attach the PWM channels to the motor PWM pins
    ledcAttachPin(MOTOR_PWN_L, pwmChannelL);
    ledcAttachPin(MOTOR_PWN_R, pwmChannelR);
}

void loop() {
    // Make motors go forward
    digitalWrite(MOTOR_DIR_L, HIGH);
    digitalWrite(MOTOR_DIR_R, HIGH);

    // Increase speed from 0 to max in 1 second
    for (int dutyCycle = 0; dutyCycle <= 255; dutyCycle++) {
        ledcWrite(pwmChannelL, dutyCycle);
        ledcWrite(pwmChannelR, dutyCycle);
        delay(10); // 1000 ms / 255 steps ≈ 4 ms per step
    }

    // Decrease speed from max to 0 in 1 second
    for (int dutyCycle = 255; dutyCycle >= 0; dutyCycle--) {
        ledcWrite(pwmChannelL, dutyCycle);
        ledcWrite(pwmChannelR, dutyCycle);
        delay(10); // 1000 ms / 255 steps ≈ 4 ms per step
    }

    // Make motors go backward
    digitalWrite(MOTOR_DIR_L, LOW);
    digitalWrite(MOTOR_DIR_R, LOW);

    // Increase speed from 0 to max in 1 second
    for (int dutyCycle = 0; dutyCycle <= 255; dutyCycle++) {
        ledcWrite(pwmChannelL, dutyCycle);
        ledcWrite(pwmChannelR, dutyCycle);
        delay(10); // 1000 ms / 255 steps ≈ 4 ms per step
    }

    // Decrease speed from max to 0 in 1 second
    for (int dutyCycle = 255; dutyCycle >= 0; dutyCycle--) {
        ledcWrite(pwmChannelL, dutyCycle);
        ledcWrite(pwmChannelR, dutyCycle);
        delay(10); // 1000 ms / 255 steps ≈ 4 ms per step
    }

    delay(1000); // Wait for 1 second before repeating
}
