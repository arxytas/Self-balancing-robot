#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"

#define leftwheelPWM   7
#define leftwheeldir   6
#define rightwheelPWM  8
#define rightwheeldir  9
#define samplingtime  0.005
#define correctionAngle -2.0
#define p  20
#define d  0.1
#define i  40

int16_t accY, accZ, gyroX;
volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle=0, error, prevError=0, errorSum=0;
volatile byte count=0;

MPU6050 mpu;

void setMotors(int leftwheelSpeed, int rightwheelSpeed) {
  if(leftwheelSpeed >= 0) {
    analogWrite(leftwheelPWM, leftwheelSpeed);
    digitalWrite(leftwheeldir, LOW);
  }
  else {
    analogWrite(leftwheelPWM, 255 + leftwheelSpeed);
    digitalWrite(leftwheeldir, HIGH);
  }
  if(rightwheelSpeed >= 0) {
    analogWrite(rightwheelPWM, rightwheelSpeed);
    digitalWrite(rightwheeldir, LOW);
  }
  else {
    analogWrite(rightwheelPWM, 255 + rightwheelSpeed);
    digitalWrite(rightwheeldir, HIGH);
  }
}

void init_PID() {  
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B    
  // set compare match register to set sample time 5ms
  OCR1A = 9999;    
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for prescaling by 8
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}

void setup() {
  Serial.begin(9600);
  // set the motor control and PWM pins to output mode
  pinMode(leftwheelPWM, OUTPUT);
  pinMode(leftwheeldir, OUTPUT);
  pinMode(rightwheelPWM, OUTPUT);
  pinMode(rightwheeldir, OUTPUT);
  // set the status LED to output mode 
  pinMode(13, OUTPUT);
  // initialize the MPU6050 and set offset values
  mpu.initialize();
  mpu.setYAccelOffset(-274);
  mpu.setZAccelOffset(165);
  mpu.setXGyroOffset(-68);
  // initialize PID sampling loop
  init_PID();
}

void loop() {
  // read acceleration and gyroscope values
  accY = mpu.getAccelerationY();
  accZ = mpu.getAccelerationZ();  
  gyroX = mpu.getRotationX();
  Serial.println(gyroX);
  // set motor power after constraining it 
  motorPower = constrain(motorPower, -255, 255);
  //Serial.println("i'm here");
  setMotors(motorPower, motorPower);
  
}
// The ISR will be called every 5 milliseconds
ISR(TIMER1_COMPA_vect)
{
  // calculate the angle of inclination
  accAngle = atan2(accY, accZ)*RAD_TO_DEG;
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate*samplingtime;  
  currentAngle = 0.9934*(prevAngle + gyroAngle) + 0.0066*(accAngle);
  error = currentAngle - correctionAngle;
  errorSum = errorSum + error;  
  //Serial.println(errorSum);
  errorSum = constrain(errorSum, -300, 300);
  //Serial.println("error sum=%f", errorSum);
  //calculate output from P, I and D values
  motorPower = p*(error) + i*(errorSum)*samplingtime - d*(currentAngle-prevAngle)/samplingtime;
  prevAngle = currentAngle;
  // toggle the led on pin13 every second
  count++;
  if(count == 200)  {
    count = 0;
    digitalWrite(13, !digitalRead(13));
  }
}
