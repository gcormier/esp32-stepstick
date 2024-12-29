// #include <SPI.h>
#include <TMC2209.h>
#include <FastAccelStepper.h>
#include <HardwareSerial.h>

struct MotorParams
{
  int en_pin;
  int dir_pin;
  int step_pin;
  int stall_pin;
  int serial_rx;
  int serial_tx;
  int run_current;
  int hold_current;
  int stall_threshold; // Higher number, higher sensitivity
  int up_direction;
};

MotorParams MotorAParams = {
    .en_pin = 4,
    .dir_pin = 2,
    .step_pin = 1,
    .stall_pin = 7,
    .serial_rx = 6,
    .serial_tx = 5,
    .run_current = 200,
    .hold_current = 100,
    .stall_threshold = 75,
    .up_direction = 1};

MotorParams MotorBParams = {
    .en_pin = 36,
    .dir_pin = 21,
    .step_pin = 35,
    .stall_pin = 37,
    .serial_rx = 17,
    .serial_tx = 14,
    .run_current = 200,
    .hold_current = 100,
    .stall_threshold = 85,
    .up_direction = -1};

unsigned long ticker = 0L;

#define HOMINGSPEED 2000

HardwareSerial &serialA = Serial1;
HardwareSerial &serialB = Serial2;

TMC2209 driverA, driverB;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepperA = NULL;
FastAccelStepper *stepperB = NULL;

constexpr uint8_t pulley_teeth = 20;
constexpr float stepper_degrees = 1.80;
constexpr uint8_t microsteps = 8;
constexpr uint8_t beltpitch = 2; // 2mm gt2 belt

constexpr float steps_per_revolution = (360.0 / stepper_degrees) * microsteps;
constexpr float belt_travel_per_revolution_mm = pulley_teeth * beltpitch;
constexpr float steps_per_mm = steps_per_revolution / belt_travel_per_revolution_mm;
constexpr float steps_per_cm = steps_per_mm * 10; // 1 cm = 10 mm

volatile bool stalled_A = false;
volatile bool stalled_B = false;

bool lostComms = false;

void IRAM_ATTR stallInterruptA()
{ // flag set for motor A when motor stalls
  stalled_A = true;
}

void IRAM_ATTR stallInterruptB()
{ // flag set for motor B when motor stalls
  stalled_B = true;
}

void homeStepper(MotorParams &motorParams, TMC2209 &driver)
{
  Serial.print("Homing started... ");

  // Enable stallguard
  driver.setStallGuardThreshold(motorParams.stall_threshold);
  driver.setCoolStepDurationThreshold(0xFFFFF); // TCOOLTHRS
  // driverA.setRunCurrent(100);

  driver.moveAtVelocity(HOMINGSPEED * motorParams.up_direction);
  attachInterrupt(digitalPinToInterrupt(motorParams.stall_pin), stallInterruptA, RISING);
  stalled_A = false;
  while (!stalled_A)
    ;

  driver.moveAtVelocity(0);
  detachInterrupt(digitalPinToInterrupt(motorParams.stall_pin));

  Serial.println("Stall on motor detected, setting home!");
  driver.disable();
  // stepperA->setCurrentPosition(0);

  // Disable stallguard
  driver.setStallGuardThreshold(0);
  driver.setCoolStepDurationThreshold(0);

  driver.enable();
}

void setupDriver(MotorParams &motorParams, TMC2209 &driver)
{
  driver.setHardwareEnablePin(motorParams.en_pin);

  // driver.disableCoolStep();
  driver.useExternalSenseResistors();
  driver.setRunCurrent(motorParams.run_current);
  driver.setRMSCurrent(motorParams.hold_current, 0.11f);
  driver.setMicrostepsPerStep(microsteps);
  driver.disableStealthChop();
  delay(50);
  driver.enableStealthChop();
  driver.enableAutomaticCurrentScaling();
  driver.enableAutomaticGradientAdaptation();

  delay(150);

  driver.enable();
}

void setup()
{
  delay(300);         // Leave time for native USB to come up for proper logging
  Serial.begin(9600); // ESP32S3 Usb logging output
  delay(50);
  Serial.println("setup() entered");
  pinMode(MotorAParams.stall_pin, INPUT);
  pinMode(MotorBParams.stall_pin, INPUT);

  // First, setup the actual stepper driver configuration via serial
  driverA.setup(serialA, 115200L, TMC2209::SERIAL_ADDRESS_0, MotorAParams.serial_rx, MotorAParams.serial_tx);
  driverB.setup(serialB, 115200L, TMC2209::SERIAL_ADDRESS_0, MotorBParams.serial_rx, MotorBParams.serial_tx);

  setupDriver(MotorAParams, driverA);
  setupDriver(MotorBParams, driverB);

  while (!driverA.isSetupAndCommunicating() && !driverB.isSetupAndCommunicating())
  {
    Serial.println("Waiting for comms to both motors!");
    delay(500);
    setupDriver(MotorAParams, driverA);
    setupDriver(MotorBParams, driverB);
  }

  // Now, setup the stepper input/output steps
  engine.init();

  stepperA = engine.stepperConnectToPin(MotorAParams.step_pin);
  stepperA->setDirectionPin(MotorAParams.dir_pin);
  stepperA->setEnablePin(MotorAParams.en_pin);
  stepperA->setAutoEnable(false); // You will lose holding current
  stepperA->enableOutputs();

  stepperB = engine.stepperConnectToPin(MotorBParams.step_pin);
  stepperB->setDirectionPin(MotorBParams.dir_pin);
  stepperB->setEnablePin(MotorBParams.en_pin);
  stepperB->setAutoEnable(false); // You will lose holding current
  stepperB->enableOutputs();

  homeStepper(MotorAParams, driverA);
  stepperA->setCurrentPosition(0);
  homeStepper(MotorBParams, driverB);
  stepperB->setCurrentPosition(0);
  delay(100);

  stepperA->setAcceleration(20000);
  stepperB->setAcceleration(20000);

  stepperA->setSpeedInTicks(3000);
  stepperB->setSpeedInTicks(3000);

  driverA.setRunCurrent(40);
  driverB.setRunCurrent(40);
}

bool flip = true;

void loop()
{
  while (lostComms)
  {
    ; // Force a hard reset since we lost comms, don't know the state
  }

  if (!driverA.isSetupAndCommunicating() || !driverB.isSetupAndCommunicating())
  {
    lostComms = true;
    Serial.println("Lost communication with one or both drivers! Reboot!");
  }

  int moveCM = 29;

  if (!stepperA->isRunning() && flip)
  {
    delay(200);
    stepperA->move(steps_per_cm * moveCM);
    stepperB->move(-steps_per_cm * moveCM);

    flip = false;
  }

  if (!stepperA->isRunning() && !flip)
  {
    delay(1000);
    stepperA->move(-steps_per_cm * moveCM);
    stepperB->move(steps_per_cm * moveCM);
    flip = true;
  }
}