#include <SPI.h>
#include <TMC2209.h>
#include <FastAccelStepper.h>
#include <HardwareSerial.h>

#define STEPA_EN_PIN 4    // Enable
#define STEPA_DIR_PIN 2   // Direction
#define STEPA_STEP_PIN 1  // Step
#define STEPA_STALL_PIN 7 // Connected to DIAG pin on the TMC2209
#define SERIALA_RX 6
#define SERIALA_TX 5

#define STEPB_EN_PIN 36   // Enable
#define STEPB_DIR_PIN 21  // Direction
#define STEPB_STEP_PIN 35 // Step
#define STEPB_STALL_PIN 37
#define SERIALB_RX 17
#define SERIALB_TX 14

#define STEPPER_RUN_CURRENT 250
#define STEPPER_HOLD_CURRENT 125

int aMin = 50;
int aMax = 9500;
int bMin = 50;
int bMax = 8000;

unsigned long ticker = 0L;

// The numbers after 'b' are determined by the state of pins MS2 and MS1 pins of
// TMC2209 respectively. 1->VCC 2->GND
// Both the driver's UART pins are connected to the same UART pins of the microcontroller.
// The distinct addresses are used to indentify each and control the parameters individually.
#define driverA_ADDRESS 0b00 // Pins MS1 and MS2 connected to GND.
#define driverB_ADDRESS 0b00 // Pin MS1 connected to VCC and MS2 connected to GND.

// Stallguard values for each driver(0-255), higher number -> higher sensitivity.
#define STALLA_VALUE 220
#define STALLB_VALUE 200

#define RA_SENSE 0.11f // Sense resistor value, match to your driverA
#define RB_SENSE 0.11f

// TMC2209Stepper driverA(&Serial1, RA_SENSE, driverA_ADDRESS);
// TMC2209Stepper driverB(&Serial2, RB_SENSE, driverB_ADDRESS);

TMC2209 driverA, driverB;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepperA = NULL;
FastAccelStepper *stepperB = NULL;

constexpr uint32_t steps_per_round = 200 * (60 / 16); // Claculated for the belt and pulley system.

bool startup = true; // set false after homing
volatile uint8_t stalled_A = 0;
volatile bool stalled_B = false;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

volatile bool pauseA = false;
volatile bool pauseB = false;

void IRAM_ATTR stallInterruptA()
{ // flag set for motor A when motor stalls
  portENTER_CRITICAL_ISR(&mux);
  stalled_A = 1;
  portEXIT_CRITICAL_ISR(&mux);
}

void stallInterruptB()
{ // flag set for motor B when motor stalls
  stalled_B = true;
}

void homeStepperA()
{
  Serial.print("Homing A started... ");

  attachInterrupt(digitalPinToInterrupt(STEPA_STALL_PIN), stallInterruptA, HIGH);

  // Enable stallguard
  driverA.setStallGuardThreshold(40);
  driverA.setCoolStepDurationThreshold(0xFFFFF); // TCOOLTHRS
  driverA.setRunCurrent(30);

  driverA.moveAtVelocity(1000);

  while (!stalled_A)
    ;
  detachInterrupt(digitalPinToInterrupt(STEPA_STALL_PIN));

  Serial.println("done!");
  driverA.disable();
  stepperA->setCurrentPosition(0);

  // Disable stallguard
  driverA.setStallGuardThreshold(0);
  driverA.setCoolStepDurationThreshold(0);

  driverA.enable();
}

void setup()
{
  delay(200);         // Leave time for native USB to come up for proper logging
  Serial.begin(9600); // ESP32S3 Usb logging output
  pinMode(STEPA_STALL_PIN, INPUT);
  // First, setup the actual stepper driver configuration via serial
  HardwareSerial serialA(1);
  // serialA.begin(115200, SERIAL_8N1, SERIALA_RX, SERIALA_TX);

  driverA.setup(serialA, 115200L, TMC2209::SERIAL_ADDRESS_0, SERIALA_RX, SERIALA_TX);
  delay(50);

  driverA.setRunCurrent(40);
  driverA.setHoldCurrent(30);
  driverA.setMicrostepsPerStep(8);
  // driverA.enableStealthChop();
  // driverA.enableAutomaticGradientAdaptation();
  // driverA.enableAutomaticCurrentScaling();
  // driverA.enableAutomaticGradientAdaptation();
  // driverA.enableAutomaticCurrentScaling();
  driverA.setPwmOffset(100);
  // driverA.setPwmGradient(120);
  // driverA.setStandstillMode(TMC2209::NORMAL);
  // driverA.setHoldDelay(10);
  driverA.enable();

  delay(100);

  // driverA.moveAtVelocity(5000);

  // Serial2.begin(115200, SERIAL_8N1, SERIALB_RX, SERIALB_TX);

  // attachInterrupt(digitalPinToInterrupt(STEPA_STALL_PIN), stallInterruptA, RISING);
  // attachInterrupt(digitalPinToInterrupt(STEPB_STALL_PIN), stallInterruptB, RISING);

  if (!driverA.isSetupAndCommunicating())
    Serial.println("Stepper driver not setup and communicating!");
  else
    Serial.println("Communication established!");

  /*
    driverB.begin();          // Initiate pins and registeries
    driverB.rms_current(STEPPER_RUN_CURRENT); // Set stepperA current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
    // driverA.en_pwm_mode(1);      // Enable extremely quiet stepping
    driverB.pwm_autoscale(1);
    driverB.microsteps(16);
    driverB.TCOOLTHRS(0xFFFFF); // 20bit max
    driverB.SGTHRS(STALLB_VALUE);
  */
  // Now, setup the stepper input/output steps
  engine.init();

  stepperA = engine.stepperConnectToPin(STEPA_STEP_PIN);
  stepperA->setDirectionPin(STEPA_DIR_PIN);
  stepperA->setEnablePin(STEPA_EN_PIN);
  // stepperA->setAutoEnable(true);  // You will lose holding current
  stepperA->enableOutputs();

  homeStepperA();
  delay(500);

  if (stepperA)
  {
    Serial.println("doit");

    // If auto enable/disable need delays, just add (one or both):
    // stepperA->setDelayToEnable(50);
    // stepperA->setDelayToDisable(1000);

    stepperA->setSpeedInUs(150); // the parameter is us/step !!!

    stepperA->setAcceleration(4000);
    stepperA->move(1600);
  }

  // motorAHome();
  // motorBHome();

  // stepperB.moveTo(2000);
}

void loop()
{
  // Serial.println("Entering loop()");

  if (stalled_A)
  {
    Serial.println("Stalled A");
    stalled_A = false;
    driverA.disable();
    delay(10);
    driverA.enable();
    delay(10);
    pauseA = false;
  }

  if (millis() - ticker > 2500)
  {
    ticker = millis();
    // stepperA->setCurrentPosition(0);
    stepperA->move(1600);
  }

  /*
  if(Serial.available()>0){
    char readVal = Serial.read();
    if (readVal == 'a'){
    int steps = Serial.parseInt();
    stepperA.moveTo(steps);

    } else if (readVal == 'A'){
    motorAHome();
    }

    if (readVal == 'b'){
    int steps = Serial.parseInt();
    stepperB.moveTo(steps);

    } else if (readVal == 'B'){
    motorBHome();
    }
  }
  */

  // stepperA.run();
  // stepperB.run();
}