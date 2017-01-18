#include <SparkFunAutoDriver.h>
#include <SPI.h>

// Pin definitions for the X-NUCLEO-IHM03A1 connected to an Uno-compatible board
#define nCS_PIN 10
#define nSTBY_nRESET_PIN 8
#define nBUSY_PIN 4

// powerSTEP library instance, parameters are distance from the end of a daisy-chain
// of drivers, !CS pin, !STBY/!Reset pin
AutoDriver driver(0, nCS_PIN, nSTBY_nRESET_PIN);

void setup() 
{
  // Start serial
  Serial.begin(9600);
  Serial.println("powerSTEP control initialising...");

  // Prepare pins
  pinMode(nSTBY_nRESET_PIN, OUTPUT);
  pinMode(nCS_PIN, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, OUTPUT);
  pinMode(SCK, OUTPUT);

  // Reset powerSTEP and set CS
  digitalWrite(nSTBY_nRESET_PIN, HIGH);
  digitalWrite(nSTBY_nRESET_PIN, LOW);
  digitalWrite(nSTBY_nRESET_PIN, HIGH);
  digitalWrite(nCS_PIN, HIGH);

  // Start SPI
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);

  // Configure powerSTEP
  driver.SPIPortConnect(&SPI); // give library the SPI port (only the one on an Uno)
  driver.configSyncPin(BUSY_PIN, 0); // use SYNC/nBUSY pin as nBUSY, 
                                     // thus syncSteps (2nd paramater) does nothing
  driver.configStepMode(STEP_FS_128); // full steps, eg. 1/8 microstepping = STEP_FS_8,
                                // options: 1, 1/2, 1/4, 1/8, 1/16, 1/32, 1/64, 1/128
  driver.setMaxSpeed(1000); // steps/s 
  driver.setFullSpeed(1000); // steps/s threshold above which will use full steps
  driver.setAcc(1000); // steps/s^2 acceleration
  driver.setDec(1000); // steps/s^2 deceleration
  driver.setSlewRate(SR_520V_us); // faster may give more torque and signal noise,
                                  // options are: 114, 220, 400, 520, 790, 980(V/us)
  driver.setOCThreshold(0x03); // Over-Current, ------------------------
  driver.setPWMFreq(PWM_DIV_1, PWM_MUL_0_75); // 16MHz*0.75/(512*1) = 23.4375kHz 
                            // power is supplied to stepper phases as a sin wave,  
                            // frequency is set by two PWM modulators,
                            // Fpwm = Fosc*m/(512*N), N and m are set by DIV and MUL,
                            // options: DIV: 1, 2, 3, 4, 5, 6, 7, 
                            // MUL: 0.625, 0.75, 0.875, 1, 1.25, 1.5, 1.75, 2
  driver.setOCShutdown(OC_SD_DISABLE); // shutdown motor bridge on over-current event
                                      // to protect against permanant damage
  driver.setVoltageComp(VS_COMP_DISABLE); // no compensation for variation in Vs as
                                          // ADC voltage divider is not populated
  driver.setSwitchMode(SW_HARD_STOP); // TP1 causes hard stop on connection to GND,
                                      // SW_USER no stop, status can still be read
  driver.setOscMode(INT_16MHZ); // 16MHz internal oscillator as clock source

  // KVAL registers set
  driver.setRunKVAL(80);
  driver.setAccKVAL(80);
  driver.setDecKVAL(80);
  driver.setHoldKVAL(100);

  driver.setParam(ALARM_EN, 0x00CF); // disable ADC UVLO, , 
                                     // Stall detection flag triggers

  driver.getStatus();

  Serial.println("Initialisation complete");
}

void loop() 
{ 
  Serial.println(driver.getParam(STEP_MODE), HEX);
  Serial.println((int)driver.getStatus(), HEX);
  
  //driver.move(FWD, 200000);
  while(driver.busyCheck());
  driver.softStop();
  while(driver.busyCheck());
  
  //driver.move(REV, 200000);
  while(driver.busyCheck());
  //driver.softStop();
  while(driver.busyCheck());
  
  Serial.println((int)driver.getStatus(), HEX);
  
  while(1);
}
