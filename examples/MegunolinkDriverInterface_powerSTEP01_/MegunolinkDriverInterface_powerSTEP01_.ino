// Program to allow a MegunoLink interface to control a powerSTEP01-based ST
// X-NUCLEO-IHM03A1 stepper motor driver shield on an Arduino Uno-compatible board

// Written 2017/1/17 by Elliot Baptist for Number Eight Innovation
// Using a version of the SparkFun AutoDriver library modified for the powerSTEP01 

// Configuration structures used for EEPROM saving of configuration
#include "powerSTEP01MegunolinkConfiguration.h"
//#include "powerSTEP01MegunolinkCommands.h"

// Driver includes
#include "SparkFunAutoDriver.h"
#include "SPI.h"

// MegunoLink control includes
#include "MegunoLink.h"
#include "CommandHandler.h"
#include "EEPROMStore.h"


// Pin definitions for the X-NUCLEO-IHM03A1 connected to an Uno-compatible board
#define nCS_PIN 10
#define STCK_PIN 9
#define nSTBY_nRESET_PIN 8
#define nBUSY_PIN 4

// powerSTEP library instance, parameters are distance from the end of a daisy-chain
// of drivers, !CS pin, !STBY/!Reset pin
AutoDriver driver(0, nCS_PIN, nSTBY_nRESET_PIN);


CommandHandler<40> SerialCommandHandler;
EEPROMStore<basicPowerSTEP01Configuration> Config;


void setup() 
{
  // Start serial
  Serial.begin(9600);
  Serial.println(F("powerSTEP01 MegunoLink control initialising..."));

  // Setup commands
  SerialCommandHandler.AddCommand(F("GetStatus"), Cmd_GetStatus);

  SerialCommandHandler.AddCommand(F("SetMaxSpeed"), Cmd_SetMaxSpeed);
  SerialCommandHandler.AddCommand(F("SetOCThreshold"), Cmd_SetOCThreshold);
  SerialCommandHandler.AddCommand(F("SetRunKVAL"), Cmd_SetRunKVAL);
  SerialCommandHandler.AddCommand(F("SetHoldKVAL"), Cmd_SetHoldKVAL);
  
  SerialCommandHandler.AddCommand(F("Run"), Cmd_Run);
  SerialCommandHandler.AddCommand(F("StepClock"), Cmd_StepClock);
  SerialCommandHandler.AddCommand(F("Move"), Cmd_Move);
  SerialCommandHandler.AddCommand(F("GoTo"), Cmd_GoTo);
  SerialCommandHandler.AddCommand(F("GoUntil"), Cmd_GoUntil);
  SerialCommandHandler.AddCommand(F("ReleaseSw"), Cmd_ReleaseSw);
  SerialCommandHandler.AddCommand(F("GoHome"), Cmd_GoHome);
  //SerialCommandHandler.AddCommand(F("AutoHome"), Cmd_AutoHome);
  SerialCommandHandler.AddCommand(F("SetPos"), Cmd_SetPos);
  SerialCommandHandler.AddCommand(F("ResetPos"), Cmd_ResetPos); 
  SerialCommandHandler.AddCommand(F("SoftStop"), Cmd_SoftStop);
  SerialCommandHandler.AddCommand(F("HardStop"), Cmd_HardStop);
  SerialCommandHandler.AddCommand(F("SoftHiZ"), Cmd_SoftHiZ);
  SerialCommandHandler.AddCommand(F("HardHiZ"), Cmd_HardHiZ);

  SerialCommandHandler.SetDefaultHandler(Cmd_Unknown);

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
  
  driver.configSyncPin(BUSY_PIN, 0);// use SYNC/nBUSY pin as nBUSY, 
                                     // thus syncSteps (2nd paramater) does nothing
                                     
  driver.configStepMode(STEP_FS_128); // full steps, eg. 1/8 microstepping = STEP_FS_8,
                                // options: 1, 1/2, 1/4, 1/8, 1/16, 1/32, 1/64, 1/128
  
  driver.setMaxSpeed(Config.Data.maxSpeed); // full steps/s 
  driver.setFullSpeed(2000); // full steps/s threshold above which will use full steps
  driver.setAcc(2000); // full steps/s^2 acceleration
  driver.setDec(2000); // full steps/s^2 deceleration
  
  driver.setSlewRate(SR_520V_us); // faster may give more torque (and EM noise),
                                  // options are: 114, 220, 400, 520, 790, 980(V/us)
                                  
  driver.setOCThreshold(Config.Data.overCurrentThreshold);
  driver.setOCShutdown(OC_SD_ENABLE); // shutdown motor bridge on over-current event
                                      // to protect against permanant damage
  
  driver.setPWMFreq(PWM_DIV_1, PWM_MUL_0_75); // 16MHz*0.75/(512*1) = 23.4375kHz 
                            // power is supplied to stepper phases as a PWM signal,  
                            // the frequency is set by two PWM modulators,
                            // Fpwm = Fosc*m/(512*N), N and m are set by DIV and MUL,
                            // options: DIV: 1, 2, 3, 4, 5, 6, 7, 
                            // MUL: 0.625, 0.75, 0.875, 1, 1.25, 1.5, 1.75, 2
                            
  driver.setVoltageComp(VS_COMP_DISABLE); // no compensation for variation in Vs as
                                          // ADC voltage divider is not populated
                                          
  driver.setSwitchMode(SW_USER); // switch doesn't trigger stop, status can be read
                                 // SW_HARD_STOP: TP1 causes hard stop on connection 
                                 // to GND, get stuck on switch after homing
                                      
  driver.setOscMode(INT_16MHZ); // 16MHz internal oscillator as clock source

  // KVAL registers set
  driver.setRunKVAL(Config.Data.runKval);
  driver.setAccKVAL(Config.Data.runKval);
  driver.setDecKVAL(Config.Data.runKval);
  driver.setHoldKVAL(Config.Data.holdKval);

  driver.setParam(ALARM_EN, 0x8F); // disable ADC UVLO (divider not populated),
                                   // stall detection (not configured),
                                   // switch (not using as hard stop)
                                     
  driver.getStatus();

  Serial.println(F("Initialisation complete"));
}

void loop() 
{
  SerialCommandHandler.Process();
}


float MicrostepsToSteps(float microsteps)
{
  byte stepMode = driver.getStepMode();
  return microsteps/(0x1<<stepMode); // microsteps to full steps
}


//-----------------------------------------------------------------------------


void Cmd_GetStatus(CommandParameter &Parameters)
{
  Serial.print(F("Status: "));
  Serial.println(((unsigned int)driver.getStatus()), HEX);
}


void Cmd_SetMaxSpeed(CommandParameter &Parameters)
{
  float microStepSpeed = (float)Parameters.NextParameterAsDouble();
  
  float stepSpeed = MicrostepsToSteps(microStepSpeed);
  driver.setMaxSpeed(stepSpeed);
  
  Config.Data.maxSpeed = stepSpeed;
  Config.Save();

  Serial.print(F("New Max Speed: "));
  Serial.print(microStepSpeed);
  Serial.print(F(" (microsteps/s) "));
  Serial.print(stepSpeed);
  Serial.println(F(" (full steps/s)"));
}

void Cmd_SetOCThreshold(CommandParameter &Parameters)
{
  byte thresh = (byte)Parameters.NextParameterAsInteger();
  driver.setOCThreshold(thresh);

  Config.Data.overCurrentThreshold = thresh;
  Config.Save();

  Serial.print(F("New Over-Current Threshold: "));
  Serial.println(thresh);
}

void Cmd_SetRunKVAL(CommandParameter &Parameters)
{
  byte val = (byte)Parameters.NextParameterAsInteger();
  driver.setRunKVAL(val);
  driver.setAccKVAL(val);
  driver.setDecKVAL(val);

  Config.Data.runKval = val;
  Config.Save();

  Serial.print(F("New Running KVAL: "));
  Serial.println(val);
}

void Cmd_SetHoldKVAL(CommandParameter &Parameters)
{
  byte val = (byte)Parameters.NextParameterAsInteger();
  driver.setHoldKVAL(val);

  Config.Data.holdKval = val;
  Config.Save();

  Serial.print(F("New Holding KVAL: "));
  Serial.println(val);
}


void Cmd_Run(CommandParameter &Parameters)
{
  byte dir = (byte)Parameters.NextParameterAsInteger();
  float microStepSpeed = (float)Parameters.NextParameterAsDouble();

  float stepSpeed = MicrostepsToSteps(microStepSpeed);
  driver.run(dir, stepSpeed);
 
  Serial.print(F("Run "));
  Serial.print(dir);
  Serial.print(" ");
  Serial.print(microStepSpeed);
  Serial.print(F(" (microsteps/s) "));
  Serial.print(stepSpeed);
  Serial.println(F(" (full steps/s)"));
}

void Cmd_StepClock(CommandParameter &Parameters)
{
  byte dir = (byte)Parameters.NextParameterAsInteger(); 
  driver.stepClock(dir);
 
  Serial.print(F("Stepping on STCK input in dir: "));
  Serial.println(dir);
}

void Cmd_Move(CommandParameter &Parameters)
{
  // the next line wouldn't work as the call order is reversed
  //driver.move((byte)(Parameters.NextParameterAsInteger()), Parameters.NextParameterAsUnsignedLong());
  
  byte dir = (byte)Parameters.NextParameterAsInteger();
  unsigned long steps = Parameters.NextParameterAsUnsignedLong();
  driver.move(dir, steps);
 
  Serial.print(F("Move "));
  Serial.print(dir);
  Serial.print(" ");
  Serial.println(steps);
}

void Cmd_GoTo(CommandParameter &Parameters)
{
  long pos = Parameters.NextParameterAsLong();
  driver.goTo(pos);
 
  Serial.print(F("Goto "));
  Serial.println(pos);
}

void Cmd_GoUntil(CommandParameter &Parameters)
{
  byte act = (byte)Parameters.NextParameterAsInteger();
  byte dir = (byte)Parameters.NextParameterAsInteger();
  float microStepSpeed = (float)Parameters.NextParameterAsDouble(); 

  float stepSpeed = MicrostepsToSteps(microStepSpeed);
  driver.goUntil(act, dir, stepSpeed);
 
  Serial.print(F("GoUntil "));
  Serial.print(act);
  Serial.print(" ");
  Serial.print(dir);
  Serial.print(" ");
  Serial.print(microStepSpeed);
  Serial.print(F(" (microsteps/s) "));
  Serial.print(stepSpeed);
  Serial.println(F(" (full steps/s)"));
}

void Cmd_ReleaseSw(CommandParameter &Parameters)
{
  byte act = (byte)Parameters.NextParameterAsInteger();
  byte dir = (byte)Parameters.NextParameterAsInteger();

  //float oldMinSpeed = driver.getMinSpeed();
  //driver.setMinSpeed(driver.getMaxSpeed()/10);
  driver.releaseSw(act, dir);
  //while(driver.busyCheck());
  //driver.setMinSpeed(oldMinSpeed);
  
  Serial.print(F("Backing off Switch "));
  Serial.print(act);
  Serial.print(" ");
  Serial.println(dir);
}

void Cmd_GoHome(CommandParameter &Parameters)
{
  driver.goHome();
  Serial.println(F("Homing"));
}

/*void Cmd_AutoHome(CommandParameter &Parameters)
{
  CommandParameter Parameters2 = Parameters;
  Cmd_GoUntil(Parameters);
  while(driver.busyCheck());
  Cmd_ReleaseSw(Parameters2);

  Serial.println(F("Auto Homing"));
}*/

void Cmd_SetPos(CommandParameter &Parameters)
{
  unsigned long steps = Parameters.NextParameterAsUnsignedLong();
  driver.setPos(steps);
  Serial.print(F("Position set to "));
  Serial.println(steps);
}

void Cmd_ResetPos(CommandParameter &Parameters)
{
  driver.resetPos();
  Serial.println(F("Position Reset to 0"));
}

void Cmd_SoftStop(CommandParameter &Parameters)
{
  driver.softStop();
  Serial.println(F("Slowing for Hold"));
}

void Cmd_HardStop(CommandParameter &Parameters)
{
  driver.hardStop();
  Serial.println(F("Bridge set Hold"));
}

void Cmd_SoftHiZ(CommandParameter &Parameters)
{
  driver.softHiZ();
  Serial.println(F("Slowing for High Impedance"));
}

void Cmd_HardHiZ(CommandParameter &Parameters)
{
  driver.hardHiZ();
  Serial.println(F("Bridge set High Impedance"));
}

void Cmd_Unknown()
{
  Serial.println(F("Unknown Command"));
}

