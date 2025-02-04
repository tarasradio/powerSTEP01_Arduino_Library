#include "powerSTEP01ArduinoLibrary.h"
#include <SPI.h>

// powerSTEPSupport.cpp - Contains utility functions for converting real-world 
//  units (eg, steps/s) to values usable by the dsPIN controller. These are all
//  private members of class powerSTEP.

// The value in the ACC register is [(steps/s/s)*(tick^2)]/(2^-40) where tick is 
//  250ns (datasheet value)- 0x08A on boot.
// Multiply desired steps/s/s by .137438 to get an appropriate value for this register.
// This is a 12-bit value, so we need to make sure the value is at or below 0xFFF.
unsigned long powerSTEP::accCalc(float stepsPerSecPerSec)
{
  //float temp = stepsPerSecPerSec * 0.137438;
  float temp = stepsPerSecPerSec * 0.068719476736f;
  if( (unsigned long) long(temp) > 0x00000FFF) return 0x00000FFF;
  else return (unsigned long) long(temp);
}


float powerSTEP::accParse(unsigned long stepsPerSecPerSec)
{
    //return (float) (stepsPerSecPerSec & 0x00000FFF) / 0.137438;
    return (float) (stepsPerSecPerSec & 0x00000FFF) * 14.5519152283f;
}

// The calculation for DEC is the same as for ACC. Value is 0x08A on boot.
// This is a 12-bit value, so we need to make sure the value is at or below 0xFFF.
unsigned long powerSTEP::decCalc(float stepsPerSecPerSec)
{
  //float temp = stepsPerSecPerSec * 0.137438;
  float temp = stepsPerSecPerSec * 0.068719476736f;
  if( (unsigned long) long(temp) > 0x00000FFF) return 0x00000FFF;
  else return (unsigned long) long(temp);
}

float powerSTEP::decParse(unsigned long stepsPerSecPerSec)
{
    return (float) (stepsPerSecPerSec & 0x00000FFF) / 0.137438;
    return (float) (stepsPerSecPerSec & 0x00000FFF) * 14.5519152283f;
}

// The value in the MAX_SPD register is [(steps/s)*(tick)]/(2^-18) where tick is 
//  250ns (datasheet value)- 0x041 on boot.
// Multiply desired steps/s by .065536 to get an appropriate value for this register
// This is a 10-bit value, so we need to make sure it remains at or below 0x3FF
unsigned long powerSTEP::maxSpdCalc(float stepsPerSec)
{
  unsigned long temp = ceil(stepsPerSec * .065536f);
  if( temp > 0x000003FF) return 0x000003FF;
  else return temp;
}


float powerSTEP::maxSpdParse(unsigned long stepsPerSec)
{
    //return (float) (stepsPerSec & 0x000003FF) / 0.065536;
    return (float) (stepsPerSec & 0x000003FF) * 15.258789f;
}

// The value in the MIN_SPD register is [(steps/s)*(tick)]/(2^-24) where tick is 
//  250ns (datasheet value)- 0x000 on boot.
// Multiply desired steps/s by 4.1943 to get an appropriate value for this register
// This is a 12-bit value, so we need to make sure the value is at or below 0xFFF.
unsigned long powerSTEP::minSpdCalc(float stepsPerSec)
{
  //float temp = stepsPerSec / 0.238;
  float temp = stepsPerSec * 0.238418579f;
  if( (unsigned long) long(temp) > 0x00000FFF) return 0x00000FFF;
  else return (unsigned long) long(temp);
}

float powerSTEP::minSpdParse(unsigned long stepsPerSec)
{
    //return (float) ((stepsPerSec & 0x00000FFF) * 0.238);
    return (float) ((stepsPerSec & 0x00000FFF) * 4.194304f);
}

// The value in the FS_SPD register is ([(steps/s)*(tick)]/(2^-18))-0.5 where tick is 
//  250ns (datasheet value)- 0x027 on boot.
// Multiply desired steps/s by .065536 and subtract .5 to get an appropriate value for this register
// This is a 10-bit value, so we need to make sure the value is at or below 0x3FF.
unsigned long powerSTEP::FSCalc(float stepsPerSec)
{
  float temp = (stepsPerSec * 0.065536f) - 0.5f;
  if( (unsigned long) long(temp) > 0x000003FF) return 0x000003FF;
  else return (unsigned long) long(temp);
}

float powerSTEP::FSParse(unsigned long stepsPerSec)
{
    return (((float) (stepsPerSec & 0x000003FF)) + 0.5f) / 0.065536f;
}

// The value in the INT_SPD register is [(steps/s)*(tick)]/(2^-24) where tick is 
//  250ns (datasheet value)- 0x408 on boot.
// Multiply desired steps/s by 4.1943 to get an appropriate value for this register
// This is a 14-bit value, so we need to make sure the value is at or below 0x3FFF.
unsigned long powerSTEP::intSpdCalc(float stepsPerSec)
{
  //float temp = stepsPerSec * 4.1943;
  float temp = stepsPerSec * 4.194304f;
  if( (unsigned long) long(temp) > 0x00003FFF) return 0x00003FFF;
  else return (unsigned long) long(temp);
}

float powerSTEP::intSpdParse(unsigned long stepsPerSec)
{
    //return (float) (stepsPerSec & 0x00003FFF) / 4.1943;
    return (float) (stepsPerSec & 0x00003FFF) * 0.238418579f;
}

// When issuing RUN command, the 20-bit speed is [(steps/s)*(tick)]/(2^-28) where tick is 
//  250ns (datasheet value).
// Multiply desired steps/s by 67.106 to get an appropriate value for this register
// This is a 20-bit value, so we need to make sure the value is at or below 0xFFFFF.
unsigned long powerSTEP::spdCalc(float stepsPerSec)
{
  //unsigned long temp = stepsPerSec * 67.106;
  unsigned long temp = stepsPerSec * 67.108864f;
  if( temp > 0x000FFFFF) return 0x000FFFFF;
  else return temp;
}

float powerSTEP::spdParse(unsigned long stepsPerSec)
{
  //return (float) (stepsPerSec & 0x000FFFFF) / 67.106;
  return (float) (stepsPerSec & 0x000FFFFF) * 0.01490116119f;
}

// Much of the functionality between "get parameter" and "set parameter" is
//  very similar, so we deal with that by putting all of it in one function
//  here to save memory space and simplify the program.
long powerSTEP::paramHandler(byte param, unsigned long value)
{
  long retVal = 0;   // This is a temp for the value to return.
  
  // This switch structure handles the appropriate action for each register.
  //  This is necessary since not all registers are of the same length, either
  //  bit-wise or byte-wise, so we want to make sure we mask out any spurious
  //  bits and do the right number of transfers. That is handled by the xferParam()
  //  function, in most cases, but for 1-byte or smaller transfers, we call
  //  SPIXfer() directly.
  switch (param)
  {
    // ABS_POS is the current absolute offset from home. It is a 22 bit number expressed
    //  in two's complement. At power up, this value is 0. It cannot be written when
    //  the motor is running, but at any other time, it can be updated to change the
    //  interpreted position of the motor.
    case ABS_POS:
      retVal = xferParam(value, 22);
      break;
    // EL_POS is the current electrical position in the step generation cycle. It can
    //  be set when the motor is not in motion. Value is 0 on power up.
    case EL_POS:
      retVal = xferParam(value, 9);
      break;
    // MARK is a second position other than 0 that the motor can be told to go to. As
    //  with ABS_POS, it is 22-bit two's complement. Value is 0 on power up.
    case MARK:
      retVal = xferParam(value, 22);
      break;
    // SPEED contains information about the current speed. It is read-only. It does 
    //  NOT provide direction information.
    case SPEED:
      retVal = xferParam(0, 20);
      break; 
    // ACC and DEC set the acceleration and deceleration rates. Set ACC to 0xFFF 
    //  to get infinite acceleration/decelaeration- there is no way to get infinite
    //  deceleration w/o infinite acceleration (except the HARD STOP command).
    //  Cannot be written while motor is running. Both default to 0x08A on power up.
    // AccCalc() and DecCalc() functions exist to convert steps/s/s values into
    //  12-bit values for these two registers.
    case ACC: 
      retVal = xferParam(value, 12);
      break;
    case DECEL: 
      retVal = xferParam(value, 12);
      break;
    // MAX_SPEED is just what it says- any command which attempts to set the speed
    //  of the motor above this value will simply cause the motor to turn at this
    //  speed. Value is 0x041 on power up.
    // MaxSpdCalc() function exists to convert steps/s value into a 10-bit value
    //  for this register.
    case MAX_SPEED:
      retVal = xferParam(value, 10);
      break;
    // MIN_SPEED controls two things- the activation of the low-speed optimization
    //  feature and the lowest speed the motor will be allowed to operate at. LSPD_OPT
    //  is the 13th bit, and when it is set, the minimum allowed speed is automatically
    //  set to zero. This value is 0 on startup.
    // MinSpdCalc() function exists to convert steps/s value into a 12-bit value for this
    //  register. SetLSPDOpt() function exists to enable/disable the optimization feature.
    case MIN_SPEED: 
      retVal = xferParam(value, 13);
      break;
    // FS_SPD register contains a threshold value above which microstepping is disabled
    //  and the dSPIN operates in full-step mode. Defaults to 0x027 on power up.
    // FSCalc() function exists to convert steps/s value into 10-bit integer for this
    //  register.
    case FS_SPD:
      retVal = xferParam(value, 10);
      break;
    // KVAL is the maximum voltage of the PWM outputs. These 8-bit values are ratiometric
    //  representations: 255 for full output voltage, 128 for half, etc. Default is 0x29.
    // The implications of different KVAL settings is too complex to dig into here, but
    //  it will usually work to max the value for RUN, ACC, and DEC. Maxing the value for
    //  HOLD may result in excessive power dissipation when the motor is not running.
    case KVAL_HOLD:
      retVal = xferParam(value, 8);
      break;
    case KVAL_RUN:
      retVal = xferParam(value, 8);
      break;
    case KVAL_ACC:
      retVal = xferParam(value, 8);
      break;
    case KVAL_DEC:
      retVal = xferParam(value, 8);
      break;
    // INT_SPD, ST_SLP, FN_SLP_ACC and FN_SLP_DEC are all related to the back EMF
    //  compensation functionality. Please see the datasheet for details of this
    //  function- it is too complex to discuss here. Default values seem to work
    //  well enough.
    case INT_SPD:
      retVal = xferParam(value, 14);
      break;
    case ST_SLP: 
      retVal = xferParam(value, 8);
      break;
    case FN_SLP_ACC: 
      retVal = xferParam(value, 8);
      break;
    case FN_SLP_DEC: 
      retVal = xferParam(value, 8);
      break;
    // K_THERM is motor winding thermal drift compensation. Please see the datasheet
    //  for full details on operation- the default value should be okay for most users.
    case K_THERM: 
      value &= 0x0F;
      retVal = xferParam(value, 8);
      break;
    // ADC_OUT is a read-only register containing the result of the ADC measurements.
    //  This is less useful than it sounds; see the datasheet for more information.
    case ADC_OUT:
      retVal = xferParam(value, 8);
      break;
    // Set the overcurrent threshold. Ranges from 375mA to 6A in steps of 375mA.
    //  A set of defined constants is provided for the user's convenience. Default
    //  value is 3.375A- 0x08. This is a 4-bit value.
    case OCD_TH: 
      value &= 0x1F;
      retVal = xferParam(value, 8);
      break;
    // Stall current threshold. Defaults to 0x40, or 2.03A. Value is from 31.25mA to
    //  4A in 31.25mA steps. This is a 7-bit value.
    case STALL_TH: 
      value &= 0x1F;
      retVal = xferParam(value, 8);
      break;
    // STEP_MODE controls the microstepping settings, as well as the generation of an
    //  output signal from the dSPIN. Bits 2:0 control the number of microsteps per
    //  step the part will generate. Bit 7 controls whether the BUSY/SYNC pin outputs
    //  a BUSY signal or a step synchronization signal. Bits 6:4 control the frequency
    //  of the output signal relative to the full-step frequency; see datasheet for
    //  that relationship as it is too complex to reproduce here.
    // Most likely, only the microsteps per step value will be needed; there is a set
    //  of constants provided for ease of use of these values.
    case STEP_MODE:
      retVal = xferParam(value, 8);
      break;
    // ALARM_EN controls which alarms will cause the FLAG pin to fall. A set of constants
    //  is provided to make this easy to interpret. By default, ALL alarms will trigger the
    //  FLAG pin.
    case ALARM_EN: 
      retVal = xferParam(value, 8);
      break;
    // GATECFG1 controls driver transistor gate discharging and clock source monitoring
    case GATECFG1:
      retVal = xferParam(value, 16);
      break;
    // GATECFG2 controls driver dead time and blanking
    case GATECFG2:
      retVal = xferParam(value, 8);
      break;
    // CONFIG contains some assorted configuration bits and fields. A fairly comprehensive
    //  set of reasonably self-explanatory constants is provided, but users should refer
    //  to the datasheet before modifying the contents of this register to be certain they
    //  understand the implications of their modifications. Value on boot is 0x2E88; this
    //  can be a useful way to verify proper start up and operation of the dSPIN chip.
    case CONFIG: 
      retVal = xferParam(value, 16);
      break;
    // STATUS contains read-only information about the current condition of the chip. A
    //  comprehensive set of constants for masking and testing this register is provided, but
    //  users should refer to the datasheet to ensure that they fully understand each one of
    //  the bits in the register.
    case STATUS:  // STATUS is a read-only register
      retVal = xferParam(0, 16);;
      break;
    default:
      SPIXfer((byte)value);
      break;
  }
  return retVal;
}

// Generalization of the subsections of the register read/write functionality.
//  We want the end user to just write the value without worrying about length,
//  so we pass a bit length parameter from the calling function.
long powerSTEP::xferParam(unsigned long value, byte bitLen)
{
  byte byteLen = bitLen/8;      // How many BYTES do we have?
  if (bitLen%8 > 0) byteLen++;  // Make sure not to lose any partial byte values.
  
  byte temp;

  unsigned long retVal = 0; 

  for (int i = 0; i < byteLen; i++)
  {
    retVal = retVal << 8;
    temp = SPIXfer((byte)(value>>((byteLen-i-1)*8)));
    retVal |= temp;
  }

  unsigned long mask = 0xffffffff >> (32-bitLen);
  return retVal & mask;
}

byte powerSTEP::SPIXfer(byte data)
{
  byte dataPacket[_numBoards];
  int i;
  for (i=0; i < _numBoards; i++)
  {
    dataPacket[i] = 0;
  }
  dataPacket[_position] = data;
  digitalWrite(_CSPin, LOW);
  _SPI->beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));
  _SPI->transfer(dataPacket, _numBoards);
  _SPI->endTransaction();
  digitalWrite(_CSPin, HIGH);
  return dataPacket[_position];
}

