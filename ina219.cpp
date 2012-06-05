/******************************************************************************
* TI INA219 hi-side i2c current/power monitor Library
*
* http://www.ti.com/product/ina219
*
* 6 May 2012 by John De Cristofaro
*
*
* Only tested at standard i2c 100kbps signaling rate
*
* This library does not handle triggered conversion modes. It uses the INA219
* in continuous conversion mode. All reads are from continous conversions.
*
* MIT license
******************************************************************************/

#include "INA219.H"
#include <util/delay.h>

INA219::INA219() {
}


void INA219::begin(uint8_t addr)
{
  Wire.begin();
  i2c_address = addr;
  gain = D_GAIN;
}


// calibration of equations and device
// shunt_val 		= value of shunt in Ohms
// v_shunt_max 		= maximum value of voltage across shunt
// v_bus_max 		= maximum voltage of bus
// i_max_expected 	= maximum current draw of bus + shunt
// default values are for a 0.25 Ohm shunt on a 5V bus with max current of 1A
void INA219::calibrate(float shunt_val, float v_shunt_max, float v_bus_max, float i_max_expected)
{
  uint16_t cal;
  float i_max_possible, min_lsb, max_lsb, swap;

  r_shunt = shunt_val;

  i_max_possible = v_shunt_max / r_shunt;
  min_lsb = i_max_expected / 32767;
  max_lsb = i_max_expected / 4096;

  current_lsb = (uint16_t)(min_lsb * 100000000) + 1;
  current_lsb /= 100000000;
  swap = (0.04096)/(current_lsb*r_shunt);
  cal = (uint16_t)swap;
  power_lsb = current_lsb * 20;

#if (INA219_DEBUG == 1)
  Serial.print("v_bus_max:	"); Serial.println(v_bus_max, 8);
  Serial.print("v_shunt_max:	"); Serial.println(v_shunt_max, 8);
  Serial.print("i_max_possible:	"); Serial.println(i_max_possible, 8);
  Serial.print("i_max_expected: "); Serial.println(i_max_expected, 8);
  Serial.print("min_lsb:	"); Serial.println(min_lsb, 12);
  Serial.print("max_lsb:	"); Serial.println(max_lsb, 12);
  Serial.print("current_lsb:	"); Serial.println(current_lsb, 12);
  Serial.print("power_lsb:	"); Serial.println(power_lsb, 8);
  Serial.println("  ");
  Serial.print("cal:		"); Serial.println(cal);
  Serial.print("r_shunt:	"); Serial.println(r_shunt);
#endif

  write16(CAL_R, cal);

}


// config values (gain, bus adc, shunt adc, mode) can be derived from pp26-27 in the datasheet
// defaults are:
// range = 1 (0-32V bus voltage range)
// gain = 3 (unity gain - 320mV range)
// bus adc = 3 (12-bit, single sample, 532uS conversion time)
// shunt adc = 3 (12-bit, single sample, 532uS conversion time)
// mode = 7 (continuous conversion)
void INA219::configure(uint8_t range, uint8_t gain, uint8_t bus_adc, uint8_t shunt_adc, uint8_t mode)
{
  config = 0;

  config |= (range << BRNG | gain << PG0 | bus_adc << BADC1 | shunt_adc << SADC1 | mode);

  write16(CONFIG_R, config);		
}

// resets the INA219
void INA219::reset()
{
  write16(CONFIG_R, INA_RESET);
  _delay_ms(5);
}

// returns the raw binary value of the shunt voltage
int16_t INA219::shuntVoltageRaw()
{
  return read16(V_SHUNT_R);
}

// returns the shunt voltage in volts.
float INA219::shuntVoltage()
{
  float temp;
  temp = read16(V_SHUNT_R);
  return (temp / 100000);
}

// returns raw bus voltage binary value
int16_t INA219::busVoltageRaw()
{
  return read16(V_BUS_R);
}

// returns the bus voltage in volts
float INA219::busVoltage()
{
  int16_t temp;
  temp = read16(V_BUS_R);
  temp >>= 3;
  return (temp * 0.004);
}

// returns the shunt current in amps
float INA219::shuntCurrent()
{
  return (read16(I_SHUNT_R) * current_lsb);
}

// returns the bus power in watts
float INA219::busPower()
{
  return (read16(P_BUS_R) * power_lsb);
}


/**********************************************************************
* 			INTERNAL I2C FUNCTIONS			      *
**********************************************************************/

// writes a 16-bit word (d) to register pointer (a)
// when selecting a register pointer to read from, (d) = 0
void INA219::write16(uint8_t a, uint16_t d) {
  uint8_t temp;
  temp = (uint8_t)d;
  d >>= 8;
  Wire.beginTransmission(i2c_address); // start transmission to device 
  Wire.send(a); // sends register address to read from
  Wire.send((uint8_t)d);  // write data hibyte 
  Wire.send(temp); // write data lobyte;
  Wire.endTransmission(); // end transmission
  delay(1);
}


int16_t INA219::read16(uint8_t a) {
  uint16_t ret;

  // move the pointer to reg. of interest, null argument
  write16(a, 0);
  
  Wire.requestFrom((int)i2c_address, 2);	// request 2 data bytes
  ret = Wire.receive(); // rx hi byte
  ret <<= 8;
  ret |= Wire.receive(); // rx lo byte
  Wire.endTransmission(); // end transmission

  return ret;
}

