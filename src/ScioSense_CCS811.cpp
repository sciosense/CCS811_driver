/*
  ScioSense_CCS811.h - Library for the CCS811 digital gas sensor for monitoring indoor air quality from ams.
  2020 apr 06  v12  Christoph Friese  Changed nomenclature to ScioSense as product shifted from ams
  2019 sep 17  v11  Christoph Friese  changed I2C communication to adjust to M0 
  2019 jan 22  v10  Maarten Pennings  Added F() on all strings, added get/set_baseline()
  2019 jan 15   v9  Maarten Pennings  Flash&i2cwrite now use const array
  2018 dec 06   v8  Maarten Pennings  Added firmware flash routine
  2018 Dec 04   v7  Maarten Pennings  Added support for older CCS811's (fw 1100)
  2018 Nov 11   v6  Maarten Pennings  uint16 -> uint16_t
  2018 Nov 02   v5  Maarten Pennings  Added clearing of ERROR_ID
  2018 Oct 23   v4  Maarten Pennings  Added envdata/i2cdelay
  2018 Oct 21   v3  Maarten Pennings  Added hw-version
  2018 Oct 21   v2  Maarten Pennings  Simplified I2C, added version mngt
  2017 Dec 11   v1  Maarten Pennings  Created
*/
#include <Arduino.h>
#include <Wire.h>
#include "ScioSense_CCS811.h"

// begin() and flash() prints errors to help diagnose startup problems.
// Change these macro's to empty to suppress those prints.
#define PRINTLN(s)    Serial.println(s)
#define PRINT(s)      Serial.print(s)
#define PRINTLN2(s,m) Serial.println(s,m)
#define PRINT2(s,m)   Serial.print(s,m)

// Timings
#define CCS811_WAIT_AFTER_RESET_US     2000 // The CCS811 needs a wait after reset
#define CCS811_WAIT_AFTER_APPSTART_US  1000 // The CCS811 needs a wait after app start
#define CCS811_WAIT_AFTER_WAKE_US        50 // The CCS811 needs a wait after WAKE signal
#define CCS811_WAIT_AFTER_APPERASE_MS   500 // The CCS811 needs a wait after app erase (300ms from spec not enough)
#define CCS811_WAIT_AFTER_APPVERIFY_MS   70 // The CCS811 needs a wait after app verify
#define CCS811_WAIT_AFTER_APPDATA_MS     50 // The CCS811 needs a wait after writing app data


// Main interface =====================================================================================================


// CCS811 registers/mailboxes, all 1 byte except when stated otherwise
#define CCS811_STATUS           0x00
#define CCS811_MEAS_MODE        0x01
#define CCS811_ALG_RESULT_DATA  0x02 // up to 8 bytes
#define CCS811_RAW_DATA         0x03 // 2 bytes
#define CCS811_ENV_DATA         0x05 // 4 bytes
#define CCS811_THRESHOLDS       0x10 // 5 bytes
#define CCS811_BASELINE         0x11 // 2 bytes
#define CCS811_HW_ID            0x20
#define CCS811_HW_VERSION       0x21
#define CCS811_FW_BOOT_VERSION  0x23 // 2 bytes
#define CCS811_FW_APP_VERSION   0x24 // 2 bytes
#define CCS811_ERROR_ID         0xE0
#define CCS811_APP_ERASE        0xF1 // 4 bytes
#define CCS811_APP_DATA         0xF2 // 9 bytes
#define CCS811_APP_VERIFY       0xF3 // 0 bytes
#define CCS811_APP_START        0xF4 // 0 bytes
#define CCS811_SW_RESET         0xFF // 4 bytes


// Pin number connected to nWAKE (nWAKE can also be bound to GND, then pass -1), slave address (5A or 5B)
ScioSense_CCS811::ScioSense_CCS811(int nwake, int slaveaddr) {
  _nwake= nwake;
  _slaveaddr= slaveaddr;
  _i2cdelay_us = 50;
  wake_init();
}


// Reset the CCS811, switch to app mode and check HW_ID. Returns false on problems.
bool ScioSense_CCS811::begin( void ) {
	uint8_t sw_reset[]= {0x11,0xE5,0x72,0x8A};
	uint8_t app_start[]= {};
	uint8_t hw_id;
	uint8_t hw_version;
	uint8_t app_version[2];
	uint8_t status;
	uint8_t ok;

	_i2c_init();
	_available = false;
	
	// Wakeup CCS811
	wake_up();
	delayMicroseconds(CCS811_WAIT_AFTER_WAKE_US);

	// Invoke a SW reset (bring CCS811 in a know state)
	ok = this->write(_slaveaddr, CCS811_SW_RESET, sw_reset, 4);
	if( !(ok == 0) ) {
	  PRINTLN(F("ccs811: reset failed"));
	  goto abort_begin;
	}
	delayMicroseconds(CCS811_WAIT_AFTER_RESET_US);

	// Check that HW_ID is 0x81
	ok = this->read(_slaveaddr, CCS811_HW_ID, &hw_id, 1); 
	if( !(ok == 0) ) {
	  PRINTLN(F("ccs811: HW_ID read failed"));
	  goto abort_begin;
	}
	if( hw_id!=0x81 ) {
	  PRINT(F("ccs811: Wrong HW_ID: "));
	  PRINTLN2(hw_id,HEX);
	  goto abort_begin;
	}

	// Check that HW_VERSION is 0x1X
	ok = this->read(_slaveaddr, CCS811_HW_VERSION, &hw_version, 1);
	if( !(ok == 0) ) {
	  PRINTLN(F("ccs811: HW_VERSION read failed"));
	  goto abort_begin;
	}
	if( (hw_version&0xF0)!=0x10 ) {
	  PRINT(F("ccs811: Wrong HW_VERSION: "));
	  PRINTLN2(hw_version,HEX);
	  goto abort_begin;
	}

	// Check status (after reset, CCS811 should be in boot mode with valid app)
	ok = this->read(_slaveaddr, CCS811_STATUS, &status, 1);
	if( !(ok == 0) ) {
	  PRINTLN(F("ccs811: STATUS read (boot mode) failed"));
	  goto abort_begin;
	}
	if( status!=0x10 ) {
	  PRINT(F("ccs811: Not in boot mode, or no valid app: "));
	  PRINTLN2(status,HEX);
	  goto abort_begin;
	}

	// Read the application version
	ok = this->read(_slaveaddr, CCS811_FW_APP_VERSION, app_version, 2);
	if( !(ok == 0) ) {
	  PRINTLN(F("ccs811: APP_VERSION read failed"));
	  goto abort_begin;
	}
	_appversion= app_version[0]*256+app_version[1];

	// Switch CCS811 from boot mode into app mode
	ok = this->write(_slaveaddr, CCS811_APP_START, NULL, 0);
	if( !(ok == 0) ) {
	  PRINTLN(F("ccs811: Goto app mode failed"));
	  goto abort_begin;
	}
	delayMicroseconds(CCS811_WAIT_AFTER_APPSTART_US);

	// Check if the switch was successful
	ok = this->read(_slaveaddr, CCS811_STATUS, &status, 1);
	if( !(ok == 0) ) {
	  PRINTLN(F("ccs811: STATUS read (app mode) failed"));
	  goto abort_begin;
	}
	if( status!=0x90 ) {
	  PRINT(F("ccs811: Not in app mode, or no valid app: "));
	  PRINTLN2(status,HEX);
	  goto abort_begin;
	}

	// CCS811 back to sleep
	wake_down();
	_available = true;

	// Return success
	return _available;

abort_begin:
	// CCS811 back to sleep
	wake_down();
	// Return failure
	return _available;
}


// Switch CCS811 to `mode`, use constants CCS811_MODE_XXX. Returns false on problems.
bool ScioSense_CCS811::start( int mode ) {
  uint8_t meas_mode[]= {(uint8_t)(mode<<4)};
  wake_up();
  uint8_t ok= this->write(_slaveaddr, CCS811_MEAS_MODE, meas_mode, 2);
  wake_down();
  return (ok == 0);
}


// Get measurement results from the CCS811, check status via errstat, e.g. ccs811_errstat(errstat)
//void ScioSense_CCS811::read( uint16_t*eco2, uint16_t*etvoc, uint16_t*errstat,uint16_t*raw) {
void ScioSense_CCS811::read() {
	uint8_t    ok;
	uint8_t buf[8];
	uint8_t stat;
	wake_up();
 
	if( _appversion<0x2000 ) {
		ok = this->read(_slaveaddr, CCS811_STATUS, &stat, 1);
		if( ok && stat==CCS811_ERRSTAT_OK ) ok = this->read(_slaveaddr, CCS811_ALG_RESULT_DATA, buf, 8); else buf[5]=0;
		buf[4]= stat; // Update STATUS field with correct STATUS
    } else {
		ok = this->read(_slaveaddr, CCS811_ALG_RESULT_DATA, buf, 8);
    }
	wake_down();
	
	// Status and error management
	uint16_t combined = buf[5]*256+buf[4];
	if( combined & ~(CCS811_ERRSTAT_HWERRORS|CCS811_ERRSTAT_OK) ) ok= false; // Unused bits are 1: I2C transfer error
	combined &= CCS811_ERRSTAT_HWERRORS|CCS811_ERRSTAT_OK; // Clear all unused bits
	if( !(ok == 0) ) combined |= CCS811_ERRSTAT_I2CFAIL;
	
	// Clear ERROR_ID if flags are set
	if( combined & CCS811_ERRSTAT_HWERRORS ) {
		int err = get_errorid();
		if( err == -1 ) combined |= CCS811_ERRSTAT_I2CFAIL; // Propagate I2C error
	}
	
	// Outputs
	this->_eCO2 = buf[0]*256+buf[1];
	this->_eTVOC = buf[2]*256+buf[3];
	this->_errstat = combined;
	this->_raw = buf[6]*256+buf[7];
}


// Returns a string version of an errstat. Note, each call, this string is updated.
const char * ScioSense_CCS811::errstat_str(uint16_t errstat) {
  static char s[17]; // 16 bits plus terminating zero
  // First the ERROR_ID flags
                                                  s[ 0]='-';
                                                  s[ 1]='-';
  if( errstat & CCS811_ERRSTAT_HEATER_SUPPLY    ) s[ 2]='V'; else s[2]='v';
  if( errstat & CCS811_ERRSTAT_HEATER_FAULT     ) s[ 3]='H'; else s[3]='h';
  if( errstat & CCS811_ERRSTAT_MAX_RESISTANCE   ) s[ 4]='X'; else s[4]='x';
  if( errstat & CCS811_ERRSTAT_MEASMODE_INVALID ) s[ 5]='M'; else s[5]='m';
  if( errstat & CCS811_ERRSTAT_READ_REG_INVALID ) s[ 6]='R'; else s[6]='r';
  if( errstat & CCS811_ERRSTAT_WRITE_REG_INVALID) s[ 7]='W'; else s[7]='w';
  // Then the STATUS flags
  if( errstat & CCS811_ERRSTAT_FW_MODE          ) s[ 8]='F'; else s[8]='f';
                                                  s[ 9]='-';
                                                  s[10]='-';
  if( errstat & CCS811_ERRSTAT_APP_VALID        ) s[11]='A'; else s[11]='a';
  if( errstat & CCS811_ERRSTAT_DATA_READY       ) s[12]='D'; else s[12]='d';
                                                  s[13]='-';
  // Next bit is used by SW to signal I2C transfer error
  if( errstat & CCS811_ERRSTAT_I2CFAIL          ) s[14]='I'; else s[14]='i';
  if( errstat & CCS811_ERRSTAT_ERROR            ) s[15]='E'; else s[15]='e';
                                                  s[16]='\0';
  return s;
}


// Extra interface ========================================================================================


// Gets version of the CCS811 hardware (returns 0 on I2C failure)
int ScioSense_CCS811::hardware_version(void) {
  uint8_t buf[1];
  wake_up();
  uint8_t ok = this->read(_slaveaddr, CCS811_HW_VERSION, buf, 1);
  wake_down();
  int version= -1;
  if(ok == 0) version= buf[0];
  return version;
}


// Gets version of the CCS811 boot loader (returns 0 on I2C failure)
int ScioSense_CCS811::bootloader_version(void) {
  uint8_t buf[2];
  wake_up();
  uint8_t ok = this->read(_slaveaddr, CCS811_FW_BOOT_VERSION, buf, 2);
  wake_down();
  int version= -1;
  if(ok == 0) version= buf[0]*256+buf[1];
  return version;
}


// Gets version of the CCS811 application (returns 0 on I2C failure)
int ScioSense_CCS811::application_version(void) {
  uint8_t buf[2];
  wake_up();
  uint8_t ok = this->read(_slaveaddr, CCS811_FW_APP_VERSION, buf, 2);
  wake_down();
  int version= -1;
  if(ok == 0) version= buf[0]*256+buf[1];
  return version;
}


// Gets the ERROR_ID [same as 'err' part of 'errstat' in 'read'] (returns -1 on I2C failure)
// Note, this actually clears CCS811_ERROR_ID (hardware feature)
int ScioSense_CCS811::get_errorid(void) {
  uint8_t buf[1];
  wake_up();
  uint8_t ok = this->read(_slaveaddr, CCS811_ERROR_ID, buf, 1);
  wake_down();
  int version= -1;
  if(ok == 0) version= buf[0];
  return version;
}


#define HI(u16) ( (uint8_t)( ((u16)>>8)&0xFF ) )
#define LO(u16) ( (uint8_t)( ((u16)>>0)&0xFF ) )


// Writes t and h to ENV_DATA (see datasheet for format). Returns false on I2C problems.
bool ScioSense_CCS811::set_envdata(uint16_t t, uint16_t h) {
  uint8_t envdata[]= { HI(h), LO(h), HI(t), LO(t) };
  wake_up();
  // Serial.print(" [T="); Serial.print(t); Serial.print(" H="); Serial.print(h); Serial.println("] ");
  uint8_t ok = this->write(_slaveaddr, CCS811_ENV_DATA, envdata, 5);
  wake_down();
  return (ok == 0);
}


// Writes t and h (in ENS210 format) to ENV_DATA. Returns false on I2C problems.
bool ScioSense_CCS811::set_envdata210(uint16_t t, uint16_t h) {
  // Humidity formats of ENS210 and CCS811 are equal, we only need to map temperature.
  // The lowest and highest (raw) ENS210 temperature values the CCS811 can handle
  uint16_t lo= 15882; // (273.15-25)*64 = 15881.6 (float to int error is 0.4)
  uint16_t hi= 24073; // 65535/8+lo = 24073.875 (24074 would map to 65539, so overflow)
  // Check if ENS210 temperature is within CCS811 range, if not clip, if so map
  uint8_t ok;
  if( t<lo )      ok= set_envdata(0,h);
  else if( t>hi ) ok= set_envdata(65535,h);
  else            ok= set_envdata( (t-lo)*8+3 , h); // error in 'lo' is 0.4; times 8 is 3.2; so we correct 3
  // Returns I2C transaction status
  return (ok == 0);
}


// Reads (encoded) baseline from BASELINE (see datasheet). Returns false on I2C problems.
bool ScioSense_CCS811::get_baseline(uint16_t *baseline) {
  uint8_t buf[2];
  wake_up();
  uint8_t ok = this->read(_slaveaddr, CCS811_BASELINE, buf, 2);
  wake_down();
  *baseline= (buf[0]<<8) + buf[1];
  return (ok == 0);
}


// Writes (encoded) baseline to BASELINE (see datasheet). Returns false on I2C problems.
bool ScioSense_CCS811::set_baseline(uint16_t baseline) {
  uint8_t buf[]= { HI(baseline), LO(baseline) };
  wake_up();
  uint8_t ok= this->write(_slaveaddr, CCS811_BASELINE, buf, 3);
  wake_down();
  return (ok == 0);
}


// Flashes the firmware of the CCS811 with size bytes from image - image _must_ be in PROGMEM
bool ScioSense_CCS811::flash(const uint8_t * image, int size) {
  uint8_t sw_reset[]=   {0x11,0xE5,0x72,0x8A};
  uint8_t app_erase[]=  {0xE7,0xA7,0xE6,0x09};
  uint8_t app_verify[]= {};
  uint8_t status;
  int count;
  uint8_t ok;
  wake_up();

    // Try to ping CCS811 (can we reach CCS811 via I2C?)
    PRINT(F("ccs811: ping "));
	ok= this->write8(_slaveaddr, 0, 0);
    if( !(ok == 0) ) {
      PRINTLN(F("FAILED"));
      goto abort_begin;
    }
    PRINTLN(F("ok"));

    // Invoke a SW reset (bring CCS811 in a know state)
    PRINT(F("ccs811: reset "));
	ok= this->write(_slaveaddr, CCS811_SW_RESET, sw_reset, 5);
    if( !(ok == 0) ) {
      PRINTLN(F("FAILED"));
      goto abort_begin;
    }
    delayMicroseconds(CCS811_WAIT_AFTER_RESET_US);
    PRINTLN(F("ok"));

    // Check status (after reset, CCS811 should be in boot mode with or without valid app)
    PRINT(F("ccs811: status (reset1) "));
	ok = this->read(_slaveaddr, CCS811_STATUS, &status, 1);
    if( !(ok == 0) ) {
      PRINTLN(F("FAILED"));
      goto abort_begin;
    }
    PRINT2(status,HEX);
    PRINT(F(" "));
    if( status!=0x00 && status!=0x10 ) {
      PRINTLN(F("ERROR - ignoring")); // Seems to happens when there is no valid app
    } else {
      PRINTLN(F("ok"));
    }

    // Invoke app erase
    PRINT(F("ccs811: app-erase "));
	ok= this->write(_slaveaddr, CCS811_APP_ERASE, app_erase, 5);
    if( !(ok == 0) ) {
      PRINTLN(F("FAILED"));
      goto abort_begin;
    }
    delay(CCS811_WAIT_AFTER_APPERASE_MS);
    PRINTLN(F("ok"));

    // Check status (CCS811 should be in boot mode without valid app, with erase completed)
    PRINT(F("ccs811: status (app-erase) "));
	ok = this->read(_slaveaddr, CCS811_STATUS, &status, 1);
    if( !(ok == 0) ) {
      PRINTLN(F("FAILED"));
      goto abort_begin;
    }
    PRINT2(status,HEX);
    PRINT(F(" "));
    if( status!=0x40 ) {
      PRINTLN(F("ERROR"));
      goto abort_begin;
    }
    PRINTLN(F("ok"));

    // Write all blocks
    count= 0;
    while( size>0 ) {
        if( count%64==0 ) { PRINT(F("ccs811: writing ")); PRINT(size); PRINT(F(" ")); }
        int len= size<8 ? size : 8;
        // Copy PROGMEM to RAM
        uint8_t ram[8];
        memcpy_P(ram, image, len);
        // Send 8 bytes from RAM to CCS811
		ok = this->write(_slaveaddr, CCS811_APP_DATA, ram, len);
        if( !ok ) {
          PRINTLN(F("ccs811: app data failed"));
          goto abort_begin;
        }
        PRINT(F("."));
        delay(CCS811_WAIT_AFTER_APPDATA_MS);
        image+= len;
        size-= len;
        count++;
        if( count%64==0 ) { PRINT(F(" ")); PRINTLN(size); }
    }
    if( count%64!=0 ) { PRINT(F(" ")); PRINTLN(size); }

    // Invoke app verify
    PRINT(F("ccs811: app-verify "));
	ok = this->write8(_slaveaddr, CCS811_APP_VERIFY, app_verify[0]);
    if( !(ok == 0) ) {
      PRINTLN(F("FAILED"));
      goto abort_begin;
    }
    delay(CCS811_WAIT_AFTER_APPVERIFY_MS);
    PRINTLN(F("ok"));

    // Check status (CCS811 should be in boot mode with valid app, and erased and verified)
    PRINT(F("ccs811: status (app-verify) "));
	ok = this->read(_slaveaddr, CCS811_STATUS, &status, 1);
    if( !(ok == 0) ) {
      PRINTLN(F("FAILED"));
      goto abort_begin;
    }
    PRINT2(status,HEX);
    PRINT(F(" "));
    if( status!=0x30 ) {
      PRINTLN(F("ERROR"));
      goto abort_begin;
    }
    PRINTLN(F("ok"));

    // Invoke a second SW reset (clear flashing flags)
    PRINT(F("ccs811: reset2 "));
	ok= this->write(_slaveaddr, CCS811_SW_RESET, sw_reset, 4);
    if( !(ok == 0) ) {
      PRINTLN(F("FAILED"));
      goto abort_begin;
    }
    delayMicroseconds(CCS811_WAIT_AFTER_RESET_US);
    PRINTLN(F("ok"));

    // Check status (after reset, CCS811 should be in boot mode with valid app)
    PRINT(F("ccs811: status (reset2) "));
	ok = this->read(_slaveaddr, CCS811_STATUS, &status, 1);
    if( !(ok == 0) ) {
      PRINTLN(F("FAILED"));
      goto abort_begin;
    }
    PRINT2(status,HEX);
    PRINT(F(" "));
    if( status!=0x10 ) {
      PRINTLN(F("ERROR"));
      goto abort_begin;
    }
    PRINTLN(F("ok"));

  // CCS811 back to sleep
  wake_down();
  // Return success
  return true;

abort_begin:
  // CCS811 back to sleep
  wake_down();
  // Return failure
  return false;
}


// Advanced interface: i2cdelay ========================================================================================


// Delay before a repeated start - needed for e.g. ESP8266 because it doesn't handle I2C clock stretch correctly
void ScioSense_CCS811::set_i2cdelay(int us) {
  if( us<0 ) us= 0;
  _i2cdelay_us= us;
}


// Get current delay
int  ScioSense_CCS811::get_i2cdelay(void) {
  return _i2cdelay_us;
}


// Helper interface: nwake pin ========================================================================================


// _nwake<0 means nWAKE is not connected to a pin of the host, so no action needed
void ScioSense_CCS811::wake_init( void ) {
  if( _nwake>=0 ) pinMode(_nwake, OUTPUT);
}


void ScioSense_CCS811::wake_up( void) {
  if( _nwake>=0 ) { digitalWrite(_nwake, LOW); delayMicroseconds(CCS811_WAIT_AFTER_WAKE_US);  }
}


void ScioSense_CCS811::wake_down( void) {
  if( _nwake>=0 ) digitalWrite(_nwake, HIGH);
}


// Helper interface: i2c wrapper ======================================================================================

void ScioSense_CCS811::_i2c_init()
{
	Wire.begin();
}

uint8_t ScioSense_CCS811::read8(uint8_t addr, byte reg)
{
	uint8_t ret;
	this->read(addr, reg, &ret, 1);
	
	return ret;
}

uint8_t ScioSense_CCS811::read(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t num)
{
	uint8_t pos = 0;
	uint8_t result = 0;
	
	//on arduino we need to read in 32 byte chunks
	while(pos < num){
		
		uint8_t read_now = min((uint8_t)32, (uint8_t)(num - pos));
		Wire.beginTransmission((uint8_t)addr);
		
		Wire.write((uint8_t)reg + pos);
		result = Wire.endTransmission();
		
		Wire.requestFrom((uint8_t)addr, read_now);
		
		for(int i=0; i<read_now; i++){
			buf[pos] = Wire.read();
			pos++;
		}
	}
	return result;
}

/**************************************************************************/

uint8_t ScioSense_CCS811::write8(uint8_t addr, byte reg, byte value)
{
	uint8_t result = this->write(addr, reg, &value, 1);
	return result;
}

uint8_t ScioSense_CCS811::write(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t num)
{

	Wire.beginTransmission((uint8_t)addr);
	Wire.write((uint8_t)reg);
	Wire.write((uint8_t *)buf, num);
	uint8_t result = Wire.endTransmission();
	return result;
}

/**************************************************************************/
