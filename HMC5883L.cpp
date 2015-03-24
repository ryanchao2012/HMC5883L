/*****************************************************************************/
//    Function:     Cpp file for HMC5883L
//  Hardware:    Grove - 3-Axis Digital Compass
//    Arduino IDE: Arduino-1.0
//    Author:     FrankieChu
//    Date:      Jan 10,2013
//    Version: v1.0
//    by www.seeedstudio.com
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//
/*******************************************************************************/

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>

#include "HMC5883L.h"

uint16_t calibRef[8] = {1370, 1090, 820, 660, 440, 390, 330, 230};
/**************************************************************************/
/*!
    @brief  Instantiates a new HMC5883L class with default setting
*/
/**************************************************************************/
HMC5883L::HMC5883L()
{
    // default setting
    _operateMode    = HMC5883L_OPMODE_CONTIUOUS;
    _deviceRange    = HMC5883L_RANGE_5_6_GA;
    _measureMode    = HMC5883L_MEASMODE_NORMAL;
    _dataRate       = HMC5883L_DATARATE_75_HZ;
    _sampleAvg      = HMC5883L_1_AVG_SAMPLE;

}


/**************************************************************************/
/*!
    @brief  Setups the HW (reads coefficients values, etc.)
*/
/**************************************************************************/
bool HMC5883L::begin()
{
    uint8_t status;

    Wire.begin();
    if(!checkID()) return false;

    setRegCfgA(_measureMode, _dataRate, _sampleAvg);
    setRegCfgB(_deviceRange);
    setRegMode(_operateMode);

    status = getStatus();
    printStatus(status);
    resetData();
    // Serial.println(calibRef[(uint8_t)_deviceRange]);
    return true;
}

void HMC5883L::setRegCfgA(measureMode_t measMode, dataRate_t rate, sampleAvg_t sample)
{
    uint8_t newData =   (measMode)
                      | (rate << 2)
                      | (sample << 5);

    newData &= REG_A_DEFAULT;
    writeRegister(HMC5883L_REG_CFG_A, newData);
}


void HMC5883L::setRegCfgB(deviceRange_t range)
{
    uint8_t newData = (range << 5);
    newData &= REG_B_DEFAULT;
    writeRegister(HMC5883L_REG_CFG_B, newData);
}


void HMC5883L::setRegMode(operateMode_t opMode)
{
    uint8_t newData = opMode;
    newData &= REG_MODE_DEFAULT;
    writeRegister(HMC5883L_REG_MODE, newData);
}


void HMC5883L::setRange(deviceRange_t range)
{
    if( _deviceRange == range) return;
    uint8_t tempData, newData;
    tempData = readRegister(HMC5883L_REG_CFG_B);
    /* Update the data */
    newData = tempData & 0b00011111;
    newData |= (range << 5);
    newData &= REG_B_DEFAULT;
    /* Write the register back to the IC */
    writeRegister(HMC5883L_REG_CFG_B, newData);
    /* Keep track of the current range (to avoid readbacks) */
    _deviceRange = range;
}


void HMC5883L::setMeasureMode(measureMode_t measMode)
{
    if(_measureMode == measMode) return;
    uint8_t tempData, newData;
    tempData = readRegister(HMC5883L_REG_CFG_A);
    /* Update the data */
    newData = tempData & 0b11111100;
    newData |= measMode;
    newData &= REG_A_DEFAULT;
    /* Write the register back to the IC */
    writeRegister(HMC5883L_REG_CFG_A, newData);
    /* Keep track of the current measurement mode (to avoid readbacks) */
    _measureMode = measMode;
}


void HMC5883L::setSampleAvg(sampleAvg_t sample)
{
    if(_sampleAvg == sample) return;
    uint8_t tempData, newData;
    tempData = readRegister(HMC5883L_REG_CFG_A);
    /* Update the data */
    newData = tempData & 0b10011111;
    newData |= (sample << 5);
    newData &= REG_A_DEFAULT;
    /* Write the register back to the IC */
    writeRegister(HMC5883L_REG_CFG_A, newData);
    /* Keep track of the number of samples averaged (to avoid readbacks) */
    _sampleAvg = sample;
}


void HMC5883L::setDataRate(dataRate_t rate)
{
    if(_dataRate == rate) return;
    uint8_t tempData, newData;
    tempData = readRegister(HMC5883L_REG_CFG_A);
    /* Update the data */
    newData = tempData & 0b11100011;
    newData |= (rate << 2);
    newData &= REG_A_DEFAULT;
    /* Write the register back to the IC */
    writeRegister(HMC5883L_REG_CFG_A, newData);
    /* Keep track of the data rate (to avoid readbacks) */
    _dataRate = rate;
}


void HMC5883L::setOperateMode(operateMode_t opMode)
{
    if(_operateMode == opMode) return;
    uint8_t tempData, newData;
    tempData = readRegister(HMC5883L_REG_MODE);
    /* Update the data */
    newData = tempData & 0b11111100;
    newData |= opMode;
    newData &= REG_MODE_DEFAULT;
    /* Write the register back to the IC */
    writeRegister(HMC5883L_REG_MODE, newData);
    /* Keep track of the current operating mode (to avoid readbacks) */
    _operateMode = opMode;
}


/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
inline void HMC5883L::i2cwrite(uint8_t x) 
{
  #if ARDUINO >= 100
  Wire.write((uint8_t)x);
  #else
  Wire.send(x);
  #endif
}


/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
inline uint8_t HMC5883L::i2cread(void) 
{
  #if ARDUINO >= 100
  return Wire.read();
  #else
  return Wire.receive();
  #endif
}


/**************************************************************************/
/*!
    @brief  Check LOCK or READY status in device
    LOCK:
        This bit is set when:
        1. some but not all for of the six data output registers have been read,
        2. Mode register has been read.
        
        When this bit is set, the six data output registers are locked 
        and any new data will not be placed in these register until 
        one of these conditions are met:
        1. all six bytes have been read, 
        2. the mode register is changed,
        3. the measurement configuration (CRA) is changed,
        4. power is reset.

    READY:
        Set when data is written to all six data registers. 
        Cleared when device initiates a write to the data output registers 
        and after one or more of the data output registers are written to. 
        When RDY bit is clear it shall remain cleared for a 250 μs. 
        DRDY pin can be used as an alternative to the status register 
        for monitoring the device for measurement data.
*/
/**************************************************************************/
uint8_t HMC5883L::getStatus(void)
{
    return readRegister(HMC5883L_REG_STATUS);  
}


/**************************************************************************/
/*!
    @brief  Writes 8-bits to the specified destination register
*/
/**************************************************************************/
void HMC5883L::writeRegister(uint8_t reg, uint8_t value) 
{
    Wire.beginTransmission(HMC5883L_ADDRESS);
    i2cwrite((uint8_t)reg);
    i2cwrite((uint8_t)(value));
    Wire.endTransmission();
}


/**************************************************************************/
/*!
    @brief  Reads 8-bits from the specified register
*/
/**************************************************************************/
uint8_t HMC5883L::readRegister(uint8_t reg) 
{
    uint8_t data;
    Wire.beginTransmission(HMC5883L_ADDRESS);
    i2cwrite(reg);
    Wire.endTransmission();
    Wire.requestFrom(HMC5883L_ADDRESS, 1);
    data = i2cread();
    Wire.endTransmission();
    return data; 
}


/**************************************************************************/
/*!
    @brief  Used to identify the device.
*/
/**************************************************************************/
bool HMC5883L::checkID(void)
{
    uint16_t _idA, _idB, _idC;
    _idA = readRegister(HMC5883L_REG_ID_A);
    _idB = readRegister(HMC5883L_REG_ID_B);
    _idC = readRegister(HMC5883L_REG_ID_C);
    return ((_idA + _idB + _idC) == ID_SUM) ? true : false;
}


bool HMC5883L::getRawData(void)
{
    uint8_t   status, discard;
    short     reqNum = 6;

    status = getStatus();

    Wire.beginTransmission(HMC5883L_ADDRESS);
    i2cwrite(HMC5883L_REG_X_MSB);
    Wire.endTransmission();
    Wire.requestFrom(HMC5883L_ADDRESS, reqNum);
    
    if(!(status & 0x1) | ((status >> 1) & 0x1))
    {
        // Serial.println("Device is busy!!");

        while(reqNum--) discard = i2cread();
        return false;
    }

    rawData.x = (int16_t)((i2cread() << 8) | i2cread());
    rawData.z = (int16_t)((i2cread() << 8) | i2cread());
    rawData.y = (int16_t)((i2cread() << 8) | i2cread());

    return true;
}


bool HMC5883L::getCalibData(void)
{
    if(!getRawData()) return false;
    int16_t x = rawData.x, y = rawData.y, z = rawData.z;
    calibData.x = (int16_t)(offset.x + x * (x >= 0 ? scale.xp : scale.xn));
    calibData.y = (int16_t)(offset.y + y * (y >= 0 ? scale.yp : scale.yn));
    calibData.z = (int16_t)(offset.z + z * (z >= 0 ? scale.zp : scale.zn));

    return true;
}


bool HMC5883L::calibrate(void)
{
    int16_t offsetNum = 400;
    int16_t scaleNum = 10;
    resetData();
    calibOffset(offsetNum);
    calibScale(scaleNum);
    return true;
}


bool HMC5883L::calibOffset(int16_t calibNum)
{
    bool _max = true, _min = false;
    int8_t  bufferSize = 8, _idx = bufferSize;
    int16_t x = rawData.x, y = rawData.y, _calibNum = (calibNum > 300 ? calibNum : 300);
    int16_t _ofstXmin[bufferSize], _ofstYmin[bufferSize], _ofstXmax[bufferSize], _ofstYmax[bufferSize];
    int16_t _sumXmin = 0, _sumXmax = 0, _sumYmin = 0, _sumYmax = 0;

    bufferInitiate(_ofstXmin, x, bufferSize);
    bufferInitiate(_ofstXmax, x, bufferSize);
    bufferInitiate(_ofstYmin, y, bufferSize);
    bufferInitiate(_ofstYmax, y, bufferSize);

    Serial.println("********** Calibration(Offset) Start **********");
    Serial.println("********** Keep the Compass Level and Rotate **********");

    while(_calibNum--)
    {
        if(!getRawData()) {  _calibNum++; continue; }

        getOffestBoundary(_ofstXmax, rawData.x, bufferSize, _max);
        getOffestBoundary(_ofstXmin, rawData.x, bufferSize, _min);
        getOffestBoundary(_ofstYmax, rawData.y, bufferSize, _max);
        getOffestBoundary(_ofstYmin, rawData.y, bufferSize, _min);

        Serial.print('.');
        if(_calibNum % 100 == 0) Serial.println();
        delay(30);
    }

    _sumXmax = sum(_ofstXmax, bufferSize);
    _sumXmin = sum(_ofstXmin, bufferSize);
    _sumYmax = sum(_ofstYmax, bufferSize);
    _sumYmin = sum(_ofstYmin, bufferSize);

    offset.x = -((_sumXmax >> 3) + (_sumXmin >> 3)) >> 1;
    offset.y = -((_sumYmax >> 3) + (_sumYmin >> 3)) >> 1;
    Serial.println("\n\rCheck result offset(x, y, z):\t");
    printData(offset.x, offset.y, 0);

    Serial.println("Calibration(Offset) Finished!");
    return true; 
}


void HMC5883L::getOffestBoundary(int16_t *pBuffer, int16_t ref, int8_t bufferSize, bool sw)
{
    int8_t _idx = bufferSize;
    
    if(sw)
    {
        while(--_idx >= 0)
        {  
            if(ref > pBuffer[_idx]) 
            { 
                pBuffer[_idx] = ref;  
                break; 
            } 
        } 
    }
    else
    {
        while(--_idx >= 0)
        {  
            if(ref < pBuffer[_idx]) 
            { 
                pBuffer[_idx] = ref;  
                break; 
            } 
        }
    }

}


bool HMC5883L::calibScale(int8_t calibNum)
{
    float _scaleXp = 1, _scaleYp = 1, _scaleZp = 1, _scaleXn = 1, _scaleYn = 1, _scaleZn = 1;

    Serial.println("********** Calibration(Scale) Start **********");

    setRegCfgA(HMC5883L_MEASMODE_P_BIAS, HMC5883L_DATARATE_75_HZ, HMC5883L_8_AVG_SAMPLE);
    delay(300);

    getBiasScale(&_scaleXp, &_scaleYp, &_scaleZp, calibNum);
    Serial.println("Check positive scale(x, y, z):\t");
    printData(_scaleXp, _scaleYp, _scaleZp);


    setRegCfgA(HMC5883L_MEASMODE_N_BIAS, HMC5883L_DATARATE_75_HZ, HMC5883L_8_AVG_SAMPLE);
    delay(300);

    getBiasScale(&_scaleXn, &_scaleYn, &_scaleZn, calibNum);  
    Serial.println("Check negative scale(x, y, z):\t");
    printData(_scaleXn, _scaleYn, _scaleZn);

    scale.xp = abs(_scaleXp);
    scale.yp = abs(_scaleYp);
    scale.zp = abs(_scaleZp);
    scale.xn = abs(_scaleXn);
    scale.yn = abs(_scaleYn);
    scale.zn = abs(_scaleZn);

    setRegCfgA(HMC5883L_MEASMODE_NORMAL, HMC5883L_DATARATE_75_HZ, HMC5883L_1_AVG_SAMPLE);
    Serial.println("Calibration(Scale) Finished!");
    return true;
}


void HMC5883L::getBiasScale(float *px, float *py, float *pz, int8_t calibNum)
{
    int8_t _calibNum = calibNum;

    while(_calibNum--)
    {
        if(!getRawData()){
            delay(20);
            _calibNum++;
            continue;
        } 
        *px += (float)rawData.x / (float)calibRef[(uint8_t)_deviceRange];
        *py += (float)rawData.y / (float)calibRef[(uint8_t)_deviceRange];
        *pz += (float)rawData.z / (float)calibRef[(uint8_t)_deviceRange];

        delay(100);
    }

    *px /= (float)calibNum;
    *py /= (float)calibNum;
    *pz /= (float)calibNum;
}


void HMC5883L::resetData(void)
{
    int8_t count = 5;

    scale.xp     = 1;
    scale.yp     = 1;
    scale.zp     = 1;
    scale.xn     = 1;
    scale.yn     = 1;
    scale.zn     = 1;

    offset.x     = 0;
    offset.y     = 0;
    offset.z     = 0;

    rawData.x    = 0;
    rawData.y    = 0;
    rawData.z    = 0;
    calibData.x  = 0;
    calibData.y  = 0;
    calibData.z  = 0;

    while(count--)
    {
        while(!getRawData());
        delay(30);
    }
}


void HMC5883L::bufferInitiate(int16_t *pBuffer, int16_t value, int8_t bufferSize)
{
    int16_t _idx = bufferSize;
    while(--_idx >= 0) pBuffer[_idx] = value;
}


int16_t HMC5883L::sum(int16_t *pBuffer, int8_t bufferSize)
{
    int16_t _idx = bufferSize;
    int16_t _sum = 0;
    while(--_idx >= 0) _sum += pBuffer[_idx];
    return _sum;  
}


void HMC5883L::printData(int16_t x, int16_t y, int16_t z)
{
    Serial.print("\t");
    Serial.print(x);
    Serial.print(" ,\t");
    Serial.print(y);
    Serial.print(" ,\t");
    Serial.println(z);
}

void HMC5883L::printData(float x, float y, float z)
{
    Serial.print("\t");
    Serial.print(x);
    Serial.print(" ,\t");
    Serial.print(y);
    Serial.print(" ,\t");
    Serial.println(z);
}


void HMC5883L::printStatus(uint8_t status)
{
    Serial.print("HMC5883L Status: \n\r LOCK Bit: ");
    Serial.print((status >> 1) & 0x1);
    Serial.print(" ,\t READY bit: ");
    Serial.println(status & 0x1);    
}
/*
HMC5883L::HMC5883L()
{
    m_Scale = 1;
}

void HMC5883L::initCompass()
{
    
    delay(5);
    
    int error = setScale(1.3);                              // Set the scale of the compass.
    
    if(error != 0)                                                  // If there is an error, print it out.
    {
        Serial.println(getErrorText(error));
    }
    
    error = setMeasurementMode(MEASUREMENT_CONTINUOUS);     // Set the measurement mode to Continuous
    
    if(error != 0)                                                  // If there is an error, print it out.
    {
        Serial.println(getErrorText(error));
    }
    
#if __Dbg
    //cout << "val_origin = " << val_origin << endl;
    //cout <<"init ok" << endl;
#endif
}


int HMC5883L::getCompass()
{
    MagnetometerRaw raw = readRawAxis();
    // Retrived the scaled values from the compass (scaled to the configured scale).
    MagnetometerScaled scaled = readScaledAxis();

    // Values are accessed like so:
    int MilliGauss_OnThe_XAxis = scaled.XAxis;// (or YAxis, or ZAxis)

    // Calculate heading when the magnetometer is level, then correct for signs of axis.
    float heading = atan2(scaled.YAxis, scaled.XAxis);

    // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
    // Find yours here: http://www.magnetic-declination.com/
    // Mine is: -2??37' which is -2.617 Degrees, or (which we need) -0.0456752665 radians, I will use -0.0457
    // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
    float declinationAngle = -0.0457;
    heading += declinationAngle;

    // Correct for when signs are reversed.
    if(heading < 0)
    heading += 2*PI;

    // Check for wrap due to addition of declination.
    if(heading > 2*PI)
    heading -= 2*PI;

    // Convert radians to degrees for readability.
    float headingDegrees = heading * 180/M_PI;

    // Output the data via the serial port.
    
    int degree = headingDegrees*10;
    
    return degree;
}



MagnetometerRaw HMC5883L::readRawAxis()
{
    uint8_t* buffer = read(DATA_REGISTER_BEGIN, 6);
    MagnetometerRaw raw = MagnetometerRaw();
    raw.XAxis = (buffer[0] << 8) | buffer[1];
    raw.ZAxis = (buffer[2] << 8) | buffer[3];
    raw.YAxis = (buffer[4] << 8) | buffer[5];
    return raw;
}

MagnetometerScaled HMC5883L::readScaledAxis()
{
    MagnetometerRaw raw = readRawAxis();
    MagnetometerScaled scaled = MagnetometerScaled();
    scaled.XAxis = raw.XAxis * m_Scale;
    scaled.ZAxis = raw.ZAxis * m_Scale;
    scaled.YAxis = raw.YAxis * m_Scale;
    return scaled;
}

short HMC5883L::setScale(float gauss)
{
    uint8_t regValue = 0x00;
    if(gauss == 0.88)
    {
        regValue = 0x00;
        m_Scale = 0.73;
    }
    else if(gauss == 1.3)
    {
        regValue = 0x01;
        m_Scale = 0.92;
    }
    else if(gauss == 1.9)
    {
        regValue = 0x02;
        m_Scale = 1.22;
    }
    else if(gauss == 2.5)
    {
        regValue = 0x03;
        m_Scale = 1.52;
    }
    else if(gauss == 4.0)
    {
        regValue = 0x04;
        m_Scale = 2.27;
    }
    else if(gauss == 4.7)
    {
        regValue = 0x05;
        m_Scale = 2.56;
    }
    else if(gauss == 5.6)
    {
        regValue = 0x06;
        m_Scale = 3.03;
    }
    else if(gauss == 8.1)
    {
        regValue = 0x07;
        m_Scale = 4.35;
    }
    else
    return ERRORCODE_1_NUM;

    // Setting is in the top 3 bits of the register.
    regValue = regValue << 5;
    write(CONFIGURATION_REGISTERB, regValue);
}

short HMC5883L::setMeasurementMode(uint8_t mode)
{
    write(MODE_REGISTER, mode);
}

void HMC5883L::write(short address, short data)
{
    Wire.beginTransmission(HMC5883L_ADDRESS);
    Wire.write(address);
    Wire.write(data);
    Wire.endTransmission();
}

uint8_t* HMC5883L::read(short address, short length)
{
    Wire.beginTransmission(HMC5883L_ADDRESS);
    Wire.write(address);
    Wire.endTransmission();

    Wire.beginTransmission(HMC5883L_ADDRESS);
    Wire.requestFrom(HMC5883L_ADDRESS, length);

    uint8_t buffer[length];
    
    if(Wire.available() == length)
    {
        for(uint8_t i = 0; i < length; i++)
        {
            buffer[i] = Wire.read();
        }
    }
    
    Wire.endTransmission();
    return buffer;
}

char* HMC5883L::getErrorText(short errorCode)
{
    if(ERRORCODE_1_NUM == 1)
    return ERRORCODE_1;

    return "Error not defined.";
}
*/