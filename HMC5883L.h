/*****************************************************************************/
//    Function:     Header file for HMC5883L
//  Hardware:    Grove - 3-Axis Digital Compass
//    Arduino IDE: Arduino-1.0
//    Author:     Frankie.Chu
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

#ifndef __HMC5883L_H__
#define __HMC5883L_H__


/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define HMC5883L_ADDRESS                    (0x1E)
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
#define HMC5883L_REG_CFG_A                  (0x00)  // Configuration Register A,    R/W
#define HMC5883L_REG_CFG_B                  (0x01)  // Configuration Register B,    R/W
#define HMC5883L_REG_MODE                   (0x02)  // Mode Register,               R/W
#define HMC5883L_REG_X_MSB                  (0x03)  // Data Output X MSB Register,  R
#define HMC5883L_REG_X_LSB                  (0x04)
#define HMC5883L_REG_Y_MSB                  (0x05)
#define HMC5883L_REG_Y_LSB                  (0x06)
#define HMC5883L_REG_Z_MSB                  (0x07)
#define HMC5883L_REG_Z_LSB                  (0x08)
#define HMC5883L_REG_STATUS                 (0x09)
#define HMC5883L_REG_ID_A                   (0x0A)
#define HMC5883L_REG_ID_B                   (0x0B)
#define HMC5883L_REG_ID_C                   (0x0C)
/*=========================================================================*/

#define ID_SUM                              (0x48 + 0x33 + 0x34)    // ID: 'H' + '3' + '4'

#define REG_A_DEFAULT                       (0b01111111)            // bit[7] is reserved for future function. 
                                                                    // set to 0 when configuring REG_A.

#define REG_B_DEFAULT                       (0b11100000)            // bit[4] - bit[0] must be cleared for correct operation.

#define REG_MODE_DEFAULT                    (0b00000011)            // bit[7] for high speed I2C, disabled.
                                                                    // bit[6] - bit[2] reserved bits
/*=========================================================================
    Used with register 0x00 (HMC5883L_REG_CFG_A) to set these features
    bit[7] reserved, set to 0.
    -----------------------------------------------------------------------*/
/* bit[6] - bit[5] */
typedef enum
{
    HMC5883L_1_AVG_SAMPLE   =                  0b00,  // default
    HMC5883L_2_AVG_SAMPLE   =                  0b01,
    HMC5883L_4_AVG_SAMPLE   =                  0b10,
    HMC5883L_8_AVG_SAMPLE   =                  0b11

}   sampleAvg_t;

/* bit[4] - bit[2] */
typedef enum 
{
    HMC5883L_DATARATE_0_75_HZ   =              0b000,
    HMC5883L_DATARATE_1_5_HZ    =              0b001,
    HMC5883L_DATARATE_3_HZ      =              0b010,
    HMC5883L_DATARATE_7_5_HZ    =              0b011,
    HMC5883L_DATARATE_15_HZ     =              0b100,  // default
    HMC5883L_DATARATE_30_HZ     =              0b101,
    HMC5883L_DATARATE_75_HZ     =              0b110

}   dataRate_t;

/* bit[1] - bit[0] */
typedef enum
{
    HMC5883L_MEASMODE_NORMAL    =              0b00,  // default
    HMC5883L_MEASMODE_P_BIAS    =              0b01,
    HMC5883L_MEASMODE_N_BIAS    =              0b10,

}   measureMode_t;
/*=========================================================================*/

/*=========================================================================
    Used with register 0x01 (HMC5883L_REG_CFG_B) to set sensor field range
    bit[4] - bit[0] must be cleared for correct operation.
    -----------------------------------------------------------------------*/
/* bit[7] - bit[5] */
typedef enum 
{
    HMC5883L_RANGE_0_88_GA       =              0b000,
    HMC5883L_RANGE_1_3_GA        =              0b001,  // default
    HMC5883L_RANGE_1_9_GA        =              0b010,
    HMC5883L_RANGE_2_5_GA        =              0b011,
    HMC5883L_RANGE_4_GA          =              0b100,
    HMC5883L_RANGE_4_7_GA        =              0b101,
    HMC5883L_RANGE_5_6_GA        =              0b110,
    HMC5883L_RANGE_8_1_GA        =              0b111

}   deviceRange_t;


/*=========================================================================
    Used with register 0x02 (HMC5883L_REG_MODE) to set operating mode
    bit[7] Set this pin to enable High Speed I2C, 3400kHz.
    -----------------------------------------------------------------------*/
/* bit[1] - bit[0] */
typedef enum 
{
    HMC5883L_OPMODE_CONTIUOUS   =               0b00,
    HMC5883L_OPMODE_SINGLE      =               0b01,  // default
    HMC5883L_OPMODE_IDLE        =               0b10,

}   operateMode_t;




struct rawData_t
{
    int16_t x;
    int16_t y;
    int16_t z;
};

struct calibData_t
{
    int16_t x;
    int16_t y;
    int16_t z;
};

struct scale_t
{
    float xp;
    float yp;
    float zp;
    float xn;
    float yn;
    float zn;
};

struct offset_t
{
    int16_t x;
    int16_t y;
    int16_t z;
};

/*
#define MEASUREMENT_CONTINUOUS 0x00
#define MEASUREMENT_SINGLE_SHOT 0x01
#define MEASUREMENT_IDLE 0x03
*/

/*
struct MagnetometerScaled
{
    float XAxis;
    float YAxis;
    float ZAxis;
};

struct MagnetometerRaw
{
    short XAxis;
    short YAxis;
    short ZAxis;
};
*/

class HMC5883L
{   
public:
    HMC5883L(void);

    bool        begin(void);
    uint8_t     getStatus(void);
    void        writeRegister(uint8_t reg, uint8_t value);
    uint8_t     readRegister(uint8_t reg);
    
    void        setRange(deviceRange_t range);
    void        setOperateMode(operateMode_t opMode);
    void        setMeasureMode(measureMode_t measMode);
    void        setSampleAvg(sampleAvg_t sample);
    void        setDataRate(dataRate_t rate);

    void        setRegCfgA(measureMode_t measMode, dataRate_t rate, sampleAvg_t sample);
    void        setRegCfgB(deviceRange_t range);
    void        setRegMode(operateMode_t opMode);

    bool        getRawData(void);
    bool        getCalibData(void);
    bool        calibrate(void);
    bool        calibScale(int8_t calibNum);
    bool        calibOffset(int16_t calibNum);

    rawData_t       rawData;
    calibData_t     calibData;
    offset_t        offset;
    scale_t         scale;

private:
    inline uint8_t      i2cread(void);
    inline void         i2cwrite(uint8_t x);
    bool                checkID(void);
    void                resetData(void);
    void                getBiasScale(float *px, float *py, float *pz, int8_t calibNum);
    void                getOffestBoundary(int16_t *pBuffer, int16_t ref, int8_t bufferSize, bool sw);
    void                printData(int16_t x, int16_t y, int16_t z);
    void                printData(float x, float y, float z);
    void                printStatus(uint8_t status);
    int16_t             sum(int16_t *pBuffer, int8_t bufferSize);
    void                bufferInitiate(int16_t *pBuffer, int16_t value, int8_t bufferSize);
    operateMode_t       _operateMode;
    deviceRange_t       _deviceRange;
    measureMode_t       _measureMode;
    dataRate_t          _dataRate;
    sampleAvg_t         _sampleAvg;

        // float m_Scale;
};

#endif
