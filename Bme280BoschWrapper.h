#ifndef __BMP280_BW_H__
#define __BMP280_BW_H__

#include <bme280.h>

class Bme280BoschWrapper
{
  public:
    //true: uses forced mode, sensore measures values on demand
    //false: uses continuous measuring mode
    Bme280BoschWrapper(bool forcedMode);

    bool beginI2C(uint8_t dev_addr = 0x77);
    bool beginSPI(int8_t cspin);

    //this method performs measurement
    //be sure to call it before reading values
    bool measure();

    //Temperature in degrees of Celsius * 100
    int32_t getTemperature();

    //Relative humidity in % * 1024
    uint32_t getHumidity();

    //Air pressure in Pa
    uint32_t getPressure();

    // set oversampling
    void setPressureOversampling( uint8_t osr ); 
    void setTemperatureOversampling( uint8_t osr ); 
    void setHumidityOversampling( uint8_t osr );
    // set filter coeff
    void setFilterCoeff( uint8_t fcoeff ); 

  private:
    void I2CInit();
    void SPIInit();
    int8_t setSensorSettings();


    static int8_t I2CRead(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t cnt);
    static int8_t I2CWrite(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t cnt);
    static int8_t SPIRead(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t cnt);
    static int8_t SPIWrite(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t cnt);
    static void delaymsec(uint32_t msec);

    static int _cs;

    struct bme280_dev bme280;
    struct bme280_data comp_data;

    bool forced;
    bool error = false;

    uint8_t osr_t = BME280_OVERSAMPLING_8X;
    uint8_t osr_p = BME280_OVERSAMPLING_8X;
    uint8_t osr_h = BME280_OVERSAMPLING_8X;
    uint8_t osr2macro( uint8_t osr);

    uint8_t filter_coeff = BME280_FILTER_COEFF_OFF;
};

#endif

