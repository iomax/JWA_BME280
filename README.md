# BME280_Bosch_Wrapper

## News 24-12-2019
Added fuctions to set normal mode standby time ( default 500ms )
```
	setStandbyTime( BME280_STANDBY_TIME[ms] )

	BME280_STANDBY_TIME_0_5_MS
	BME280_STANDBY_TIME_62_5_MS
	BME280_STANDBY_TIME_125_MS
	BME280_STANDBY_TIME_250_MS
	BME280_STANDBY_TIME_500_MS
	BME280_STANDBY_TIME_1000_MS
	BME280_STANDBY_TIME_10_MS
	BME280_STANDBY_TIME_20_MS
```

## News 22-12-2019
Updated BME280 driver to version 3.3.7.

Added fuctions to set the oversampling value ( default 8x )
```
	setTemperatureOversampling( <0,1,2,4,8,16> )
	setPressureOversampling( <0,1,2,4,8,16> )
	setHumidityOversampling( <0,1,2,4,8,16> )
```
Added function to set the filter coefficent ( default 0 )
```
	setFilterCoeff( <0,2,4,8,16> )
```
Replaced fixed measure delay (255ms) in line with the spec at bme280 datasheet ( *Appendix B: Measurement time and current calculation* )

```
𝑡𝑚𝑒𝑎𝑠𝑢𝑟𝑒,𝑚𝑎𝑥 = 1.25 + [2.3 ⋅ 𝑇_𝑜𝑣𝑒𝑟𝑠𝑎𝑚𝑝𝑙𝑖𝑛𝑔]𝑜𝑠𝑟𝑠_𝑡≠0 + [2.3 ⋅ 𝑃_𝑜𝑣𝑒𝑟𝑠𝑎𝑚𝑝𝑙𝑖𝑛𝑔 + 0.575]𝑜𝑠𝑟𝑠_𝑝≠0 + [2.3 ⋅ 𝐻_𝑜𝑣𝑒𝑟𝑠𝑎𝑚𝑝𝑙𝑖𝑛𝑔 + 0.575]𝑜𝑠𝑟𝑠_h≠0
```

## About
This is Arduino library for BME280 environmental sensor. This library is based on reference code released by Bosch Sensortec.

BME280 is very small sensor for measuring air temperature, relative humidity and pressure. It features I2C and SPI connection. https://www.bosch-sensortec.com/bst/products/all_products/bme280

Bosch driver repository: https://github.com/BoschSensortec/BME280_driver

## Function
The original Bosch driver is included in this package and it has not been modified in any way. It only supplied wrapper code which enables compilation with Arduino IDE and provides implementation of Arduino I2C and SPI for the driver.

## Copyright
Files bme280.c, bme280.h and bme280_defs.h are Copyright (c) 2013 - 2017 Bosch Sensortec GmbH

The original [JWA_BME280](https://github.com/janw-cz/JWA_BME280) implementation written by [Jan Wasserbauer](https://github.com/janw-cz) and are licensed under the terms of GNU GPL v3. 

