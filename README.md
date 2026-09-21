# LSM6DSR
Arduino library to support the LSM6DSR 3D accelerometer and 3D gyroscope

## API

This sensor uses I2C, I3C or SPI to communicate.
For I2C it is then required to create a TwoWire interface before accessing to the sensors:  

    TwoWire dev_i2c(I2C_SDA, I2C_SCL);  
    dev_i2c.begin();

For SPI it is then required to create a SPI interface before accessing to the sensors:  

    SPIClass dev_spi(SPI_MOSI, SPI_MISO, SPI_SCK);  
    dev_spi.begin();

For I3C it is then required to create an I3C interface before accessing to the sensors:

    I3C.begin(I3C_SDA, I3C_SCL, 1000000U);

An instance can be created and enabled when the I2C bus is used following the procedure below:  

    LSM6DSRSensor AccGyr(&dev_i2c);
    AccGyr.begin();
    AccGyr.Enable_X();  
    AccGyr.Enable_G();

An instance can be created and enabled when the SPI bus is used following the procedure below:  

    LSM6DSRSensor AccGyr(&dev_spi, CS_PIN);
    AccGyr.begin();	
    AccGyr.Enable_X();  
    AccGyr.Enable_G();

An instance can be created and enabled when the I3C bus is used with SETDASA (static-to-dynamic address assignment):

    LSM6DSRSensor AccGyr(&I3C, LSM6DSR_I3C_ADD_H);
    I3C.resetDynamicAddresses();
    I3C.assignDynamicAddress(AccGyr.getStaticAddress(), LSM6DSR_DYNAMIC_ADDRESS);
    AccGyr.begin(LSM6DSR_DYNAMIC_ADDRESS);
    I3C.setClock(12500000);
    AccGyr.Enable_X();
    AccGyr.Enable_G();

An instance can be created and enabled when the I3C bus is used with ENTDAA (dynamic address discovery):

    LSM6DSRSensor AccGyr(&I3C);
    I3C.begin(I3C_SDA, I3C_SCL, 1000000U);
    I3C.discover(devices, 8, &found);
    // find dynAddr by matching LSM6DSR_I3C_PID in discovered devices
    AccGyr.begin(dynAddr);
    I3C.setClock(12500000);
    AccGyr.Enable_X();
    AccGyr.Enable_G();

The access to the sensor values is done as explained below:  

  Read accelerometer and gyroscope.

    int32_t accelerometer[3];
    int32_t gyroscope[3];
    AccGyr->Get_X_Axes(accelerometer);  
    AccGyr->Get_G_Axes(gyroscope);

# Examples

There are several examples with the LSM6DSR library.
* LSM6DSR_HelloWorld_I2C: This application provides a simple example of usage of the LSM6DSR IMU 6-axis over I2C.
* LSM6DSR_HelloWorld_SPI: This application provides a simple example of usage of the LSM6DSR IMU 6-axis over SPI.
* LSM6DSR_Datalog_Terminal_I3C: This application shows how to use LSM6DSR accelerometer and gyroscope over I3C using SETDASA.
* LSM6DSR_Datalog_Terminal_I3C_ENTDAA: This application shows how to discover and use LSM6DSR dynamic address over I3C.

## Documentation

You can find the source files at  
https://github.com/stm32duino/LSM6DSR

The LSM6DSR datasheet is available at  
https://www.st.com/content/st_com/en/products/mems-and-sensors/inemo-inertial-modules/lsm6dsr.html
