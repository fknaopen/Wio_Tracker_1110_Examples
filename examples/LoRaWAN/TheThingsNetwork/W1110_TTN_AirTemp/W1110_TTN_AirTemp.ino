/*
 * TRACKER1110_TTN_AirTemp.ino
 * Copyright (C) 2024 N.Fukuoka
 * MIT License
 */
// [specification]
// - location mode = Ble
// - valid send data =
//          1)Air Temperature
//          2)Air Humidity 
// - sensor measurement period = 60min
//
////////////////////////////////////////////////////////////////////////////////
// Includes

#include <Arduino.h>

#include <LbmWm1110.hpp>
#include <Lbmx.hpp>

#include <Lbm_Modem_Common.hpp>
#include <WM1110_Geolocation.hpp>
#include <WM1110_BLE.hpp>
#include <WM1110_Storage.hpp>
#include <WM1110_At_Config.hpp>
#include <Tracker_Peripheral.hpp>

// Set a execution period
static constexpr uint32_t EXECUTION_PERIOD = 1000;    // [msec.]
// Set sensor measurement period
static constexpr uint32_t SENSOR_MEASUREMENT_PERIOD = 60;    // [min.]

// Instance
static WM1110_Geolocation& wm1110_geolocation = WM1110_Geolocation::getInstance();

// Sensor measurement
uint32_t start_sensor_read_time = 0; 
uint32_t sensor_read_period = 0; 

// Packed buf & size
uint8_t sensor_data_buf[64] = {0};
uint8_t sensor_data_size = 0;

// Sensor data
float x; 
float y; 
float z;
float temperature; 
float humidity;

// Receice cmd buf & size
uint8_t cmd_data_buf[244] = {0};
uint8_t cmd_data_size = 0;

////////////////////////////////////////////////////////////////////////////////
// setup and loop
void setup()
{
    // Initializes the storage area
    wm1110_storage.begin();
    wm1110_storage.loadBootConfigParameters(); // Load all parameters (WM1110_Param_Var.h)

    delay(1000);

    wm1110_ble.begin(); // Init BLE
    wm1110_ble.setName(); // Set the  Bluetooth broadcast name

    // Set broadcast parameters
    // true: central,  false:peripheral   empty:both
    wm1110_ble.setStartParameters();

    // Start broadcast
    wm1110_ble.startAdv();

    // Initializes the detected IIC peripheral sensors(include LIS3DHTR,SHT4x,Si1151,SGP40,DPS310)    
    tracker_peripheral.begin();

    // Set the location mode
    wm1110_geolocation.begin(Track_Scan_Ble,true);

    // Initialize command parsing
    wm1110_at_config.begin();

    wm1110_geolocation.setSensorMeasurementPeriod(SENSOR_MEASUREMENT_PERIOD);
    sensor_read_period = wm1110_geolocation.getSensorMeasurementPeriod();    // Get period for reading sensor data
    sensor_read_period = sensor_read_period*60*1000;  // Convert to ms

    // Start running    
    wm1110_geolocation.run();
}

void loop()
{
    static uint32_t now_time = 0;
    static uint32_t start_scan_time = 0;   

    bool result = false;    

    // Run process 
    // sleepTime is the desired sleep time for LoRaWAN's next task (Periodic positioning is not included)    
    uint32_t sleepTime = wm1110_geolocation.lbmxProcess(); 
    uint32_t consume_time = 0;

    // Light action for Join 
    wm1110_geolocation.modemLedActionProcess();

    //if(wm1110_geolocation.time_sync_flag== true) // The device has been synchronized time from the LNS
    {
        //printf("sleepTime=%d  start_sensor_read_time=%d\r\n", sleepTime, start_sensor_read_time);

        // sensor 
        if(sleepTime > 5000) // Free time
        { 
            now_time = smtc_modem_hal_get_time_in_ms();
            if(now_time - start_sensor_read_time > sensor_read_period ||(start_sensor_read_time == 0))
            {
                printf("Reading sensor data...\r\n");
                tracker_peripheral.measureLIS3DHTRDatas(&x,&y,&z); // Get the 3-axis data
                tracker_peripheral.measureSHT4xDatas(&temperature,&humidity); // Get the Temperture & humidity data
                // Pack datas      
                tracker_peripheral.packUplinkSensorDatas();
                // Display sensor raw datas 
                tracker_peripheral.displaySensorDatas();
                // Get packed datas
                tracker_peripheral.getUplinkSensorDatas( sensor_data_buf, &sensor_data_size );

                // Insert all sensor data to lora tx buffer
                wm1110_geolocation.insertIntoTxQueue(sensor_data_buf,sensor_data_size);
                start_sensor_read_time = smtc_modem_hal_get_time_in_ms( );
                consume_time = start_sensor_read_time - now_time; 
                sleepTime = sleepTime - consume_time;

            }
        }
    }  
    if(wm1110_ble.getBleRecData(cmd_data_buf,&cmd_data_size))    // Communicate commands with APP
    {
        cmd_parse_type = 1;
        wm1110_at_config.parseCmd((char *)cmd_data_buf,cmd_data_size);
        memset(cmd_data_buf,0,cmd_data_size);
        cmd_data_size = 0;
        cmd_parse_type = 0;
    }
    //The Bluetooth connection is disconnected. Restart the device to make the configuration take effect 
    if(wm1110_ble.getBleStatus() == BleRunState::StateDisconnect)
    {
        smtc_modem_hal_reset_mcu();
    }

    delay(min(sleepTime, EXECUTION_PERIOD));
}

////////////////////////////////////////////////////////////////////////////////
