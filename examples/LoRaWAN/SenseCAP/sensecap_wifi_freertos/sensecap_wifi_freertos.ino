#include <Arduino.h>

#include <LbmWm1110.hpp>
#include <Lbmx.hpp>

#include <Lbm_packet.hpp>

//taskhandle

static TaskHandle_t  LORAWAN_ENGINE_Handle;
static TaskHandle_t  LORAWAN_TX_Handle;
static TaskHandle_t  LORAWAN_RX_Handle;
static TaskHandle_t  WIFI_SCAN_Handle;

static TaskHandle_t  COLLECT_VOC_Handle;
static TaskHandle_t  COLLECT_SENSOR_Handle;
static TaskHandle_t  COLLECT_SOUND_Handle;
static TaskHandle_t  COLLECT_ULTRASONIC_Handle;

void app_ble_scan_task_wakeup( void );
void app_lora_engine_task_wakeup( void );
void app_lora_tx_task_wakeup( void );
void app_lora_rx_task_wakeup( void );

/* most important thing:
// Please follow local regulations to set lorawan duty cycle limitations
// smtc_modem_set_region_duty_cycle()
//
//
*/

////////////////////////////////////////////////////////////////////////////////
// Types

enum class StateType
{
    Startup,
    Joining,
    Joined,
    Failed,
};

////////////////////////////////////////////////////////////////////////////////
// Constants



static constexpr uint32_t TIME_SYNC_VALID_TIME = 60 * 60 * 24;  // [sec.] 

////////////////////////////////////////////////////////////////////////////////
// Variables
uint32_t position_period = 300*1000;   // [msec.]
uint32_t sensor_read_period = 180*1000;   // [msec.]

static LbmWm1110& lbmWm1110 = LbmWm1110::getInstance();
static StateType state = StateType::Startup;

uint8_t DEV_EUI[8];
uint8_t JOIN_EUI[8];
uint8_t APP_KEY[16];
static  smtc_modem_region_t REGION = SMTC_MODEM_REGION_EU_868;
////////////////////////////////////////////////////////////////////////////////
void print_current_lorawan_param(void)
{
    printf("DevEui:\r\n");
    for(uint8_t u8i = 0;u8i < 8; u8i++)
    {
        printf("%02x ",DEV_EUI[u8i]);
    }
    printf("\r\nJoinEui:\r\n");
    for(uint8_t u8i = 0;u8i < 8; u8i++)
    {
        printf("%02x ",JOIN_EUI[u8i]);
    }
    printf("\r\nAppKey:\r\n");
    for(uint8_t u8i = 0;u8i < 16; u8i++)
    {
        printf("%02x ",APP_KEY[u8i]);
    }
    printf("\r\n");    

    position_period = app_append_param.position_interval*60*1000;
    sensor_read_period = app_append_param.sample_interval*60*1000;

    printf("position_period:%umin\r\n",app_append_param.position_interval);
    printf("sensor_read_period:%umin\r\n",app_append_param.sample_interval);

}

void init_current_lorawan_param(void)
{
    memcpy(DEV_EUI, app_param.lora_info.DevEui, sizeof(DEV_EUI));
    memcpy(JOIN_EUI, app_param.lora_info.JoinEui, sizeof(JOIN_EUI));
    memcpy(APP_KEY, app_param.lora_info.AppKey, sizeof(APP_KEY));

    REGION = sensecap_lorawan_region();

    print_current_lorawan_param();    
}

// MyLbmxEventHandlers
class MyLbmxEventHandlers : public LbmxEventHandlers
{
protected:
    void reset(const LbmxEvent& event) override;
    void joined(const LbmxEvent& event) override;
    void joinFail(const LbmxEvent& event) override;
    void time(const LbmxEvent& event) override;
    void almanacUpdate(const LbmxEvent& event) override;
    void txDone(const LbmxEvent& event) override;
    void downData(const LbmxEvent& event) override;
    void wifiScanDone(const LbmxEvent& event) override;
    void wifiTerminated(const LbmxEvent& event) override;
    void wifiScanStopped(const LbmxEvent& event) override;
    void wifiScanCancelled(const LbmxEvent& event) override;
    void wifiErrorUnknown(const LbmxEvent& event) override;

};

void MyLbmxEventHandlers::reset(const LbmxEvent& event)
{
    if (LbmxEngine::setRegion(REGION) != SMTC_MODEM_RC_OK) abort();
    if (LbmxEngine::setOTAA(DEV_EUI, JOIN_EUI, APP_KEY) != SMTC_MODEM_RC_OK) abort();

    if (smtc_modem_dm_set_info_interval(SMTC_MODEM_DM_INFO_INTERVAL_IN_DAY, 1) != SMTC_MODEM_RC_OK) abort();
    {
        const uint8_t infoField = SMTC_MODEM_DM_FIELD_ALMANAC_STATUS;
        if (smtc_modem_dm_set_info_fields(&infoField, 1) != SMTC_MODEM_RC_OK) abort();
    }

    printf("Join the LoRaWAN network.\n");
    if (LbmxEngine::joinNetwork() != SMTC_MODEM_RC_OK) abort();

    // if((REGION == SMTC_MODEM_REGION_EU_868) || (REGION == SMTC_MODEM_REGION_RU_864))
    // {
    //     smtc_modem_set_region_duty_cycle( false );
    // }

    state = StateType::Joining;
}

void MyLbmxEventHandlers::joined(const LbmxEvent& event)
{
    state = StateType::Joined;

    //Configure ADR, It is necessary to set up ADR,Tx useable payload must large than 51 bytes
    app_get_profile_list_by_region(REGION,adr_custom_list_region);
    if (smtc_modem_adr_set_profile(0, SMTC_MODEM_ADR_PROFILE_CUSTOM, adr_custom_list_region) != SMTC_MODEM_RC_OK) abort();              //adr_custom_list_region  CUSTOM_ADR    

    if (smtc_modem_time_set_sync_interval_s(TIME_SYNC_VALID_TIME / 3) != SMTC_MODEM_RC_OK) abort();     // keep call order
    if (smtc_modem_time_set_sync_invalid_delay_s(TIME_SYNC_VALID_TIME) != SMTC_MODEM_RC_OK) abort();    // keep call order

    printf("Start time sync.\n");
    if (smtc_modem_time_start_sync_service(0, SMTC_MODEM_TIME_ALC_SYNC) != SMTC_MODEM_RC_OK) abort();

    app_lora_tx_task_wakeup();
}

void MyLbmxEventHandlers::joinFail(const LbmxEvent& event)
{
    state = StateType::Failed;
}

void MyLbmxEventHandlers::time(const LbmxEvent& event)
{
    if (event.event_data.time.status == SMTC_MODEM_EVENT_TIME_NOT_VALID) return;

    static bool first = true;
    if (first)
    {
        if( is_first_time_sync == false )
        {
            is_first_time_sync = true;
            app_wifi_scan_task_wakeup();
        }
        printf("time sync ok:current time:%lu\r\n",app_task_track_get_utc( ));
        // Configure transmissions
        if (smtc_modem_set_nb_trans(0, 1) != SMTC_MODEM_RC_OK) abort();
        if (smtc_modem_connection_timeout_set_thresholds(0, 0, 0) != SMTC_MODEM_RC_OK) abort();

        first = false;
    }
}


void MyLbmxEventHandlers::almanacUpdate(const LbmxEvent& event)
{
    if( event.event_data.almanac_update.status == SMTC_MODEM_EVENT_ALMANAC_UPDATE_STATUS_REQUESTED )
    {
        printf( "Almanac update is not completed: sending new request\n" );
    }
    else
    {
        printf( "Almanac update is completed\n" );
    }
}
void MyLbmxEventHandlers::txDone(const LbmxEvent& event)
{
    static uint32_t uplink_count = 0;
    if( event.event_data.txdone.status == SMTC_MODEM_EVENT_TXDONE_CONFIRMED )
    {
        app_lora_confirmed_count_increment();
    }
    uint32_t tick = smtc_modem_hal_get_time_in_ms( );
    uint32_t confirmed_count = app_lora_get_confirmed_count();
    printf( "LoRa tx done at %lu, %lu, %lu\r\n", tick, ++uplink_count, confirmed_count );    
}
void MyLbmxEventHandlers::downData(const LbmxEvent& event)
{
    uint8_t port;
    printf("Downlink received:\n");
    printf("  - LoRaWAN Fport = %d\n", event.event_data.downdata.fport);
    printf("  - Payload size  = %d\n", event.event_data.downdata.length);
    printf("  - RSSI          = %d dBm\n", event.event_data.downdata.rssi - 64);
    printf("  - SNR           = %d dB\n", event.event_data.downdata.snr / 4);

    if (event.event_data.downdata.length != 0)
    {
        port = event.event_data.downdata.fport;
        gnss_mw_handle_downlink(event.event_data.downdata.fport, event.event_data.downdata.data, event.event_data.downdata.length);
        if( port == app_lora_port )
        {
            app_lora_data_rx_size = event.event_data.downdata.length;
            memcpy( app_lora_data_rx_buffer, event.event_data.downdata.data, app_lora_data_rx_size );
            app_lora_rx_task_wakeup();
        }
    }
}
void MyLbmxEventHandlers::wifiScanDone(const LbmxEvent& event)
{
    printf("----- Wi-Fi - %s -----\n", event.getWifiEventString(WIFI_MW_EVENT_SCAN_DONE).c_str());

    wifi_mw_get_event_data_scan_done(&wifi_results);
    wifi_mw_display_results(&wifi_results);
}

void MyLbmxEventHandlers::wifiTerminated(const LbmxEvent& event)
{
    printf("----- Wi-Fi - %s -----\n", event.getWifiEventString(WIFI_MW_EVENT_TERMINATED).c_str());

    wifi_mw_event_data_terminated_t eventData;
    wifi_mw_get_event_data_terminated(&eventData);
    // printf("TERMINATED info:\n");
    // printf("-- number of scans sent: %u\n", eventData.nb_scans_sent);  if set wifi_mw_send_bypass,  eventData.nb_scans_sent = 0
}

void MyLbmxEventHandlers::wifiScanCancelled(const LbmxEvent& event)
{
    printf("----- Wi-Fi - %s -----\n", event.getWifiEventString(WIFI_MW_EVENT_TERMINATED).c_str());
}
void MyLbmxEventHandlers::wifiErrorUnknown(const LbmxEvent& event)
{
    printf("----- Wi-Fi - %s -----\n", event.getWifiEventString(WIFI_MW_EVENT_TERMINATED).c_str());
}
void MyLbmxEventHandlers::wifiScanStopped(const LbmxEvent& event)
{
    wifi_mw_custom_clear_scan_busy();
}

////////////////////////////////////////////////////////////////////////////////
// ModemEventHandler

static void ModemEventHandler()
{
    static LbmxEvent event;
    static MyLbmxEventHandlers handlers;

    while (event.fetch())
    {
        printf("----- %s -----\n", event.getEventString().c_str());
        handlers.invoke(event);
    }
}


//---------------Task---------------- ---------------
void app_wifi_scan_task_wakeup( void )
{
    xTaskNotifyGive( WIFI_SCAN_Handle );
}
void app_lora_tx_task_wakeup( void )
{
    xTaskNotifyGive( LORAWAN_TX_Handle );
}
void app_lora_rx_task_wakeup( void )
{
    xTaskNotifyGive( LORAWAN_RX_Handle );
}
void app_Lora_engine_task_wakeup( void )
{
    xTaskNotifyGive( LORAWAN_ENGINE_Handle );
}
// LoraWan_Engine_Task
void LoraWan_Engine_Task(void *parameter) {
    while (true) 
    {
        uint32_t sleepTime = LbmxEngine::doWork();
        ( void )ulTaskNotifyTake( pdTRUE, sleepTime );
    }
}

// Wifi_Scan_Task
void Wifi_Scan_Task(void *parameter) {

    while (true) 
    {
        if(is_first_time_sync == false)
        {
            ( void )ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
            xTaskNotifyGive( COLLECT_SENSOR_Handle );
        }
        while(1)
        {
            printf("start scan wifi\r\n");
            app_wifi_scan_start();
            app_Lora_engine_task_wakeup( );
            ulTaskNotifyTake(pdTRUE,4000);
            bool result = false; 
            result = app_wifi_get_results( tracker_wifi_scan_data, &tracker_wifi_scan_len );
            if( result )
            {
                app_wifi_display_results( );
            }
            app_wifi_scan_stop( );
            printf("stop scan wifi\r\n");
            //send data to LoRaWAN
            app_task_track_scan_send();
            vTaskDelay(300000);
        }
    }
}

// LoraWan_Tx_Task
void LoraWan_Tx_Task(void *parameter) {
    while (true) 
    {
        ( void )ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
        while(1)
        {
            if(app_task_lora_tx_engine() == true)
            {
                app_Lora_engine_task_wakeup(  );
            }
            vTaskDelay( 15000 );
        }
    }
}
// LoraWan_Rx_Task
void LoraWan_Rx_Task( void * pvParameter )
{   
    while( true )
    {
        ( void )ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
        app_task_packet_downlink_decode( app_lora_data_rx_buffer, app_lora_data_rx_size );
        memset(app_lora_data_rx_buffer,0,app_lora_data_rx_size);
        app_lora_data_rx_size = 0;
    }
}
// Collect_Voc_Task
void Collect_Voc_Task(void *parameter) {
    while (true) 
    {
        //get temperture&humidity for SGP internal compensation
        single_fact_sensor_data_get(sht4x_sensor_type);     
        single_fact_sensor_data_get(sgp41_sensor_type);
        vTaskDelay( 10000 );
    }
}

// Collect_Sensor_Task
void Collect_Sensor_Task(void *parameter) {
    while (true) 
    {
        if(is_first_time_sync == false)
        {
            ( void )ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
        }
        while(1)
        {
            single_fact_sensor_data_get(lis3dhtr_sensor_type);                      // 1ms
            single_fact_sensor_data_get(dps310_sensor_type);                        //219ms
            single_fact_sensor_data_get(si1151_sensor_type);                        //4ms
            factory_sensor_data_combined();
            app_sensor_data_display_results();
            //Insert all sensor data to lora tx buffer
            app_task_factory_sensor_data_send();              //30ms
            vTaskDelay( 180000 );
        }
    }
}

// Collect_Sound_Task
void Collect_Sound_Task(void *parameter) {
    while (true) 
    {
        single_fact_sensor_data_get(sound_sensor_type);                 //30ms
        vTaskDelay( 10000 );
    }
}

// Collect_Ultrasonic_Task
void Collect_Ultrasonic_Task(void *parameter) {
    while (true) 
    {
        //get temperture&humidity for SGP internal compensation
        single_fact_sensor_data_get(ultrasonic_sensor_type);                 //5ms
        if(ultrasonic_distance_cm < 10)
        {
            if(!relay_status_on())
            {
                relay_status_control(true); 
            }
        }
        else if(relay_status_on())
        {
            relay_status_control(false);     
        }
        vTaskDelay( 10000 );
    }
}


void setup() {

    default_param_load();
    init_current_lorawan_param();
    delay(1000);
    sensor_init_detect();
    track_scan_type_set(TRACKER_SCAN_WIFI);

    printf("\n---------- STARTUP ----------\n");
    // custom_lora_adr_compute(0,6,adr_custom_list_region);

    lbmWm1110.begin();

    app_wifi_scan_init();
    //Initialize wifi middleware
    wifi_mw_init(lbmWm1110.getRadio(), 0);
    wifi_mw_send_bypass(true);

    LbmxEngine::begin(lbmWm1110.getRadio(), ModemEventHandler);
    LbmxEngine::printVersions(lbmWm1110.getRadio());

    xTaskCreate(LoraWan_Engine_Task, "LoraWan_engine_Task", 256*8, NULL, 1, &LORAWAN_ENGINE_Handle);

    xTaskCreate(Wifi_Scan_Task, "Wifi_Scan_Task", 256*4, NULL, 3, &WIFI_SCAN_Handle);

    xTaskCreate(LoraWan_Tx_Task, "LoraWan_Tx_Task", 256*2, NULL, 2, &LORAWAN_TX_Handle);
    xTaskCreate(LoraWan_Rx_Task, "LoraWan_Rx_Task", 256*2, NULL, 1, &LORAWAN_RX_Handle);

    xTaskCreate(Collect_Voc_Task, "Collect_Voc_Task", 256*2, NULL, 1, &COLLECT_VOC_Handle); 

    xTaskCreate(Collect_Sensor_Task, "Collect_Sensor_Task", 256*8, NULL, 1, &COLLECT_SENSOR_Handle); 

    xTaskCreate(Collect_Sound_Task, "Collect_Sound_Task", 256*2, NULL, 1, &COLLECT_SOUND_Handle); 

    xTaskCreate(Collect_Ultrasonic_Task, "Collect_Ultrasonic_Task", 256*2, NULL, 1, &COLLECT_ULTRASONIC_Handle);

}
 
void loop() {
	
}