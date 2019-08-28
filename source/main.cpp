#include "mbed.h"
#include "platform/CircularBuffer.h"
#include "ctype.h"

#include "mbed_trace.h"
#include "mbed_events.h"
#include "lora_radio_helper.h"
#include "dev_eui_helper.h"
#include "LoRaWANInterface.h"
#include "platform/Callback.h"
#include "nvstore.h"

// Macro name to string 
#define xstr(a) str(a)
#define str(a) #a

// Version information
#define MAJOR_VERSION 1
#define MINOR_VERSION 1
#define PATCH_VERSION 1

// Commands
#define SET_TX_INTERVAL           1
#define SET_UPLINK_MSGTYPE        2
#define SET_ADR_STATE             3
#define SET_DEVICE_CLASS          4
#define SET_PING_SLOT_PERIODICITY 5
#define SEND_LINK_CHECK_REQ       6
#define SEND_DEVICE_TIME_REQ      7
#define RESET_NONVOL_CMD          254 
#define SW_RESET_CMD              255 

// NVStore Keys
#define NVSTORE_TX_INTERVAL_KEY       1
#define NVSTORE_UPLINK_MSGTYPE_KEY    2
#define NVSTORE_ADR_ON_KEY            3 
#define NVSTORE_DEVICE_CLASS_KEY      4 
#define NVSTORE_PING_SLOT_PERIODICITY 5 
#define DEVICE_CLASS xstr(MBED_CONF_APP_LORA_DEVICE_CLASS)

// Network time display interval 
#define PRINT_NETWORK_TIME_INTERVAL 60000

// Transmit Interval
#define MIN_TX_INTERVAL 5

#define PING_SLOT_PERIODICITY_MAX 7
#define PING_SLOT_PERIODICITY  MBED_CONF_LORA_PING_SLOT_PERIODICITY
MBED_STATIC_ASSERT(PING_SLOT_PERIODICITY <= PING_SLOT_PERIODICITY_MAX , "Valid Ping Slot Periodicity values are 0 to 7");

static uint32_t       app_tx_interval        = MBED_CONF_APP_TX_INTERVAL;
static uint8_t        adr_on                 = MBED_CONF_LORA_ADR_ON;
static uint8_t        tx_flags               = MSG_UNCONFIRMED_FLAG;
static uint8_t        ping_slot_periodicity = MBED_CONF_LORA_PING_SLOT_PERIODICITY;
static device_class_t app_device_class      = CLASS_A;
static int            send_queued           = 0;
static bool           fastTransmit          = false;
static bool           class_b_on            = false;
static bool           ping_slot_synched     = false;
static bool           device_time_synched   = false;
static bool           beacon_found          = false;
static bool           use_builtin_deveui    = true;

// Debug RX LED
static mbed::DigitalOut dbg_rx(MBED_CONF_APP_LORA_RX_PIN); 
static RawSerial pc(USBTX, USBRX);

#define SERIAL_RX_BUF_SIZE 80
static EventQueue serial_rx_queue;
static CircularBuffer<char, SERIAL_RX_BUF_SIZE> serial_rx_buffer;
static uint8_t  serial_command[SERIAL_RX_BUF_SIZE/2];

typedef struct {
    uint16_t rx;
    uint16_t beacon_lock;
    uint16_t beacon_miss;
} app_data_frame_t;

app_data_frame_t app_data;

// Device credentials, register device as OTAA in The Things Network and copy credentials here
static uint8_t DEV_EUI[] = MBED_CONF_LORA_DEVICE_EUI;
static uint8_t APP_EUI[] = MBED_CONF_LORA_APPLICATION_EUI;
static uint8_t APP_KEY[] = MBED_CONF_LORA_APPLICATION_KEY;

static void queue_next_send_message();
static void print_received_beacon();
static void receive_command(uint8_t* buffer, int size);
static void display_command_help();
static void display_app_info();

// EventQueue is required to dispatch events around
static EventQueue ev_queue;

// Constructing Mbed LoRaWANInterface and passing it down the radio object.
static LoRaWANInterface lorawan(radio);

// Application specific callbacks
static lorawan_app_callbacks_t callbacks;

// LoRaWAN stack event handler
static void lora_event_handler(lorawan_event_t event);

// LoRaWAN LinkCheck Answer handler
static void link_check_response(uint8_t demod_margin, uint8_t gw_cnt);

// Set device class helper
static lorawan_status_t set_device_class(device_class_t device_class);

const char* get_device_class_string(device_class_t device_class)
{
    switch(device_class)
    {
        case CLASS_A:
            return "A";
        case CLASS_B:
            return "B";
        case CLASS_C:
            return "C";
        default:
            return "?";
    }
}

bool serial_rx_irq_enable = true;

bool atoh(uint8_t &hex, char high_nibble, char low_nibble)
{
    hex = 0;

    if(!isxdigit(high_nibble) || !isxdigit(low_nibble))
        return false;

    high_nibble = toupper(high_nibble);
    low_nibble  = toupper(low_nibble);

    hex = (high_nibble <= '9') ? (high_nibble - '0') * 16  :  (((high_nibble - 'A') + 10) << 4);
    hex |= (low_nibble <= '9') ? (low_nibble - '0') : (low_nibble - 'A'  + 10);

    return true;
}

void receive_serial_command()
{
    uint8_t  size = serial_rx_buffer.size();
    bool     is_valid = true;

    if(size == 1)
    {
        char c;
        serial_rx_buffer.pop(c);
        if(c == '?')
        {
            display_app_info();
            display_command_help();
        }
    }
    else
    {
        // size has to be a multiple of two
        for(uint16_t i=0; i < size && is_valid; i+=2) 
        {
            char hn, ln;

            if(serial_rx_buffer.pop(hn) && serial_rx_buffer.pop(ln))
                is_valid = atoh(serial_command[i/2], hn, ln);
            else
                is_valid = false;
        }

        if(is_valid)
            receive_command(serial_command, size/2);
    }

    if(!serial_rx_buffer.empty())
        serial_rx_buffer.reset();


    serial_rx_irq_enable = true;
}

void serial_rx_irq()
{
    if(!pc.readable())
        return;

    char c = pc.getc();

    if(!serial_rx_irq_enable)
        return;

    if(!serial_rx_buffer.full())
    {
        // Ignore LF
        if(c != '\n')
        {
            bool eol = c == '\r';

            // Carriage Return is EOL
            if(!eol)
                serial_rx_buffer.push(c);

            if(eol || serial_rx_buffer.full())
            {
                serial_rx_irq_enable = false;
                ev_queue.call(receive_serial_command);
            }
        }
    }
}


// Send a message over LoRaWAN
static void send_message()
{
    send_queued = 0;

    uint8_t tx_buffer[6];
    tx_buffer[0] = (app_data.beacon_lock >> 8) & 0xff;
    tx_buffer[1] = app_data.beacon_lock & 0xff;
    tx_buffer[2] = (app_data.beacon_miss >> 8) & 0xff;
    tx_buffer[3] = app_data.beacon_miss & 0xff;
    tx_buffer[4] = (app_data.rx >> 8) & 0xff;
    tx_buffer[5] = app_data.rx & 0xff;

    int packet_len = sizeof(tx_buffer);
    printf("Sending %d bytes\n", packet_len);

    int16_t retcode = lorawan.send(MBED_CONF_APP_LORA_UPLINK_PORT, tx_buffer, packet_len,tx_flags); 

    if (retcode < 0) {
        retcode == LORAWAN_STATUS_WOULD_BLOCK ? printf("send - duty cycle violation\n")
        : printf("send() - Error code %d\n", retcode);

        queue_next_send_message();
        return;
    }

    printf("%d bytes scheduled for transmission\n", retcode);
}

static void queue_next_send_message()
{
    int backoff;
    int txInterval = fastTransmit ? MIN_TX_INTERVAL : app_tx_interval;

    if (send_queued) {
        return;
    }

    lorawan.get_backoff_metadata(backoff);
    if(backoff < txInterval){
        backoff = txInterval*1000;
    }

    printf("Next uplink in %d seconds\r\n", backoff / 1000);
    send_queued = ev_queue.call_in(backoff, &send_message);
}


void print_network_time(){
    printf("Network Time = %llu\n",lorawan.get_current_gps_time());
}

void print_return_code(int rc, int expected_rc)
{
    printf("Return code is %d ", rc);
    if(rc == expected_rc)
        printf("(as expected).\n");
    else
        printf("(expected %d!).\n",expected_rc);
}

void restore_config()
{
    uint32_t value;
    uint16_t size;
    int rc;

    NVStore &nvstore = NVStore::get_instance();
    rc = nvstore.init();
    printf("Init NVStore. ");
    print_return_code(rc, NVSTORE_SUCCESS);

    #if 0
    // Show NVStore size, maximum number of keys and area addresses and sizes
    printf("NVStore size is %d.\n", nvstore.size());
    printf("NVStore max number of keys is %d (out of %d possible ones in this flash configuration).\n",
            nvstore.get_max_keys(), nvstore.get_max_possible_keys());

    printf("NVStore areas:\n");
    for (uint8_t area = 0; area < NVSTORE_NUM_AREAS; area++) {
        uint32_t area_address;
        size_t area_size;
        nvstore.get_area_params(area, area_address, area_size);
        printf("Area %d: address 0x%08lx, size %d (0x%x).\n", area, area_address, area_size, area_size);
    }
    #endif

    rc = nvstore.get(NVSTORE_TX_INTERVAL_KEY, sizeof(value), &value, size);
    if(rc == NVSTORE_SUCCESS)
    {
        app_tx_interval = value;
    }

    rc = nvstore.get(NVSTORE_ADR_ON_KEY, sizeof(value), &value, size);
    if(rc == NVSTORE_SUCCESS)
    {
        if(value <= 1)
            adr_on = value; 
        else
            printf("restore() - invalid ADR=%lu\n", value);
    }

    rc = nvstore.get(NVSTORE_UPLINK_MSGTYPE_KEY, sizeof(value), &value, size); 
    if(rc == NVSTORE_SUCCESS)
    {
        if(value <= 1)
            tx_flags = (value == 0) ? MSG_UNCONFIRMED_FLAG :  MSG_CONFIRMED_FLAG;
        else
            printf("restore() - invalid uplink type=%lu\n", value);
    }

    rc = nvstore.get(NVSTORE_DEVICE_CLASS_KEY, sizeof(value), &value, size); 
    if(rc == NVSTORE_SUCCESS)
    {
        if(value <= 2)
            app_device_class = static_cast<device_class_t>(value);
        else
            printf("restore() - invalid device class=%lu\n", value);
    }

    rc = nvstore.get(NVSTORE_PING_SLOT_PERIODICITY, sizeof(value), &value, size); 
    if(rc == NVSTORE_SUCCESS)
    {
        printf("restore() - ping slot periodicity=%lu\n", value);
        if(value <= PING_SLOT_PERIODICITY_MAX)
            ping_slot_periodicity = value; 
        else
            printf("restore() - invalid ping slot periodicity=%lu\n", value);
    }
}

void display_command_help()
{
    printf("\n\n");
    printf("Command                   Format\n");
    printf("------------------------- ------------------------------------\n");
    printf("Set Tx Interval            %02x + [seconds encoded in 2 bytes (eg. 0x000F = 15 seconds)]\n", SET_TX_INTERVAL);
    printf("Set Msg Type               %02x + [unconfirmed=00, confirmed=01]\n", SET_UPLINK_MSGTYPE);
    printf("Set ADR                    %02x + [on=01, off=00]\n", SET_ADR_STATE);
    printf("Set Device Class           %02x + [A=00, B=01, C=02]\n", SET_DEVICE_CLASS);
    printf("Set Ping Slot Periodicity  %02x + [00 - 07]\n", SET_PING_SLOT_PERIODICITY);
    printf("Send LinkCheckReq          %02x\n", SEND_LINK_CHECK_REQ);
    printf("Send DeviceTimeReq         %02x\n", SEND_DEVICE_TIME_REQ);
    printf("Reset Persistent Settings  %02x\n", RESET_NONVOL_CMD);
    printf("Device Reset               %02x\n", SW_RESET_CMD);

    printf("\nLoRaWAN Command FPort=%d\n", MBED_CONF_APP_LORA_CONFIG_PORT);
    printf("--------------------------------------------------------------\n\n");
}

void display_app_info()
{
    printf("\n\n");
    printf("Application            : Device Class Test\n");
    printf("LoRaWAN                : Mbed Native\n");
    printf("Region                 : %s\n",xstr(MBED_CONF_LORA_PHY));
    printf("Version                : %d.%d.%d\n", MAJOR_VERSION, MINOR_VERSION, PATCH_VERSION);
    printf("Build                  : %s %s\n\n", __DATE__, __TIME__);
    printf("DevEui%c               : %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\n",
        use_builtin_deveui?'*':' ',
        DEV_EUI[0], DEV_EUI[1], DEV_EUI[2], DEV_EUI[3], 
        DEV_EUI[4], DEV_EUI[5], DEV_EUI[6], DEV_EUI[7]);
    printf("AppEui                : %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\n",
           APP_EUI[0], APP_EUI[1], APP_EUI[2], APP_EUI[3], 
           APP_EUI[4], APP_EUI[5], APP_EUI[6], APP_EUI[7]);
    printf("AppKey                : %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\n",
           APP_KEY[0], APP_KEY[1], APP_KEY[2], APP_KEY[3], 
           APP_KEY[4], APP_KEY[5], APP_KEY[6], APP_KEY[7],
           APP_KEY[8], APP_KEY[9], APP_KEY[10], APP_KEY[11], 
           APP_KEY[12], APP_KEY[13], APP_KEY[14], APP_KEY[15]); 

    printf("Device Class          : %s\n", get_device_class_string(app_device_class));
    printf("Tx Interval           : %lu\n", app_tx_interval);
    printf("ADR                   : %u\n", adr_on);
    printf("Msg Type              : %u\n", tx_flags);
    printf("Ping Slot Periodicity : %u\n", ping_slot_periodicity);
    printf("\n\n");
}

int main()
{
    pc.baud(115200);

    // Serial Rx interrupt handler
    pc.attach(mbed::callback(serial_rx_irq), Serial::RxIrq);

    memset(&app_data, 0, sizeof(app_data));

    // Add delay for debugger connection 
    wait(3);

    // Initialize device class
    if(strcmp(DEVICE_CLASS, "A") == 0)
        app_device_class = CLASS_A;
    else if(strcmp(DEVICE_CLASS, "B") == 0)
        app_device_class = CLASS_B;
    else if(strcmp(DEVICE_CLASS, "C") == 0)
        app_device_class = CLASS_C;
    else
    {
        printf("Invalid device class=%s\n",DEVICE_CLASS);
        app_device_class = CLASS_A;
    }

    // Restore persisted configuration 
    restore_config();

    for(uint8_t i=0; i< 8; i++)
    {
        if(DEV_EUI[i] != 0)
        {
            use_builtin_deveui = false;
            break;
        }
    }

    if(use_builtin_deveui)
        get_built_in_dev_eui(DEV_EUI, sizeof(DEV_EUI));

    if (DEV_EUI[0] == 0x0 && DEV_EUI[1] == 0x0 &&
        DEV_EUI[2] == 0x0 && DEV_EUI[3] == 0x0 &&
        DEV_EUI[4] == 0x0 && DEV_EUI[5] == 0x0 &&
        DEV_EUI[6] == 0x0 && DEV_EUI[7] == 0x0) {
        printf("Set your LoRaWAN credentials first!\n");
        return -1;
    }

    display_app_info();
    display_command_help();

    // Enable trace output for this demo, so we can see what the LoRaWAN stack does
    mbed_trace_init();

    if (lorawan.initialize(&ev_queue) != LORAWAN_STATUS_OK) {
        printf("LoRa initialization failed!\n");
        return -1;
    }

    // prepare application callbacks
    callbacks.events = mbed::callback(lora_event_handler);
    callbacks.link_check_resp = mbed::callback(link_check_response);
    lorawan.add_app_callbacks(&callbacks);

    lorawan_connect_t connect_params;
    connect_params.connect_type = LORAWAN_CONNECTION_OTAA;
    connect_params.connection_u.otaa.dev_eui = DEV_EUI;
    connect_params.connection_u.otaa.app_eui = APP_EUI;
    connect_params.connection_u.otaa.app_key = APP_KEY;
    connect_params.connection_u.otaa.nwk_key = APP_KEY;
    connect_params.connection_u.otaa.nb_trials = MBED_CONF_LORA_NB_TRIALS;
    lorawan_status_t retcode = lorawan.connect(connect_params);

    if (retcode == LORAWAN_STATUS_OK ||
        retcode == LORAWAN_STATUS_CONNECT_IN_PROGRESS) {
    } else {
        printf("Connection error, code = %d\n", retcode);
        return -1;
    }

    printf("Connection - In Progress ...\r\n");


    if(app_device_class == CLASS_B){
        ev_queue.call_every(PRINT_NETWORK_TIME_INTERVAL, &print_network_time);
    }

    // make your event queue dispatching events forever
    ev_queue.dispatch_forever();

    return 0;
}

static void receive_command(uint8_t* buffer, int size)
{
    int rc;
    lorawan_status_t status;

    NVStore &nvstore = NVStore::get_instance();

    switch(buffer[0])
    {
        case SET_TX_INTERVAL:
        {
            // Check size
            if(size == 3)
            {
                app_tx_interval = (buffer[1]<<8 | buffer[2]);
                rc = nvstore.set(NVSTORE_TX_INTERVAL_KEY, sizeof(app_tx_interval), &app_tx_interval);
                printf("Set Transmit interval=%lu. ",app_tx_interval);
                print_return_code(rc, NVSTORE_SUCCESS);

                // Restart send with new interval
                if(send_queued) 
                {
                     ev_queue.cancel(send_queued);
                     send_queued = 0;
                     queue_next_send_message();
                }
            }
            break;
        }
        case SET_ADR_STATE:
        {
            if((size == 2) && (buffer[1] <= 1))
            {
                adr_on = buffer[1];
                printf("Set ADR=%u. ",adr_on);
                rc = nvstore.set(NVSTORE_ADR_ON_KEY, sizeof(adr_on), &adr_on);
                print_return_code(rc, NVSTORE_SUCCESS);
                if(adr_on)
                    status = lorawan.enable_adaptive_datarate();
                else
                    status = lorawan.disable_adaptive_datarate();

                if(status != LORAWAN_STATUS_OK)
                    printf("Configuration Error - EventCode = %d\n", status);
            }
            break;
        }
        case SET_UPLINK_MSGTYPE:
        {
            if((size == 2) && (buffer[1] <= 1))
            {
                tx_flags = (buffer[1] == 0) ? MSG_UNCONFIRMED_FLAG : MSG_CONFIRMED_FLAG;
                printf("Message type=%s. ",tx_flags == MSG_UNCONFIRMED_FLAG ?"unconfirmed":"confirmed");
                rc = nvstore.set(NVSTORE_UPLINK_MSGTYPE_KEY, 1, buffer + 1);
                print_return_code(rc, NVSTORE_SUCCESS);
            }
            break;
        }
        case SEND_DEVICE_TIME_REQ:
        {
            printf("Send device time request\n");
            status = lorawan.add_device_time_request();
            if(status == LORAWAN_STATUS_OK)
            {
                // Send now 
                if(send_queued) 
                {
                    ev_queue.cancel(send_queued);
                    send_queued = 0;
                }
                send_queued = ev_queue.call(&send_message);
            }
            else
            {
                printf("Configuration Error - EventCode = %d\n", status);
            }

            break;
        }
        case SEND_LINK_CHECK_REQ:
        {
            printf("Send link check request\n");
            status = lorawan.add_link_check_request();
            if(status == LORAWAN_STATUS_OK)
            {
                // Send now 
                if(send_queued) 
                {
                    ev_queue.cancel(send_queued);
                    send_queued = 0;
                }
                send_queued = ev_queue.call(&send_message);
            }
            else
            {
                printf("Configuration Error - EventCode = %d\n", status);
            }
            break;
        }
        case SET_DEVICE_CLASS:
        {
            if((size == 2) && (buffer[1] <= 2))
            {
                uint8_t rx_device_class = buffer[1];
                rc = nvstore.set(NVSTORE_DEVICE_CLASS_KEY, 1, &rx_device_class);
                printf("Set device class=%s. ",get_device_class_string(static_cast<device_class_t>(rx_device_class)));
                print_return_code(rc, NVSTORE_SUCCESS);
                set_device_class(static_cast<device_class_t>(rx_device_class));
            }
            break;
        }
        case SW_RESET_CMD:
        {
            printf("Software Reset\n");
            NVIC_SystemReset();
            break;
        }
        case RESET_NONVOL_CMD:
        {
            printf("Reset NVStore\n");
            nvstore.reset();
            break;

        }
        case SET_PING_SLOT_PERIODICITY:
        {
            if((size == 2) && (buffer[1] <= PING_SLOT_PERIODICITY_MAX))
            {
                ping_slot_periodicity = buffer[1];
                ping_slot_synched = false;

                status = lorawan.add_ping_slot_info_request(ping_slot_periodicity);
                if (status != LORAWAN_STATUS_OK) {
                    printf("Add ping slot info request Error - EventCode = %d", status);
                }
                else{
                    rc = nvstore.set(NVSTORE_PING_SLOT_PERIODICITY, 1, &ping_slot_periodicity); 
                    printf("Set ping slot periodicity=%u. ",ping_slot_periodicity);
                    print_return_code(rc, NVSTORE_SUCCESS);
                }
            }
            break;
        }
        default:
        {
            printf("receive_cmd() - Unknown command=%u\n",buffer[0]);
            break;
        }
    }
}

// This is called from RX_DONE, so whenever a message came in
static void receive_message()
{
    uint8_t rx_buffer[255] = { 0 };
    uint8_t port;
    int flags;

    int16_t retcode = lorawan.receive(rx_buffer, sizeof(rx_buffer), port, flags);
    if (retcode < 0) {
        printf("receive() - Error code %d\n", retcode);
        return;
    }
    app_data.rx++;

    printf("Received %d bytes on port %u\n", retcode, port);

    printf("Data received on port %d (length %d): ", port, retcode);

    for (uint8_t i = 0; i < retcode; i++) {
        printf("%02x ", rx_buffer[i]);
    }
    printf("\n");

    if((port == MBED_CONF_APP_LORA_CONFIG_PORT) && (retcode >= 1))
        receive_command(rx_buffer, retcode);

}

lorawan_status_t enable_beacon_acquisition()
{
    lorawan_status_t status = LORAWAN_STATUS_NO_OP;

    if(class_b_on == true){
        printf("enable_beacon_acquisition - Class B already enabled\n" ); 
    }
    else{ 
        beacon_found = false;
        if(device_time_synched)
        {
            status = lorawan.enable_beacon_acquisition();
            if (status != LORAWAN_STATUS_OK) {
                printf("Beacon Acquisition Error - EventCode = %d\n", status);
            } 
            fastTransmit = false;
        }
        else
        {
            // Send device time request. Beacon acquisition is optimized when device time is synched
            status = lorawan.add_device_time_request();
            if (status == LORAWAN_STATUS_OK) {
                fastTransmit = true;          
                if(send_queued) {
                    ev_queue.cancel(send_queued);
                    send_queued = 0;
                }
                send_queued = ev_queue.call(send_message);
            }
            else{
                printf("Add device time request Error - EventCode = %d", status);
            }
        }
    }
    return status;
}

void switch_to_class_b(void)
{
    lorawan_status_t status = LORAWAN_STATUS_NO_OP;

    if(app_device_class != CLASS_B) {
        printf("Switch To Device Class B: Configured device class=%s\n", get_device_class_string(app_device_class)); 
    }
    else if(!class_b_on && beacon_found && ping_slot_synched){
        status = lorawan.set_device_class(CLASS_B);
        if (status == LORAWAN_STATUS_OK) {
            class_b_on = true;
            // Send uplink now to notify server device is class B
            uint8_t dummy_value;
            lorawan.send(MBED_CONF_APP_LORA_UPLINK_PORT, &dummy_value, 1, MSG_UNCONFIRMED_FLAG);

        } else {
            printf("Switch Device Class -> B Error - EventCode = %d\n", status);
        }
    } 
}

void debug_rx_led(uint8_t count)
{
    if(dbg_rx.is_connected())
    {
        for(uint8_t i=0; i< count; i++)
        {
            dbg_rx = 1;
            wait(.1);
            dbg_rx = 0;
            wait(.1);
        }
    }
}

lorawan_status_t set_device_class(device_class_t device_class)
{
    lorawan_status_t status = LORAWAN_STATUS_NO_OP;

    switch(device_class)
    {
        case CLASS_A:
        case CLASS_C:
            status = lorawan.set_device_class(device_class);
            break;
        case CLASS_B:
            // Send ping slot configuration to the server
            if(!ping_slot_synched){
                status = lorawan.add_ping_slot_info_request(ping_slot_periodicity);
                if (status == LORAWAN_STATUS_OK) 
                {
                    fastTransmit = true;          
                    if(send_queued) 
                    {
                        ev_queue.cancel(send_queued);
                        send_queued = 0;
                    }
                    send_queued = ev_queue.call(send_message);
                }
                else{
                    printf("Add ping slot info request Error - EventCode = %d", status);
                }
            }
            else{
                // Enable beacon acquisition.
                status = enable_beacon_acquisition();
            }
            break;
    }
    if(status == LORAWAN_STATUS_OK)
        app_device_class = device_class;

    return status;
}

// Event handler
static void lora_event_handler(lorawan_event_t event)
{
    switch (event) {
        case CONNECTED:
            printf("Connection - Successful\n");
            set_device_class(app_device_class);
            send_message();
            break;
        case DISCONNECTED:
            ev_queue.break_dispatch();
            printf("Disconnected Successfully\n");
            break;
        case TX_DONE:
            printf("Message sent to Network Server\n");
            queue_next_send_message();
            break;
        case TX_TIMEOUT:
        case TX_ERROR:
        case TX_CRYPTO_ERROR:
        case TX_SCHEDULING_ERROR:
            printf("Transmission Error - EventCode = %d\n", event);
            queue_next_send_message();
            break;
        case RX_DONE:
            debug_rx_led(3);
            printf("Received Message from Network Server\n");
            receive_message();
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
            printf("Error in reception - Code = %d\n", event);
            break;
        case JOIN_FAILURE:
            printf("OTAA Failed - Check Keys\n");
            break;
        case DEVICE_TIME_SYNCHED:
            printf("Device Time received from Network Server\n");
            print_network_time();
            device_time_synched = true;
            if(app_device_class == CLASS_B)
                enable_beacon_acquisition();
            break;
        case PING_SLOT_INFO_SYNCHED:
            printf("Ping Slots = %u Synchronized with Network Server\n", 1 << (7 - PING_SLOT_PERIODICITY));
            ping_slot_synched = true;
            if(app_device_class == CLASS_B)
                enable_beacon_acquisition();
            break;
        case BEACON_NOT_FOUND:
            debug_rx_led(2);
            app_data.beacon_miss++; // This is not accurate since acquisition can span multiple beacon periods
            printf("Beacon Acquisition Failed\n");
            // Restart beacon acquisition
            if(app_device_class == CLASS_B)
                enable_beacon_acquisition();
            break;
        case BEACON_FOUND:
            debug_rx_led(1);
            beacon_found = true;
            app_data.beacon_lock++;
            printf("Beacon Acquisiton Success\n");
            print_received_beacon();
            switch_to_class_b();
            break;
        case BEACON_LOCK:
            debug_rx_led(1);
            app_data.beacon_lock++;
            print_received_beacon();
            printf("Beacon Lock Count=%u\r\n", app_data.beacon_lock);
            break;
        case BEACON_MISS:
            debug_rx_led(2);
            app_data.beacon_miss++;
            printf("Beacon Miss Count=%u\r\n", app_data.beacon_miss);
            break;
        case SWITCH_CLASS_B_TO_A:
            printf("Reverted Class B -> A\n");
            class_b_on = false;
            if(app_device_class == CLASS_B)
                enable_beacon_acquisition();
            break;
        default:
            MBED_ASSERT("Unknown Event");
    }
}

static void link_check_response(uint8_t demod_margin, uint8_t gw_cnt)
{
    printf("LinkCheckAns Margin=%u, GwCnt=%u\n",demod_margin, gw_cnt);
    lorawan.remove_link_check_request();
}

void print_received_beacon()
{
    loramac_beacon_t beacon;
    lorawan_status_t status;

    status = lorawan.get_last_rx_beacon(beacon);
    if (status != LORAWAN_STATUS_OK) {
        printf("Get Received Beacon Error - EventCode = %d\n", status);
    }

    printf("\nReceived Beacon Time=%lu, GwSpecific=", beacon.time);
    for (uint8_t i = 0; i < sizeof(beacon.gw_specific); i++) {
        printf("%02X", beacon.gw_specific[i]);
    }
    printf("\n");

}

