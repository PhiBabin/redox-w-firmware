/*
 * Either COMPILE_RIGHT or COMPILE_LEFT has to be defined from the make call to allow proper functionality
 */

#include "redox-w.h"
#include "nrf_drv_config.h"
#include "nrf_gzll.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_rtc.h"
#include "nrf51_bitfields.h"
#include "nrf51.h"


/*****************************************************************************/
/** Configuration */
/*****************************************************************************/

const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(1); /**< Declaring an instance of nrf_drv_rtc for RTC1. */


// Data and acknowledgement payloads
static uint8_t ack_payload[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH]; ///< Placeholder for received ACK payloads from Host.

// Debounce time (dependent on tick frequency)
#define DEBOUNCE 5
// Mark as inactive after a number of ticks:
#define KEY_INACTIVITY_THRESHOLD 500 // 0.5sec
#define ENC_INACTIVITY_THRESHOLD 3000 // 3sec

#ifdef COMPILE_LEFT
static uint8_t channel_table[3] = {4, 42, 77};
#endif
#ifdef COMPILE_RIGHT
static uint8_t channel_table[3] = {25, 63, 33};
#endif

// Setup switch pins with pullups
static void gpio_config(void)
{
    nrf_gpio_cfg_sense_input(R01, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);
    nrf_gpio_cfg_sense_input(R02, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);
    nrf_gpio_cfg_sense_input(R03, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);
    nrf_gpio_cfg_sense_input(R04, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);
    nrf_gpio_cfg_sense_input(R05, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);

    #ifdef ENCODER_ENABLED
    // nrf_gpio_cfg_input(ENC_A, NRF_GPIO_PIN_PULLUP);
    // nrf_gpio_cfg_input(ENC_B, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_sense_input(ENC_A, NRF_GPIO_PIN_PULLUP, nrf_gpio_pin_sense_get(ENC_A));
    nrf_gpio_cfg_sense_input(ENC_B, NRF_GPIO_PIN_PULLUP, nrf_gpio_pin_sense_get(ENC_B));
    #endif

    nrf_gpio_cfg_output(C01);
    nrf_gpio_cfg_output(C02);
    nrf_gpio_cfg_output(C03);
    nrf_gpio_cfg_output(C04);
    nrf_gpio_cfg_output(C05);
    nrf_gpio_cfg_output(C06);
    nrf_gpio_cfg_output(C07);
    

    nrf_gpio_pin_clear(C01);
    nrf_gpio_pin_clear(C02);
    nrf_gpio_pin_clear(C03);
    nrf_gpio_pin_clear(C04);
    nrf_gpio_pin_clear(C05);
    nrf_gpio_pin_clear(C06);
    nrf_gpio_pin_clear(C07);
}

// Return the key states
static void read_keys(uint8_t *row_stat)
{
    unsigned short c;
    uint32_t input = 0;
    static const uint32_t COL_PINS[] = { C01, C02, C03, C04, C05, C06, C07 };

    // scan matrix by columns
    for (c = 0; c < COLUMNS; ++c) {
        // Force the compiler to add one cycle gap between activating
        // the column pin and reading the input to allow some time for
        // it to be come stable. Note that compile optimizations are
        // not allowed for next three statements. Setting the pin
        // write a location which is marked as volatile
        // (NRF_GPIO->OUTSET) and reading the input is also a memory
        // access to a location which is marked as volatile too
        // (NRF_GPIO->IN).
        nrf_gpio_pin_set(COL_PINS[c]);
        asm volatile("nop");
        input = NRF_GPIO->IN;
        row_stat[0] = (row_stat[0] << 1) | ((input >> R01) & 1);
        row_stat[1] = (row_stat[1] << 1) | ((input >> R02) & 1);
        row_stat[2] = (row_stat[2] << 1) | ((input >> R03) & 1);
        row_stat[3] = (row_stat[3] << 1) | ((input >> R04) & 1);
        row_stat[4] = (row_stat[4] << 1) | ((input >> R05) & 1);
        nrf_gpio_pin_clear(COL_PINS[c]);
    }

}

struct Encoder
{
    uint8_t state;
    int16_t pulse;
};


static int8_t read_encoder(uint8_t* enc_state)
{
    static const int8_t encoder_LUT[] =
    {
        //       BA BA
        0,    // 00 00 => no change 
        -1,   // 00 01 => anti clockwise  
        1,    // 00 10 => clockwise
        20,    // 00 11 => invalid
        1,    // 01 00 => clockwise
        0,    // 01 01 => same
        20,    // 01 10 => invalid
        -1,   // 01 11 => anti clockwise
        -1,   // 10 00 => anti clockwise
        20,    // 10 01 => invalid
        0,    // 10 10 => same
        1,    // 10 11 => clockwise
        20,    // 11 00 => invalid
        1,    // 11 01 => clockwise
        -1,   // 11 10 => anti clockwise
        0     // 11 11 => same
    };

    bool changed = false;

    *enc_state <<= 2;
    const uint32_t input = NRF_GPIO->IN;
    *enc_state |= (((input >> ENC_A) & 1) << 0) | (((input >> ENC_B) & 1) << 1);

    const int8_t delta = encoder_LUT[*enc_state & 0xF];

     // Caught an invalid change
    if (delta == 20)
    {
        return 0;
    }
    else
    {
        return delta;
    }
}

static bool compare_keys(const uint8_t* first, const uint8_t* second,
                         uint32_t size)
{
    for(int i=0; i < size; i++)
    {
        if (first[i] != second[i])
        {
          return false;
        }
    }
    return true;
}

static bool empty_keys(const uint8_t* keys_buffer)
{
    for(int i=0; i < ROWS; i++)
    {
        if (keys_buffer[i])
        {
          return false;
        }
    }
    return true;
}

static void handle_inactivity(const uint8_t *keys_buffer, const bool has_enc_change)
{
    static uint32_t key_inactivity_ticks = 0;
    static uint32_t enc_inactivity_ticks = 0;

    // looking for 500 ticks of no keys pressed or 5000 ticks of no encoder, to go back to deep sleep
    if (empty_keys(keys_buffer)) {
        key_inactivity_ticks++;
    }
    else
    {
        key_inactivity_ticks = 0;
    }
    if (!has_enc_change)
    {
        enc_inactivity_ticks++;
    }
    else
    {
        enc_inactivity_ticks = 0;
    }
    #ifdef ENCODER_ENABLED
    if (key_inactivity_ticks > KEY_INACTIVITY_THRESHOLD && enc_inactivity_ticks > ENC_INACTIVITY_THRESHOLD) {
    #else
    if (key_inactivity_ticks > KEY_INACTIVITY_THRESHOLD) {
    #endif
        nrf_drv_rtc_disable(&rtc);
        nrf_gpio_pin_set(C01);
        nrf_gpio_pin_set(C02);
        nrf_gpio_pin_set(C03);
        nrf_gpio_pin_set(C04);
        nrf_gpio_pin_set(C05);
        nrf_gpio_pin_set(C06);
        nrf_gpio_pin_set(C07);

        // Sense if the the value changes

        #ifdef ENCODER_ENABLED
        const uint32_t input = NRF_GPIO->IN;
        nrf_gpio_cfg_sense_input(ENC_A, NRF_GPIO_PIN_PULLUP, ((input >> ENC_A) & 1) ? NRF_GPIO_PIN_SENSE_LOW : NRF_GPIO_PIN_SENSE_HIGH);
        nrf_gpio_cfg_sense_input(ENC_B, NRF_GPIO_PIN_PULLUP, ((input >> ENC_B) & 1) ? NRF_GPIO_PIN_SENSE_LOW : NRF_GPIO_PIN_SENSE_HIGH);
        #endif
        key_inactivity_ticks = 0;
        enc_inactivity_ticks = 0;

        NRF_POWER->SYSTEMOFF = 1;
    }
}

struct OutBuffer
{
    uint8_t keys_snapshot[ROWS];
    int8_t enc_delta;
} __attribute__((packed));

static void handle_send(const uint8_t* keys_buffer, const int8_t enc_delta)
{
    static struct OutBuffer out_buffer = {.keys_snapshot = {0}, .enc_delta = 0};
    static uint32_t debounce_ticks = 0;

    const bool key_no_change = compare_keys(keys_buffer, out_buffer.keys_snapshot, ROWS);
    out_buffer.enc_delta = enc_delta;
    if (out_buffer.enc_delta != 0) {
        // Assemble packet and send to receiver
        nrf_gzll_add_packet_to_tx_fifo(PIPE_NUMBER, (uint8_t*)(void *)&out_buffer, ROWS + 1);
    }
    else if (key_no_change) {
        debounce_ticks++;
        // debouncing - send only if the keys state has been stable
        // for DEBOUNCE ticks
        if (debounce_ticks == DEBOUNCE) {
            // Assemble packet and send to receiver
            nrf_gzll_add_packet_to_tx_fifo(PIPE_NUMBER, (uint8_t*)(void *)&out_buffer, ROWS + 1);
            debounce_ticks = 0;
        }
    } else {
        // change detected, start over
        debounce_ticks = 0;
        for (int k = 0; k < ROWS; k++) {
            out_buffer.keys_snapshot[k] = keys_buffer[k];
        }
    }
}

// 1000Hz debounce sampling
static void tick(nrf_drv_rtc_int_type_t int_type)
{
    uint8_t keys_buffer[ROWS] = {0, 0, 0, 0, 0};
    read_keys(keys_buffer);

#ifdef ENCODER_ENABLED
    static bool initialized = false;
    static uint8_t enc_state = 0;
    if (!initialized)
    {
        initialized = true;
        enc_state = 0;

        // On reset all information about the state is lost, therefore, we need to infer it from the config of the GPIO sense registers
        const nrf_gpio_pin_sense_t enc_a_sense = nrf_gpio_pin_sense_get(ENC_A);
        const nrf_gpio_pin_sense_t enc_b_sense = nrf_gpio_pin_sense_get(ENC_B);

        // If first reset, read the current encoder value
        if (enc_a_sense == NRF_GPIO_PIN_NOSENSE && enc_b_sense == NRF_GPIO_PIN_NOSENSE)
        {
            read_encoder(&enc_state);
        }
        else
        {
            enc_state |= (enc_a_sense == NRF_GPIO_PIN_SENSE_LOW ? 1 : 0) | ((enc_b_sense == NRF_GPIO_PIN_SENSE_LOW ? 1 : 0) << 1);
        }
    }
    const int8_t enc_delta = read_encoder(&enc_state);
#else
    const int8_t enc_delta = 0;
#endif
    handle_inactivity(keys_buffer, enc_delta != 0);

    handle_send(keys_buffer, enc_delta);
}

// Low frequency clock configuration
static void lfclk_config(void)
{
    nrf_drv_clock_init();

    nrf_drv_clock_lfclk_request(NULL);
}

// RTC peripheral configuration
static void rtc_config(void)
{
    //Initialize RTC instance
    nrf_drv_rtc_init(&rtc, NULL, tick);

    //Enable tick event & interrupt
    nrf_drv_rtc_tick_enable(&rtc, true);

    //Power on RTC instance
    nrf_drv_rtc_enable(&rtc);
}

int main()
{
    // Initialize Gazell
    nrf_gzll_init(NRF_GZLL_MODE_DEVICE);

    // Attempt sending every packet up to 100 times
    nrf_gzll_set_max_tx_attempts(100);
    nrf_gzll_set_timeslots_per_channel(4);
    nrf_gzll_set_channel_table(channel_table,3);
    nrf_gzll_set_datarate(NRF_GZLL_DATARATE_1MBIT);
    nrf_gzll_set_timeslot_period(900);

    // Addressing
    nrf_gzll_set_base_address_0(0x01020304);
    nrf_gzll_set_base_address_1(0x05060708);

    // Enable Gazell to start sending over the air
    nrf_gzll_enable();

    // Configure 32kHz xtal oscillator
    lfclk_config();

    // Configure RTC peripherals with ticks
    rtc_config();

    // Configure all keys as inputs with pullups
    gpio_config();

    // Main loop, constantly sleep, waiting for RTC and gpio IRQs
    while(1)
    {
        __SEV();
        __WFE();
        __WFE();
    }
}


/*****************************************************************************/
/** Gazell callback function definitions  */
/*****************************************************************************/

void  nrf_gzll_device_tx_success(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info)
{
    uint32_t ack_payload_length = NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH;

    if (tx_info.payload_received_in_ack)
    {
        // Pop packet and write first byte of the payload to the GPIO port.
        nrf_gzll_fetch_packet_from_rx_fifo(pipe, ack_payload, &ack_payload_length);
    }
}

// no action is taken when a packet fails to send, this might need to change
void nrf_gzll_device_tx_failed(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info)
{

}

// Callbacks not needed
void nrf_gzll_host_rx_data_ready(uint32_t pipe, nrf_gzll_host_rx_info_t rx_info)
{}
void nrf_gzll_disabled()
{}
