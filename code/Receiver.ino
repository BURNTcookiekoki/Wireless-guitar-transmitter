#include <Arduino.h>
#include <driver/i2s_std.h>
#include <SPI.h>
#include <RF24.h>

// --- PIN CONFIGURATIONS ---
#define I2S_DIN_IO      GPIO_NUM_4
#define I2S_WS_IO       GPIO_NUM_5
#define I2S_BCK_IO      GPIO_NUM_6

#define NRF_CE_PIN      10
#define NRF_CSN_PIN     9

// --- AUDIO CONFIGURATIONS ---
#define SAMPLE_RATE     44100
#define AUDIO_DEPTH     I2S_DATA_BIT_WIDTH_16BIT
#define BLOCK_SAMPLES   15 

// --- PACKET STRUCTURE ---
struct __attribute__((__packed__)) AudioPacket {
    uint8_t packet_id;
    int16_t audio_data[BLOCK_SAMPLES];
};

RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
i2s_chan_handle_t tx_handle = NULL;
AudioPacket rx_packet;

// Stereo distribution ring buffer block (15 samples * 2 channels)
int16_t stereo_buffer[BLOCK_SAMPLES * 2];

void init_i2s_dac() {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    i2s_new_channel(&chan_cfg, &tx_handle, NULL);

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        // Configure standard stereo slot format to drive both OUTL and OUTR physical DAC channels
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(AUDIO_DEPTH, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED, // PCM5102A operates internal clock extraction when SCK is grounded
            .bclk = I2S_BCK_IO,
            .ws = I2S_WS_IO,
            .dout = I2S_DIN_IO,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false }
        }
    };
    i2s_channel_init_std_tx(tx_handle, &std_cfg);
    i2s_channel_enable(tx_handle);
}

void init_nrf_radio() {
    SPI.begin(12, 13, 11, 9); // SCK=12, MISO=13, MOSI=11, CSN=9
    if (!radio.begin()) {
        while (1);
    }
    
    radio.setAddressWidth(3);
    radio.setDataRate(RF24_2MBPS);
    radio.setPALevel(RF24_PA_MAX);
    radio.setAutoAck(false);
    radio.setPayloadSize(sizeof(AudioPacket));
    
    uint8_t pipe_address[3] = {0xE7, 0xE7, 0xE7};
    radio.openReadingPipe(1, pipe_address);
    radio.startListening();
}

void setup() {
    init_i2s_dac();
    init_nrf_radio();
}

void loop() {
    if (radio.available()) {
        // Unpack raw payload straight from the radio transceiver's inner FIFO buffer
        radio.read(&rx_packet, sizeof(AudioPacket));
        
        // --- LIVE MONO TO STEREO MIRRORING CRITICAL CODE ---
        // Duplicates the incoming mono data array into adjacent memory positions 
        // to fill both output DAC pins simultaneously
        for (int i = 0; i < BLOCK_SAMPLES; i++) {
            stereo_buffer[i * 2]     = rx_packet.audio_data[i]; // Left Output Channel (OUTL)
            stereo_buffer[i * 2 + 1] = rx_packet.audio_data[i]; // Right Output Channel (OUTR)
        }
        
        size_t bytes_written = 0;
        size_t bytes_to_write = sizeof(stereo_buffer);
        
        // Push the interleaved stereo array out to the DAC DMA buffers instantly
        i2s_channel_write(tx_handle, (void*)stereo_buffer, bytes_to_write, &bytes_written, portMAX_DELAY);
    }
}
