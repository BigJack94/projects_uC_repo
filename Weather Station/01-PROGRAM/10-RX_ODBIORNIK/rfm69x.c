
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>

#include "rfm69.h"

uint8_t addr_filter_bytes;    // 1 or 2 depending on the address filter

volatile uint8_t RX_flag;    // flag for RFM69_RX_EVENT


static void (*rfm69_rx_callback)(uint8_t * data, uint8_t len );

// function which register callback for user function
void rfm69_rx_event_callback( void (*callback)(uint8_t * data, uint8_t len ) )
{
    rfm69_rx_callback = callback;
}

//--------- main RFM_RX event -------------------------
void RFM69_RX_EVENT( uint8_t * rfm69_buf ) {
    if( RX_flag ) {
        uint8_t len = rfm69_receive( rfm69_buf );
        if( rfm69_rx_callback ) {
            (*rfm69_rx_callback)( rfm69_buf, len );
        } else rfm69_fifo_clear();
        RX_flag=0;
    }
#if USE_IRQ == 0    // if we don't use interrupts
    if( rfm69_receiving() ) RX_flag=1;    // we check rx data in pooling
#endif
}



#if USE_IRQ == 1

ISR( INTVECT(IRQ_NR) ) {        // INT0, INT1 or INT2

    uint8_t flags2 = rfm69_status();    // we check only flags2 - lower byte

    if( flags2 & 1<<2 ) {
        GIFR |= (1<<INT(IRQ_NR));        // clear interrupt flag
        RX_flag=1;                        // set rx_flag for RX_EVENT
    }
}

#endif


static inline uint8_t rfm69_spi_transfer( uint8_t data ) {

    for (uint8_t i = 8; i; i--) {
        if (data & 0x80) MOSI_PORT |= MOSI;
        else MOSI_PORT &= ~MOSI;

        data <<= 1;
        SCK_PORT |= SCK;

        if (MISO_PIN & MISO) data |= 1;
        else data |= 0;

        SCK_PORT &= ~SCK;
    }

    return data;
}

void rfm69_fifo_clear( void ) {
    rfm69_cmd(0x2810, 1);
}

uint8_t rfm69_cmd( uint16_t cmd, uint8_t write ) {

    // Split command in two bytes, merge with wnr bit write-/read-flag
    uint8_t hbyte = ((cmd >> 8) | (write ? 128 : 0));
    uint8_t lbyte = (write ? (cmd & 0x00FF) : 0xFF);

    // enable RFM
    SCK_PORT &= ~SCK;
    MOSI_PORT &= ~MOSI;
    RFM_ACTIVATE;

    // spi transfer
    rfm69_spi_transfer( hbyte );
    lbyte = rfm69_spi_transfer( lbyte );

    // disable RFM
    MOSI_PORT &= ~MOSI;
    SCK_PORT &= ~SCK;
    RFM_DEACTIVATE;

    return lbyte;
}


uint8_t rfm69_fifo_transfer( uint8_t *data, uint8_t write ) {
    register uint8_t tmp;

    RFM_ACTIVATE;
    // Address FIFO-Register in write- or read-mode
    rfm69_spi_transfer(write ? 128 : 0);

    // Write data length or read data length depending on mode
    tmp = rfm69_spi_transfer(write ? data[0] : 0xFF);
    if( !write ) data[0] = tmp;

    // Make sure there's no array-overflow
    if (data[0] > MAX_RFM69_BUF) data[0] = MAX_RFM69_BUF;

    // Write/read data bytes
    for (uint8_t i = 1; i <= data[0]; i++) {
        tmp = rfm69_spi_transfer(write ? data[i] : 0xFF);
        if (!write) data[i] = tmp;
    }

    RFM_DEACTIVATE;

    return 0;
}

void rfm69_SetBitrate( uint32_t bitrate ) {
    uint8_t bw;
    uint16_t freqdev;

    switch (bitrate) {
    case 38400:
    case 57600:
        bw = 1;
        freqdev = 1475;
        break;
    case 115200:
        bw = 8;
        freqdev = 1966;
        break;
    default:
        bw = 2;
        freqdev = 737;
        break;
    }
    //Frequency Deviation
    rfm69_cmd(0x0500 + (freqdev >> 8), 1);
    rfm69_cmd(0x0600 + (freqdev & 0xFF), 1);

    //Data Rate
    rfm69_cmd(0x0300 | DATARATE_MSB, 1);
    rfm69_cmd(0x0400 | DATARATE_LSB, 1);

    //Receiver Bandwidth
    rfm69_cmd(0x1980 | bw, 1);
}


void rfm69_SetModeAndModulation( uint8_t datamode, uint8_t mod_type, uint8_t mod_shape ) {
    rfm69_cmd(0x0200 | (datamode<<5) | (mod_type<<3) | (mod_shape<<0), 1); // FSK/OOK, Packet mode, shaping
}

//------- set Over current protection in range 50 mA up to 120 mA RFM69(CW)    (default 95 mA)
void rfm69_SetOverCurrent( uint8_t enable, uint8_t mA ) {
    uint8_t ocp;
    if( mA<46 ) ocp = 0x00;
    else if( mA>120 ) ocp = 0x0f;
    else ocp = (mA-45)+(5/2)/5;

    rfm69_cmd(0x1300 | (enable<<4) | ocp, 1);
}

//------- set Power Out in range -18 dBm up to +13 dbm RFM69(CW)
void rfm69_SetPowerOut( int8_t dBm ) {
    uint8_t pwr = ( ( dBm + 18 )*(dBm>-19)*(dBm<14) + 31*(dBm>18) );
    rfm69_cmd( 0x1180 | pwr, 1 );
}

void rfm69_PacketConfig( uint8_t packet_len, uint8_t dc_encoding, uint8_t addr_filter ) {
    rfm69_cmd(0x3700 | packet_len<<7 | dc_encoding<<5 | 1<<4 | addr_filter<<1, 1);
}

void rfm69_SetNodeAddress( uint8_t node_addr ) {
    rfm69_cmd(0x3900 | node_addr , 1);
}

void rfm69_SetBroadcastAddress( uint8_t broadcast_addr ) {
    rfm69_cmd(0x3A00 | broadcast_addr , 1);
}


void set_net_id( const char * network_id ) {

    uint8_t netid_len = ( (network_id) ? strlen_P(network_id) : 0 );
    if( netid_len>8 ) netid_len=8;

    // Sync-Mode
    if( !network_id ) {
        rfm69_cmd(0x2E88, 1);                     // set FIFO mode
        rfm69_cmd(0x2F2D, 1);                     // sync word MSB
        rfm69_cmd(0x30D4, 1);                     // sync word LSB
    } else {
        uint8_t i;
        uint8_t base = 0x2F;
        rfm69_cmd(0x2E80 + ( (netid_len-1)<<3), 1);                     // set FIFO mode
        for(i=0; i<netid_len; i++) {
            rfm69_cmd( (base<<8) | pgm_read_byte( &network_id[i] )   , 1);
            base++;
        }
    }
}


void rfm69_init( uint32_t freq, const char * network_id, uint8_t address_filter, uint8_t node_adr, uint8_t broadcast_adr ) {
    uint32_t time_out = TIME_OUT;

    //------------------ init IRQ if used -------------------
#if USE_IRQ == 1

#if IRQ_NR == 0
    MCUCR |= (1<<ISC01)|(1<<ISC00);
#endif

#if IRQ_NR == 1
    MCUCR |= (1<<ISC11)|(1<<ISC10);
#endif

#if IRQ_NR == 2
    MCUCSR |= (1<<ISC2);
#endif

    // pullup IRQ pin
    IRQ_PORT |= IRQ_PIN;
    // enable INT0, INT1 or INT2
    GICR |= (1<<INT(IRQ_NR));

#endif



    // if address filtering OFF - 1 byte (length), if ON - 2 bytes (length and address)
    addr_filter_bytes = (!address_filter)?1:2;


    // Configure SPI inputs and outputs
    NSEL_PORT    |= NSEL;
    MISO_PORT    |= MISO;

    MISO_DIR     &= ~MISO;
    MOSI_DIR     |= MOSI;
    SCK_DIR     |= SCK;
    NSEL_DIR    |= NSEL;

    _delay_ms(10);        // Power-on-Reset delay before any config data is send i.e 10ms

    rfm69_fifo_clear();

        rfm69_SetModeAndModulation( PACKET_MODE, MODULATION_FSK, MOD_SHAPING_OFF );

        //Bitrate + corresponding settings (Receiver bandwidth, frequency deviation)
        rfm69_SetBitrate( BITRATE );

        // set overcurrent in range from 50 ma up to 120 mA
        rfm69_SetOverCurrent( 1, 100 );            // Overload Enabled, Over current protection = 100 mA (default 95 mA)

        rfm69_SetPowerOut( 13 );                // Set Output Power +13 dBm ( range [-18...13] dBm )

        // Carrier frequency
        rfm69_cmd(0x0700 + (uint8_t)(freq>>16), 1);
        rfm69_cmd(0x0800 + (uint8_t)(freq>>8), 1);
        rfm69_cmd(0x0900 + (uint8_t)(freq>>0), 1);

        // Packet config
        // Variable length, DC-free encoding/decoding ,  Address filter
        rfm69_PacketConfig( PACKET_VARIALBLE_LEN, PACKET_DC_FILTER_MAN, address_filter );
        //rfm69_cmd(0x3800 + MAX_RFM69_BUF, 1);     // Max. Payload-Length
        // here we set node and broadcast address if needed
        rfm69_SetNodeAddress( node_adr );            // Node address, range 0-255
        rfm69_SetBroadcastAddress( broadcast_adr );        // Broadcast address, range 0-255

        rfm69_cmd(0x3C80, 1);                     // Tx-Start-Condition: FIFO not empty
        rfm69_cmd(0x3D12, 1);                     // Packet-Config2

        // Preambel length 3 bytes
        rfm69_cmd(0x2C00, 1);
        rfm69_cmd(0x2D03, 1);


        set_net_id( network_id );

        // Receiver config
        rfm69_cmd(0x1800, 1);                     // LNA: 50 Ohm Input Impedance, Automatic Gain Control
        rfm69_cmd(0x582D, 1);                     // High sensitivity mode
        rfm69_cmd(0x6F30, 1);                     // Improved DAGC
        rfm69_cmd(0x29DC, 1);                     // RSSI mind. -110 dBm
        rfm69_cmd(0x1E2D, 1);                     // Start AFC, Auto-On


        // je li 868 MHz i wi cej
        if( freq>10000000 ) {

            // BR=9.6K
            rfm69_cmd(0x030D, 1);
            rfm69_cmd(0x0405, 1);

//            // BR=4.8K
//            rfm69_cmd(0x031A, 1);
//            rfm69_cmd(0x040B, 1);

            rfm69_cmd(0x0502, 1);
            rfm69_cmd(0x0641, 1);

            rfm69_cmd(0x130F, 1);
            rfm69_cmd(0x1952, 1);

            //---------------------------------------------
            rfm69_cmd(0x0200, 1);                   //RegDataModul, FSK Packet
            rfm69_cmd(0x0502, 1);                   //RegFdevMsb, 241*61Hz = 35KHz
            rfm69_cmd(0x0641, 1);                   //RegFdevLsb
            rfm69_cmd(0x130F, 1);                   //RegOcp, Disable OCP
            rfm69_cmd(0x1952, 1);                   //RegRxBw , RxBW, 83KHz

//            rfm69_cmd(0x2C00, 1);                   //RegPreambleMsb
//            rfm69_cmd(0x2D05, 1);                   //RegPreambleLsb, 5Byte Preamble
//            rfm69_cmd(0x2E90, 1);                   //enable Sync.Word, 2+1=3bytes
//            rfm69_cmd(0x2FAA, 1);                   //0xAA, SyncWord = aa2dd4
//            rfm69_cmd(0x302D, 1);                   //0x2D
//            rfm69_cmd(0x31D4, 1);                   //0xD4
//            rfm69_cmd(0x3700, 1);                   //RegPacketConfig1, Disable CRC  NRZ encode
//            rfm69_cmd(0x3815, 1);                   //RegPayloadLength, 21bytes for length & Fixed length
            rfm69_cmd(0x3C95, 1);                   //RegFiFoThresh

            rfm69_cmd(0x1888, 1);                   //RegLNA, 200R
            rfm69_cmd(0x581B, 1);                   //RegTestLna, Normal sensitivity
              //0x582D,                   //RegTestLna, increase sensitivity with LNA (Note: consumption also increase!)
            rfm69_cmd(0x6F30, 1);                   //RegTestDAGC, Improved DAGC
              //0x6F00,                   //RegTestDAGC, Normal DAGC
            rfm69_cmd(0x0104, 1);                   //Enter standby mode

        }

        time_out = TIME_OUT;
        while( !(rfm69_cmd(0x1EFF, 0) & (1 << 4)) && --time_out ) ;


    rfm69_cmd(0x0A80, 1);                         // Start RC-Oscillator
    time_out = TIME_OUT;
    while (!(rfm69_cmd(0x0A00, 0) & (1 << 6)) && --time_out ) ;    // Wait for RC-Oscillator


    rfm69_rxon();
}

                                                                                                        
void rfm69_txon( void ) {
    uint32_t time_out=TIME_OUT;

    rfm69_cmd(0x010C, 1);                                         // TX on (set to ter mode in RegOpMode)
    while (!(rfm69_cmd(0x27FF, 0) & 1 << 7) && --time_out) ;    // Wait for Mode-Ready-Flag


    time_out = TIME_OUT;
    while (!(rfm69_cmd(0x27FF, 0) & 1 << 5) && --time_out) ;    // Wait for TX-Ready-Flag
}



void rfm69_rxon( void ) {
    uint32_t time_out=TIME_OUT;

    rfm69_cmd(0x0110, 1);                                             // RX on (set to receiver mode in RegOpMode)
    while( !(rfm69_cmd(0x27FF, 0) & (1 << 7)) && --time_out ) ;        // Wait for Mode-Ready-Flag

    time_out = TIME_OUT;
    while( !(rfm69_cmd(0x27FF, 0) & (1 << 6)) && --time_out ) ;     // Wait for RX-Ready-Flag
}

void rfm69_standby( void ) {
    uint32_t time_out=TIME_OUT;
    rfm69_cmd( 0x0104, 1 );                                         // (set to standby mode in RegOpMode)
    while( !(rfm69_cmd(0x27FF, 0) & (1 << 7)) && --time_out ) ;        // Wait for Mode-Ready-Flag
}


void rfm69_sleep( void ) {
    uint32_t time_out=TIME_OUT;
    rfm69_cmd( 0x0100, 1 );                                         // (set to sleep mode in RegOpMode)
    while( !(rfm69_cmd(0x27FF, 0) & (1 << 7)) && --time_out ) ;        // Wait for Mode-Ready-Flag

}


uint8_t rfm69_transmit( uint8_t addr, void *data, uint8_t length ) {
    uint32_t time_out = TIME_OUT;
    uint8_t * wsk = data;
    uint8_t len=length;
    uint8_t fifobuf[MAX_RFM69_BUF + 1];

    if( !len ) len = strlen( data );    // if len=0 we have a C-string

    // Turn off receiver, switch to Standby
    rfm69_standby();

    // Clear FIFO
    rfm69_fifo_clear();

    // Limit length
    if (len > MAX_RFM69_BUF - 1) len = MAX_RFM69_BUF - 1;

    fifobuf[0] = len+( addr_filter_bytes );                // Number of data bytes LEN+ADDR+DATA
    fifobuf[1] = addr;

    // Write data to FIFO-array
    for (uint8_t i = 0; i < len; i++) fifobuf[ i + addr_filter_bytes ] = wsk[i];

    // Write data to FIFO
    rfm69_fifo_transfer(fifobuf, 1);

    // Turn on transmitter (Transmitting starts automatically if FIFO not empty)
    rfm69_txon();

    // Wait for Package Sent
    time_out = TIME_OUT;
    while (!(rfm69_cmd(0x28FF, 0) & (1 << 3)) && --time_out) ;

    rfm69_rxon();

    return 0;
}



uint8_t rfm69_receive( void *data ) {
    uint8_t fifo_buf[MAX_RFM69_BUF + 1];
    uint8_t len;
    uint8_t * wsk = data;

    // Turn off receiver, switch to Standby
    rfm69_standby();

    // Read FIFO-data into FIFO-array
    rfm69_fifo_transfer(fifo_buf, 0);

    // Read data from FIFO-array
    len = fifo_buf[0];                                                // Number of data bytes

    if (len > MAX_RFM69_BUF - 1) len = MAX_RFM69_BUF - 1;     // Limit length
    for (uint8_t i = 0; i < len; i++) {
        wsk[i] = fifo_buf[ i + addr_filter_bytes ];                                                // Data bytes
    }
    wsk[len] = 0;                                                    // Terminate string

    // Clear FIFO after readout
    rfm69_fifo_clear();

    // Turn receiver back on
    rfm69_rxon();

    // return length of received data/string
    return len - addr_filter_bytes ;
}




// check if there it's some received data ?
uint8_t rfm69_receiving( void ) {
    uint8_t status;
    RFM_ACTIVATE;
    status = ((rfm69_cmd(0x28FF, 0) & (1 << 2)) && 1); // PayloadReady?
    RFM_DEACTIVATE;
    return status;
}


// check status flags
uint16_t rfm69_status( void ) {
    uint16_t status = 0;
    RFM_ACTIVATE;
    status |= rfm69_cmd(0x2700, 0);        // flags1
    status <<= 8;
    status |= rfm69_cmd(0x2800, 0);        // flags2
    RFM_DEACTIVATE;
    return status;
}