
#ifndef RFM69_H_
#define RFM69_H_

//*****************************************************************************************************
//*****************************************************************************************************
//------------- USER CONFIG - HERE ----------------------------------------


#define BITRATE        9600                // baudrate

#define USE_IRQ        0                    // 1 - if use IRQ, 0 - if don't
#define IRQ_NR        0                    // choose INT0, INT1 or INT2

                                                                         
//------------- USER CONFIG - SOFT SPI PINS --------------------------- Atmega8
#define MOSI_PORT            PORTB 
#define MOSI                (1<<3)
#define MOSI_DIR            DDRB

#define MISO_PORT            PORTB
#define MISO                (1<<4)
#define MISO_DIR            DDRB
#define MISO_PIN            PINB

#define SCK_PORT            PORTB
#define SCK                    (1<<5)
#define SCK_DIR                DDRB

#define NSEL_PORT            PORTB
#define NSEL                (1<<2)
#define NSEL_DIR            DDRB
//------------- USER CONFIG - END ----------------------------------------
//*****************************************************************************************************
//*****************************************************************************************************


//--------------------- Atmega8 -----------------------
#if IRQ_NR == 0
    #define RFM69_IRQ_vect     INT0_vect
    #define    IRQ_PORT        PORTD
    #define IRQ_PIN            (1<<PD2)
#endif
                                                                                                                                                                       
#if IRQ_NR == 1
    #define RFM69_IRQ_vect     INT1_vect
    #define    IRQ_PORT        PORTD
    #define IRQ_PIN            (1<<PD3)
#endif
/* 
#if IRQ_NR == 2
    #define RFM69_IRQ_vect     INT2_vect   
    #define    IRQ_PORT        PORTB
    #define IRQ_PIN            (1<<PB2)
#endif
*/


#define INT(x) XINT(x)
#define XINT(x)    (INT##x)

#define INTVECT(x) XINTVECT(x)
#define XINTVECT(x)    (INT##x##_vect)



// Don't change anything from here
#define XTALFREQ            32000000UL


#define RFM_ACTIVATE            NSEL_PORT    &=    ~NSEL
#define RFM_DEACTIVATE            NSEL_PORT    |=     NSEL


#define FREQ( fr )            (( (fr*1000000LL)*524288LL + (XTALFREQ/2)) / XTALFREQ)
//#define FRF_MSB                ((FRF>>16) & 0xFF)
//#define FRF_MID                ((FRF>>8) & 0xFF)
//#define FRF_LSB                ((FRF) & 0xFF)


#define PACKET_MODE                0x00
#define CONT_MODE_W_SYNC_BIT    0x10
#define CONT_MODE_WO_SYNC_BIT    0x11

#define MODULATION_FSK            0x00
#define MODULATION_OOK            0x01

#define    MOD_SHAPING_OFF            0x00
#define MOD_FSK_SHAPE_BT_10        0x01
#define MOD_FSK_SHAPE_BT_05        0x02
#define MOD_FSK_SHAPE_BT_03        0x03
#define MOD_OOK_SHAPE_1XBR        0x01
#define MOD_OOK_SHAPE_2XBR        0x02

#define PACKET_FIXED_LEN        0x00
#define PACKET_VARIALBLE_LEN    0x01
#define PACKET_DC_FILTER_NONE    0x00
#define PACKET_DC_FILTER_MAN    0x01
#define PACKET_DC_FILTER_WHI    0x02

#define ADDR_FILTER_OFF            0x00
#define ADDR_FILTER_NODE        0x01
#define ADDR_FILTER_BROADCAST    0x02



#define DATARATE            ((XTALFREQ + (BITRATE/2)) / BITRATE)
#define DATARATE_MSB        (DATARATE>>8)
#define DATARATE_LSB        (DATARATE & 0xFF)

#ifndef MAX_RFM69_BUF
#define MAX_RFM69_BUF 30
#endif

// Value for input timeout
#ifndef TIME_OUT
#define TIME_OUT            (F_CPU/160UL)
#endif



extern volatile uint8_t RX_flag;



// MK
uint8_t rfm69_cmd( uint16_t cmd, uint8_t write );

void rfm69_init( uint32_t freq, const char * network_id, uint8_t address_filter, uint8_t node_adr, uint8_t broadcast_adr );
uint8_t rfm69_receive( void *data );
uint8_t rfm69_transmit( uint8_t addr, void *data, uint8_t length );
uint8_t rfm69_receiving( void );
uint16_t rfm69_status( void );
void rfm69_fifo_clear( void );

void RFM69_RX_EVENT( uint8_t * rfm69_buf );
void rfm69_rx_event_callback( void (*callback)(uint8_t * data, uint8_t len ) );

void rfm69_txon( void );
void rfm69_rxon( void );

void rfm69_standby( void );                                  
void rfm69_sleep( void );

void rfm69_SetBitrate( uint32_t bitrate );
void rfm69_SetOverCurrent( uint8_t enable, uint8_t mA );
void rfm69_SetPowerOut( int8_t dBm );
void rfm69_PacketConfig( uint8_t packet_len, uint8_t dc_encoding, uint8_t addr_filter );
void rfm69_SetNodeAddress( uint8_t node_addr );
void rfm69_SetBroadcastAddress( uint8_t broadcast_addr );

#endif /* MK_RFM69_H_ */