#define TXB0CTRL_REGISTER 0b00110000
    #define ABTF_FLAG         0b01000000 // Message Aborted
    #define MLOA_FLAG         0b00100000 // Message Lost Arbitration
    #define TXERR_FLAG        0b00010000 // Transmission Error
    #define TXREQ_FLAG        0b00001000 // Message Transmit Request
    #define TXP1_BIT          0b00000010 // Transmit Buffer Priority 1
    #define TXP0_BIT          0b00000001 // Transmit Buffer Priority 0
#define TXB0SIDH_REGISTER 0b00110001 // TANDARD IDENTIFIER REGISTER HIGH
    #define SID10_BIT         0b10000000
    #define SID9_BIT          0b01000000
    #define SID8_BIT          0b00100000
    #define SID7_BIT          0b00010000
    #define SID6_BIT          0b00001000
    #define SID5_BIT          0b00000100
    #define SID4_BIT          0b00000010
    #define SID3_BIT          0b00000001
#define TXB0SIDL_REGISTER 0b00110010 // TANDARD IDENTIFIER REGISTER LOW
    #define SID2_BIT          0b10000000
    #define SID1_BIT          0b01000000
    #define SID0_BIT          0b00100000
    #define EXIDE             0b00001000
    #define EID17             0b00000010
    #define EID16             0b00000001
#define TXB0DLC_REGISTER  0b00110101 // DATA LENGTH CODE REGISTER
    #define DLC3              0b00001000
    #define DLC2              0b00000100
    #define DLC1              0b00000010
    #define DLC0              0b00000001
#define TXB0D0_REGISTER   0b00110110
#define CNF3_REGISTER     0x28 // Budrate Configuration Registers
#define CNF2_REGISTER     0x29
#define CNF1_REGISTER     0x2A
#define CANINTE_REGISTER  0b00101010
    #define MERRE_FLAG    7
    #define WAKIE_FLAG    6
    #define ERRIE_FLAG    5
    #define TX1IE_FLAG    4
    #define TX0IE_FLAG    3
    #define RX1IE_FLAG    2
    #define RX0IE_FLAG    1
#define BFPCTRL_REGISTER   0x0C 
#define TXRTSCTRL_REGISTER 0x0D // PIN CONTROL AND STATUS REGISTER
#define CANCTRL_REGISTER   0x0F // CAN CONTROL REGISTER
    #define REQOP2_BIT    7 // Request Operation Mode bit 2
    #define REQOP1_BIT    6 // Request Operation Mode bit 1
    #define REQOP0_BIT    5 // Request Operation Mode bit 0
    #define ABAT_FLAG     4 // Abort All Pending Transmissions
    #define OSM_FLAG      3 // One-Shot Mode
    #define CLKEN_FLAG    2 // CLKOUT Pin Enable
    #define CLKPRE1_BIT   1 // CLKOUT Pin Prescaler bit 1
    #define CLKPRE0_BIT   0 // CLKOUT Pin Prescaler bit 0