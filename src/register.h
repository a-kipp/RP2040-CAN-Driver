// Configuration Registers
#define BFPCTRL_REGISTER   0x0C // PIN CONTROL AND STATUS REGISTER
    #define B1BFS          5 // Pin State bit (Digital Output mode only) - Reads as ‘0’ when RX1BF is configured as an interrupt pin
    #define B0BFS          4 // 
    #define B1BFE          3 // Pin Function Enable bit
    #define B0BFE          2 //
    #define B1BFM          1 // Pin Operation Mode bit
    #define B0BFM          0 //
#define TXRTSCTRL_REGISTER 0x0D // PIN CONTROL AND STATUS REGISTER
    #define B2RTS          5 // Pin State bit
    #define B1RTS          4 // 
    #define B0RTS          3 //
    #define B2RTSM         2 // Pin Mode bit
    #define B1RTSM         1 //
    #define B0RTSM         0 //
#define CANSTAT_REGISTER   0x0E // CAN STATUS REGISTER 
    #define OPMOD2_BIT     7 // Operation Mode bits
    #define OPMOD1_BIT     6 //
    #define OPMOD0_BIT     5 //
    #define ICOD2_BIT      3 // Interrupt Flag Code bits 
    #define ICOD1_BIT      2 //
    #define ICOD0_BIT      1 //
#define CANCTRL_REGISTER   0x0F // CAN CONTROL REGISTER
    #define REQOP2_BIT     7 // Request Operation Mode bit 2
    #define REQOP1_BIT     6 // Request Operation Mode bit 1
    #define REQOP0_BIT     5 // Request Operation Mode bit 0
    #define ABAT_FLAG      4 // Abort All Pending Transmissions
    #define OSM_FLAG       3 // One-Shot Mode
    #define CLKEN_FLAG     2 // CLKOUT Pin Enable
    #define CLKPRE1_BIT    1 // CLKOUT Pin Prescaler bit 1
    #define CLKPRE0_BIT    0 // CLKOUT Pin Prescaler bit 0
#define TEC_REGISTER       0x1C // TRANSMIT ERROR COUNTER
#define REC_REGISTER       0x1D // RECIEVE ERRROR COUNTER
#define CNF3_REGISTER      0x28 // Budrate Configuration Registers
    #define SOF_BIT        7 // clockout on start of frame
    #define WAKFIL         6 // Wake-up filter is enabled
    #define PHSEG22        2 // PS2 Length bits
    #define PHSEG21        1 //
    #define PHSEG20        0 //
#define CNF2_REGISTER      0x29
#define CNF1_REGISTER      0x2A
#define CANINTE_REGISTER   0x2B // CAN INTERRUPT ENABLE REGISTER 
    #define MERRE_FLAG     7 // Message Error Interrupt Enable bi
    #define WAKIE_FLAG     6 // Wake-up Interrupt Enable bi
    #define ERRIE_FLAG     5 // Error Interrupt Enable bit (multiple sources in EFLG registe
    #define TX2IE_FLAG     4 // Transmit Buffer 2 Empty Interrupt Enable bit
    #define TX1IE_FLAG     3 // Transmit Buffer 1 Empty Interrupt Enable bit
    #define TX0IE_FLAG     2 // Transmit Buffer 0 Empty Interrupt Enable bit
    #define RX1IE_FLAG     1 // Receive Buffer 1 Full Interrupt Enable bit
    #define RX0IE_FLAG     0 // Receive Buffer 0 Full Interrupt Enable bit
#define CANINTF_REGISTER   0x2C
#define EFLG_REGISTER      0x2D
    
    
// Transmit Registers
#define TXB0CTRL_REGISTER  0x30 // TRANSMIT BUFFER 0 CONTROL REGISTER
    #define ABTF_FLAG      6 // Message Aborted
    #define MLOA_FLAG      5 // Message Lost Arbitration
    #define TXERR_FLAG     4 // Transmission Error
    #define TXREQ_FLAG     3 // Message Transmit Request
    #define TXP1_BIT       1 // Transmit Buffer Priority 1
    #define TXP0_BIT       0 // Transmit Buffer Priority 0
#define TXB0SIDH_REGISTER  0x31 // STANDARD IDENTIFIER REGISTER HIGH
    #define SID10_BIT      7 // Standard Identifier bits
    #define SID9_BIT       6 //
    #define SID8_BIT       5 //
    #define SID7_BIT       4 //
    #define SID6_BIT       3 //
    #define SID5_BIT       2 //
    #define SID4_BIT       1 //
    #define SID3_BIT       0 //
#define TXB0SIDL_REGISTER  0x32 // STANDARD IDENTIFIER REGISTER LOW
    #define SID2_BIT       7 // Standard Identifier bits
    #define SID1_BIT       6 //
    #define SID0_BIT       5 //
    #define EXIDE          3 // Extended Identifier Enable bit
    #define EID17          1 // Extended Identifier bits
    #define EID16          0 //
#define TXB0EID8           0x33 // EXTENDED IDENTIFIER 8 REGISTER HIGH
#define TXB0EID0           0x34 // EXTENDED IDENTIFIER 0 REGISTER LOW
#define TXB0DLC_REGISTER   0x35 // DATA LENGTH CODE REGISTER
    #define RTR_BIT        6 // Remote Transmission Request bit
    #define DLC3_BIT       3 // Data Length Code bits
    #define DLC2_BIT       2 //
    #define DLC1_BIT       1 //
    #define DLC0_BIT       0 //
#define TXB0D0_REGISTER    0x36 // TRANSMIT BUFFER DATA BYTE 0 REGISTER
#define TXB0D1_REGISTER    0x37 //
#define TXB0D2_REGISTER    0x38 //
#define TXB0D3_REGISTER    0x39 //
#define TXB0D4_REGISTER    0x3A //
#define TXB0D5_REGISTER    0x3B //
#define TXB0D6_REGISTER    0x3C //
#define TXB0D7_REGISTER    0x3D //

#define TXB1CTRL_REGISTER  0x40
#define TXB1SIDH_REGISTER  0x41
#define TXB1SIDL_REGISTER  0x42
#define TXB1EID8_REGISTER  0x43
#define TXB1EID0_REGISTER  0x44
#define TXB1DLC_REGISTER   0x45
#define TXB1D0_REGISTER    0x46
#define TXB1D1_REGISTER    0x47
#define TXB1D2_REGISTER    0x48
#define TXB1D3_REGISTER    0x49
#define TXB1D4_REGISTER    0x4A
#define TXB1D5_REGISTER    0x4B
#define TXB1D6_REGISTER    0x4C
#define TXB1D7_REGISTER    0x4D 

#define TXB2CTRL_REGISTER  0x50
#define TXB2SIDH_REGISTER  0x51
#define TXB2SIDL_REGISTER  0x52
#define TXB2EID8_REGISTER  0x53
#define TXB2EID0_REGISTER  0x54
#define TXB2DLC_REGISTER   0x55
#define TXB2D0_REGISTER    0x56
#define TXB2D1_REGISTER    0x57
#define TXB2D2_REGISTER    0x58
#define TXB2D3_REGISTER    0x59
#define TXB2D4_REGISTER    0x5A
#define TXB2D5_REGISTER    0x5B
#define TXB2D6_REGISTER    0x5C
#define TXB2D7_REGISTER    0x5D


// Recieve Registers
#define RXB0CTRL_REGISTER  0x60 // RECEIVE BUFFER 0 CONTROL REGISTER
    #define RXM1           6 // Receive Buffer Operating mode bits
    #define RXM0           5 //
    #define RXRTR          3 // Received Remote Transfer Request bit
    #define BUKT           2 // Rollover Enable bit
    #define BUKT1          1 // Read-Only Copy of BUKT bit (used internally)
    #define FILTH0         0 // Filter Hit bit (indicates which acceptance filter enabled reception of message)
#define RXB0SIDH_REGISTER  0x61 // RECEIVE BUFFER n STANDARD IDENTIFIER REGISTER HIG
    #define SID10_BIT      7 // Standard Identifier bits
    #define SID9_BIT       6 //
    #define SID8_BIT       5 //
    #define SID7_BIT       4 //
    #define SID6_BIT       3 //
    #define SID5_BIT       2 //
    #define SID4_BIT       1 //
    #define SID3_BIT       0 //
#define RXB0SIDL_REGISTER  0x62 // RECEIVE BUFFER 0 STANDARD IDENTIFIER REGISTER LOW
    #define SID2_BIT       7 // Standard Identifier bits
    #define SID1_BIT       6 //
    #define SID0_BIT       5 // 
    #define SRR_BIT        4 // Standard Frame Remote Transmit Request bit (valid only if IDE bit = 0)
    #define IDE_BIT        3 // Extended Identifier Flag bit
    #define EID17          1 // Extended Identifier bits
    #define EID16          0 //
#define RXB0EID8_REGISTER  0x63 // RECEIVE BUFFER 0 EXTENDED IDENTIFIER REGISTER HIGH
#define RXB0EID0_REGISTER  0x64 // RECEIVE BUFFER 0 EXTENDED IDENTIFIER REGISTER LOW
#define RXB0DLC_REGISTER   0x65 // RECEIVE BUFFER 0 DATA LENGTH CODE REGISTER
    #define RTR_BIT        6 // Extended Frame Remote Transmission Request bit (valid only when IDE (RXBnSIDL[3]) = 1)    
    #define DLC3           3 // Data Length Code bits
    #define DLC2           2 //
    #define DLC1           1 //
    #define DLC0           0 //
#define RXB0D0_REGISTER    0x66 // RECEIVE BUFFER 0 DATA BYTE m REGISTER
#define RXB0D1_REGISTER    0x67 //
#define RXB0D2_REGISTER    0x68 //
#define RXB0D3_REGISTER    0x69 //
#define RXB0D4_REGISTER    0x6A //
#define RXB0D5_REGISTER    0x6B //
#define RXB0D6_REGISTER    0x6C //
#define RXB0D7_REGISTER    0x6D //

#define RXB1CTRL_REGISTER  0x70
    #define RXM1_BIT       6 // Receive Buffer Operating mode bits
    #define RXM0_BIT       5 //
    #define RXRTR_BIT      3 // Received Remote Transfer Request bit
    #define FILTH2_BIT     2 // Filter Hit bits (indicates which acceptance filter enabled reception of message)
    #define FILTH1_BIT     1 // 
    #define FILTH0_BIT     0 // 
#define RXB1SIDH_REGISTER  0x71
#define RXB1SIDL_REGISTER  0x72
#define RXB1EID8_REGISTER  0x63
#define RXB1EID0_REGISTER  0x64
#define RXB1DLC_REGISTER   0x75
#define RXB1D0_REGISTER    0x76
#define RXB1D1_REGISTER    0x77
#define RXB1D2_REGISTER    0x78
#define RXB1D3_REGISTER    0x79
#define RXB1D4_REGISTER    0x7A
#define RXB1D5_REGISTER    0x7B
#define RXB1D6_REGISTER    0x7C
#define RXB1D7_REGISTER    0x7D 



// Filter Control Registers
#define RXF0SIDH_REGISTER  0x00 // FILTER STANDARD IDENTIFIER REGISTER HIGH
#define RXF0SIDL_REGISTER  0x01 // FILTER STANDARD IDENTIFIER REGISTER LOW
#define RXF0EID8_REGISTER  0x02 // FILTER EXTENDED IDENTIFIER REGISTER HIGH
#define RXF0EID0_REGISTER  0x03 // FILTER EXTENDED IDENTIFIER REGISTER LOW

#define RXF1SIDH_REGISTER  0x04
#define RXF1SIDL_REGISTER  0x05
#define RXF1EID8_REGISTER  0x06
#define RXF1EID0_REGISTER  0x07

#define RXF2SIDH_REGISTER  0x08
#define RXF2SIDL_REGISTER  0x09
#define RXF2EID8_REGISTER  0x0A
#define RXF2EID0_REGISTER  0x0B

#define RXF3SIDH_REGISTER  0x10
#define RXF3SIDL_REGISTER  0x11
#define RXF3EID8_REGISTER  0x12
#define RXF3EID0_REGISTER  0x13

#define RXF4SIDH_REGISTER  0x14
#define RXF4SIDL_REGISTER  0x15
#define RXF4EID8_REGISTER  0x16
#define RXF4EID0_REGISTER  0x17

#define RXF5SIDH_REGISTER  0x18
#define RXF5SIDL_REGISTER  0x19
#define RXF5EID8_REGISTER  0x1A
#define RXF5EID0_REGISTER  0x1B


// Mask Control Registers
#define RXMF0SIDH_REGISTER  0x20 // MASK STANDARD IDENTIFIER REGISTER HIGH
#define RXMF0SIDL_REGISTER  0x21 // MASK STANDARD IDENTIFIER REGISTER LOW
#define RXMF0EID8_REGISTER  0x22 // MASK EXTENDED IDENTIFIER REGISTER HIGH
#define RXMF0EID0_REGISTER  0x23 // MASK EXTENDED IDENTIFIER REGISTER LORXM

#define RXMF1SIDH_REGISTER  0x24
#define RXMF1SIDL_REGISTER  0x25
#define RXMF1EID8_REGISTER  0x26
#define RXMF1EID0_REGISTER  0x27