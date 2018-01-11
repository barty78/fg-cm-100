#ifndef PACKETS_H
#define PACKETS_H

#ifdef DP100
#define SOF_RX    '>'
#define SOF_TX    '<'
#else
#define SOF_RX    '<'
#define SOF_TX    '>'
#endif

#define LED       '1'
#define ALL       '1'
#define SNGL      '2'
#define BLINK     '3'
#define ALLOFF    '4'
#define RAW7SEG   '6'

#define SEPARATOR ','
#define UUID_SEP  '-'
#define TENS      '00'
#define ONES      '00'


// Display Registration Packets
static char regDispCmdResp[35] = { SOF_TX, SEPARATOR, '8','3', SEPARATOR, '0', SEPARATOR,
          '0','0','0','0','0','0','0','0',UUID_SEP,
          '0','0','0','0','0','0','0','0',UUID_SEP,
          '0','0','0','0','0','0','0','0',
          SEPARATOR, '0'};

// Display Heartbeat Packet
static char dispHeartbeat[] = { SOF_TX, SEPARATOR, '0', '0', SEPARATOR, '0', SEPARATOR};

// LED Packets
static char dispSinglLedCmd[] = { SOF_TX, SEPARATOR, LED, SEPARATOR, SNGL, SEPARATOR};
static char dispAllLedCmd[] = { SOF_TX, SEPARATOR, LED, SEPARATOR, ALL, SEPARATOR};
static char dispLedBlinkCmd[] = { SOF_TX, SEPARATOR, LED, SEPARATOR, BLINK, SEPARATOR};
static char dispAllLedOffCmd[] = { SOF_TX, SEPARATOR, LED, SEPARATOR, ALLOFF, SEPARATOR};
static char dispRaw7SegCmd[] = { SOF_TX, SEPARATOR, LED, RAW7SEG, SEPARATOR, TENS, SEPARATOR, ONES, SEPARATOR};

#endif
