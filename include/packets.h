#ifndef PACKETS_H
#define PACKETS_H

#ifdef DP100
#define SOF_RX '>'
#define SOF_TX '<'
#else
#define SOF_RX '<'
#define SOF_TX '>'
#endif

#define SEPARATOR ','

static char regDispCmdResp[35] = { SOF_TX, SEPARATOR, '8','3', SEPARATOR, '0', SEPARATOR,
          '0','0','0','0','0','0','0','0','-',
          '0','0','0','0','0','0','0','0','-',
          '0','0','0','0','0','0','0','0',
          SEPARATOR, '0'};

#endif
