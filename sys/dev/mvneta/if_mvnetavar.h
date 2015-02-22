#ifndef __IF_MVNETA_H__
#define __IF_MVNETA_H__

#define MVNETA_RXQ_NUM_PER_PORT 8
#define MVNETA_TXQ_NUM_PER_PORT 8
#define MVNETA_IDMA_NUM 4
#define MVNETA_IDMA_MAX_SIZE (16 * 1024 * 1024)
#define MVNETA_BMU_NUM_BUFPOOL 4
#define MVNETA_LONGFRAME_SIZE = 9700
#define MVNETA_SHORTFRAME_SIZE = 64

#define MVNETA_PACC   /* Port Acceleration Mode Register */
#define MVNETA_EUDA   /* Ethernet Unit Default Address Register */
#define MVNETA_EUDID  /* Ethernet Unit Default ID Register */
#define MVNETA_PXC    /* Port Configuration Register */
#define MVNETA_SDC    /* SDMA Configuration Register */

struct mvneta_rx_desc {};

struct mvneta_tx_desc {};

struct mvneta_softc {};

#endif /* __IF_MVNETA_H__ */
