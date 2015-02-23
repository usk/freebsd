#ifndef __IF_MVNETA_H__
#define __IF_MVNETA_H__

#define MVNETA_RXQ_NUM_PER_PORT	8
#define MVNETA_TXQ_NUM_PER_PORT	8
#define MVNETA_TX_DESC_NUM		256
#define MVNETA_RX_DESC_NUM		256
#define MVNETA_IDMA_NUM			4
#define MVNETA_IDMA_MAX_SIZE	(16 * 1024 * 1024)
#define MVNETA_BMU_NUM_BUFPOOL	4
#define MVNETA_LONGFRAME_SIZE	9700
#define MVNETA_SHORTFRAME_SIZE	64

struct mvneta_tx_desc {
	uint32_t	command;		/* Options used by H/W */
	uint16_t	reserved0;
	uint16_t	size;			/* Payload size in bytes */
	bus_addr_t	buf_pa;			/* Physical address of TX buffer */
	uint32_t	reserved1[5];
};

struct mvneta_rx_desc {
	uint32_t	status;			/* Received packet status */
	uint16_t	reserved0;
	uint16_t	size;			/* Payload size in bytes */
	bus_addr_t	buf_pa;			/* Physical address of RX buffer */
	uint32_t	reserved1;
	bus_addr_t	buf_va;			/* Virtual address of RX buffer */
	uint32_t	reserved2[4];
};

struct mvneta_softc {
	device_t	dev;
	struct mvneta_tx_desc		*tx_desc;
	struct mvneta_desc_wrapper	*rx_desc;
};

#define MVNETA_PACC   /* Port Acceleration Mode Register */
#define MVNETA_EUDA   /* Ethernet Unit Default Address Register */
#define MVNETA_EUDID  /* Ethernet Unit Default ID Register */
#define MVNETA_PXC    /* Port Configuration Register */
#define MVNETA_SDC    /* SDMA Configuration Register */

#endif /* __IF_MVNETA_H__ */
