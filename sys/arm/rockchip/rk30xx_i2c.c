/*-
 * Copyright (C) 2015 usk
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/limits.h>
#include <sys/module.h>
#include <sys/resource.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <sys/rman.h>

#include <sys/lock.h>
#include <sys/mutex.h>
#include <machine/atomic.h>

#include <dev/iicbus/iiconf.h>
#include <dev/iicbus/iicbus.h>
#include "iicbus_if.h"

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#define I2C_CON_REG		0x0000
#define I2C_CLKDIV_REG		0x0004
#define I2C_MRXADDR_REG	0x0008
#define I2C_MRXRADDR_REG	0x000c
#define I2C_MTXCNT_REG		0x0010
#define I2C_MRXCNT_REG		0x0014
#define I2C_IEN_REG		0x0018
#define I2C_IPD_REG		0x001c
#define I2C_FCNT_REG		0x0020
#define I2C_TXDATA_REG(n)	(0x0100 + ((n) * 4))
#define I2C_RXDATA_REG(n)	(0x0200 + ((n) * 4))
#define I2C_TXDATA_REG_MAX	8
#define I2C_RXDATA_REG_MAX	8

#define I2C_CON_EN		(1 << 0)
#define I2C_CON_MODE_TX	(0 << 1)
#define I2C_CON_MODE_TXADDR	(1 << 1)
#define I2C_CON_MODE_RX	(2 << 1)
#define I2C_CON_MODE_RXADDR	(3 << 1)
#define I2C_CON_START		(1 << 3)  /* this bit is cleared by H/W */
#define I2C_CON_STOP		(1 << 4)  /* ditto */
#define I2C_CON_ACK		(0 << 5)
#define I2C_CON_NAK		(1 << 5)
#define I2C_CON_ACK2NAK_IGNORE	(0 << 6)
#define I2C_CON_ACK2NAK_STOP	(1 << 6)

#define I2C_CLKDIV_LOW		0x0000ffff
#define I2C_CLKDIV_HIGH	0xffff0000

#define I2C_MRXADDR_SADDR	0x007fffff
#define I2C_MRXADDR_ADDLVLD	(1 << 24)
#define I2C_MRXADDR_ADDMVLD	(1 << 25)
#define I2C_MRXADDR_ADDHVLD	(1 << 26)

#define I2C_MRXRADDR_SRADDR	0x007fffff
#define I2C_MRXRADDR_SRADDLVLD	(1 << 24)
#define I2C_MRXRADDR_SRADDMVLD	(1 << 25)
#define I2C_MRXRADDR_SRADDHVLD	(1 << 26)

#define I2C_MTXCNT_COUNT	0x0000003f

#define I2C_MRXCNT_COUNT	0x0000003f

#define I2C_INT_BTF	(1 << 0)
#define I2C_INT_BRF	(1 << 1)
#define I2C_INT_MBTF	(1 << 2)
#define I2C_INT_MBRF	(1 << 3)
#define I2C_INT_START	(1 << 4)
#define I2C_INT_STOP	(1 << 5)
#define I2C_INT_NAKRCV	(1 << 6)

#define I2C_FCNT_COUNT	0x0000003f

#define I2C_CLOCK_RATE	100000

struct i2c_softc {
	device_t		dev;
	device_t		iicbus;
	struct mtx		mutex;
	struct resource	*res[2];  /* 0: MMIO, 1:INTR */
	bus_space_handle_t	bsh;
	bus_space_tag_t	bst;
};

static struct resource_spec i2c_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE },
	{ -1, 0 }
};

static int i2c_probe(device_t);
static int i2c_attach(device_t);

static phandle_t i2c_get_node(device_t, device_t);

static int i2c_repeated_start(device_t, u_char, int);
static int i2c_start(device_t, u_char, int);
static int i2c_stop(device_t);
static int i2c_reset(device_t, u_char, u_char, u_char *);
static int i2c_read(device_t, char *, int, int *, int, int);
static int i2c_write(device_t, const char *, int, int *, int);

static device_method_t i2c_methods[] = {
	DEVMETHOD(device_probe,		i2c_probe),
	DEVMETHOD(device_attach,		i2c_attach),

	/* OFW methods */
	DEVMETHOD(ofw_bus_get_node,		i2c_get_node),

	DEVMETHOD(iicbus_callback,		iicbus_null_callback),
	DEVMETHOD(iicbus_repeated_start,	i2c_repeated_start),
	DEVMETHOD(iicbus_start,		i2c_start),
	DEVMETHOD(iicbus_stop,			i2c_stop),
	DEVMETHOD(iicbus_reset,		i2c_reset),
	DEVMETHOD(iicbus_read,			i2c_read),
	DEVMETHOD(iicbus_write,		i2c_write),
	DEVMETHOD(iicbus_transfer,		iicbus_transfer_gen),

	{ 0, 0 }
};

static driver_t i2c_driver = {
	"iichb",
	i2c_methods,
	sizeof(struct i2c_softc),
};

static devclass_t i2c_devclass;

DRIVER_MODULE(i2c, simplebus, i2c_driver, i2c_devclass, 0, 0);
DRIVER_MODULE(iicbus, i2c, iicbus_driver, iicbus_devclass, 0, 0);
MODULE_DEPEND(i2c, iicbus, 1, 1, 1);

static struct ofw_compat_data compat_data[] = {
	{"rockchip,rk30xx-i2c", 1},
	{NULL,                  0}
};

static __inline void
i2c_write_reg(struct i2c_softc *sc, bus_size_t off, uint32_t val)
{

	bus_space_write_4(sc->bst, sc->bsh, off, val);
}

static __inline uint32_t
i2c_read_reg(struct i2c_softc *sc, bus_size_t off)
{

	return bus_space_read_4(sc->bst, sc->bsh, off);
}

static void
i2c_dump_reg(struct i2c_softc *sc)
{
	device_t dev = sc->dev;
	int i;

	device_printf(dev, "I2C_CON_REG:       0x%08x\n", i2c_read_reg(sc, I2C_CON_REG));
	device_printf(dev, "I2C_CLKDIV_REG:    0x%08x\n", i2c_read_reg(sc, I2C_CLKDIV_REG));
	device_printf(dev, "I2C_MRXADDR_REG:   0x%08x\n", i2c_read_reg(sc, I2C_MRXADDR_REG));
	device_printf(dev, "I2C_MRXRADDR_REG:  0x%08x\n", i2c_read_reg(sc, I2C_MRXRADDR_REG));
	device_printf(dev, "I2C_MTXCNT_REG:    0x%08x\n", i2c_read_reg(sc, I2C_MTXCNT_REG));
	device_printf(dev, "I2C_MRXCNT_REG:    0x%08x\n", i2c_read_reg(sc, I2C_MRXCNT_REG));
	device_printf(dev, "I2C_IEN_REG:       0x%08x\n", i2c_read_reg(sc, I2C_IEN_REG));
	device_printf(dev, "I2C_IPD_REG:       0x%08x\n", i2c_read_reg(sc, I2C_IPD_REG));
	device_printf(dev, "I2C_FCNT_REG:      0x%08x\n", i2c_read_reg(sc, I2C_FCNT_REG));

	for (i = 0; i < I2C_TXDATA_REG_MAX; i++) {
		device_printf(dev, "I2C_TXDATA_REG[%d]: 0x%08x\n", i, i2c_read_reg(sc, I2C_TXDATA_REG(i)));
	}

	for (i = 0; i < I2C_RXDATA_REG_MAX; i++) {
		device_printf(dev, "I2C_RXDATA_REG[%d]: 0x%08x\n", i, i2c_read_reg(sc, I2C_RXDATA_REG(i)));
	}
}

/*
 * todo:
 *   branch by rockchip chip id
 *   implement CRU driver
 */
#define __BIT(n)			(1 << (n))
#define __BITS(hi,lo)			((~((~0)<<((hi)+1)))&((~0)<<(lo)))
#define __LOWEST_SET_BIT(__mask)	((((__mask) - 1) & (__mask)) ^ (__mask))
#define __SHIFTOUT(__x, __mask)	(((__x) & (__mask)) / __LOWEST_SET_BIT(__mask))

#define CRU_VA						((volatile uint8_t *)arm_devmap_ptov(0x20000000, 0x1000))
#define CRU_APLL_CON_REG(n)				(0x0000 + 4 * (n))	/* ARM PLL configuration registers */
#define CRU_DPLL_CON_REG(n)				(0x0010 + 4 * (n))	/* DDR PLL configuration registers */
#define CRU_CPLL_CON_REG(n)				(0x0020 + 4 * (n))	/* CODEC PLL configuration registers */
#define CRU_GPLL_CON_REG(n)				(0x0030 + 4 * (n))	/* GENERAL PLL configuration registers */
#define CRU_MODE_CON_REG				0x0040			/* System work mode control register */
#define CRU_CLKSEL_CON_REG(n)				(0x0044 + 4 * (n))	/* Internal clock select and divide registers */
#define CRU_CLKGATE_CON_REG(n)				(0x00d0 + 4 * (n))	/* Internal clock gating control registers */
#define CRU_GLB_SRST_FST_VALUE_REG			0x0100			/* The 1st global software reset config value */
#define CRU_GLB_SRST_SND_VALUE_REG			0x0104			/* The 2nd global software reset config value */
#define CRU_SOFTRST_CON_REG(n)				(0x0110 + 4 * (n))	/* Internal software reset control registers */
#define CRU_MISC_CON_REG				0x0134			/* SCU control register */
#define CRU_GLB_CNT_TH_REG				0x0140			/* Global reset wait counter threshold */

#define CRU_CLKSEL_CON0_CPU_CLK_PLL_SEL		__BIT(8)
#define CRU_CLKSEL_CON10_PERI_ACLK_DIV_CON		__BITS(4,0)
#define CRU_CLKSEL_CON10_PERI_PCLK_DIV_CON		__BITS(13,12)
#define CRU_CLKSEL_CON10_PERI_PLL_SEL			__BIT(15)
#define CRU_CLKSEL_CON1_CPU_PCLK_DIV_CON		__BITS(13,12)
#define CRU_PLL_CON0_CLKR				__BITS(13,8)
#define RK3188_CRU_CLKSEL_CON0_A9_CORE_DIV_CON		__BITS(13,9)
#define RK3188_CRU_CLKSEL_CON1_CPU_ACLK_DIV_CON	__BITS(5,3)
#define RK3188_CRU_PLL_CON0_CLKOD			__BITS(5,0)
#define RK3188_CRU_PLL_CON1_CLKF			__BITS(15,0)
#define ROCKCHIP_REF_FREQ				24000000L		/* 24MHz */

static uint32_t
rockchip_pll_get_rate(uint32_t *con0_reg, uint32_t *con1_reg)
{
	uint32_t pll_con0, pll_con1;
	uint32_t nr, no, nf;

	pll_con0 = *((volatile uint32_t *)(CRU_VA+con0_reg));
	pll_con1 = *((volatile uint32_t *)(CRU_VA+con1_reg));
	rmb();

	nr = __SHIFTOUT(pll_con0, CRU_PLL_CON0_CLKR) + 1;
	no = __SHIFTOUT(pll_con0, RK3188_CRU_PLL_CON0_CLKOD) + 1;
	nf = __SHIFTOUT(pll_con1, RK3188_CRU_PLL_CON1_CLKF) + 1;

	return ((uint64_t)ROCKCHIP_REF_FREQ * nf) / (nr * no);
}

static uint32_t
rockchip_gpll_get_rate()
{

	return rockchip_pll_get_rate(CRU_GPLL_CON_REG(0), CRU_GPLL_CON_REG(1));
}

static uint32_t
rockchip_apll_get_rate()
{

	return rockchip_pll_get_rate(CRU_APLL_CON_REG(0), CRU_APLL_CON_REG(1));
}

static uint32_t
rockchip_cpu_get_rate()
{
	uint32_t clksel_con0;
	uint32_t a9_core_div;

	clksel_con0 = *((volatile uint32_t *)(CRU_VA+CRU_CLKSEL_CON_REG(1)));
	rmb();

	a9_core_div = __SHIFTOUT(clksel_con0, RK3188_CRU_CLKSEL_CON0_A9_CORE_DIV_CON) + 1;
	if (clksel_con0 & CRU_CLKSEL_CON0_CPU_CLK_PLL_SEL) {
		return rockchip_gpll_get_rate() / a9_core_div;
	} else {
		return rockchip_apll_get_rate() / a9_core_div;
	}
}

static uint32_t
rockchip_pclk_cpu_get_rate()
{
	uint32_t clksel_con1;
	uint32_t aclk_div, core_axi_div;

	clksel_con1 = *((volatile uint32_t *)(CRU_VA+CRU_CLKSEL_CON_REG(1)));
	rmb();

	aclk_div = __SHIFTOUT(clksel_con1, RK3188_CRU_CLKSEL_CON1_CPU_ACLK_DIV_CON);
	switch (aclk_div) {
	case 0: core_axi_div = 1; break;
	case 1: core_axi_div = 2; break;
	case 2: core_axi_div = 3; break;
	case 3: core_axi_div = 4; break;
	case 4: core_axi_div = 8; break;
	default: return EINVAL;
	}
	pclk_div = 1 << __SHIFTOUT(clksel_con1, CRU_CLKSEL_CON1_CPU_PCLK_DIV_CON);

	return rockchip_cpu_get_rate() / (core_axi_div * pclk_div);
}

static uint32_t
rockchip_apb_get_rate()
{
	uint32_t clksel_con10;
	uint32_t pclk_div, aclk_div;

	clksel_con10 = *((volatile uint32_t *)(CRU_VA+CRU_CLKSEL_CON_REG(10)));
	rmb();

	if (clksel_con10 & CRU_CLKSEL_CON10_PERI_PLL_SEL) {
		rate = rockchip_gpll_get_rate();
	} else {
		rate = rockchip_cpll_get_rate();
	}
	aclk_div = __SHIFTOUT(clksel_con10, CRU_CLKSEL_CON10_PERI_ACLK_DIV_CON) + 1;
	pclk_div = 1 << __SHIFTOUT(clksel_con10, CRU_CLKSEL_CON10_PERI_PCLK_DIV_CON);

	return rate / (aclk_div * pclk_div);
}

static uint32_t
rockchip_i2c_get_rate(uint32_t port)
{
	if (port == 0 || port == 1) {
		return rockchip_pclk_cpu_get_rate();
	} else {
		return rockchip_apb_get_rate();
	}
}

static int
i2c_set_rate(struct i2c_softc *sc, uint32_t rate)
{
	int port;
	uint32_t i2c_rate, clkdiv, divh, divl;

	port = device_get_unit(dev);
	i2c_rate = rockchip_i2c_get_rate(port);
	if (i2c_rate == 0) {
		return ENXIO;
	}

	/*
	 * SCL Divisor = 8 * (CLKDIVL + CLKDIVH)
	 * SCL = PCLK / SCLK Divisor
	 */
	div = (i2c_rate + (rate * 8 - 1)) / (rate * 8);
	divh = divl = (div + (2 - 1)) / 2;
	clkdiv = (divh << 16) | (divl & I2C_CLKDIV_LOW);
	i2c_write_reg(sc, I2C_CLKDIV_REG, clkdiv);

	return 0;
}

static int
i2c_intr(void *arg)
{
	struct i2c_softc *sc;
	uint32_t ipd;

	sc = (struct i2c_softc *)arg;
	ipd = i2c_read_reg(sc, I2C_IPD_REG);

	if (ipd & I2C_INT_BTF) {
		device_printf(sc->dev, "I2C_INT_BTF");
	}

	if (ipd & I2C_INT_BRF) {
		device_printf(sc->dev, "I2C_INT_BRF");
	}

	if (ipd & I2C_INT_MBTF) {
		device_printf(sc->dev, "I2C_INT_MBTF");
	}

	if (ipd & I2C_INT_MBRF) {
		device_printf(sc->dev, "I2C_INT_MBRF");
	}

	if (ipd & I2C_INT_START) {
		device_printf(sc->dev, "I2C_INT_START");
	}

	if (ipd & I2C_INT_STOP) {
		device_printf(sc->dev, "I2C_INT_STOP");
	}

	if (ipd & I2C_INT_NAKRCV) {
		device_printf(sc->dev, "I2C_INT_NAKRCV");
	}

	return 0;
}

static int
i2c_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev)) {
		return (ENXIO);
	}

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0) {
		return (ENXIO);
	}

	device_set_desc(dev, "Rockchip RK30XX I2C bus controller");

	return (BUS_PROBE_DEFAULT);
}

static int
i2c_attach(device_t dev)
{
	struct i2c_softc *sc;
	void *ihl;

	sc = device_get_softc(dev);
	sc->dev = dev;

	mtx_init(&sc->mutex, device_get_nameunit(dev), "rkiic", MTX_DEF);

	if (bus_alloc_resources(dev, i2c_spec, sc->res)) {
		device_printf(dev, "could not allocate resources");
		mtx_destroy(&sc->mutex);
		return (ENXIO);
	}

	sc->bst = rman_get_bustag(sc->res[0]);
	sc->bsh = rman_get_bushandle(sc->res[0]);
	i2c_dump_reg(sc);
	i2c_write_reg(sc, I2C_CON_REG,      0x00000000);
	i2c_write_reg(sc, I2C_CLKDIV_REG,   0x00060006);
	i2c_write_reg(sc, I2C_MRXADDR_REG,  0x00000000);
	i2c_write_reg(sc, I2C_MRXRADDR_REG, 0x00000000);
	i2c_write_reg(sc, I2C_MTXCNT_REG,   0x00000000);
	i2c_write_reg(sc, I2C_MRXCNT_REG,   0x00000000);
	i2c_write_reg(sc, I2C_IEN_REG,      0x00000000);
	i2c_write_reg(sc, I2C_IPD_REG,      0x00000000);
	i2c_write_reg(sc, I2C_FCNT_REG,     0x00000000);
	i2c_set_rate(sc, I2C_CLOCK_RATE);

	if (bus_setup_intr(dev, sc->res[1], INTR_TYPE_CLK, i2c_intr, NULL, sc, &ihl) != 0) {
		device_printf(dev, "could not setup interrupt");
		bus_release_resources(dev, i2c_spec, sc->res);
		mtx_destroy(&sc->mutex);
		return (ENXIO);
	}

	sc->iicbus = device_add_child(dev, "iicbus", -1);
	if (sc->iicbus == NULL) {
		device_printf(dev, "could not add iicbus child");
		bus_release_resources(dev, i2c_spec, sc->res);
		mtx_destroy(&sc->mutex);
		return (ENXIO);
	}

	bus_generic_attach(dev);
	return (IIC_NOERR);
}

static phandle_t
i2c_get_node(device_t bus, device_t dev)
{
	/*
	 * Share controller node with iicbus device
	 */
	return ofw_bus_get_node(bus);
}

static int
i2c_repeated_start(device_t dev, u_char slave, int timeout)
{

	return 0;
}

static int
i2c_start(device_t dev, u_char slave, int timeout)
{

	return 0;
}

static int
i2c_stop(device_t dev)
{

	return 0;
}

static int
i2c_reset(device_t dev, u_char speed, u_char addr, u_char *oldadr)
{

	return 0;
}

static int
i2c_read(device_t dev, char *buf, int len, int *read, int last, int delay)
{

	return 0;
}

static int
i2c_write(device_t dev, const char *buf, int len, int *sent, int timeout)
{

	return 0;
}
