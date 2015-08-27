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

#define I2C_FCNT_COUNT		0x0000003f

#define I2C_CLOCK_RATE	100000

struct i2c_softc {
	device_t		dev;
	device_t		iicbus;
	struct resource	*res;
	struct mtx		mutex;
	int			rid;
	bus_space_handle_t	bsh;
	bus_space_tag_t	bst;
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

	return (bus_space_read_4(sc->bst, sc->bsh, off));
}

static int
i2c_intr(void *priv)
{

	return 0;
}

static int
i2c_probe(device_t dev)
{
	struct i2c_softc *sc;

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Rockchip rk30xx I2C bus controller");

	return (BUS_PROBE_DEFAULT);
}

static int
i2c_attach(device_t dev)
{
	struct i2c_softc *sc;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->rid = 0;

	mtx_init(&sc->mutex, device_get_nameunit(dev), "rkiic", MTX_DEF);

	sc->res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->rid,
	    RF_ACTIVE);
	if (sc->res == NULL) {
		device_printf(dev, "could not allocate resources");
		mtx_destroy(&sc->mutex);
		return (ENXIO);
	}

	sc->bst = rman_get_bustag(sc->res);
	sc->bsh = rman_get_bushandle(sc->res);

	i2c_write_reg(sc, I2C_CON_REG, 0);
	i2c_write_reg(sc, I2C_IEN_REG, 0);

	sc->iicbus = device_add_child(dev, "iicbus", -1);
	if (sc->iicbus == NULL) {
		device_printf(dev, "could not add iicbus child");
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
