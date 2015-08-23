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

static int i2c_probe(device_t d) { return 0; }
static int i2c_attach(device_t d) { return 0; }

static phandle_t i2c_get_node(device_t d, device_t d_) { return 0; }

static int i2c_repeated_start(device_t d, u_char c, int i) { return 0; }
static int i2c_start(device_t d, u_char c, int i) { return 0; }
static int i2c_stop(device_t d) { return 0; }
static int i2c_reset(device_t d, u_char c, u_char c_, u_char *c__) { return 0; }
static int i2c_read(device_t d, char *c, int i, int *j, int k, int m) { return 0; }
static int i2c_write(device_t d, const char *c, int i, int *j, int k) { return 0; }
