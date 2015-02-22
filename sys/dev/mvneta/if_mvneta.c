#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/endian.h>
#include <sys/mbuf.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/socket.h>
#include <sys/sysctl.h>

#include <net/ethernet.h>
#include <net/bpf.h>
#include <net/if.h>
#include <net/if_var.h>
#include <net/if_arp.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>
#include <net/if_vlan_var.h>

#include <netinet/in_systm.h>
#include <netinet/in.h>
#include <netinet/ip.h>

#include <sys/sockio.h>
#include <sys/bus.h>
#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/mge/if_mvnetavar.h>
#include <arm/mv/mvreg.h>
#include <arm/mv/mvvar.h>

#include "miibus_if.h"

static int mvneta_probe(device_t dev);
static int mvneta_attach(device_t dev);
static int mvneta_detach(device_t dev);
static int mvneta_shutdown(device_t dev);
static int mvneta_suspend(device_t dev);
static int mvneta_resume(device_t dev);

static int mvneta_miibus_readreg(device_t dev, int phy, int reg);
static int mvneta_miibus_writereg(device_t dev, int phy, int reg, int value);

static device_method_t mvneta_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		mvneta_probe),
	DEVMETHOD(device_attach,	mvneta_attach),
	DEVMETHOD(device_detach,	mvneta_detach),
	DEVMETHOD(device_shutdown,	mvneta_shutdown),
	DEVMETHOD(device_suspend,	mvneta_suspend),
	DEVMETHOD(device_resume,	mvneta_resume),
	/* MII interface */
	DEVMETHOD(miibus_readreg,	mvneta_miibus_readreg),
	DEVMETHOD(miibus_writereg,	mvneta_miibus_writereg),
	{ 0, 0 }
};

static driver_t mvneta_driver = {
	"mvneta",
	mvneta_methods,
	sizeof(struct mvneta_softc),
};

static devclass_t mvneta_devclass;

DRIVER_MODULE(mvneta, simplebus, mvneta_driver, mvneta_devclass, 0, 0);
DRIVER_MODULE(miibus, mvneta, miibus_driver, miibus_devclass, 0, 0);
MODULE_DEPEND(mvneta, ether, 1, 1, 1);
MODULE_DEPEND(mvneta, miibus, 1, 1, 1);

static int
mvneta_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "mrvl,ge"))
		return (ENXIO);

	if (!ofw_bus_has_prop(dev, "has-neta"))
		return (ENXIO);

	device_set_desc(dev, "Marvell NETA Gigabit Ethernet controller");
	return (BUS_PROBE_DEFAULT);
}

static int
mvneta_attach(device_t dev)
{
	struct mvneta_softc *sc;

	sc = device_get_softc(dev);
	sc->dev = dev;

	return (0);
}

static int
mvneta_detach(device_t dev)
{
	struct mvneta_softc *sc;

	sc = device_get_softc(dev);

	return (0);
}

static int
mvneta_shutdown(device_t dev)
{
	struct mvneta_softc *sc;

	sc = device_get_softc(dev);

	return (0);
}

static int
mvneta_suspend(device_t dev)
{

	device_printf(dev, "%s\n", __FUNCTION__);
	return (0);
}

static int
mvneta_resume(device_t dev)
{

	device_printf(dev, "%s\n", __FUNCTION__);
	return (0);
}

static int
mvneta_miibus_readreg(device_t dev, int phy, int reg)
{
	struct mvneta_softc *sc;

	sc = device_get_softc(dev);

	return (0);
}

static int
mvneta_miibus_writereg(device_t dev, int phy, int reg, int value)
{
	struct mvneta_softc *sc;

	sc = device_get_softc(dev);

	return (0);
}
