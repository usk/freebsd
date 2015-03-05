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

#include <dev/mvneta/if_mvnetavar.h>
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

static void mvneta_init(void *arg);
static void mvneta_start(struct ifnet *ifp);
static int mvneta_ioctl(struct ifnet *ifp, u_long command, caddr_t data);

static void mvneta_intr_rxtx(void *arg);
static void mvneta_get_mac_address(struct mvneta_softc *sc, uint8_t *addr);

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

static struct resource_spec res_spec[] = {
	{ SYS_RES_MEMORY, 0, RF_ACTIVE },		/* MVNETA_READ, MVNETA_WRITE */
	{ SYS_RES_IRQ, 0, RF_ACTIVE | RF_SHAREABLE },	/* mvneta_intr_rxtx */
	{ -1, 0 }
};

static struct {
	driver_intr_t *handler;
	char * description;
} mvneta_intrs[MVNETA_INTR_COUNT] = {
	{ mvneta_intr_rxtx,	"GbE aggregated interrupt" },
};

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
	return (BUS_PROBE_SPECIFIC);
}

static int
mvneta_attach(device_t dev)
{
	struct mvneta_softc *sc;
	int phy;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->node = ofw_bus_get_node(dev);

	/* Get phy address and used softc from fdt */
	if (fdt_get_phyaddr(sc->node, sc->dev, &phy, (void **)&sc->phy_sc) != 0)
		return (ENXIO);

	/* Initialize mutexes */
	mtx_init(&sc->tx_lock, device_get_nameunit(dev), "mvneta TX lock", MTX_DEF);
	mtx_init(&sc->rx_lock, device_get_nameunit(dev), "mvneta RX lock", MTX_DEF);

	/* Allocate IO and IRQ resources */
	error = bus_alloc_resources(dev, res_spec, sc->res);
	if (error) {
		device_printf(dev, "could not allocate resources\n");
		mvneta_detach(dev);
		return (ENXIO);
	}

	/* Allocate DMA, buffers, buffer descriptors */
	/* WIP */

	/* Allocate network interface */
	ifp = sc->ifp = if_alloc(IFT_ETHER);
	if (ifp == NULL) {
		device_printf(dev, "if_alloc() failed\n");
		mvneta_detach(dev);
		return (ENOMEM);
	}

	if_initname(ifp, device_get_name(dev), device_get_unit(dev));
	ifp->if_softc = sc;
	ifp->if_flags = IFF_SIMPLEX | IFF_MULTICAST | IFF_BROADCAST;
	ifp->if_capabilities = IFCAP_VLAN_MTU | IFCAP_HWCSUM;
	ifp->if_hwassist = MVNETA_CHECKSUM_FEATURES;
	ifp->if_capenable = ifp->if_capabilities;

	ifp->if_init = mvneta_init;
	ifp->if_start = mvneta_start;
	ifp->if_ioctl = mvneta_ioctl;

	ifp->if_snd.ifq_drv_maxlen = MVNETA_TX_DESC_COUNT - 1;
	IFQ_SET_MAXLEN(&ifp->if_snd, ifp->if_snd.ifq_drv_maxlen);
	IFQ_SET_READY(&ifp->if_snd);

	mvneta_get_mac_address(sc, hwaddr);
	ether_ifattach(ifp, hwaddr);
	callout_init(&sc->wd_callout, 0);

	/* Attach PHY(s) */
	/* WIP */

	/* Tell the MAC where to find the PHY so autoneg works */
	/* WIP */

	/* Attach interrupt handlers */
	for (i = 1; i <= MVNETA_INTR_COUNT; ++i) {
		error = bus_setup_intr(dev, sc->res[i],
		    INTR_TYPE_NET | INTR_MPSAFE,
		    NULL, *mvneta_intrs[(sc->mvneta_intr_cnt == 1 ? 0 : i)].handler,
		    sc, &sc->ih_cookie[i - 1]);
		if (error) {
			device_printf(dev, "could not setup %s\n",
			    mvneta_intrs[(sc->mvneta_intr_cnt == 1 ? 0 : i)].description);
			mvneta_detach(dev);
			return (error);
		}
	}

	return (0);
}

static int
mvneta_detach(device_t dev)
{

	return (0);
}

static int
mvneta_shutdown(device_t dev)
{

	return (0);
}

static int
mvneta_suspend(device_t dev)
{

	return (0);
}

static int
mvneta_resume(device_t dev)
{

	return (0);
}

static int
mvneta_miibus_readreg(device_t dev, int phy, int reg)
{

	return (0);
}

static int
mvneta_miibus_writereg(device_t dev, int phy, int reg, int value)
{

	return (0);
}

static void
mvneta_intr_rxtx(void *arg)
{

	return;
}

static void
mvneta_get_mac_address(struct mvneta_softc *sc, uint8_t *addr)
{

	return;
}
