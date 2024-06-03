/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2010-2016 Intel Corporation
 */

#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#include <stdarg.h>
#include <uk/list.h>
#include <uk/bus/pci.h>
#include <uk/config.h>
#include <uk/arch/types.h>
#include <uk/plat/lcpu.h>
#include <uk/intctlr.h>
#include <uk/netdev.h>
#include <uk/netdev_core.h>
#include <uk/netdev_driver.h>

#include <e1000/e1000_api.h>
#include <e1000/e1000_ethdev.h>

#define EM_EIAC			0x000DC

/* Utility constants */
#define ETH_LINK_HALF_DUPLEX 0 /**< Half-duplex connection (see link_duplex). */
#define ETH_LINK_FULL_DUPLEX 1 /**< Full-duplex connection (see link_duplex). */
#define ETH_LINK_DOWN        0 /**< Link is down (see link_status). */
#define ETH_LINK_UP          1 /**< Link is up (see link_status). */
#define ETH_LINK_FIXED       0 /**< No autonegotiation (see link_autoneg). */
#define ETH_LINK_AUTONEG     1 /**< Autonegotiated (see link_autoneg). */

#define ETH_SPEED_NUM_NONE         0 /**< Not defined */
#define ETH_LINK_SPEED_FIXED    (1 <<  0)  /**< Disable autoneg (fixed speed) */

#define RTE_ETHER_MAX_LEN   1518  /**< Maximum frame len, including CRC. */

#define PMD_ROUNDUP(x,y)	(((x) + (y) - 1)/(y) * (y))
#define DRIVER_NAME           "e1000"

static struct uk_alloc *a;

#define to_e1000dev(ndev) \
	__containerof(ndev, struct e1000_hw, netdev)

static const char *drv_name = DRIVER_NAME;

static int eth_em_configure(struct uk_netdev *dev, const struct uk_netdev_conf *conf);
static int eth_em_start(struct uk_netdev *dev);
static void eth_em_stop(struct uk_netdev *dev);
// static void eth_em_close(struct pci_device *dev);
static unsigned eth_em_promiscuous_get(struct uk_netdev *dev);
static int eth_em_link_update(struct uk_netdev *dev,
				int wait_to_complete);
static void eth_em_infos_get(struct uk_netdev *dev,
				struct uk_netdev_info *dev_info);
static int eth_em_interrupt_handler(void *param);

static int em_hw_init(struct e1000_hw *hw);
static int em_hardware_init(struct e1000_hw *hw);
static void em_hw_control_acquire(struct e1000_hw *hw);
static void em_hw_control_release(struct e1000_hw *hw);

static uint16_t eth_em_mtu_get(struct uk_netdev *dev);
static int eth_em_mtu_set(struct uk_netdev *dev, uint16_t mtu);

static int eth_em_rx_queue_intr_enable(struct uk_netdev *dev, struct uk_netdev_rx_queue *queue);
static int eth_em_rx_queue_intr_disable(struct uk_netdev *dev, struct uk_netdev_rx_queue *queue);

static int em_get_rx_buffer_size(struct e1000_hw *hw);
static const struct uk_hwaddr * eth_em_default_mac_addr_get(struct uk_netdev *n);
static int eth_em_default_mac_addr_set(struct uk_netdev *dev,
					const struct uk_hwaddr *addr);

#define EM_FC_PAUSE_TIME 0x0680
#define EM_LINK_UPDATE_CHECK_TIMEOUT  90  /* 9s */
#define EM_LINK_UPDATE_CHECK_INTERVAL 100 /* ms */

static enum e1000_fc_mode em_fc_setting = e1000_fc_full;

static const struct uk_netdev_ops eth_em_dev_ops = {
    /** RX queue interrupts. */
	.rxq_intr_enable                = eth_em_rx_queue_intr_enable,  /* optional */
	.rxq_intr_disable               = eth_em_rx_queue_intr_disable, /* optional */

	/** Set/Get hardware address. */
	.hwaddr_get                     = eth_em_default_mac_addr_get,       /* recommended */
	.hwaddr_set                     = eth_em_default_mac_addr_set,       /* optional */

	/** Set/Get MTU. */
	.mtu_get                        = eth_em_mtu_get,
	.mtu_set                        = eth_em_mtu_set,          /* optional */

	/** Promiscuous mode. */
	.promiscuous_get                = eth_em_promiscuous_get,

	/** Device/driver capabilities and info. */
	.info_get                       = eth_em_infos_get,
	.txq_info_get                   = em_txq_info_get,
	.rxq_info_get                   = em_rxq_info_get,
	.einfo_get                      = NULL,        /* optional */

	/** Device life cycle. */
	.configure                      = eth_em_configure,
	.txq_configure                  = eth_em_tx_queue_setup,
	.rxq_configure                  = eth_em_rx_queue_setup,
	.start                          = eth_em_start,
};

static int
eth_em_dev_init(struct pci_device * pci_dev)
{
	int rc = 0;
	struct e1000_hw *hw = NULL;

    UK_ASSERT(pci_dev != NULL);

	hw = uk_calloc(a, sizeof(*hw), 1);
	hw->a = a;
	if (!hw) {
		uk_pr_err("Failed to allocate e1000 device\n");
		return -ENOMEM;
	}

	hw->pdev = pci_dev;
	hw->netdev.ops = &eth_em_dev_ops;
	hw->netdev.rx_one = &eth_em_recv_pkts;
	hw->netdev.tx_one = &eth_em_xmit_pkts;
	hw->hw_addr = (void *)(hw->pdev->bar0 & 0xFFFFFFF0);
	hw->device_id = pci_dev->id.device_id;

	rc = uk_netdev_drv_register(&hw->netdev, a, drv_name);
	if (rc < 0) {
		uk_pr_err("Failed to register virtio-net device with libuknet\n");
		return 0;
	}

	if (e1000_setup_init_funcs(hw, TRUE) != E1000_SUCCESS ||
			em_hw_init(hw) != 0) {
		uk_pr_crit("addr %X %X %X %X %X %X ", hw->mac.addr[0],
			hw->mac.addr[1], hw->mac.addr[2], hw->mac.addr[3],
			hw->mac.addr[4], hw->mac.addr[5]);
		uk_pr_crit("perm_addr %X %X %X %X %X %X ", hw->mac.perm_addr[0],
			hw->mac.perm_addr[1], hw->mac.perm_addr[2], hw->mac.perm_addr[3],
			hw->mac.perm_addr[4], hw->mac.perm_addr[5]);
		uk_pr_err("vendorID=0x%x deviceID=0x%x: "
			"failed to init HW", pci_dev->id.vendor_id,
			pci_dev->id.device_id);
		return -ENODEV;
	}

	rc = uk_intctlr_irq_register(pci_dev->irq, eth_em_interrupt_handler, hw);
	if (rc != 0) {
		uk_pr_err("Failed to register the interrupt\n");
		return rc;
	}

	return 0;
}

static int
em_hw_init(struct e1000_hw *hw)
{
	int diag;
	diag = hw->mac.ops.init_params(hw);
	if (diag != 0) {
		uk_pr_err("MAC Initialization Error\n");
		return diag;
	}
	diag = hw->nvm.ops.init_params(hw);
	if (diag != 0) {
		uk_pr_err("NVM Initialization Error\n");
		return diag;
	}
	diag = hw->phy.ops.init_params(hw);
	if (diag != 0) {
		uk_pr_err("PHY Initialization Error\n");
		return diag;
	}
	(void) e1000_get_bus_info(hw);

	hw->mac.autoneg = 1;
	hw->phy.autoneg_wait_to_complete = 0;
	hw->phy.autoneg_advertised = E1000_ALL_SPEED_DUPLEX;

	/* Copper options */
	if (hw->phy.media_type == e1000_media_type_copper) {
		hw->phy.mdix = 0; /* AUTO_ALL_MODES */
		hw->phy.disable_polarity_correction = 0;
		hw->phy.ms_type = e1000_ms_hw_default;
	}

	/*
	 * Start from a known state, this is important in reading the nvm
	 * and mac from that.
	 */
	e1000_reset_hw(hw);

	/* Make sure we have a good EEPROM before we read from it */
	if (e1000_validate_nvm_checksum(hw) < 0) {
		/*
		 * Some PCI-E parts fail the first check due to
		 * the link being in sleep state, call it again,
		 * if it fails a second time its a real issue.
		 */
		diag = e1000_validate_nvm_checksum(hw);
		if (diag < 0) {
			goto error;
		}
	}

	/* Read the permanent MAC address out of the EEPROM */
	diag = e1000_read_mac_addr(hw);
	if (diag != 0) {
		uk_pr_err("EEPROM error while reading MAC address\n");
		goto error;
	}

	/* Now initialize the hardware */
	diag = em_hardware_init(hw);
	if (diag != 0) {
		uk_pr_err("Hardware initialization failed\n");
		goto error;
	}

	hw->mac.get_link_status = 1;

	/* Indicate SOL/IDER usage */
	diag = e1000_check_reset_block(hw);
	if (diag < 0) {
		uk_pr_err("PHY reset is blocked due to SOL/IDER session\n");
		goto error;
	}

	uint32_t rctl = E1000_READ_REG(hw, E1000_RCTL);
	rctl |= (E1000_RCTL_UPE | E1000_RCTL_MPE);
	E1000_WRITE_REG(hw, E1000_RCTL, rctl);
	rctl = E1000_READ_REG(hw, E1000_RCTL);

	return 0;

error:
	em_hw_control_release(hw);
	return diag;
}

static int
eth_em_configure(__unused struct uk_netdev *dev, __unused const struct uk_netdev_conf *conf)
{
	return 0;
}

static void
em_set_pba(struct e1000_hw *hw)
{
	uint32_t pba;

	/*
	 * Packet Buffer Allocation (PBA)
	 * Writing PBA sets the receive portion of the buffer
	 * the remainder is used for the transmit buffer.
	 * Devices before the 82547 had a Packet Buffer of 64K.
	 * After the 82547 the buffer was reduced to 40K.
	 */
    pba = E1000_PBA_40K; /* 40K for Rx, 24K for Tx */
	
	E1000_WRITE_REG(hw, E1000_PBA, pba);
}

static void
eth_em_rxtx_control(struct uk_netdev *dev,
		    bool enable)
{
	struct e1000_hw *hw =
		to_e1000dev(dev);
	uint32_t tctl, rctl;

	tctl = E1000_READ_REG(hw, E1000_TCTL);
	rctl = E1000_READ_REG(hw, E1000_RCTL);
	if (enable) {
		/* enable Tx/Rx */
		tctl |= E1000_TCTL_EN;
		rctl |= E1000_RCTL_EN;
	} else {
		/* disable Tx/Rx */
		tctl &= ~E1000_TCTL_EN;
		rctl &= ~E1000_RCTL_EN;
	}
	E1000_WRITE_REG(hw, E1000_TCTL, tctl);
	E1000_WRITE_REG(hw, E1000_RCTL, rctl);
	E1000_WRITE_FLUSH(hw);
}

static int
eth_em_start(struct uk_netdev *dev)
{
	struct e1000_hw *hw = to_e1000dev(dev);
	int ret;
	
	eth_em_stop(dev);
	e1000_power_up_phy(hw);
	/* Set default PBA value */
	em_set_pba(hw);
	/* Put the address into the Receive Address Array */
	e1000_rar_set(hw, hw->mac.addr, 0);
	/* Initialize the hardware */
	if (em_hardware_init(hw)) {
		uk_pr_err("Unable to initialize the hardware\n");
		return -EIO;
	}

	E1000_WRITE_REG(hw, E1000_VET, 0x8100);

	eth_em_tx_init(hw);

	ret = eth_em_rx_init(hw);
	if (ret) {
		uk_pr_err("Unable to initialize RX hardware\n");
		em_dev_clear_queues(dev);
		return ret;
	}

	e1000_clear_hw_cntrs_base_generic(hw);

	/* Set Interrupt Throttling Rate to maximum allowed value. */
	E1000_WRITE_REG(hw, E1000_ITR, UINT16_MAX);

	/* Setup link speed and duplex */
	hw->phy.autoneg_advertised = E1000_ALL_SPEED_DUPLEX;
	hw->mac.autoneg = 1;

	e1000_setup_link(hw);
	eth_em_rxtx_control(dev, true);
	eth_em_link_update(dev, 0);

	return 0;
}

/*********************************************************************
 *
 *  This routine disables all traffic on the adapter by issuing a
 *  global reset on the MAC.
 *
 **********************************************************************/
static void
eth_em_stop(__unused struct uk_netdev *dev)
{
	
}

static int
em_get_rx_buffer_size(struct e1000_hw *hw)
{
	uint32_t rx_buf_size;
	rx_buf_size = ((E1000_READ_REG(hw, E1000_PBA) & UINT16_MAX) << 10);

	return rx_buf_size;
}

/*********************************************************************
 *
 *  Initialize the hardware
 *
 **********************************************************************/
static int
em_hardware_init(struct e1000_hw *hw)
{
	uint32_t rx_buf_size;
	int diag;

	/* Issue a global reset */
	e1000_reset_hw(hw);

	/* Let the firmware know the OS is in control */
	em_hw_control_acquire(hw);

	/*
	 * These parameters control the automatic generation (Tx) and
	 * response (Rx) to Ethernet PAUSE frames.
	 * - High water mark should allow for at least two standard size (1518)
	 *   frames to be received after sending an XOFF.
	 * - Low water mark works best when it is very near the high water mark.
	 *   This allows the receiver to restart by sending XON when it has
	 *   drained a bit. Here we use an arbitrary value of 1500 which will
	 *   restart after one full frame is pulled from the buffer. There
	 *   could be several smaller frames in the buffer and if so they will
	 *   not trigger the XON until their total number reduces the buffer
	 *   by 1500.
	 * - The pause time is fairly large at 1000 x 512ns = 512 usec.
	 */
	rx_buf_size = em_get_rx_buffer_size(hw);

	hw->fc.high_water = rx_buf_size -
		PMD_ROUNDUP(RTE_ETHER_MAX_LEN * 2, 1024);
	hw->fc.low_water = hw->fc.high_water - 1500;

	hw->fc.pause_time = EM_FC_PAUSE_TIME;

	hw->fc.send_xon = 1;

	/* Set Flow control, use the tunable location if sane */
	if (em_fc_setting <= e1000_fc_full)
		hw->fc.requested_mode = em_fc_setting;
	else
		hw->fc.requested_mode = e1000_fc_none;

	diag = e1000_init_hw(hw);
	if (diag < 0)
		return diag;
	e1000_check_for_link(hw);
	return 0;
}

static int
eth_em_rx_queue_intr_enable(__unused struct uk_netdev *dev, __unused struct uk_netdev_rx_queue *queue)
{
	return 0;
}

static int
eth_em_rx_queue_intr_disable(__unused struct uk_netdev *dev, __unused struct uk_netdev_rx_queue *queue)
{
	return 0;
}

static void
eth_em_infos_get(struct uk_netdev *dev, struct uk_netdev_info *dev_info)
{

	dev_info->max_rx_queues = 1;
	dev_info->max_tx_queues = 1;

	dev_info->max_mtu = EM_TX_MAX_MTU_SEG;
	dev_info->ioalign = EM_RXD_ALIGN;
}

/* return 0 means link status changed, -1 means not changed */
static int
eth_em_link_update(struct uk_netdev *dev, int wait_to_complete)
{
	struct e1000_hw *hw =
		to_e1000dev(dev);
	struct rte_eth_link link;
	int link_check, count;

	link_check = 0;
	hw->mac.get_link_status = 1;

	/* possible wait-to-complete in up to 9 seconds */
	for (count = 0; count < EM_LINK_UPDATE_CHECK_TIMEOUT; count ++) {
		/* Read the real link status */
		/* Do the work to read phy */
		e1000_check_for_link(hw);
		link_check = !hw->mac.get_link_status;

		if (link_check || wait_to_complete == 0)
			break;
		DELAY(EM_LINK_UPDATE_CHECK_INTERVAL);
	}
	memset(&link, 0, sizeof(link));

	/* Now we check if a transition has happened */
	if (link_check && (link.link_status == ETH_LINK_DOWN)) {
		uint16_t duplex, speed;
		hw->mac.ops.get_link_up_info(hw, &speed, &duplex);
		link.link_duplex = (duplex == FULL_DUPLEX) ?
				ETH_LINK_FULL_DUPLEX :
				ETH_LINK_HALF_DUPLEX;
		link.link_speed = speed;
		link.link_status = ETH_LINK_UP;
		link.link_autoneg = 0;
	} else if (!link_check && (link.link_status == ETH_LINK_UP)) {
		link.link_speed = ETH_SPEED_NUM_NONE;
		link.link_duplex = ETH_LINK_HALF_DUPLEX;
		link.link_status = ETH_LINK_DOWN;
		link.link_autoneg = ETH_LINK_FIXED;
	}

	return 0;
}

/*
 * em_hw_control_acquire sets {CTRL_EXT|FWSM}:DRV_LOAD bit.
 * For ASF and Pass Through versions of f/w this means
 * that the driver is loaded. For AMT version type f/w
 * this means that the network i/f is open.
 */
static void
em_hw_control_acquire(struct e1000_hw *hw)
{
	uint32_t ctrl_ext;

    ctrl_ext = E1000_READ_REG(hw, E1000_CTRL_EXT);
    E1000_WRITE_REG(hw, E1000_CTRL_EXT,
        ctrl_ext | E1000_CTRL_EXT_DRV_LOAD);
}

/*
 * em_hw_control_release resets {CTRL_EXTT|FWSM}:DRV_LOAD bit.
 * For ASF and Pass Through versions of f/w this means that the
 * driver is no longer loaded. For AMT versions of the
 * f/w this means that the network i/f is closed.
 */
static void
em_hw_control_release(struct e1000_hw *hw)
{
	uint32_t ctrl_ext;

	/* Let firmware taken over control of h/w */
    ctrl_ext = E1000_READ_REG(hw, E1000_CTRL_EXT);
    E1000_WRITE_REG(hw, E1000_CTRL_EXT,
        ctrl_ext & ~E1000_CTRL_EXT_DRV_LOAD);
}

static unsigned
eth_em_promiscuous_get(__unused struct uk_netdev *dev) {
	return 0;
}

/**
 * Interrupt handler which shall be registered at first.
 *
 * @param handle
 *  Pointer to interrupt handle.
 * @param param
 *  The address of parameter (struct uk_netdev *) regsitered before.
 *
 * @return
 *  void
 */
static int
eth_em_interrupt_handler(__unused void *param)
{
	return 0;
}

static const struct uk_hwaddr *
eth_em_default_mac_addr_get(struct uk_netdev *n)
{
	struct e1000_hw *d;

	UK_ASSERT(n);
	d = to_e1000dev(n);

	return &d->mac.addr;
}

static int
eth_em_default_mac_addr_set(__unused struct uk_netdev *dev,
			    __unused const struct uk_hwaddr *hwaddr)
{
	return 0;
}

static uint16_t
eth_em_mtu_get(__unused struct uk_netdev *dev)
{
	return EM_TX_MAX_MTU_SEG;
}


static int
eth_em_mtu_set(__unused struct uk_netdev *dev, __unused uint16_t mtu)
{
	return 0;
}

/*
 * The set of PCI devices this driver supports
 */
static const struct pci_device_id e1000_pci_ids[] = {
	{ PCI_DEVICE_ID(E1000_INTEL_VENDOR_ID, E1000_DEV_ID_82545EM_COPPER) },
	{ PCI_DEVICE_ID(E1000_INTEL_VENDOR_ID, E1000_DEV_ID_82545EM_FIBER) },
	{ PCI_ANY_DEVICE_ID },
};


static int e1000_drv_init(struct uk_alloc *allocator)
{
	/* driver initialization */
	if (!allocator)
		return -EINVAL;

	a = allocator;
	return 0;
}

static struct pci_driver e1000_pci_drv = {
	.device_ids = e1000_pci_ids,
	.init = e1000_drv_init,
	.add_dev = eth_em_dev_init
};

PCI_REGISTER_DRIVER(&e1000_pci_drv);
