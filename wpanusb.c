/*
 * wpanusb.c - Driver for the WPANUSB IEEE 802.15.4 dongle
 *
 * Copyright (C) 2018 Intel Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * The driver implements SoftMAC 802.15.4 protocol based on atusb
 * driver for ATUSB IEEE 802.15.4 dongle.
 *
 * Written by Andrei Emeltchenko <andrei.emeltchenko@intel.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/usb.h>
#include <linux/skbuff.h>

#include <net/cfg802154.h>
#include <net/mac802154.h>

#include "wpanusb.h"

#define VERBOSE_DEBUG

#define WPANUSB_NUM_RX_URBS	4	/* allow for a bit of local latency */
#define WPANUSB_ALLOC_DELAY_MS	100	/* delay after failed allocation */

#define VENDOR_OUT		(USB_TYPE_VENDOR | USB_DIR_OUT)
#define VENDOR_IN		(USB_TYPE_VENDOR | USB_DIR_IN)

/* TODO: kernel doc format */

struct wpanusb {
	struct ieee802154_hw *hw;
	struct usb_device *usb_dev;
	int shutdown;			/* non-zero if shutting down */

	/* RX variables */
	struct delayed_work work;	/* memory allocations */
	struct usb_anchor idle_urbs;	/* URBs waiting to be submitted */
	struct usb_anchor rx_urbs;	/* URBs waiting for reception */

	/* TX variables */
	struct usb_ctrlrequest tx_dr;
	struct urb *tx_urb;
	struct sk_buff *tx_skb;
	uint8_t tx_ack_seq;		/* current TX ACK sequence number */
};

/* ----- USB commands without data ----------------------------------------- */

static int wpanusb_control_send(struct wpanusb *wpanusb, unsigned int pipe,
				__u8 request, __u8 requesttype,
				void *data, __u16 size, int timeout)
{
	struct usb_device *usb_dev = wpanusb->usb_dev;
	int ret;

	ret = usb_control_msg(usb_dev, pipe, request, requesttype,
			      0, 0, data, size, timeout);
	if (ret < 0) {
		dev_err(&usb_dev->dev, "%s: failed: req 0x%02x error %d\n",
			__func__, request, ret);
	}

	return ret;
}

/* ----- skb allocation ---------------------------------------------------- */

#define MAX_PSDU	127
#define MAX_RX_XFER	(1 + MAX_PSDU + 2 + 1)	/* PHR+PSDU+CRC+LQI */

#define SKB_WPANUSB(skb)	(*(struct wpanusb **)(skb)->cb)

static void wpanusb_bulk_complete(struct urb *urb);

static int wpanusb_submit_rx_urb(struct wpanusb *wpanusb, struct urb *urb)
{
	struct usb_device *usb_dev = wpanusb->usb_dev;
	struct sk_buff *skb = urb->context;
	int ret;

	if (!skb) {
		skb = alloc_skb(MAX_RX_XFER, GFP_KERNEL);
		if (!skb) {
			dev_warn_ratelimited(&usb_dev->dev,
					     "can't allocate skb\n");
			return -ENOMEM;
		}
		skb_put(skb, MAX_RX_XFER);
		SKB_WPANUSB(skb) = wpanusb;
	}

	usb_fill_bulk_urb(urb, usb_dev, usb_rcvbulkpipe(usb_dev, 1),
			  skb->data, MAX_RX_XFER, wpanusb_bulk_complete, skb);
	usb_anchor_urb(urb, &wpanusb->rx_urbs);

	ret = usb_submit_urb(urb, GFP_KERNEL);
	if (ret) {
		usb_unanchor_urb(urb);
		kfree_skb(skb);
		urb->context = NULL;
	}

	return ret;
}

static void wpanusb_work_urbs(struct work_struct *work)
{
	struct wpanusb *wpanusb =
		container_of(to_delayed_work(work), struct wpanusb, work);
	struct usb_device *usb_dev = wpanusb->usb_dev;
	struct urb *urb;
	int ret;

	if (wpanusb->shutdown)
		return;

	do {
		urb = usb_get_from_anchor(&wpanusb->idle_urbs);
		if (!urb)
			return;
		ret = wpanusb_submit_rx_urb(wpanusb, urb);
	} while (!ret);

	usb_anchor_urb(urb, &wpanusb->idle_urbs);
	dev_warn_ratelimited(&usb_dev->dev, "can't allocate/submit URB (%d)\n",
			     ret);
	schedule_delayed_work(&wpanusb->work,
			      msecs_to_jiffies(WPANUSB_ALLOC_DELAY_MS) + 1);
}

/* ----- Asynchronous USB -------------------------------------------------- */

static void wpanusb_tx_done(struct wpanusb *wpanusb, uint8_t seq)
{
	struct usb_device *usb_dev = wpanusb->usb_dev;
	uint8_t expect = wpanusb->tx_ack_seq;

	dev_dbg(&usb_dev->dev, "seq 0x%02x expect 0x%02x\n", seq, expect);

	if (seq == expect) {
		/* TODO check for ifs handling in firmware */
		ieee802154_xmit_complete(wpanusb->hw, wpanusb->tx_skb, false);
	} else {
		/* TODO I experience this case when wpanusb has a tx complete
		 * irq before probing, we should fix the firmware it's an
		 * unlikely case now that seq == expect is then true, but can
		 * happen and fail with a tx_skb = NULL;
		 */
		dev_dbg(&usb_dev->dev, "unknown ack %u\n", seq);

		ieee802154_wake_queue(wpanusb->hw);
		if (wpanusb->tx_skb)
			dev_kfree_skb_irq(wpanusb->tx_skb);
	}
}

static void wpanusb_process_urb(struct urb *urb)
{
	struct usb_device *usb_dev = urb->dev;
	struct sk_buff *skb = urb->context;
	struct wpanusb *wpanusb = SKB_WPANUSB(skb);
	uint8_t len, lqi;

	if (!urb->actual_length) {
		dev_dbg(&usb_dev->dev, "zero-sized URB ?\n");
		return;
	}

	len = *skb->data;

	dev_dbg(&usb_dev->dev, "urb %p urb len %u pkt len %u", urb,
		urb->actual_length, len);

	/* Handle ACK */
	if (urb->actual_length == 1) {
		wpanusb_tx_done(wpanusb, len);
		return;
	}

	if (len + 1 > urb->actual_length - 1) {
		dev_dbg(&usb_dev->dev, "frame len %d+1 > URB %u-1\n",
			len, urb->actual_length);
		return;
	}

	if (!ieee802154_is_valid_psdu_len(len)) {
		dev_dbg(&usb_dev->dev, "frame corrupted\n");
		return;
	}

	print_hex_dump_bytes("> ", DUMP_PREFIX_OFFSET, skb->data,
			     urb->actual_length);

	/* Get LQI at the end of the packet */
	lqi = skb->data[len + 1];
	dev_dbg(&usb_dev->dev, "rx len %d lqi 0x%02x\n", len, lqi);
	skb_pull(skb, 1);	/* remove length */
	skb_trim(skb, len);	/* remove LQI */
	ieee802154_rx_irqsafe(wpanusb->hw, skb, lqi);
	urb->context = NULL;	/* skb is gone */
}

static void wpanusb_bulk_complete(struct urb *urb)
{
	struct usb_device *usb_dev = urb->dev;
	struct sk_buff *skb = urb->context;
	struct wpanusb *wpanusb = SKB_WPANUSB(skb);

	dev_dbg(&usb_dev->dev, "status %d len %d\n",
		urb->status, urb->actual_length);

	if (urb->status) {
		if (urb->status == -ENOENT) { /* being killed */
			kfree_skb(skb);
			urb->context = NULL;
			return;
		}
		dev_dbg(&usb_dev->dev, "URB error %d\n", urb->status);
	} else {
		wpanusb_process_urb(urb);
	}

	usb_anchor_urb(urb, &wpanusb->idle_urbs);
	if (!wpanusb->shutdown)
		schedule_delayed_work(&wpanusb->work, 0);
}

/* ----- URB allocation/deallocation --------------------------------------- */

static void wpanusb_free_urbs(struct wpanusb *wpanusb)
{
	struct urb *urb;

	do {
		urb = usb_get_from_anchor(&wpanusb->idle_urbs);
		if (!urb)
			break;
		kfree_skb(urb->context);
		usb_free_urb(urb);
	} while (true);
}

static int wpanusb_alloc_urbs(struct wpanusb *wpanusb, unsigned int n)
{
	struct urb *urb;

	while (n--) {
		urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!urb) {
			wpanusb_free_urbs(wpanusb);
			return -ENOMEM;
		}
		usb_anchor_urb(urb, &wpanusb->idle_urbs);
	}

	return 0;
}

/* ----- IEEE 802.15.4 interface operations -------------------------------- */

static void wpanusb_xmit_complete(struct urb *urb)
{
	dev_dbg(&urb->dev->dev, "urb transmit completed");
}

static int wpanusb_xmit(struct ieee802154_hw *hw, struct sk_buff *skb)
{
	struct wpanusb *wpanusb = hw->priv;
	struct usb_device *usb_dev = wpanusb->usb_dev;
	int ret;

	dev_dbg(&usb_dev->dev, "len %u", skb->len);

	/* ack_seq range is 0 - 0xff */
	wpanusb->tx_ack_seq++;
	if (!wpanusb->tx_ack_seq)
		wpanusb->tx_ack_seq++;

	wpanusb->tx_skb = skb;
	wpanusb->tx_dr.wIndex = cpu_to_le16(wpanusb->tx_ack_seq);
	wpanusb->tx_dr.wLength = cpu_to_le16(skb->len);

	usb_fill_control_urb(wpanusb->tx_urb, usb_dev,
			     usb_sndctrlpipe(usb_dev, 0),
			     (unsigned char *)&wpanusb->tx_dr, skb->data,
			     skb->len, wpanusb_xmit_complete, NULL);
	ret = usb_submit_urb(wpanusb->tx_urb, GFP_ATOMIC);

	dev_dbg(&usb_dev->dev, "wpanusb_xmit ret %d len %u seq %u\n", ret,
		skb->len, wpanusb->tx_ack_seq);

	return ret;
}

static int wpanusb_channel(struct ieee802154_hw *hw, u8 page, u8 channel)
{
	struct wpanusb *wpanusb = hw->priv;
	struct usb_device *usb_dev = wpanusb->usb_dev;
	struct set_channel *req;
	int ret;

	req = kmalloc(sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	req->page = page;
	req->channel = channel;

	dev_dbg(&usb_dev->dev, "page %u channel %u", page, channel);

	ret = wpanusb_control_send(wpanusb, usb_sndctrlpipe(usb_dev, 0),
				   SET_CHANNEL, VENDOR_OUT, req, sizeof(*req),
				   1000);
	if (ret < 0) {
		dev_err(&usb_dev->dev, "%s: Failed set channel, ret %d",
			__func__, ret);

		kfree(req);
		return ret;
	}

	kfree(req);

	return 0;
}

static int wpanusb_ed(struct ieee802154_hw *hw, u8 *level)
{
	BUG_ON(!level);

	/* TODO: verify value */
	*level = 0xbe;

	return 0;
}

static int wpanusb_set_hw_addr_filt(struct ieee802154_hw *hw,
				    struct ieee802154_hw_addr_filt *filt,
				    unsigned long changed)
{
	struct wpanusb *wpanusb = hw->priv;
	struct usb_device *usb_dev = wpanusb->usb_dev;
	int ret = 0;

	if (changed & IEEE802154_AFILT_SADDR_CHANGED) {
		struct set_short_addr *req;

		req = kmalloc(sizeof(*req), GFP_KERNEL);
		if (!req)
			return -ENOMEM;

		dev_dbg(&usb_dev->dev, "short addr changed to 0x%04x",
			le16_to_cpu(filt->short_addr));

		req->short_addr = filt->short_addr;

		ret = wpanusb_control_send(wpanusb, usb_sndctrlpipe(usb_dev, 0),
					   SET_SHORT_ADDR, VENDOR_OUT, req,
					   sizeof(*req), 1000);
		if (ret < 0) {
			dev_err(&usb_dev->dev, "%s: Failed to set short_addr",
				__func__);

			kfree(req);
			return ret;
		}

		kfree(req);
	}

	if (changed & IEEE802154_AFILT_PANID_CHANGED) {
		struct set_pan_id *req;

		req = kmalloc(sizeof(*req), GFP_KERNEL);
		if (!req)
			return -ENOMEM;

		dev_dbg(&usb_dev->dev, "pan id changed to 0x%04x",
			le16_to_cpu(filt->pan_id));

		req->pan_id = filt->pan_id;

		ret = wpanusb_control_send(wpanusb, usb_sndctrlpipe(usb_dev, 0),
					   SET_PAN_ID, VENDOR_OUT, req,
					   sizeof(*req), 1000);
		if (ret < 0) {
			dev_err(&usb_dev->dev, "%s: Failed to set pan_id",
				__func__);

			kfree(req);
			return ret;
		}

		kfree(req);
	}

	if (changed & IEEE802154_AFILT_IEEEADDR_CHANGED) {
		struct set_ieee_addr *req;

		req = kmalloc(sizeof(*req), GFP_KERNEL);
		if (!req)
			return -ENOMEM;

		dev_dbg(&usb_dev->dev, "IEEE addr changed");

		memcpy(&req->ieee_addr, &filt->ieee_addr,
		       sizeof(req->ieee_addr));

		ret = wpanusb_control_send(wpanusb, usb_sndctrlpipe(usb_dev, 0),
					   SET_IEEE_ADDR, VENDOR_OUT, req,
					   sizeof(*req), 1000);
		if (ret < 0) {
			dev_err(&usb_dev->dev, "%s: Failed to set ieee_addr",
				__func__);

			kfree(req);
			return ret;
		}

		kfree(req);
	}

	if (changed & IEEE802154_AFILT_PANC_CHANGED) {
		dev_vdbg(&usb_dev->dev, "panc changed");

		dev_err(&usb_dev->dev, "%s: Not handled", __func__);
	}

	return ret;
}

static int wpanusb_start(struct ieee802154_hw *hw)
{
	struct wpanusb *wpanusb = hw->priv;
	struct usb_device *usb_dev = wpanusb->usb_dev;
	int ret;

	schedule_delayed_work(&wpanusb->work, 0);

	ret = wpanusb_control_send(wpanusb, usb_sndctrlpipe(usb_dev, 0),
				   START, VENDOR_OUT, NULL, 0, 1000);
	if (ret < 0)
		usb_kill_anchored_urbs(&wpanusb->idle_urbs);

	return ret;
}

static void wpanusb_stop(struct ieee802154_hw *hw)
{
	struct wpanusb *wpanusb = hw->priv;
	struct usb_device *usb_dev = wpanusb->usb_dev;

	dev_dbg(&usb_dev->dev, "stop");

	usb_kill_anchored_urbs(&wpanusb->idle_urbs);

	wpanusb_control_send(wpanusb, usb_sndctrlpipe(usb_dev, 0),
			     STOP, VENDOR_OUT, NULL, 0, 1000);
}

#define WPANUSB_MAX_TX_POWERS 0xF
static const s32 wpanusb_powers[WPANUSB_MAX_TX_POWERS + 1] = {
	300, 280, 230, 180, 130, 70, 0, -100, -200, -300, -400, -500, -700,
	-900, -1200, -1700,
};

static int wpanusb_set_txpower(struct ieee802154_hw *hw, s32 mbm)
{
	struct wpanusb *wpanusb = hw->priv;
	struct usb_device *usb_dev = wpanusb->usb_dev;

	dev_err(&usb_dev->dev, "%s: Not handled, mbm %d", __func__, mbm);

	/* TODO: */

	return 0;
}

static int wpanusb_set_cca_mode(struct ieee802154_hw *hw,
				const struct wpan_phy_cca *cca)
{
	struct wpanusb *wpanusb = hw->priv;
	struct usb_device *usb_dev = wpanusb->usb_dev;

	dev_err(&usb_dev->dev, "%s: Not handled, mode %u opt %u",
		__func__, cca->mode, cca->opt);

	switch (cca->mode) {
	case NL802154_CCA_ENERGY:
		break;
	case NL802154_CCA_CARRIER:
		break;
	case NL802154_CCA_ENERGY_CARRIER:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int wpanusb_set_cca_ed_level(struct ieee802154_hw *hw, s32 mbm)
{
	struct wpanusb *wpanusb = hw->priv;
	struct usb_device *usb_dev = wpanusb->usb_dev;

	dev_err(&usb_dev->dev, "%s: Not handled, mbm %d", __func__, mbm);

	return 0;
}

static int wpanusb_set_csma_params(struct ieee802154_hw *hw, u8 min_be,
				   u8 max_be, u8 retries)
{
	struct wpanusb *wpanusb = hw->priv;
	struct usb_device *usb_dev = wpanusb->usb_dev;

	dev_err(&usb_dev->dev, "%s: Not handled, min_be %u max_be %u retr %u",
		__func__, min_be, max_be, retries);

	return 0;
}

static int wpanusb_set_promiscuous_mode(struct ieee802154_hw *hw, const bool on)
{
	struct wpanusb *wpanusb = hw->priv;
	struct usb_device *usb_dev = wpanusb->usb_dev;

	dev_err(&usb_dev->dev, "%s: Not handled, on %d", __func__, on);

	return 0;
}

static const struct ieee802154_ops wpanusb_ops = {
	.owner			= THIS_MODULE,
	.xmit_async		= wpanusb_xmit,
	.ed			= wpanusb_ed,
	.set_channel		= wpanusb_channel,
	.start			= wpanusb_start,
	.stop			= wpanusb_stop,
	.set_hw_addr_filt	= wpanusb_set_hw_addr_filt,
	.set_txpower		= wpanusb_set_txpower,
	.set_cca_mode		= wpanusb_set_cca_mode,
	.set_cca_ed_level	= wpanusb_set_cca_ed_level,
	.set_csma_params	= wpanusb_set_csma_params,
	.set_promiscuous_mode	= wpanusb_set_promiscuous_mode,
};

/* ----- Setup ------------------------------------------------------------- */

static int wpanusb_probe(struct usb_interface *interface,
			 const struct usb_device_id *id)
{
	struct usb_device *usb_dev = interface_to_usbdev(interface);
	struct ieee802154_hw *hw;
	struct wpanusb *wpanusb;
	int ret;

	hw = ieee802154_alloc_hw(sizeof(struct wpanusb), &wpanusb_ops);
	if (!hw)
		return -ENOMEM;

	wpanusb = hw->priv;
	wpanusb->hw = hw;
	wpanusb->usb_dev = usb_get_dev(usb_dev);
	usb_set_intfdata(interface, wpanusb);

	wpanusb->shutdown = 0;
	INIT_DELAYED_WORK(&wpanusb->work, wpanusb_work_urbs);
	init_usb_anchor(&wpanusb->idle_urbs);
	init_usb_anchor(&wpanusb->rx_urbs);

	ret = wpanusb_alloc_urbs(wpanusb, WPANUSB_NUM_RX_URBS);
	if (ret)
		goto fail;

	wpanusb->tx_dr.bRequestType = VENDOR_OUT;
	wpanusb->tx_dr.bRequest = TX;
	wpanusb->tx_dr.wValue = cpu_to_le16(0);

	wpanusb->tx_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!wpanusb->tx_urb)
		goto fail;

	hw->parent = &usb_dev->dev;
	hw->flags = IEEE802154_HW_TX_OMIT_CKSUM | IEEE802154_HW_AFILT |
		    IEEE802154_HW_PROMISCUOUS;

	hw->phy->flags = WPAN_PHY_FLAG_TXPOWER;

	hw->phy->current_page = 0;
	hw->phy->current_channel = 11;
	hw->phy->supported.channels[0] = 0x7FFF800;

	hw->phy->supported.tx_powers = wpanusb_powers;
	hw->phy->supported.tx_powers_size = ARRAY_SIZE(wpanusb_powers);
	hw->phy->transmit_power = hw->phy->supported.tx_powers[0];

	ieee802154_random_extended_addr(&hw->phy->perm_extended_addr);

	/* TODO: Do some initialization if needed */
	wpanusb_control_send(wpanusb, usb_sndctrlpipe(usb_dev, 0), RESET,
			     VENDOR_OUT, NULL, 0, 1000);

	ret = ieee802154_register_hw(hw);
	if (ret)
		goto fail;

	return 0;

fail:
	wpanusb_free_urbs(wpanusb);
	usb_kill_urb(wpanusb->tx_urb);
	usb_free_urb(wpanusb->tx_urb);
	usb_put_dev(usb_dev);
	ieee802154_free_hw(hw);

	return ret;
}

static void wpanusb_disconnect(struct usb_interface *interface)
{
	struct wpanusb *wpanusb = usb_get_intfdata(interface);

	wpanusb->shutdown = 1;
	cancel_delayed_work_sync(&wpanusb->work);

	usb_kill_anchored_urbs(&wpanusb->rx_urbs);
	wpanusb_free_urbs(wpanusb);
	usb_kill_urb(wpanusb->tx_urb);
	usb_free_urb(wpanusb->tx_urb);

	ieee802154_unregister_hw(wpanusb->hw);

	ieee802154_free_hw(wpanusb->hw);

	usb_set_intfdata(interface, NULL);
	usb_put_dev(wpanusb->usb_dev);
}

/* The devices we work with */
static const struct usb_device_id wpanusb_device_table[] = {
	{
		.match_flags		= USB_DEVICE_ID_MATCH_DEVICE |
					  USB_DEVICE_ID_MATCH_INT_INFO,
		.idVendor		= WPANUSB_VENDOR_ID,
		.idProduct		= WPANUSB_PRODUCT_ID,
		.bInterfaceClass	= USB_CLASS_VENDOR_SPEC
	},
	/* end with null element */
	{}
};
MODULE_DEVICE_TABLE(usb, wpanusb_device_table);

static struct usb_driver wpanusb_driver = {
	.name		= "wpanusb",
	.probe		= wpanusb_probe,
	.disconnect	= wpanusb_disconnect,
	.id_table	= wpanusb_device_table,
};
module_usb_driver(wpanusb_driver);

MODULE_AUTHOR("Andrei Emeltchenko <andrei.emeltchenko@intel.com>");
MODULE_DESCRIPTION("WPANUSB IEEE 802.15.4 over USB Driver");
MODULE_LICENSE("GPL");
