/*
 * wpanusb.h - Definitions shared between kernel and WPANUSB firmware
 *
 * Copyright (C) 2018 Intel Corp.
 *
 * Written by Andrei Emeltchenko <andrei.emeltchenko@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define WPANUSB_VENDOR_ID	0x2fe3
#define WPANUSB_PRODUCT_ID	0x0101

enum wpanusb_requests {
	RESET,
	TX,
	XMIT_ASYNC,
	ED,
	SET_CHANNEL,
	START,
	STOP,
	SET_SHORT_ADDR,
	SET_PAN_ID,
	SET_IEEE_ADDR,
	SET_TXPOWER,
	SET_CCA_MODE,
	SET_CCA_ED_LEVEL,
	SET_CSMA_PARAMS,
	SET_PROMISCUOUS_MODE,
};

struct set_channel {
	__u8 page;
	__u8 channel;
} __packed;

struct set_short_addr {
	__le16 short_addr;
} __packed;

struct set_pan_id {
	__le16 pan_id;
} __packed;

struct set_ieee_addr {
	__le64 ieee_addr;
} __packed;
