/*
 * Copyright (c) 2011 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 */

#include <common.h>
#include <command.h>
#include <part.h>

int do_raw_read(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	char *ep;
	struct blk_desc *dev_desc = NULL;
	int dev;
	int part = 0;
	disk_partition_t part_info;
	ulong offset = 0u;
	ulong limit = 0u;
	void *addr;
	uint blk;
	uint cnt;

	if (argc != 6) {
		cmd_usage(cmdtp);
		return 1;
	}

	dev = (int)simple_strtoul(argv[2], &ep, 16);
	if (*ep) {
		debug("%s: dev:%d arg[1]:%s arg[2]:%s\n", __func__, dev, argv[1], argv[2]);
		if (*ep != ':') {
			debug("%s: Invalid block device %s return 1\n", __func__, argv[2]);
			return 1;
		}
		part = (int)simple_strtoul(++ep, NULL, 16);
	}

	dev_desc = blk_get_dev(argv[1], dev);
	if (dev_desc == NULL) {
		debug("%s: Block device %s %d not supported return 1\n", __func__, argv[1], dev);
		return 1;
	}

	addr = (void *)simple_strtoul(argv[3], NULL, 16);
	blk = simple_strtoul(argv[4], NULL, 16);
	cnt = simple_strtoul(argv[5], NULL, 16);
	if (part != 0) {
		if (part_get_info(dev_desc, part, &part_info)) {
			debug("%s: Cannot find partition %d return 1\n", __func__, part);
			return 1;
		}
		offset = part_info.start;
		limit = part_info.size;
	} else {
		/* Largest address not available in struct blk_desc. */
		limit = ~0;
	}

	if (cnt + blk > limit) {
		debug("%s: Read out of range return 1\n", __func__);
		return 1;
	}

	if (blk_dread(dev_desc, offset + blk, cnt, addr) < 0) {
		debug("%s: Error reading blocks return 1\n", __func__);
		return 1;
	}
	debug("%s: addr:%p part:%u block:%u count:%u\n", __func__, addr, part, (offset + blk), cnt);
	return 0;
}

U_BOOT_CMD(
	read,	6,	0,	do_raw_read,
	"Load binary data from a partition",
	"<interface> <dev[:part]> addr blk# cnt"
);
