// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026, Juan Schroeder <jcschroeder@gmail.com>
 */

#include <dm.h>
#include <dm/ofnode.h>
#include <env.h>
#include <fdtdec.h>
#include <image.h>
#include <log.h>
#include <spl.h>
#include <init.h>
#include <usb.h>
#include <virtio_types.h>
#include <virtio.h>
#include <configs/openhwgroup_cvwsoc_virt.h>
#include <../common/cvwsoc_common.h>

int board_init(void) {

    return 0;
}


int ft_board_setup(void *fdt, struct bd_info *bd) {
    return 0;
}
