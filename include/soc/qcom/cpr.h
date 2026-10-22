/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2013-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2019 Linaro Limited
 * Copyright (c) 2021, AngeloGioacchino Del Regno
 *                     <angelogioacchino.delregno@somainline.org>
 */

#ifndef __CPR_H__
#define __CPR_H__

#define CPR_MAX_CORNER_VOLTAGES	40

/**
 * struct cpr_corner_voltage - voltage computed by the CPR for one corner
 * @freq: frequency (Hz) of the CPU OPP this corner corresponds to
 * @uV: open loop target voltage
 * @min_uV: minimum allowed voltage
 * @max_uV: maximum allowed voltage
 *
 * On CPR-Hardened the per-chip voltages are computed while attaching to the
 * power domain, but the CPU OPP table does not exist yet at that point (the
 * OPP core forbids pinning a required_dev once it is populated). The driver
 * therefore publishes them here and the consumer applies them to its OPP
 * table right after installing it.
 */
struct cpr_corner_voltage {
	unsigned long freq;
	int uV;
	int min_uV;
	int max_uV;
};

struct cpr_ext_data {
	int mem_acc_threshold_uV;
	int apm_threshold_uV;
	unsigned int num_corner_voltages;
	struct cpr_corner_voltage corner_voltages[CPR_MAX_CORNER_VOLTAGES];
};

#endif /* __CPR_H__ */
