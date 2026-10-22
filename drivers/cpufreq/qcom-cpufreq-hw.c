// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018, The Linux Foundation. All rights reserved.
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/clk-provider.h>
#include <linux/cpufreq.h>
#include <linux/init.h>
#include <linux/interconnect.h>
#include <linux/iopoll.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/limits.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_opp.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/units.h>

#include <linux/firmware/qcom/qcom_scm.h>
#include <soc/qcom/cpr.h>

#define LUT_MAX_ENTRIES			40U
#define LUT_SRC				GENMASK(31, 30)
#define LUT_L_VAL			GENMASK(7, 0)
#define LUT_CORE_COUNT			GENMASK(18, 16)
#define LUT_VOLT			GENMASK(11, 0)
#define CLK_HW_DIV			2
#define LUT_TURBO_IND			1

#define GT_IRQ_STATUS			BIT(2)

#define MAX_FREQ_DOMAINS		4

#define LUT_SRC_8998			GENMASK(27, 26)
#define LUT_PLL_DIV			GENMASK(25, 24)
#define LUT_VOLT_VC			GENMASK(21, 16)
#define OSM_BOOT_TIME_US		5

#define CYCLE_COUNTER_CLK_RATIO		GENMASK(5, 1)
#define OSM_XO_RATIO_VAL		(10 - 1)
#define CYCLE_COUNTER_USE_XO_EDGE	BIT(8)

#define CC_BOOST_EN			BIT(0)
#define PS_BOOST_EN			BIT(1)
#define DCVS_BOOST_EN			BIT(2)
#define BOOST_TIMER_REG_HI		GENMASK(31, 16)
#define BOOST_TIMER_REG_LO		GENMASK(15, 0)
#define PLL_WAIT_LOCK_TIME_NS		2000
#define SAFE_FREQ_WAIT_NS		1000
#define DEXT_DECREMENT_WAIT_NS		200
#define BOOST_SYNC_DELAY		5

#define HYSTERESIS_UP_MASK		GENMASK(31, 16)
#define HYSTERESIS_DN_MASK		GENMASK(15, 0)
#define HYSTERESIS_CC_NS		200
#define HYSTERESIS_LLM_NS		65535

#define PC_RET_EXIT_DROOP_EN		BIT(3)
#define WFX_DROOP_EN			BIT(4)
#define DCVS_DROOP_EN			BIT(5)
#define DROOP_TIMER1			GENMASK(31, 16)
#define DROOP_TIMER0			GENMASK(15, 0)
#define DROOP_CTRL_VAL			(BIT(3) | BIT(17) | BIT(31))
#define DROOP_TIMER_NS			100
#define DROOP_WAIT_RELEASE_TIMER_NS	50
#define DROOP_RELEASE_TIMER_NS		1
#define PLL_OVERRIDE_DROOP_EN		BIT(0)

#define SEQUENCER_REG(base, n)		(base + (n * 4))
#define SEQ_APM_THRESH_VC		15
#define SEQ_APM_THRESH_PREVC		31
#define SEQ_MEM_ACC_LVAL		32
#define SEQ_MEM_ACC_0			55
#define SEQ_APM_CROSSOVER_VC		72
#define SEQ_APM_PARAM			76
#define SEQ_MEM_ACC_CROSSOVER_VC	88
#define SEQ_MEM_ACC_MAX_LEVELS		4
#define SEQ_MEMACC_REG(base, n)		SEQUENCER_REG(base, SEQ_MEM_ACC_0 + n)

#define ACD_WRITE_CTL_UPDATE_EN		BIT(0)
#define ACD_WRITE_CTL_SELECT_SHIFT	1


struct qcom_cpufreq_soc_setup_data {
	/* OSM physical register offset of the sequencer */
	u16 reg_osm_sequencer;

	/* Frequency domain register offsets */
	u16 reg_override;
	u16 reg_spare;
	u16 reg_cc_zero_behav;
	u16 reg_spm_cc_hyst;
	u16 reg_spm_cc_dcvs_dis;
	u16 reg_spm_core_ret_map;
	u16 reg_llm_freq_vote_hyst;
	u16 reg_llm_volt_vote_hyst;
	u16 reg_llm_intf_dcvs_dis;
	u16 reg_seq1;
	u16 reg_pdn_fsm_ctrl;
	u16 reg_cc_boost_timer;
	u16 reg_dcvs_boost_timer;
	u16 reg_ps_boost_timer;
	u16 boost_timer_reg_len;
	u16 reg_boost_sync_delay;
	u16 reg_droop_ctrl;
	u16 reg_droop_release_ctrl;
	u16 reg_droop_unstall_ctrl;
	u16 reg_droop_wait_release_ctrl;
	u16 reg_droop_timer_ctrl;
	u16 reg_droop_sync_delay;
	u16 reg_pll_override;
	u16 reg_cycle_counter;
};

struct qcom_cpufreq_soc_acd_data {
	u8 tl_delay_reg;
	u8 acd_ctrl_reg;
	u8 softstart_reg;
	u8 ext_intf_reg;
	u8 auto_xfer_reg;
	u8 auto_xfer_cfg_reg;
	u8 auto_xfer_ctl_reg;
	u8 auto_xfer_sts_reg;
	u8 dcvs_sw_reg;
	u8 gfmux_cfg_reg;
	u8 write_ctl_reg;
	u8 write_sts_reg;
	u32 tl_delay_val;
	u32 acd_ctrl_val;
	u32 softstart_val;
	u32 ext_intf0_val;
	u32 ext_intf1_val;
	u32 auto_xfer_val;
};

struct qcom_cpufreq_hw_params {
	u32 freq_lut_val;
	u32 volt_lut_val;
	u32 override_val;
	u32 spare_val;
};

static const struct qcom_cpufreq_soc_setup_data msm8998_setup_regs = {
	/* Physical offset for sequencer SCM calls */
	.reg_osm_sequencer = 0x300,

	/* Frequency domain offsets */
	.reg_override = 0x15c,
	.reg_spare = 0x164,
	.reg_cc_zero_behav = 0x0c,
	.reg_spm_cc_hyst = 0x1c,
	.reg_spm_cc_dcvs_dis = 0x20,
	.reg_spm_core_ret_map = 0x24,
	.reg_llm_freq_vote_hyst = 0x2c,
	.reg_llm_volt_vote_hyst = 0x30,
	.reg_llm_intf_dcvs_dis = 0x34,
	.reg_seq1 = 0x48,
	.reg_pdn_fsm_ctrl = 0x70,
	.reg_cc_boost_timer = 0x74,
	.reg_dcvs_boost_timer = 0x84,
	.reg_ps_boost_timer = 0x94,
	.boost_timer_reg_len = 0x4,
	.reg_boost_sync_delay = 0xa0,
	.reg_droop_ctrl = 0xa4,
	.reg_droop_release_ctrl = 0xa8,
	.reg_droop_unstall_ctrl = 0xac,
	.reg_droop_wait_release_ctrl = 0xb0,
	.reg_droop_timer_ctrl = 0xb8,
	.reg_droop_sync_delay = 0xbc,
	.reg_pll_override = 0xc0,
	.reg_cycle_counter = 0xf00,
};

static const struct qcom_cpufreq_soc_acd_data msm8998_acd_data = {
	.acd_ctrl_reg = 0x4,
	.tl_delay_reg = 0x8,
	.softstart_reg = 0x28,
	.ext_intf_reg = 0x30,
	.dcvs_sw_reg = 0x34,
	.gfmux_cfg_reg = 0x3c,
	.auto_xfer_cfg_reg = 0x80,
	.auto_xfer_reg = 0x84,
	.auto_xfer_ctl_reg = 0x88,
	.auto_xfer_sts_reg = 0x8c,
	.write_ctl_reg = 0x90,
	.write_sts_reg = 0x94,
	.tl_delay_val = 38417,
	.acd_ctrl_val = 0x2b5ffd,
	.softstart_val = 0x501,
	.ext_intf0_val = 0x2cf9ae8,
	.ext_intf1_val = 0x2cf9afe,
	.auto_xfer_val = 0x15,
};

struct qcom_cpufreq_soc_data {
	u32 reg_enable;
	u32 reg_domain_state;
	u32 reg_dcvs_ctrl;
	u32 reg_freq_lut;
	u32 reg_freq_lut_src_mask;
	u32 reg_volt_lut;
	u32 reg_intr_clr;
	u32 reg_current_vote;
	u32 reg_perf_state;
	u32 lut_max_entries;
	u8 lut_row_size;
	u8 clk_hw_div;
	bool uses_tz;
	const struct qcom_cpufreq_soc_setup_data *setup_regs;
	const struct qcom_cpufreq_soc_acd_data *acd_data;
};

struct qcom_cpufreq_data {
	void __iomem *base;
	const struct qcom_cpufreq_soc_data *soc_data;

	/*
	 * Mutex to synchronize between de-init sequence and re-starting LMh
	 * polling/interrupts
	 */
	struct mutex throttle_lock;
	int throttle_irq;
	char irq_name[15];
	bool cancel_throttle;
	struct delayed_work throttle_work;
	struct cpufreq_policy *policy;
	struct clk_hw cpu_clk;

	bool per_core_dcvs;
};

static struct {
	struct qcom_cpufreq_data *data;
	const struct qcom_cpufreq_soc_data *soc_data;
} qcom_cpufreq;

static unsigned long cpu_hw_rate, xo_rate;
static bool icc_scaling_enabled;

static int qcom_cpufreq_set_bw(struct cpufreq_policy *policy,
			       unsigned long freq_khz)
{
	unsigned long freq_hz = freq_khz * 1000;
	struct dev_pm_opp *opp;
	struct device *dev;
	int ret;

	dev = get_cpu_device(policy->cpu);
	if (!dev)
		return -ENODEV;

	opp = dev_pm_opp_find_freq_exact(dev, freq_hz, true);
	if (IS_ERR(opp))
		return PTR_ERR(opp);

	ret = dev_pm_opp_set_opp(dev, opp);
	dev_pm_opp_put(opp);
	return ret;
}

static int qcom_cpufreq_update_opp(struct device *cpu_dev,
				   unsigned long freq_khz,
				   unsigned long volt)
{
	unsigned long freq_hz = freq_khz * 1000;
	int ret;

	/* Skip voltage update if the opp table is not available */
	if (!icc_scaling_enabled)
		return dev_pm_opp_add(cpu_dev, freq_hz, volt);

	ret = dev_pm_opp_adjust_voltage(cpu_dev, freq_hz, volt, volt, volt);
	if (ret) {
		dev_err(cpu_dev, "Voltage update failed freq=%ld\n", freq_khz);
		return ret;
	}

	return dev_pm_opp_enable(cpu_dev, freq_hz);
}

static int qcom_cpufreq_hw_target_index(struct cpufreq_policy *policy,
					unsigned int index)
{
	struct qcom_cpufreq_data *data = policy->driver_data;
	const struct qcom_cpufreq_soc_data *soc_data = qcom_cpufreq.soc_data;
	unsigned long freq = policy->freq_table[index].frequency;
	unsigned int i;

	writel_relaxed(index, data->base + soc_data->reg_perf_state);

	if (data->per_core_dcvs)
		for (i = 1; i < cpumask_weight(policy->related_cpus); i++)
			writel_relaxed(index, data->base + soc_data->reg_perf_state + i * 4);

	if (icc_scaling_enabled)
		qcom_cpufreq_set_bw(policy, freq);

	return 0;
}

static unsigned long qcom_lmh_get_throttle_freq(struct qcom_cpufreq_data *data)
{
	unsigned int lval;

	if (qcom_cpufreq.soc_data->reg_current_vote)
		lval = readl_relaxed(data->base + qcom_cpufreq.soc_data->reg_current_vote) & 0x3ff;
	else
		lval = readl_relaxed(data->base + qcom_cpufreq.soc_data->reg_domain_state) & 0xff;

	return lval * xo_rate;
}

/* Get the frequency requested by the cpufreq core for the CPU */
static unsigned int qcom_cpufreq_get_freq(struct cpufreq_policy *policy)
{
	struct qcom_cpufreq_data *data;
	const struct qcom_cpufreq_soc_data *soc_data;
	unsigned int index;

	if (!policy)
		return 0;

	data = policy->driver_data;
	soc_data = qcom_cpufreq.soc_data;

	index = readl_relaxed(data->base + soc_data->reg_perf_state);
	index = min(index, soc_data->lut_max_entries - 1);

	return policy->freq_table[index].frequency;
}

static unsigned int __qcom_cpufreq_hw_get(struct cpufreq_policy *policy)
{
	struct qcom_cpufreq_data *data;

	if (!policy)
		return 0;

	data = policy->driver_data;

	if (data->throttle_irq >= 0)
		return qcom_lmh_get_throttle_freq(data) / HZ_PER_KHZ;

	return qcom_cpufreq_get_freq(policy);
}

static unsigned int qcom_cpufreq_hw_get(unsigned int cpu)
{
	return __qcom_cpufreq_hw_get(cpufreq_cpu_get_raw(cpu));
}

static unsigned int qcom_cpufreq_hw_fast_switch(struct cpufreq_policy *policy,
						unsigned int target_freq)
{
	struct qcom_cpufreq_data *data = policy->driver_data;
	const struct qcom_cpufreq_soc_data *soc_data = qcom_cpufreq.soc_data;
	unsigned int index;
	unsigned int i;

	index = policy->cached_resolved_idx;
	writel_relaxed(index, data->base + soc_data->reg_perf_state);

	if (data->per_core_dcvs)
		for (i = 1; i < cpumask_weight(policy->related_cpus); i++)
			writel_relaxed(index, data->base + soc_data->reg_perf_state + i * 4);

	return policy->freq_table[index].frequency;
}

static int qcom_cpufreq_hw_read_lut(struct device *cpu_dev,
				    struct cpufreq_policy *policy)
{
	u32 data, src, lval, i, core_count, prev_freq = 0, freq;
	u32 volt;
	struct cpufreq_frequency_table	*table;
	struct dev_pm_opp *opp;
	unsigned long rate;
	int ret;
	struct qcom_cpufreq_data *drv_data = policy->driver_data;
	const struct qcom_cpufreq_soc_data *soc_data = qcom_cpufreq.soc_data;

	table = kzalloc_objs(*table, soc_data->lut_max_entries + 1);
	if (!table)
		return -ENOMEM;

	ret = dev_pm_opp_of_add_table(cpu_dev);
	if (!ret) {
		/* Disable all opps and cross-validate against LUT later */
		icc_scaling_enabled = true;
		for (rate = 0; ; rate++) {
			opp = dev_pm_opp_find_freq_ceil(cpu_dev, &rate);
			if (IS_ERR(opp))
				break;

			dev_pm_opp_put(opp);
			dev_pm_opp_disable(cpu_dev, rate);
		}
	} else if (ret != -ENODEV) {
		dev_err(cpu_dev, "Invalid opp table in device tree\n");
		kfree(table);
		return ret;
	} else {
		policy->fast_switch_possible = true;
		icc_scaling_enabled = false;
	}

	for (i = 0; i < soc_data->lut_max_entries; i++) {
		u32 src_mask = soc_data->reg_freq_lut_src_mask ?: (u32)LUT_SRC;

		data = readl_relaxed(drv_data->base + soc_data->reg_freq_lut +
				      i * soc_data->lut_row_size);
		src = (data & src_mask) >> __ffs(src_mask);
		lval = FIELD_GET(LUT_L_VAL, data);
		core_count = FIELD_GET(LUT_CORE_COUNT, data);

		data = readl_relaxed(drv_data->base + soc_data->reg_volt_lut +
				      i * soc_data->lut_row_size);
		volt = FIELD_GET(LUT_VOLT, data) * 1000;

		if (src)
			freq = xo_rate * lval / 1000;
		else
			freq = cpu_hw_rate / 1000;

		/*
		 * Sanity check: a mis-decoded LUT (wrong stride/mask) would make
		 * us program bogus performance states.  Refuse to register then.
		 */
		if (freq < 100000 || freq > 3200000) {
			dev_err(cpu_dev, "LUT entry %u decodes to bogus %u kHz, aborting\n",
				i, freq);
			kfree(table);
			return -EINVAL;
		}

		if (freq != prev_freq && core_count != LUT_TURBO_IND) {
			if (!qcom_cpufreq_update_opp(cpu_dev, freq, volt)) {
				table[i].frequency = freq;
				dev_dbg(cpu_dev, "index=%d freq=%d, core_count %d\n", i,
				freq, core_count);
			} else {
				dev_warn(cpu_dev, "failed to update OPP for freq=%d\n", freq);
				table[i].frequency = CPUFREQ_ENTRY_INVALID;
			}

		} else if (core_count == LUT_TURBO_IND) {
			table[i].frequency = CPUFREQ_ENTRY_INVALID;
		}

		/*
		 * Two of the same frequencies with the same core counts means
		 * end of table
		 */
		if (i > 0 && prev_freq == freq) {
			struct cpufreq_frequency_table *prev = &table[i - 1];

			/*
			 * Only treat the last frequency that might be a boost
			 * as the boost frequency
			 */
			if (prev->frequency == CPUFREQ_ENTRY_INVALID) {
				if (!qcom_cpufreq_update_opp(cpu_dev, prev_freq, volt)) {
					prev->frequency = prev_freq;
					prev->flags = CPUFREQ_BOOST_FREQ;
				} else {
					dev_warn(cpu_dev, "failed to update OPP for freq=%d\n",
						 freq);
				}
			}

			break;
		}

		prev_freq = freq;
	}

	table[i].frequency = CPUFREQ_TABLE_END;
	policy->freq_table = table;
	dev_pm_opp_set_sharing_cpus(cpu_dev, policy->cpus);

	return 0;
}

static void qcom_get_related_cpus(int index, struct cpumask *m)
{
	struct device_node *cpu_np;
	struct of_phandle_args args;
	int cpu, ret;

	for_each_present_cpu(cpu) {
		cpu_np = of_cpu_device_node_get(cpu);
		if (!cpu_np)
			continue;

		ret = of_parse_phandle_with_args(cpu_np, "qcom,freq-domain",
						 "#freq-domain-cells", 0,
						 &args);
		of_node_put(cpu_np);
		if (ret < 0)
			continue;

		if (index == args.args[0])
			cpumask_set_cpu(cpu, m);
	}
}

static void qcom_lmh_dcvs_notify(struct qcom_cpufreq_data *data)
{
	struct cpufreq_policy *policy = data->policy;
	int cpu = cpumask_first(policy->related_cpus);
	struct device *dev = get_cpu_device(cpu);
	unsigned long freq_hz, throttled_freq;
	struct dev_pm_opp *opp;

	/*
	 * Get the h/w throttled frequency, normalize it using the
	 * registered opp table and use it to calculate thermal pressure.
	 */
	freq_hz = qcom_lmh_get_throttle_freq(data);

	opp = dev_pm_opp_find_freq_floor(dev, &freq_hz);
	if (IS_ERR(opp) && PTR_ERR(opp) == -ERANGE)
		opp = dev_pm_opp_find_freq_ceil(dev, &freq_hz);

	if (IS_ERR(opp)) {
		dev_warn(dev, "Can't find the OPP for throttling: %pe!\n", opp);
	} else {
		dev_pm_opp_put(opp);
	}

	throttled_freq = freq_hz / HZ_PER_KHZ;

	/* Update HW pressure (the boost frequencies are accepted) */
	arch_update_hw_pressure(policy->related_cpus, throttled_freq);

	/*
	 * In the unlikely case policy is unregistered do not enable
	 * polling or h/w interrupt
	 */
	mutex_lock(&data->throttle_lock);
	if (data->cancel_throttle)
		goto out;

	/*
	 * If h/w throttled frequency is higher than what cpufreq has requested
	 * for, then stop polling and switch back to interrupt mechanism.
	 */
	if (throttled_freq >= qcom_cpufreq_get_freq(cpufreq_cpu_get_raw(cpu)))
		enable_irq(data->throttle_irq);
	else
		mod_delayed_work(system_highpri_wq, &data->throttle_work,
				 msecs_to_jiffies(10));

out:
	mutex_unlock(&data->throttle_lock);
}

static void qcom_lmh_dcvs_poll(struct work_struct *work)
{
	struct qcom_cpufreq_data *data;

	data = container_of(work, struct qcom_cpufreq_data, throttle_work.work);
	qcom_lmh_dcvs_notify(data);
}

static irqreturn_t qcom_lmh_dcvs_handle_irq(int irq, void *data)
{
	struct qcom_cpufreq_data *c_data = data;

	/* Disable interrupt and enable polling */
	disable_irq_nosync(c_data->throttle_irq);
	schedule_delayed_work(&c_data->throttle_work, 0);

	if (qcom_cpufreq.soc_data->reg_intr_clr)
		writel_relaxed(GT_IRQ_STATUS,
			       c_data->base + qcom_cpufreq.soc_data->reg_intr_clr);

	return IRQ_HANDLED;
}

static const struct qcom_cpufreq_soc_data qcom_soc_data = {
	.reg_enable = 0x0,
	.reg_dcvs_ctrl = 0xbc,
	.reg_freq_lut = 0x110,
	.reg_volt_lut = 0x114,
	.reg_current_vote = 0x704,
	.reg_perf_state = 0x920,
	.lut_row_size = 32,
	.lut_max_entries = LUT_MAX_ENTRIES,
};

static const struct qcom_cpufreq_soc_data epss_soc_data = {
	.reg_enable = 0x0,
	.reg_domain_state = 0x20,
	.reg_dcvs_ctrl = 0xb0,
	.reg_freq_lut = 0x100,
	.reg_volt_lut = 0x200,
	.reg_intr_clr = 0x308,
	.reg_perf_state = 0x320,
	.lut_row_size = 4,
	.lut_max_entries = LUT_MAX_ENTRIES,
};

static const struct qcom_cpufreq_soc_data shikra_epss_soc_data = {
	.reg_enable = 0x0,
	.reg_domain_state = 0x20,
	.reg_dcvs_ctrl = 0xb0,
	.reg_freq_lut = 0x100,
	.reg_volt_lut = 0x200,
	.reg_intr_clr = 0x308,
	.reg_perf_state = 0x320,
	.lut_row_size = 4,
	.lut_max_entries = 12,
};

/*
 * MSM8998: one OSM instance per cluster.  All registers the driver needs
 * (enable, frequency/voltage LUT at +0x154/+0x158 and the performance state
 * at +0xf10) live in the per-cluster "freq-domain" block.
 */
static const struct qcom_cpufreq_soc_data msm8998_soc_data = {
	.reg_enable = 0x4,
	.reg_freq_lut = 0x154,
	.reg_freq_lut_src_mask = GENMASK(27, 26),
	.reg_volt_lut = 0x158,
	.reg_perf_state = 0xf10,
	.lut_row_size = 32,
	.lut_max_entries = LUT_MAX_ENTRIES,
	.clk_hw_div = 1,
	.uses_tz = false,
	.setup_regs = &msm8998_setup_regs,
	.acd_data = &msm8998_acd_data,
};

static const struct of_device_id qcom_cpufreq_hw_match[] = {
	{ .compatible = "qcom,msm8998-cpufreq-hw", .data = &msm8998_soc_data },
	{ .compatible = "qcom,cpufreq-hw", .data = &qcom_soc_data },
	{ .compatible = "qcom,cpufreq-epss", .data = &epss_soc_data },
	{ .compatible = "qcom,shikra-epss", .data = &shikra_epss_soc_data },
	{}
};
MODULE_DEVICE_TABLE(of, qcom_cpufreq_hw_match);

static int qcom_cpufreq_hw_lmh_init(struct cpufreq_policy *policy, int index)
{
	struct qcom_cpufreq_data *data = policy->driver_data;
	struct platform_device *pdev = cpufreq_get_driver_data();
	int ret;

	/*
	 * Look for LMh interrupt. If no interrupt line is specified /
	 * if there is an error, allow cpufreq to be enabled as usual.
	 */
	data->throttle_irq = platform_get_irq_optional(pdev, index);
	if (data->throttle_irq == -ENXIO)
		return 0;
	if (data->throttle_irq < 0)
		return data->throttle_irq;

	data->cancel_throttle = false;

	mutex_init(&data->throttle_lock);
	INIT_DEFERRABLE_WORK(&data->throttle_work, qcom_lmh_dcvs_poll);

	snprintf(data->irq_name, sizeof(data->irq_name), "dcvsh-irq-%u", policy->cpu);
	ret = request_threaded_irq(data->throttle_irq, NULL, qcom_lmh_dcvs_handle_irq,
				   IRQF_ONESHOT | IRQF_NO_AUTOEN, data->irq_name, data);
	if (ret) {
		dev_err(&pdev->dev, "Error registering %s: %d\n", data->irq_name, ret);
		return 0;
	}

	ret = irq_set_affinity_and_hint(data->throttle_irq, policy->cpus);
	if (ret)
		dev_err(&pdev->dev, "Failed to set CPU affinity of %s[%d]\n",
			data->irq_name, data->throttle_irq);

	return 0;
}

static int qcom_cpufreq_hw_cpu_online(struct cpufreq_policy *policy)
{
	struct qcom_cpufreq_data *data = policy->driver_data;
	struct platform_device *pdev = cpufreq_get_driver_data();
	int ret;

	if (data->throttle_irq <= 0)
		return 0;

	mutex_lock(&data->throttle_lock);
	data->cancel_throttle = false;
	mutex_unlock(&data->throttle_lock);

	ret = irq_set_affinity_and_hint(data->throttle_irq, policy->cpus);
	if (ret)
		dev_err(&pdev->dev, "Failed to set CPU affinity of %s[%d]\n",
			data->irq_name, data->throttle_irq);

	return ret;
}

static int qcom_cpufreq_hw_cpu_offline(struct cpufreq_policy *policy)
{
	struct qcom_cpufreq_data *data = policy->driver_data;

	if (data->throttle_irq <= 0)
		return 0;

	mutex_lock(&data->throttle_lock);
	data->cancel_throttle = true;
	mutex_unlock(&data->throttle_lock);

	cancel_delayed_work_sync(&data->throttle_work);
	irq_set_affinity_and_hint(data->throttle_irq, NULL);
	disable_irq_nosync(data->throttle_irq);

	return 0;
}

static void qcom_cpufreq_hw_lmh_exit(struct qcom_cpufreq_data *data)
{
	if (data->throttle_irq <= 0)
		return;

	free_irq(data->throttle_irq, data);
}

/**
 * qcom_cpufreq_gen_params() - Build the OSM lookup table parameters
 * @cpu_dev:     CPU device
 * @data:        SoC specific register offsets
 * @hw_tbl:      Pointer to return the array of parameters
 * @apm_vc:      APM virtual corner crossover number
 * @acc_vc:      MEM-ACC virtual corner crossover number
 * @cpu_count:   Number of CPUs in the frequency domain
 * @num_entries: Number of filled elements in the returned table
 *
 * Attaching to the CPR-hardened power domain makes the CPR driver compute
 * the per-corner voltages from the chip fuses and adjust the CPU OPP table
 * accordingly; those voltages are then programmed into the OSM.
 */
static int qcom_cpufreq_gen_params(struct device *cpu_dev,
				   struct qcom_cpufreq_data *data,
				   struct qcom_cpufreq_hw_params **hw_tbl,
				   int *apm_vc, int *acc_vc, int cpu_count,
				   u8 *num_entries)
{
	struct platform_device *pdev = cpufreq_get_driver_data();
	const struct qcom_cpufreq_soc_data *soc_data = data->soc_data;
	struct cpr_ext_data *cpr_data;
	struct dev_pm_opp *genpd_opp;
	struct device *vdev;
	unsigned long rate, millivolts;
	int apm_uV, acc_uV, i, gpd_opp_cnt, ret = 0;

	vdev = dev_pm_domain_attach_by_name(cpu_dev, "cprh");
	if (IS_ERR(vdev))
		return dev_err_probe(&pdev->dev, PTR_ERR(vdev),
				     "Could not attach to CPRh power domain\n");

	/* The CPR driver stores its thresholds in the CPU device drvdata */
	cpr_data = dev_get_drvdata(cpu_dev);
	if (!cpr_data)
		return -ENODATA;

	gpd_opp_cnt = dev_pm_opp_get_opp_count(cpu_dev);
	if (gpd_opp_cnt < 2) {
		ret = gpd_opp_cnt > 0 ? -EINVAL : gpd_opp_cnt;
		return ret;
	}

	apm_uV = cpr_data->apm_threshold_uV;
	if (apm_uV <= 0) {
		dev_err(&pdev->dev, "No APM threshold, refusing to program the OSM\n");
		return -EINVAL;
	}

	*apm_vc = -1;

	if (cpr_data->mem_acc_threshold_uV <= 0) {
		acc_uV = INT_MAX;
		*acc_vc = U8_MAX;
	} else {
		acc_uV = cpr_data->mem_acc_threshold_uV;
		*acc_vc = -1;
	}

	*hw_tbl = devm_kmalloc_array(&pdev->dev, gpd_opp_cnt,
				     sizeof(**hw_tbl), GFP_KERNEL);
	if (!*hw_tbl)
		return -ENOMEM;

	for (i = 0, rate = 1000; i <= gpd_opp_cnt; rate++, i++) {
		struct qcom_cpufreq_hw_params *entry = *hw_tbl + i;
		struct device_node *np;
		u32 pll_div, f_src;

		genpd_opp = dev_pm_opp_find_freq_ceil(cpu_dev, &rate);
		if (IS_ERR(genpd_opp))
			break;

		np = dev_pm_opp_get_of_node(genpd_opp);
		if (!np) {
			ret = -ENOENT;
			break;
		}

		ret = of_property_read_u32(np, "qcom,pll-override",
					   &entry->override_val);
		if (ret) {
			of_node_put(np);
			break;
		}

		if (of_property_read_u32(np, "qcom,spare-data", &entry->spare_val))
			entry->spare_val = 0;
		if (of_property_read_u32(np, "qcom,pll-div", &pll_div))
			pll_div = 0;
		of_node_put(np);

		millivolts = dev_pm_opp_get_voltage(genpd_opp);
		if (millivolts >= apm_uV && *apm_vc < 0)
			*apm_vc = i;
		if (millivolts >= acc_uV && *acc_vc < 0)
			*acc_vc = i;

		millivolts /= 1000;
		if (millivolts < 150 || millivolts > 1400) {
			dev_err(&pdev->dev, "Read invalid voltage: %lu\n",
				millivolts);
			ret = -EINVAL;
			break;
		}

		/* Virtual corner levels start from 0 in the OSM firmware */
		entry->volt_lut_val = FIELD_PREP(LUT_VOLT_VC, i);
		entry->volt_lut_val |= FIELD_PREP(LUT_VOLT, millivolts);

		/* Only the first frequency uses the alternate clock source */
		f_src = i ? 1 : 0;
		f_src <<= ffs(soc_data->reg_freq_lut_src_mask) - 1;
		entry->freq_lut_val = f_src | div_u64(rate, xo_rate);
		entry->freq_lut_val |= FIELD_PREP(LUT_CORE_COUNT, cpu_count);
		entry->freq_lut_val |= FIELD_PREP(LUT_PLL_DIV, pll_div);

		dev_dbg(&pdev->dev, "[%d] freq=0x%x volt=0x%x override=0x%x spare=0x%x\n",
			i, entry->freq_lut_val, entry->volt_lut_val,
			entry->override_val, entry->spare_val);

		dev_pm_opp_put(genpd_opp);
	}

	/* Keep the power domain attached for the lifetime of the system */
	if (ret || i < gpd_opp_cnt) {
		dev_err(&pdev->dev, "Got bad OPP table from power domain\n");
		return ret ? ret : -EINVAL;
	}

	*num_entries = i;
	return 0;
}

static inline u32 qcom_cpufreq_acd_regbit(u8 acd_reg_offset)
{
	return BIT(acd_reg_offset / 4);
}

static int qcom_cpufreq_hw_acd_write_autoxfer(struct qcom_cpufreq_data *data,
					      void __iomem *acd_base, u32 val)
{
	const struct qcom_cpufreq_soc_acd_data *aregs = data->soc_data->acd_data;
	u32 regval = 0;

	writel(val, acd_base + aregs->auto_xfer_cfg_reg);

	/* (Clear, then) Set AUTOXFER START */
	writel(0, acd_base + aregs->auto_xfer_reg);
	writel(1, acd_base + aregs->auto_xfer_reg);

	/* Poll for status: if the first bit is set the transfer is done */
	return readl_poll_timeout(acd_base + aregs->auto_xfer_sts_reg, regval,
				  regval & BIT(0), 1, 3);
}

static int qcom_cpufreq_hw_acd_write_xfer(struct qcom_cpufreq_data *data,
					  void __iomem *acd_base, u8 reg,
					  u32 val)
{
	const struct qcom_cpufreq_soc_acd_data *aregs = data->soc_data->acd_data;
	u32 regval = 0;

	writel(val, acd_base + reg);

	/* Clear write control register */
	writel(0, acd_base + aregs->write_ctl_reg);

	regval = (reg / 4) << ACD_WRITE_CTL_SELECT_SHIFT;
	regval |= ACD_WRITE_CTL_UPDATE_EN;
	writel(regval, acd_base + aregs->write_ctl_reg);

	/* Wait until ACD Local Transfer is done */
	return readl_poll_timeout(acd_base + aregs->write_sts_reg, regval,
				  regval & qcom_cpufreq_acd_regbit(reg), 1, 3);
}

static int qcom_cpufreq_hw_acd_init(struct device *cpu_dev,
				    struct cpufreq_policy *policy, int index)
{
	struct platform_device *pdev = cpufreq_get_driver_data();
	struct qcom_cpufreq_data *ddata = policy->driver_data;
	const struct qcom_cpufreq_soc_acd_data *aregs = ddata->soc_data->acd_data;
	char acd_resname[] = "osm-acdX";
	void __iomem *acd_base;
	u32 rmask;
	int ret;

	snprintf(acd_resname, sizeof(acd_resname), "osm-acd%d", index);

	acd_base = devm_platform_ioremap_resource_byname(pdev, acd_resname);
	if (IS_ERR(acd_base)) {
		dev_dbg(cpu_dev, "Skipping ACD initialization\n");
		return 0;
	}

	writel(aregs->tl_delay_val, acd_base + aregs->tl_delay_reg);
	writel(aregs->acd_ctrl_val, acd_base + aregs->acd_ctrl_reg);
	writel(aregs->softstart_val, acd_base + aregs->softstart_reg);
	writel(aregs->ext_intf0_val, acd_base + aregs->ext_intf_reg);
	writel(aregs->auto_xfer_val, acd_base + aregs->auto_xfer_ctl_reg);

	rmask = qcom_cpufreq_acd_regbit(aregs->acd_ctrl_reg) |
		qcom_cpufreq_acd_regbit(aregs->tl_delay_reg) |
		qcom_cpufreq_acd_regbit(aregs->softstart_reg) |
		qcom_cpufreq_acd_regbit(aregs->ext_intf_reg);
	ret = qcom_cpufreq_hw_acd_write_autoxfer(ddata, acd_base, rmask);
	if (ret)
		return ret;

	/* Switch CPUSS clock source to ACD clock */
	ret = qcom_cpufreq_hw_acd_write_xfer(ddata, acd_base,
					     aregs->gfmux_cfg_reg, 1);
	if (ret)
		return ret;

	/* (Set, then) Clear DCVS_SW */
	ret = qcom_cpufreq_hw_acd_write_xfer(ddata, acd_base,
					     aregs->dcvs_sw_reg, 1);
	if (ret)
		return ret;
	ret = qcom_cpufreq_hw_acd_write_xfer(ddata, acd_base,
					     aregs->dcvs_sw_reg, 0);
	if (ret)
		return ret;

	/* Wait for clock switch time */
	udelay(1);

	/* Program the final ACD external interface */
	ret = qcom_cpufreq_hw_acd_write_xfer(ddata, acd_base,
					     aregs->ext_intf_reg,
					     aregs->ext_intf1_val);
	if (ret)
		return ret;

	/* Initiate transfer of the final ACD value */
	rmask |= qcom_cpufreq_acd_regbit(aregs->gfmux_cfg_reg);
	writel(rmask, acd_base + aregs->auto_xfer_cfg_reg);

	/* Wait for ACD to stabilize */
	udelay(OSM_BOOT_TIME_US);
	return 0;
}

/**
 * qcom_cpufreq_hw_write_lut() - Program the OSM lookup table
 */
static int qcom_cpufreq_hw_write_lut(struct device *cpu_dev,
				     struct cpufreq_policy *policy,
				     int cpu_count, int index)
{
	struct platform_device *pdev = cpufreq_get_driver_data();
	struct qcom_cpufreq_data *ddata = policy->driver_data;
	const struct qcom_cpufreq_soc_data *sdata = ddata->soc_data;
	const struct qcom_cpufreq_soc_setup_data *sregs = sdata->setup_regs;
	struct qcom_cpufreq_hw_params *hw_tbl;
	struct resource *osm_rsrc;
	char osm_resname[] = "osm-domainX";
	u32 seq_addr, sreg, acc_lval = 0, last_spare = 1;
	u8 num_entries = 0;
	int apm_vc = INT_MAX, acc_vc = U8_MAX, acc_idx = 0;
	int acc_val[SEQ_MEM_ACC_MAX_LEVELS], i, ret;

	snprintf(osm_resname, sizeof(osm_resname), "osm-domain%d", index);

	osm_rsrc = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						osm_resname);
	if (!osm_rsrc)
		return -ENODEV;

	seq_addr = osm_rsrc->start + sregs->reg_osm_sequencer;

	ret = qcom_cpufreq_gen_params(cpu_dev, ddata, &hw_tbl, &apm_vc,
				      &acc_vc, cpu_count, &num_entries);
	if (ret)
		return ret;

	/* If we get less than 2 entries, scaling doesn't make sense */
	if (num_entries < 2) {
		dev_err(&pdev->dev, "Not enough LUT entries found (%u)\n",
			num_entries);
		return -EINVAL;
	}

	for (i = 0; i < LUT_MAX_ENTRIES; i++) {
		struct qcom_cpufreq_hw_params *entry;
		int pos = i * sdata->lut_row_size;

		/* Repeat the last valid entry until the end of the table */
		if (i < num_entries)
			entry = &hw_tbl[i];
		else
			entry = &hw_tbl[num_entries - 1];

		writel(i, ddata->base + 0x150 + pos);
		writel(entry->volt_lut_val, ddata->base + sdata->reg_volt_lut + pos);
		writel(entry->freq_lut_val, ddata->base + sdata->reg_freq_lut + pos);
		writel(entry->override_val, ddata->base + sregs->reg_override + pos);
		writel(entry->spare_val, ddata->base + sregs->reg_spare + pos);

		dev_dbg(cpu_dev, "Writing [%d] v:0x%x f:0x%x ovr:0x%x s:0x%x\n", i,
			entry->volt_lut_val, entry->freq_lut_val,
			entry->override_val, entry->spare_val);

		if (entry->spare_val <= last_spare ||
		    acc_idx >= SEQ_MEM_ACC_MAX_LEVELS - 1)
			continue;

		last_spare = entry->spare_val;
		acc_val[acc_idx] = i - 1;
		acc_idx++;
		acc_val[acc_idx] = i;
		acc_idx++;
	}

	/* Sanity check: we must have two mem-acc crossovers (four values) */
	if (acc_idx < SEQ_MEM_ACC_MAX_LEVELS - 1)
		return -EINVAL;

	if (acc_vc > 0 && acc_vc != U8_MAX) {
		sreg = SEQUENCER_REG(seq_addr, SEQ_MEM_ACC_CROSSOVER_VC);
		ret = qcom_scm_io_writel(sreg, num_entries + 1);
		if (ret)
			return ret;

		/*
		 * Switch the ACC one corner lower than what we have found:
		 * this is needed on at least some MSM8998 chips to achieve
		 * full system stability, at a very small power cost.
		 */
		acc_vc--;

		if (acc_vc < acc_val[3]) {
			acc_val[2] = acc_vc - 1;
			acc_val[3] = acc_vc;
		}

		if (acc_val[2] <= acc_val[1])
			acc_val[1] = acc_val[2] - 1;
		if (acc_val[1] <= acc_val[0])
			acc_val[0] = acc_val[1] - 1;
	}

	for (i = 0; i < SEQ_MEM_ACC_MAX_LEVELS; i++) {
		ret = qcom_scm_io_writel(SEQ_MEMACC_REG(seq_addr, i),
					 acc_val[i]);
		if (ret)
			return ret;
	}

	acc_lval = FIELD_GET(LUT_L_VAL, hw_tbl[acc_val[3]].freq_lut_val);
	ret = qcom_scm_io_writel(SEQUENCER_REG(seq_addr, SEQ_MEM_ACC_LVAL),
				 acc_lval);
	if (ret)
		return ret;

	/* Array Power Mux threshold: the corner that switches MX -> APC */
	if (apm_vc == INT_MAX)
		apm_vc = LUT_MAX_ENTRIES - 1;

	writel(num_entries, ddata->base + sregs->reg_seq1);
	ret = qcom_scm_io_writel(SEQUENCER_REG(seq_addr, SEQ_APM_CROSSOVER_VC),
				 num_entries);
	if (ret)
		return ret;

	ret = qcom_scm_io_writel(SEQUENCER_REG(seq_addr, SEQ_APM_THRESH_VC),
				 apm_vc);
	if (ret)
		return ret;

	ret = qcom_scm_io_writel(SEQUENCER_REG(seq_addr, SEQ_APM_THRESH_PREVC),
				 apm_vc - 1);
	if (ret)
		return ret;

	return qcom_scm_io_writel(SEQUENCER_REG(seq_addr, SEQ_APM_PARAM),
				  (0x39 | apm_vc << 6));
}

static void qcom_cpufreq_hw_boost_setup(void __iomem *timer0_addr, u32 len)
{
	u32 val;

	/* timer_reg0 */
	val = FIELD_PREP(BOOST_TIMER_REG_LO, PLL_WAIT_LOCK_TIME_NS);
	val |= FIELD_PREP(BOOST_TIMER_REG_HI, SAFE_FREQ_WAIT_NS);
	writel(val, timer0_addr);

	/* timer_reg1 */
	val = FIELD_PREP(BOOST_TIMER_REG_LO, PLL_WAIT_LOCK_TIME_NS);
	val |= FIELD_PREP(BOOST_TIMER_REG_HI, PLL_WAIT_LOCK_TIME_NS);
	writel(val, timer0_addr + len);

	/* timer_reg2 */
	val = FIELD_PREP(BOOST_TIMER_REG_LO, DEXT_DECREMENT_WAIT_NS);
	writel(val, timer0_addr + (2 * len));
}

/**
 * qcom_cpufreq_hw_osm_setup() - Setup and enable the OSM
 *
 * On msm8998 the OSM is not programmed by the bootloader nor by
 * TrustZone, so the whole DVFS machinery has to be brought up here:
 * lookup table, sequencer thresholds, cycle counter, boost and droop
 * timers and, last but not least, the enable bit.
 */
static int qcom_cpufreq_hw_osm_setup(struct device *cpu_dev,
				     struct cpufreq_policy *policy,
				     int cpu_count, int index)
{
	struct qcom_cpufreq_data *drv_data = policy->driver_data;
	const struct qcom_cpufreq_soc_setup_data *setup_regs =
		drv_data->soc_data->setup_regs;
	u32 val;
	int ret;

	ret = qcom_cpufreq_hw_write_lut(cpu_dev, policy, cpu_count, index);
	if (ret)
		return ret;

	/* Set OSM to XO clock ratio and use the XO edge for the counter */
	val = FIELD_PREP(CYCLE_COUNTER_CLK_RATIO, OSM_XO_RATIO_VAL);
	val |= CYCLE_COUNTER_USE_XO_EDGE;

	/* Enable the CPU cycle counter */
	val |= BIT(0);
	writel(val, drv_data->base + setup_regs->reg_cycle_counter);

	/* CoreCount DCVS Policy: wait time for frequency inc/decrement */
	val = FIELD_PREP(HYSTERESIS_UP_MASK, HYSTERESIS_CC_NS);
	val |= FIELD_PREP(HYSTERESIS_DN_MASK, HYSTERESIS_CC_NS);
	writel(val, drv_data->base + setup_regs->reg_spm_cc_hyst);

	/* Set the frequency index 0 and override for cluster power collapse */
	writel(BIT(0), drv_data->base + setup_regs->reg_cc_zero_behav);

	/* Treat cores in retention as active */
	writel(0, drv_data->base + setup_regs->reg_spm_core_ret_map);

	/* Enable CoreCount based DCVS */
	writel(0, drv_data->base + setup_regs->reg_spm_cc_dcvs_dis);

	/* CoreCount DCVS-LLM Policy: wait time inc/dec for freq and volt */
	val = FIELD_PREP(HYSTERESIS_UP_MASK, HYSTERESIS_LLM_NS);
	val |= FIELD_PREP(HYSTERESIS_DN_MASK, HYSTERESIS_LLM_NS);
	writel(val, drv_data->base + setup_regs->reg_llm_freq_vote_hyst);
	writel(val, drv_data->base + setup_regs->reg_llm_volt_vote_hyst);

	/* Enable LLM frequency+voltage voting */
	writel(0, drv_data->base + setup_regs->reg_llm_intf_dcvs_dis);

	/* Setup boost FSM timers */
	qcom_cpufreq_hw_boost_setup(drv_data->base + setup_regs->reg_cc_boost_timer,
				    setup_regs->boost_timer_reg_len);
	qcom_cpufreq_hw_boost_setup(drv_data->base + setup_regs->reg_dcvs_boost_timer,
				    setup_regs->boost_timer_reg_len);
	qcom_cpufreq_hw_boost_setup(drv_data->base + setup_regs->reg_ps_boost_timer,
				    setup_regs->boost_timer_reg_len);

	/* PLL signal timing control for boost */
	writel(BOOST_SYNC_DELAY, drv_data->base + setup_regs->reg_boost_sync_delay);

	/* Setup WFx and PC/RET droop unstall */
	val = FIELD_PREP(DROOP_TIMER1, DROOP_TIMER_NS);
	val |= FIELD_PREP(DROOP_TIMER0, DROOP_TIMER_NS);
	writel(val, drv_data->base + setup_regs->reg_droop_unstall_ctrl);

	/* Setup WFx and PC/RET droop wait-to-release */
	val = FIELD_PREP(DROOP_TIMER1, DROOP_WAIT_RELEASE_TIMER_NS);
	val |= FIELD_PREP(DROOP_TIMER0, DROOP_WAIT_RELEASE_TIMER_NS);
	writel(val, drv_data->base + setup_regs->reg_droop_wait_release_ctrl);

	/* PLL signal timing control for droop */
	writel(1, drv_data->base + setup_regs->reg_droop_sync_delay);

	/* Setup DCVS timers */
	writel(DROOP_RELEASE_TIMER_NS,
	       drv_data->base + setup_regs->reg_droop_release_ctrl);
	writel(DROOP_TIMER_NS, drv_data->base + setup_regs->reg_droop_timer_ctrl);

	/* Setup droop control */
	val = readl(drv_data->base + setup_regs->reg_droop_ctrl);
	val |= DROOP_CTRL_VAL;
	writel(val, drv_data->base + setup_regs->reg_droop_ctrl);

	/* Enable CC/DCVS/PS boost, WFx, PC/RET and DCVS FSM */
	val = readl(drv_data->base + setup_regs->reg_pdn_fsm_ctrl);
	val |= CC_BOOST_EN | PS_BOOST_EN | DCVS_BOOST_EN;
	val |= WFX_DROOP_EN | PC_RET_EXIT_DROOP_EN | DCVS_DROOP_EN;
	writel(val, drv_data->base + setup_regs->reg_pdn_fsm_ctrl);

	/* Enable PLL droop override */
	writel(PLL_OVERRIDE_DROOP_EN,
	       drv_data->base + setup_regs->reg_pll_override);

	/* Initialize the Adaptive Clock Distribution */
	ret = qcom_cpufreq_hw_acd_init(cpu_dev, policy, index);
	if (ret)
		return ret;

	/* We're ready: enable the OSM and give it time to boot (5 uS) */
	writel(1, drv_data->base + drv_data->soc_data->reg_enable);
	udelay(OSM_BOOT_TIME_US);

	return 0;
}

static int qcom_cpufreq_hw_cpu_init(struct cpufreq_policy *policy)
{
	struct platform_device *pdev = cpufreq_get_driver_data();
	struct device *dev = &pdev->dev;
	struct of_phandle_args args;
	struct device_node *cpu_np;
	struct device *cpu_dev;
	struct qcom_cpufreq_data *data;
	int ret, index;

	cpu_dev = get_cpu_device(policy->cpu);
	if (!cpu_dev) {
		pr_err("%s: failed to get cpu%d device\n", __func__,
		       policy->cpu);
		return -ENODEV;
	}

	cpu_np = of_cpu_device_node_get(policy->cpu);
	if (!cpu_np)
		return -EINVAL;

	ret = of_parse_phandle_with_args(cpu_np, "qcom,freq-domain",
					 "#freq-domain-cells", 0, &args);
	of_node_put(cpu_np);
	if (ret)
		return ret;

	index = args.args[0];
	data = &qcom_cpufreq.data[index];

	qcom_get_related_cpus(index, policy->cpus);

	policy->driver_data = data;
	policy->dvfs_possible_from_any_cpu = true;
	data->policy = policy;

	if (!qcom_cpufreq.soc_data->uses_tz) {
		ret = qcom_cpufreq_hw_osm_setup(cpu_dev, policy,
						cpumask_weight(policy->cpus), index);
		if (ret) {
			dev_err(dev, "Cannot setup the OSM for CPU%d: %d\n",
				policy->cpu, ret);
			return ret;
		}
	}

	/* HW should be in enabled state to proceed */
	if (!(readl_relaxed(data->base + qcom_cpufreq.soc_data->reg_enable) & 0x1)) {
		dev_err(dev, "Domain-%d cpufreq hardware not enabled\n", index);
		return -ENODEV;
	}

	if (qcom_cpufreq.soc_data->reg_dcvs_ctrl &&
	    (readl_relaxed(data->base + qcom_cpufreq.soc_data->reg_dcvs_ctrl) & 0x1))
		data->per_core_dcvs = true;

	ret = qcom_cpufreq_hw_read_lut(cpu_dev, policy);
	if (ret) {
		dev_err(dev, "Domain-%d failed to read LUT\n", index);
		return ret;
	}

	ret = dev_pm_opp_get_opp_count(cpu_dev);
	if (ret <= 0) {
		dev_err(cpu_dev, "Failed to add OPPs\n");
		return -ENODEV;
	}

	return qcom_cpufreq_hw_lmh_init(policy, index);
}

static void qcom_cpufreq_hw_cpu_exit(struct cpufreq_policy *policy)
{
	struct device *cpu_dev = get_cpu_device(policy->cpu);
	struct qcom_cpufreq_data *data = policy->driver_data;

	dev_pm_opp_remove_all_dynamic(cpu_dev);
	dev_pm_opp_of_cpumask_remove_table(policy->related_cpus);
	qcom_cpufreq_hw_lmh_exit(data);
	kfree(policy->freq_table);
}

static void qcom_cpufreq_ready(struct cpufreq_policy *policy)
{
	struct qcom_cpufreq_data *data = policy->driver_data;

	if (data->throttle_irq >= 0)
		enable_irq(data->throttle_irq);
}

static struct cpufreq_driver cpufreq_qcom_hw_driver = {
	.flags		= CPUFREQ_NEED_INITIAL_FREQ_CHECK |
			  CPUFREQ_HAVE_GOVERNOR_PER_POLICY |
			  CPUFREQ_IS_COOLING_DEV,
	.verify		= cpufreq_generic_frequency_table_verify,
	.target_index	= qcom_cpufreq_hw_target_index,
	.get		= qcom_cpufreq_hw_get,
	.init		= qcom_cpufreq_hw_cpu_init,
	.exit		= qcom_cpufreq_hw_cpu_exit,
	.online		= qcom_cpufreq_hw_cpu_online,
	.offline	= qcom_cpufreq_hw_cpu_offline,
	.register_em	= cpufreq_register_em_with_opp,
	.fast_switch    = qcom_cpufreq_hw_fast_switch,
	.name		= "qcom-cpufreq-hw",
	.ready		= qcom_cpufreq_ready,
	.set_boost	= cpufreq_boost_set_sw,
};

static unsigned long qcom_cpufreq_hw_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct qcom_cpufreq_data *data = container_of(hw, struct qcom_cpufreq_data, cpu_clk);

	return __qcom_cpufreq_hw_get(data->policy) * HZ_PER_KHZ;
}

/*
 * Since we cannot determine the closest rate of the target rate, let's just
 * return the actual rate at which the clock is running at. This is needed to
 * make clk_set_rate() API work properly.
 */
static int qcom_cpufreq_hw_determine_rate(struct clk_hw *hw, struct clk_rate_request *req)
{
	req->rate = qcom_cpufreq_hw_recalc_rate(hw, 0);

	return 0;
}

static const struct clk_ops qcom_cpufreq_hw_clk_ops = {
	.recalc_rate = qcom_cpufreq_hw_recalc_rate,
	.determine_rate = qcom_cpufreq_hw_determine_rate,
};

static int qcom_cpufreq_hw_driver_probe(struct platform_device *pdev)
{
	struct clk_hw_onecell_data *clk_data;
	struct device *dev = &pdev->dev;
	struct device *cpu_dev;
	struct clk *clk;
	int ret, i, num_domains;

	qcom_cpufreq.soc_data = of_device_get_match_data(dev);
	if (!qcom_cpufreq.soc_data)
		return -ENODEV;

	clk = clk_get(dev, "xo");
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	xo_rate = clk_get_rate(clk);
	clk_put(clk);

	clk = clk_get(dev, "alternate");
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	cpu_hw_rate = clk_get_rate(clk) /
		      (qcom_cpufreq.soc_data->clk_hw_div ?: CLK_HW_DIV);
	clk_put(clk);

	cpufreq_qcom_hw_driver.driver_data = pdev;

	/* Check for optional interconnect paths on CPU0 */
	cpu_dev = get_cpu_device(0);
	if (!cpu_dev)
		return -EPROBE_DEFER;

	/*
	 * The OSM lookup table is built from the voltages that the CPR driver
	 * derives from the chip fuses, so the CPR-hardened power domain has to
	 * be up before we register. Probe again once it is available.
	 */
	if (qcom_cpufreq.soc_data->setup_regs) {
		struct device *pd;

		pd = dev_pm_domain_attach_by_name(cpu_dev, "cprh");
		if (IS_ERR(pd))
			return dev_err_probe(dev, PTR_ERR(pd),
					     "CPRh power domain is not ready\n");
	}

	ret = dev_pm_opp_of_find_icc_paths(cpu_dev, NULL);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to find icc paths\n");

	for (num_domains = 0; num_domains < MAX_FREQ_DOMAINS; num_domains++) {
		char fdom_name[16];

		snprintf(fdom_name, sizeof(fdom_name), "freq-domain%d",
			 num_domains);
		if (!platform_get_resource_byname(pdev, IORESOURCE_MEM,
						  fdom_name))
			break;
	}

	qcom_cpufreq.data = devm_kzalloc(dev, sizeof(struct qcom_cpufreq_data) * num_domains,
					 GFP_KERNEL);
	if (!qcom_cpufreq.data)
		return -ENOMEM;

	clk_data = devm_kzalloc(dev, struct_size(clk_data, hws, num_domains), GFP_KERNEL);
	if (!clk_data)
		return -ENOMEM;

	clk_data->num = num_domains;

	for (i = 0; i < num_domains; i++) {
		struct qcom_cpufreq_data *data = &qcom_cpufreq.data[i];
		struct clk_init_data clk_init = {};
		void __iomem *base;

		char fdom_name[16];

		snprintf(fdom_name, sizeof(fdom_name), "freq-domain%d", i);
		base = devm_platform_ioremap_resource_byname(pdev, fdom_name);
		if (IS_ERR(base)) {
			dev_err(dev, "Failed to map %s\n", fdom_name);
			return PTR_ERR(base);
		}

		data->base = base;
		data->soc_data = qcom_cpufreq.soc_data;

		/* Register CPU clock for each frequency domain */
		clk_init.name = kasprintf(GFP_KERNEL, "qcom_cpufreq%d", i);
		if (!clk_init.name)
			return -ENOMEM;

		clk_init.flags = CLK_GET_RATE_NOCACHE;
		clk_init.ops = &qcom_cpufreq_hw_clk_ops;
		data->cpu_clk.init = &clk_init;

		ret = devm_clk_hw_register(dev, &data->cpu_clk);
		if (ret < 0) {
			dev_err(dev, "Failed to register clock %d: %d\n", i, ret);
			kfree(clk_init.name);
			return ret;
		}

		clk_data->hws[i] = &data->cpu_clk;
		kfree(clk_init.name);
	}

	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, clk_data);
	if (ret < 0) {
		dev_err(dev, "Failed to add clock provider\n");
		return ret;
	}

	ret = cpufreq_register_driver(&cpufreq_qcom_hw_driver);
	if (ret)
		dev_err(dev, "CPUFreq HW driver failed to register\n");
	else
		dev_dbg(dev, "QCOM CPUFreq HW driver initialized\n");

	return ret;
}

static void qcom_cpufreq_hw_driver_remove(struct platform_device *pdev)
{
	cpufreq_unregister_driver(&cpufreq_qcom_hw_driver);
}

static struct platform_driver qcom_cpufreq_hw_driver = {
	.probe = qcom_cpufreq_hw_driver_probe,
	.remove = qcom_cpufreq_hw_driver_remove,
	.driver = {
		.name = "qcom-cpufreq-hw",
		.of_match_table = qcom_cpufreq_hw_match,
	},
};

static int __init qcom_cpufreq_hw_init(void)
{
	return platform_driver_register(&qcom_cpufreq_hw_driver);
}
postcore_initcall(qcom_cpufreq_hw_init);

static void __exit qcom_cpufreq_hw_exit(void)
{
	platform_driver_unregister(&qcom_cpufreq_hw_driver);
}
module_exit(qcom_cpufreq_hw_exit);

MODULE_DESCRIPTION("QCOM CPUFREQ HW Driver");
MODULE_LICENSE("GPL v2");
