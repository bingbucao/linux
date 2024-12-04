// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2013 - 2024 Intel Corporation
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/math64.h>
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pm_runtime.h>
#include <linux/scatterlist.h>
#include <linux/types.h>

#include "ipu7.h"
#include "ipu7-bus.h"
#include "ipu7-buttress.h"
#include "ipu7-buttress-regs.h"

#define BOOTLOADER_STATUS_OFFSET	BUTTRESS_REG_FW_BOOT_PARAMS7

#define BOOTLOADER_MAGIC_KEY		0xb00710ad

#define ENTRY	BUTTRESS_IU2CSECSR_IPC_PEER_COMP_ACTIONS_RST_PHASE1
#define EXIT	BUTTRESS_IU2CSECSR_IPC_PEER_COMP_ACTIONS_RST_PHASE2
#define QUERY	BUTTRESS_IU2CSECSR_IPC_PEER_QUERIED_IP_COMP_ACTIONS_RST_PHASE

#define BUTTRESS_TSC_SYNC_RESET_TRIAL_MAX	10

#define BUTTRESS_POWER_TIMEOUT_US		(2 * USEC_PER_SEC)

#define BUTTRESS_CSE_BOOTLOAD_TIMEOUT_US	(5 * USEC_PER_SEC)
#define BUTTRESS_CSE_AUTHENTICATE_TIMEOUT_US	(10 * USEC_PER_SEC)
#define BUTTRESS_CSE_FWRESET_TIMEOUT_US		(100 * USEC_PER_MSEC)

#define BUTTRESS_IPC_TX_TIMEOUT_MS		MSEC_PER_SEC
#define BUTTRESS_IPC_RX_TIMEOUT_MS		MSEC_PER_SEC
#define BUTTRESS_IPC_VALIDITY_TIMEOUT_US	(1 * USEC_PER_SEC)
#define BUTTRESS_TSC_SYNC_TIMEOUT_US		(5 * USEC_PER_MSEC)

#define BUTTRESS_IPC_RESET_RETRY		2000
#define BUTTRESS_CSE_IPC_RESET_RETRY		4
#define BUTTRESS_IPC_CMD_SEND_RETRY		1

int ipu_buttress_ipc_reset(struct ipu7_device *isp,
			   struct ipu_buttress_ipc *ipc)
{
	return 0;
}

irqreturn_t ipu_buttress_isr(int irq, void *isp_ptr)
{
	struct ipu7_device *isp = isp_ptr;

	dev_warn(&isp->pdev->dev, "Unexpected btrs IRQ for IPU FPGA\n");

	return IRQ_NONE;
}

irqreturn_t ipu_buttress_isr_threaded(int irq, void *isp_ptr)
{
	struct ipu7_device *isp = isp_ptr;

	dev_warn(&isp->pdev->dev, "Unexpected btrs IRQ for IPU FPGA\n");

	return IRQ_NONE;
}

static int ipu_buttress_ipc_send(struct ipu7_device *isp,
				 u32 ipc_msg, u32 size, bool require_resp,
				 u32 expected_resp)
{
	return 0;
}

static int isys_d2d_power(struct device *dev, bool on)
{
	struct ipu7_device *isp = to_ipu7_bus_device(dev)->isp;
	int ret = 0;
	u32 val;

	dev_dbg(dev, "power %s isys d2d.\n", on ? "UP" : "DOWN");
	val = readl(isp->base + BUTTRESS_REG_D2D_CTL);
	if (!(val & BUTTRESS_D2D_PWR_ACK) ^ on) {
		dev_info(dev, "d2d already in %s state.\n",
			 on ? "UP" : "DOWN");
		return 0;
	}

	val = on ? val | BUTTRESS_D2D_PWR_EN : val & (~BUTTRESS_D2D_PWR_EN);
	writel(val, isp->base + BUTTRESS_REG_D2D_CTL);
	ret = readl_poll_timeout(isp->base + BUTTRESS_REG_D2D_CTL,
				 val, (!(val & BUTTRESS_D2D_PWR_ACK) ^ on),
				 100, BUTTRESS_POWER_TIMEOUT_US);
	if (ret)
		dev_err(dev, "power %s d2d timeout. status: 0x%x\n",
			on ? "UP" : "DOWN", val);

	return ret;
}

static void isys_nde_control(struct device *dev, bool on)
{
}

static int ipu7_buttress_powerup(struct device *dev,
				 const struct ipu_buttress_ctrl *ctrl)
{
	struct ipu7_device *isp = to_ipu7_bus_device(dev)->isp;
	u32 val, exp_sts;
	int ret = 0;

	if (!ctrl)
		return 0;

	mutex_lock(&isp->buttress.power_mutex);

	exp_sts = ctrl->pwr_sts_on << ctrl->pwr_sts_shift;
	if (ctrl->subsys_id == IPU_IS) {
		ret = isys_d2d_power(dev, true);
		if (ret)
			goto out_power;
		isys_nde_control(dev, true);
	}

	/* request clock resource ownership */
	val = readl(isp->base + BUTTRESS_REG_SLEEP_LEVEL_CFG);
	val |= ctrl->ovrd_clk;
	writel(val, isp->base + BUTTRESS_REG_SLEEP_LEVEL_CFG);
	ret = readl_poll_timeout(isp->base + BUTTRESS_REG_SLEEP_LEVEL_STS,
				 val, (val & ctrl->own_clk_ack),
				 100, BUTTRESS_POWER_TIMEOUT_US);
	if (ret)
		dev_warn(dev, "request clk ownership timeout. status 0x%x\n",
			 val);

	val = ctrl->ratio << ctrl->ratio_shift | ctrl->cdyn << ctrl->cdyn_shift;

	dev_dbg(dev, "set 0x%x to %s_WORKPOINT_REQ.\n", val,
		ctrl->subsys_id == IPU_IS ? "IS" : "PS");
	writel(val, isp->base + ctrl->freq_ctl);

	ret = readl_poll_timeout(isp->base + BUTTRESS_REG_PWR_STATUS,
				 val, ((val & ctrl->pwr_sts_mask) == exp_sts),
				 100, BUTTRESS_POWER_TIMEOUT_US);
	if (ret) {
		dev_err(dev, "%s power up timeout with status: 0x%x\n",
			ctrl->subsys_id == IPU_IS ? "IS" : "PS", val);
		goto out_power;
	}

	dev_dbg(dev, "%s power up successfully. status: 0x%x\n",
		ctrl->subsys_id == IPU_IS ? "IS" : "PS", val);

	/* release clock resource ownership */
	val = readl(isp->base + BUTTRESS_REG_SLEEP_LEVEL_CFG);
	val &= ~ctrl->ovrd_clk;
	writel(val, isp->base + BUTTRESS_REG_SLEEP_LEVEL_CFG);

out_power:
	mutex_unlock(&isp->buttress.power_mutex);

	return ret;
}

static int ipu7_buttress_powerdown(struct device *dev,
				   const struct ipu_buttress_ctrl *ctrl)
{
	struct ipu7_device *isp = to_ipu7_bus_device(dev)->isp;
	u32 val, exp_sts;
	int ret = 0;

	if (!ctrl)
		return 0;

	mutex_lock(&isp->buttress.power_mutex);

	exp_sts = ctrl->pwr_sts_off << ctrl->pwr_sts_shift;
	val = 0x8 << ctrl->ratio_shift;

	dev_dbg(dev, "set 0x%x to %s_WORKPOINT_REQ.\n", val,
		ctrl->subsys_id == IPU_IS ? "IS" : "PS");
	writel(val, isp->base + ctrl->freq_ctl);
	ret = readl_poll_timeout(isp->base + BUTTRESS_REG_PWR_STATUS,
				 val, ((val & ctrl->pwr_sts_mask) == exp_sts),
				 100, BUTTRESS_POWER_TIMEOUT_US);
	if (ret) {
		dev_err(dev, "%s power down timeout with status: 0x%x\n",
			ctrl->subsys_id == IPU_IS ? "IS" : "PS", val);
		goto out_power;
	}

	dev_dbg(dev, "%s power down successfully. status: 0x%x\n",
		ctrl->subsys_id == IPU_IS ? "IS" : "PS", val);
out_power:
	if (ctrl->subsys_id == IPU_IS && !ret) {
		isys_d2d_power(dev, false);
		isys_nde_control(dev, false);
	}

	mutex_unlock(&isp->buttress.power_mutex);

	return ret;
}

static int ipu8_buttress_powerup(struct device *dev,
				 const struct ipu_buttress_ctrl *ctrl)
{
	struct ipu7_device *isp = to_ipu7_bus_device(dev)->isp;
	u32 sleep_level_reg = BUTTRESS_REG_SLEEP_LEVEL_STS;
	u32 val, exp_sts;
	int ret = 0;

	if (!ctrl)
		return 0;

	mutex_lock(&isp->buttress.power_mutex);
	exp_sts = ctrl->pwr_sts_on << ctrl->pwr_sts_shift;
	if (ctrl->subsys_id == IPU_IS) {
		ret = isys_d2d_power(dev, true);
		if (ret)
			goto out_power;
		isys_nde_control(dev, true);
	}

	/* request ps_pll when psys freq > 400Mhz */
	if (ctrl->subsys_id == IPU_PS && ctrl->ratio > 0x10) {
		writel(1, isp->base + BUTTRESS_REG_PS_PLL_ENABLE);
		ret = readl_poll_timeout(isp->base + sleep_level_reg,
					 val, (val & ctrl->own_clk_ack),
					 100, BUTTRESS_POWER_TIMEOUT_US);
		if (ret)
			dev_warn(dev, "ps_pll req ack timeout. status 0x%x\n",
				 val);
	}

	val = ctrl->ratio << ctrl->ratio_shift | ctrl->cdyn << ctrl->cdyn_shift;
	dev_dbg(dev, "set 0x%x to %s_WORKPOINT_REQ.\n", val,
		ctrl->subsys_id == IPU_IS ? "IS" : "PS");
	writel(val, isp->base + ctrl->freq_ctl);
	ret = readl_poll_timeout(isp->base + BUTTRESS_REG_PWR_STATUS,
				 val, ((val & ctrl->pwr_sts_mask) == exp_sts),
				 100, BUTTRESS_POWER_TIMEOUT_US);
	if (ret) {
		dev_err(dev, "%s power up timeout with status: 0x%x\n",
			ctrl->subsys_id == IPU_IS ? "IS" : "PS", val);
		goto out_power;
	}

	dev_dbg(dev, "%s power up successfully. status: 0x%x\n",
		ctrl->subsys_id == IPU_IS ? "IS" : "PS", val);
out_power:
	mutex_unlock(&isp->buttress.power_mutex);

	return ret;
}

static int ipu8_buttress_powerdown(struct device *dev,
				   const struct ipu_buttress_ctrl *ctrl)
{
	struct ipu7_device *isp = to_ipu7_bus_device(dev)->isp;
	u32 val, exp_sts;
	int ret = 0;

	if (!ctrl)
		return 0;

	mutex_lock(&isp->buttress.power_mutex);
	exp_sts = ctrl->pwr_sts_off << ctrl->pwr_sts_shift;

	if (ctrl->subsys_id == IPU_PS)
		val = 0x10 << ctrl->ratio_shift;
	else
		val = 0x8 << ctrl->ratio_shift;

	dev_dbg(dev, "set 0x%x to %s_WORKPOINT_REQ.\n", val,
		ctrl->subsys_id == IPU_IS ? "IS" : "PS");
	writel(val, isp->base + ctrl->freq_ctl);
	ret = readl_poll_timeout(isp->base + BUTTRESS_REG_PWR_STATUS,
				 val, ((val & ctrl->pwr_sts_mask) == exp_sts),
				 100, BUTTRESS_POWER_TIMEOUT_US);
	if (ret) {
		dev_err(dev, "%s power down timeout with status: 0x%x\n",
			ctrl->subsys_id == IPU_IS ? "IS" : "PS", val);
		goto out_power;
	}

	dev_dbg(dev, "%s power down successfully. status: 0x%x\n",
		ctrl->subsys_id == IPU_IS ? "IS" : "PS", val);
out_power:
	if (ctrl->subsys_id == IPU_IS && !ret) {
		isys_d2d_power(dev, false);
		isys_nde_control(dev, false);
	}

	if (ctrl->subsys_id == IPU_PS) {
		val = readl(isp->base + BUTTRESS_REG_SLEEP_LEVEL_STS);
		if (val & ctrl->own_clk_ack)
			writel(0, isp->base + BUTTRESS_REG_PS_PLL_ENABLE);
	}
	mutex_unlock(&isp->buttress.power_mutex);

	return ret;
}

int ipu_buttress_powerup(struct device *dev,
			 const struct ipu_buttress_ctrl *ctrl)
{
	struct ipu7_device *isp = to_ipu7_bus_device(dev)->isp;

	if (is_ipu8(isp->hw_ver))
		return ipu8_buttress_powerup(dev, ctrl);

	return ipu7_buttress_powerup(dev, ctrl);
}

int ipu_buttress_powerdown(struct device *dev,
			   const struct ipu_buttress_ctrl *ctrl)
{
	struct ipu7_device *isp = to_ipu7_bus_device(dev)->isp;

	if (is_ipu8(isp->hw_ver))
		return ipu8_buttress_powerdown(dev, ctrl);

	return ipu7_buttress_powerdown(dev, ctrl);
}

bool ipu_buttress_get_secure_mode(struct ipu7_device *isp)
{
	return 0;
}

bool ipu_buttress_auth_done(struct ipu7_device *isp)
{
	u32 val;

	if (!isp->secure_mode)
		return true;

	val = readl(isp->base + BUTTRESS_REG_SECURITY_CTL);
	val = FIELD_GET(BUTTRESS_SECURITY_CTL_FW_SETUP_MASK, val);

	return val == BUTTRESS_SECURITY_CTL_AUTH_DONE;
}
EXPORT_SYMBOL_NS_GPL(ipu_buttress_auth_done, INTEL_IPU7);

int ipu_buttress_get_isys_freq(struct ipu7_device *isp, u32 *freq)
{
	u32 reg_val;
	int ret;

	ret = pm_runtime_get_sync(&isp->isys->auxdev.dev);
	if (ret < 0) {
		pm_runtime_put(&isp->isys->auxdev.dev);
		dev_err(&isp->pdev->dev, "Runtime PM failed (%d)\n", ret);
		return ret;
	}

	reg_val = readl(isp->base + BUTTRESS_REG_IS_WORKPOINT_REQ);

	pm_runtime_put(&isp->isys->auxdev.dev);

	if (is_ipu8(isp->hw_ver))
		*freq = (reg_val & BUTTRESS_IS_FREQ_CTL_RATIO_MASK) * 25;
	else
		*freq = (reg_val & BUTTRESS_IS_FREQ_CTL_RATIO_MASK) * 50 / 3;

	return 0;
}
EXPORT_SYMBOL_NS_GPL(ipu_buttress_get_isys_freq, INTEL_IPU7);

int ipu_buttress_get_psys_freq(struct ipu7_device *isp, u32 *freq)
{
	u32 reg_val;
	int ret;

	ret = pm_runtime_get_sync(&isp->psys->auxdev.dev);
	if (ret < 0) {
		pm_runtime_put(&isp->psys->auxdev.dev);
		dev_err(&isp->pdev->dev, "Runtime PM failed (%d)\n", ret);
		return ret;
	}

	reg_val = readl(isp->base + BUTTRESS_REG_PS_WORKPOINT_REQ);

	pm_runtime_put(&isp->psys->auxdev.dev);

	reg_val &= BUTTRESS_PS_FREQ_CTL_RATIO_MASK;
	*freq = BUTTRESS_PS_FREQ_RATIO_STEP * reg_val;

	return 0;
}
EXPORT_SYMBOL_NS_GPL(ipu_buttress_get_psys_freq, INTEL_IPU7);

int ipu_buttress_reset_authentication(struct ipu7_device *isp)
{
	struct device *dev = &isp->pdev->dev;
	int ret;
	u32 val;

	if (!isp->secure_mode) {
		dev_dbg(dev, "Skip auth for non-secure mode\n");
		return 0;
	}

	writel(BUTTRESS_FW_RESET_CTL_START, isp->base +
	       BUTTRESS_REG_FW_RESET_CTL);

	ret = readl_poll_timeout(isp->base + BUTTRESS_REG_FW_RESET_CTL, val,
				 val & BUTTRESS_FW_RESET_CTL_DONE, 500,
				 BUTTRESS_CSE_FWRESET_TIMEOUT_US);
	if (ret) {
		dev_err(dev, "Time out while resetting authentication state\n");
		return ret;
	}

	dev_dbg(dev, "FW reset for authentication done\n");
	writel(0, isp->base + BUTTRESS_REG_FW_RESET_CTL);
	/* leave some time for HW restore */
	usleep_range(800, 1000);

	return 0;
}

int ipu_buttress_authenticate(struct ipu7_device *isp)
{
	struct ipu_buttress *b = &isp->buttress;
	struct device *dev = &isp->pdev->dev;
	u32 data, mask, done, fail;
	int ret;

	if (!isp->secure_mode) {
		dev_dbg(dev, "Skip auth for non-secure mode\n");
		return 0;
	}

	mutex_lock(&b->auth_mutex);

	if (ipu_buttress_auth_done(isp)) {
		ret = 0;
		goto out_unlock;
	}

	/*
	 * BUTTRESS_REG_FW_SOURCE_BASE needs to be set with FW CPD
	 * package address for secure mode.
	 */

	writel(isp->cpd_fw->size, isp->base + BUTTRESS_REG_FW_SOURCE_SIZE);
	writel(sg_dma_address(isp->psys->fw_sgt.sgl),
	       isp->base + BUTTRESS_REG_FW_SOURCE_BASE);

	/*
	 * Write boot_load into IU2CSEDATA0
	 * Write sizeof(boot_load) | 0x2 << CLIENT_ID to
	 * IU2CSEDB.IU2CSECMD and set IU2CSEDB.IU2CSEBUSY as
	 */
	dev_info(dev, "Sending BOOT_LOAD to CSE\n");
	ret = ipu_buttress_ipc_send(isp, BUTTRESS_IU2CSEDATA0_IPC_BOOT_LOAD,
				    1, true,
				    BUTTRESS_CSE2IUDATA0_IPC_BOOT_LOAD_DONE);
	if (ret) {
		dev_err(dev, "CSE boot_load failed\n");
		goto out_unlock;
	}

	mask = BUTTRESS_SECURITY_CTL_FW_SETUP_MASK;
	done = BUTTRESS_SECURITY_CTL_FW_SETUP_DONE;
	fail = BUTTRESS_SECURITY_CTL_AUTH_FAILED;
	ret = readl_poll_timeout(isp->base + BUTTRESS_REG_SECURITY_CTL, data,
				 ((data & mask) == done ||
				  (data & mask) == fail), 500,
				 BUTTRESS_CSE_BOOTLOAD_TIMEOUT_US);
	if (ret) {
		dev_err(dev, "CSE boot_load timeout\n");
		goto out_unlock;
	}

	if ((data & mask) == fail) {
		dev_err(dev, "CSE auth failed\n");
		ret = -EINVAL;
		goto out_unlock;
	}

	ret = readl_poll_timeout(isp->base + BOOTLOADER_STATUS_OFFSET,
				 data, data == BOOTLOADER_MAGIC_KEY, 500,
				 BUTTRESS_CSE_BOOTLOAD_TIMEOUT_US);
	if (ret) {
		dev_err(dev, "Unexpected magic number 0x%x\n", data);
		goto out_unlock;
	}

	/*
	 * Write authenticate_run into IU2CSEDATA0
	 * Write sizeof(boot_load) | 0x2 << CLIENT_ID to
	 * IU2CSEDB.IU2CSECMD and set IU2CSEDB.IU2CSEBUSY as
	 */
	dev_info(dev, "Sending AUTHENTICATE_RUN to CSE\n");
	ret = ipu_buttress_ipc_send(isp, BUTTRESS_IU2CSEDATA0_IPC_AUTH_RUN,
				    1, true,
				    BUTTRESS_CSE2IUDATA0_IPC_AUTH_RUN_DONE);
	if (ret) {
		dev_err(dev, "CSE authenticate_run failed\n");
		goto out_unlock;
	}

	done = BUTTRESS_SECURITY_CTL_AUTH_DONE;
	ret = readl_poll_timeout(isp->base + BUTTRESS_REG_SECURITY_CTL, data,
				 ((data & mask) == done ||
				  (data & mask) == fail), 500,
				 BUTTRESS_CSE_AUTHENTICATE_TIMEOUT_US);
	if (ret) {
		dev_err(dev, "CSE authenticate timeout\n");
		goto out_unlock;
	}

	if ((data & mask) == fail) {
		dev_err(dev, "CSE boot_load failed\n");
		ret = -EINVAL;
		goto out_unlock;
	}

	dev_info(dev, "CSE authenticate_run done\n");

out_unlock:
	mutex_unlock(&b->auth_mutex);

	return ret;
}

static int ipu_buttress_send_tsc_request(struct ipu7_device *isp)
{
	u32 val, mask, done;
	int ret;

	mask = BUTTRESS_PWR_STATUS_HH_STATUS_MASK;

	writel(BUTTRESS_TSC_CMD_START_TSC_SYNC,
	       isp->base + BUTTRESS_REG_TSC_CMD);

	val = readl(isp->base + BUTTRESS_REG_PWR_STATUS);
	val = FIELD_GET(mask, val);
	if (val == BUTTRESS_PWR_STATUS_HH_STATE_ERR) {
		dev_err(&isp->pdev->dev, "Start tsc sync failed\n");
		return -EINVAL;
	}

	done = BUTTRESS_PWR_STATUS_HH_STATE_DONE;
	ret = readl_poll_timeout(isp->base + BUTTRESS_REG_PWR_STATUS, val,
				 FIELD_GET(mask, val) == done, 500,
				 BUTTRESS_TSC_SYNC_TIMEOUT_US);
	if (ret)
		dev_err(&isp->pdev->dev, "Start tsc sync timeout\n");

	return ret;
}

int ipu_buttress_start_tsc_sync(struct ipu7_device *isp)
{
	void __iomem *base = isp->base;
	unsigned int i;
	u32 val;

	return 0;
	if (is_ipu8(isp->hw_ver))
		return 0;

	if (is_ipu7p5(isp->hw_ver)) {
		val = readl(base + BUTTRESS_REG_TSC_CTL);
		val |= BUTTRESS_SEL_PB_TIMESTAMP;
		writel(val, base + BUTTRESS_REG_TSC_CTL);

		for (i = 0; i < BUTTRESS_TSC_SYNC_RESET_TRIAL_MAX; i++) {
			val = readl(base + BUTTRESS_REG_PB_TIMESTAMP_VALID);
			if (val == 1)
				return 0;
			usleep_range(40, 50);
		}

		dev_err(&isp->pdev->dev, "PB HH sync failed (valid %u)\n", val);

		return -ETIMEDOUT;
	}

	for (i = 0; i < BUTTRESS_TSC_SYNC_RESET_TRIAL_MAX; i++) {
		int ret;

		ret = ipu_buttress_send_tsc_request(isp);
		if (ret != -ETIMEDOUT)
			return ret;

		val = readl(base + BUTTRESS_REG_TSC_CTL);
		val = val | BUTTRESS_TSW_WA_SOFT_RESET;
		writel(val, base + BUTTRESS_REG_TSC_CTL);
		val = val & (~BUTTRESS_TSW_WA_SOFT_RESET);
		writel(val, base + BUTTRESS_REG_TSC_CTL);
	}

	dev_err(&isp->pdev->dev, "TSC sync failed (timeout)\n");

	return -ETIMEDOUT;
}
EXPORT_SYMBOL_NS_GPL(ipu_buttress_start_tsc_sync, INTEL_IPU7);

void ipu_buttress_tsc_read(struct ipu7_device *isp, u64 *val)
{
	unsigned long flags;
	u32 tsc_hi, tsc_lo;

	return;
	local_irq_save(flags);
	if (is_ipu7(isp->hw_ver)) {
		tsc_lo = readl(isp->base + BUTTRESS_REG_TSC_LO);
		tsc_hi = readl(isp->base + BUTTRESS_REG_TSC_HI);
	} else {
		tsc_lo = readl(isp->base + BUTTRESS_REG_PB_TIMESTAMP_LO);
		tsc_hi = readl(isp->base + BUTTRESS_REG_PB_TIMESTAMP_HI);
	}
	*val = (u64)tsc_hi << 32 | tsc_lo;
	local_irq_restore(flags);
}
EXPORT_SYMBOL_NS_GPL(ipu_buttress_tsc_read, INTEL_IPU7);

u64 ipu_buttress_tsc_ticks_to_ns(u64 ticks, const struct ipu7_device *isp)
{
	u64 ns = ticks * 10000;

	/*
	 * converting TSC tick count to ns is calculated by:
	 * Example (TSC clock frequency is 19.2MHz):
	 * ns = ticks * 1000 000 000 / 19.2Mhz
	 *    = ticks * 1000 000 000 / 19200000Hz
	 *    = ticks * 10000 / 192 ns
	 */
	return div_u64(ns, isp->buttress.ref_clk);
}
EXPORT_SYMBOL_NS_GPL(ipu_buttress_tsc_ticks_to_ns, INTEL_IPU7);

/* trigger uc control to wakeup fw */
void ipu_buttress_wakeup_is_uc(const struct ipu7_device *isp)
{
	u32 val;

	val = readl(isp->base + BUTTRESS_REG_DRV_IS_UCX_CONTROL_STATUS);
	val |= UCX_CTL_WAKEUP;
	writel(val, isp->base + BUTTRESS_REG_DRV_IS_UCX_CONTROL_STATUS);
}
EXPORT_SYMBOL_NS_GPL(ipu_buttress_wakeup_is_uc, INTEL_IPU7);

void ipu_buttress_wakeup_ps_uc(const struct ipu7_device *isp)
{
	u32 val;

	val = readl(isp->base + BUTTRESS_REG_DRV_PS_UCX_CONTROL_STATUS);
	val |= UCX_CTL_WAKEUP;
	writel(val, isp->base + BUTTRESS_REG_DRV_PS_UCX_CONTROL_STATUS);
}
EXPORT_SYMBOL_NS_GPL(ipu_buttress_wakeup_ps_uc, INTEL_IPU7);

static void ipu_buttress_setup(struct ipu7_device *isp)
{
	struct device *dev = &isp->pdev->dev;

	writel(0x1, isp->base + BUTTRESS_REG_WORKPOINT_CTL);
	writel(PG_FLOW_OVRD, isp->base + BUTTRESS_REG_PG_FLOW_OVERRIDE);
	writel(PWR_FSM_DELAY, isp->base + BUTTRESS_REG_PWR_FSM_CTL);

	return;
	/* program PB BAR */
	writel(0, isp->pb_base + GLOBAL_INTERRUPT_MASK);
	if (!is_ipu7(isp->hw_ver))
		writel(0x7c0100, isp->pb_base + BAR2_MISC_CONFIG);
	else
		writel(0x100, isp->pb_base + BAR2_MISC_CONFIG);

	if (is_ipu7p5(isp->hw_ver)) {
		writel(BIT(14), isp->pb_base + TLBID_HASH_ENABLE_63_32);
		writel(BIT(9), isp->pb_base + TLBID_HASH_ENABLE_95_64);
		dev_dbg(dev, "PTL TLBID_HASH %x %x\n",
			readl(isp->pb_base + TLBID_HASH_ENABLE_63_32),
			readl(isp->pb_base + TLBID_HASH_ENABLE_95_64));
	} else {
		writel(BIT(22), isp->pb_base + TLBID_HASH_ENABLE_63_32);
		writel(BIT(1), isp->pb_base + TLBID_HASH_ENABLE_127_96);
		dev_dbg(dev, "TLBID_HASH %x %x\n",
			readl(isp->pb_base + TLBID_HASH_ENABLE_63_32),
			readl(isp->pb_base + TLBID_HASH_ENABLE_127_96));
	}

	writel(BUTTRESS_IRQS, isp->base + BUTTRESS_REG_IRQ_CLEAR);
	writel(BUTTRESS_IRQS, isp->base + BUTTRESS_REG_IRQ_MASK);
	writel(BUTTRESS_IRQS, isp->base + BUTTRESS_REG_IRQ_ENABLE);
	/* LNL SW workaround for PS PD hang when PS sub-domain during PD */
	writel(PS_FSM_CG, isp->base + BUTTRESS_REG_CG_CTRL_BITS);
}

void ipu_buttress_restore(struct ipu7_device *isp)
{
	struct ipu_buttress *b = &isp->buttress;
	u32 val;

	ipu_buttress_setup(isp);

	writel(b->wdt_cached_value, isp->base + BUTTRESS_REG_IDLE_WDT);

#define IPU_DRV_SAI_RW      0x0101U
#define IPU_SEC_SAI_RW      0x2020U

	writel(0, isp->base + BUTTRESS_REG_FPGA_SUPPORT(0));
	writel(0x8, isp->base + BUTTRESS_REG_FPGA_SUPPORT(0));
	writel(0x20202020, isp->base + BUTTRESS_REG_FPGA_SUPPORT(2));
	writel(0x20, isp->base + BUTTRESS_REG_FPGA_SUPPORT(3));
	val = readl(isp->base + BUTTRESS_REG_FPGA_SUPPORT(6));
	val &= ~GENMASK(4, 1);
	writel(val, isp->base + BUTTRESS_REG_FPGA_SUPPORT(6));

	for (unsigned int i = 0; i < 12; i++)
		dev_info(&isp->pdev->dev, "reg FPGA_SUPPORT%d = 0x%x\n", i,
			 readl(isp->base + BUTTRESS_REG_FPGA_SUPPORT(i)));

	if (!isp->secure_mode) {
		dev_info(&isp->pdev->dev, "disable secure boot on FPGA.\n");
		writel(IPU_SEC_SAI_RW,
		       isp->base + BUTTRESS_REG_FPGA_SUPPORT(4));
		writel(0, isp->base + BUTTRESS_REG_SECURITY_CTL);
		writel(1, isp->base + BUTTRESS_REG_FW_CTL);
		writel(0, isp->base + BUTTRESS_REG_CAMERA_MASK);
		writel(IPU_DRV_SAI_RW,
		       isp->base + BUTTRESS_REG_FPGA_SUPPORT(4));
	}
}

int ipu_buttress_init(struct ipu7_device *isp)
{
	int ret, ipc_reset_retry = BUTTRESS_CSE_IPC_RESET_RETRY;
	struct ipu_buttress *b = &isp->buttress;
	struct device *dev = &isp->pdev->dev;
	u32 val;

	mutex_init(&b->power_mutex);
	mutex_init(&b->auth_mutex);
	mutex_init(&b->cons_mutex);
	mutex_init(&b->ipc_mutex);
	init_completion(&b->cse.send_complete);
	init_completion(&b->cse.recv_complete);

	b->cse.nack = BUTTRESS_CSE2IUDATA0_IPC_NACK;
	b->cse.nack_mask = BUTTRESS_CSE2IUDATA0_IPC_NACK_MASK;
	b->cse.csr_in = BUTTRESS_REG_CSE2IUCSR;
	b->cse.csr_out = BUTTRESS_REG_IU2CSECSR;
	b->cse.db0_in = BUTTRESS_REG_CSE2IUDB0;
	b->cse.db0_out = BUTTRESS_REG_IU2CSEDB0;
	b->cse.data0_in = BUTTRESS_REG_CSE2IUDATA0;
	b->cse.data0_out = BUTTRESS_REG_IU2CSEDATA0;

	isp->secure_mode = ipu_buttress_get_secure_mode(isp);
	val = readl(isp->base + BUTTRESS_REG_IPU_SKU);
	dev_info(dev, "IPU%u SKU %u in %s mode mask 0x%x\n", val & 0xf,
		 (val >> 4) & 0x7, isp->secure_mode ? "secure" : "non-secure",
		 readl(isp->base + BUTTRESS_REG_CAMERA_MASK));
	b->wdt_cached_value = readl(isp->base + BUTTRESS_REG_IDLE_WDT);
	b->ref_clk = 384;

	ipu_buttress_setup(isp);

	/* Retry couple of times in case of CSE initialization is delayed */
	do {
		ret = ipu_buttress_ipc_reset(isp, &b->cse);
		if (ret) {
			dev_warn(dev, "IPC reset protocol failed, retrying\n");
		} else {
			dev_dbg(dev, "IPC reset done\n");
			return 0;
		}
	} while (ipc_reset_retry--);

	dev_err(dev, "IPC reset protocol failed\n");

	mutex_destroy(&b->power_mutex);
	mutex_destroy(&b->auth_mutex);
	mutex_destroy(&b->cons_mutex);
	mutex_destroy(&b->ipc_mutex);

	return ret;
}

void ipu_buttress_exit(struct ipu7_device *isp)
{
	struct ipu_buttress *b = &isp->buttress;

	writel(0, isp->base + BUTTRESS_REG_IRQ_ENABLE);
	mutex_destroy(&b->power_mutex);
	mutex_destroy(&b->auth_mutex);
	mutex_destroy(&b->cons_mutex);
	mutex_destroy(&b->ipc_mutex);
}
