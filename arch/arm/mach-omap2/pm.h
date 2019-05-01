/*
 * OMAP2/3 Power Management Routines
 *
 * Copyright (C) 2008 Nokia Corporation
 * Jouni Hogander
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __ARCH_ARM_MACH_OMAP2_PM_H
#define __ARCH_ARM_MACH_OMAP2_PM_H

#include <linux/err.h>

#include "powerdomain.h"

extern void *omap3_secure_ram_storage;
extern void omap3_pm_off_mode_enable(int);
extern void omap_sram_idle(void);
extern int omap_pm_clkdms_setup(struct clockdomain *clkdm, void *unused);
extern int (*omap_pm_suspend)(void);
void omap_inc_mpu_core_pwrdm_usecount(void);
void omap_dec_mpu_core_pwrdm_usecount(void);
void omap_enable_core_notifier(int mpu_next_state, int core_next_state);
void omap_idle_core_notifier(int mpu_next_state, int core_next_state);

#ifdef CONFIG_CPU_IDLE
extern int omap3_idle_init(void);
extern int omap4_idle_init(void);
extern int omap5_idle_init(void);
#else
static inline int omap3_idle_init(void) { return 0; }
static inline int omap4_idle_init(void) { return 0; }
static inline int omap5_idle_init(void) { return 0; }
#endif

#ifdef CONFIG_PM
extern void omap4_device_set_state_off(u8 enable);
extern bool omap4_device_prev_state_off(void);
extern bool omap4_device_next_state_off(void);
extern void omap4_device_clear_prev_off_state(void);
#else
static inline void omap4_device_set_state_off(u8 enable)
{
}
static inline bool omap4_device_prev_state_off(void)
{
	return false;
}
static inline bool omap4_device_next_state_off(void)
{
	return false;
}
static inline void omap4_device_clear_prev_off_state(void)
{
	return;
}
#endif
extern u32 omap4_device_off_counter;

#if defined(CONFIG_PM_OPP)
extern int omap3_opp_init(void);
extern int omap4_opp_init(void);
extern int omap5_opp_init(void);
#else
static inline int omap3_opp_init(void)
{
	return -EINVAL;
}
static inline int omap4_opp_init(void)
{
	return -EINVAL;
}
static inline int omap5_opp_init(void)
{
	return -EINVAL;
}
#endif

#ifdef CONFIG_PM
int omap4_pm_cold_reset(char *reason);
#else
int omap4_pm_cold_reset(char *reason)
{
	return -EINVAL;
}
#endif

/*
 * cpuidle mach specific parameters
 *
 * The board code can override the default C-states definition using
 * omap3_pm_init_cpuidle
 */
struct cpuidle_params {
	u32 exit_latency;	/* exit_latency = sleep + wake-up latencies */
	u32 target_residency;
	u8 valid;		/* validates the C-state */
};

#if defined(CONFIG_PM) && defined(CONFIG_CPU_IDLE)
extern void omap3_pm_init_cpuidle(struct cpuidle_params *cpuidle_board_params);
#else
static
inline void omap3_pm_init_cpuidle(struct cpuidle_params *cpuidle_board_params)
{
}
#endif

extern int omap3_pm_get_suspend_state(struct powerdomain *pwrdm);
extern int omap3_pm_set_suspend_state(struct powerdomain *pwrdm, int state);

struct clk;

#ifdef CONFIG_PM_DEBUG
extern u32 enable_off_mode;
extern void pm_dbg_dump_pwrdm(struct powerdomain *pwrdm);
extern void pm_dbg_dump_voltdm(struct voltagedomain *voltdm);
#else
#define enable_off_mode 0
static inline void pm_dbg_dump_pwrdm(struct powerdomain *pwrdm) { }
static inline void pm_dbg_dump_voltdm(struct voltagedomain *voltdm) { }
#endif

#if defined(CONFIG_PM_DEBUG) && defined(CONFIG_DEBUG_FS)
extern void pm_dbg_update_time(struct powerdomain *pwrdm, int prev);
#else
#define pm_dbg_update_time(pwrdm, prev) do {} while (0);
#endif /* CONFIG_PM_DEBUG */

/* 24xx */
extern void omap24xx_idle_loop_suspend(void);
extern unsigned int omap24xx_idle_loop_suspend_sz;

extern void omap24xx_cpu_suspend(u32 dll_ctrl, void __iomem *sdrc_dlla_ctrl,
					void __iomem *sdrc_power);
extern unsigned int omap24xx_cpu_suspend_sz;

/* 3xxx */
extern void omap34xx_cpu_suspend(int save_state);

/* omap3_do_wfi function pointer and size, for copy to SRAM */
extern void omap3_do_wfi(void);
extern unsigned int omap3_do_wfi_sz;
/* ... and its pointer from SRAM after copy */
extern void (*omap3_do_wfi_sram)(void);

/* save_secure_ram_context function pointer and size, for copy to SRAM */
extern int save_secure_ram_context(u32 *addr);
extern unsigned int save_secure_ram_context_sz;

extern void omap3_save_scratchpad_contents(void);

#define PM_RTA_ERRATUM_i608		(1 << 0)
#define PM_SDRC_WAKEUP_ERRATUM_i583	(1 << 1)

#if defined(CONFIG_PM) && defined(CONFIG_ARCH_OMAP3)
extern u16 pm34xx_errata;
#define IS_PM34XX_ERRATUM(id)		(pm34xx_errata & (id))
extern void enable_omap3630_toggle_l2_on_restore(void);
#else
#define IS_PM34XX_ERRATUM(id)		0
static inline void enable_omap3630_toggle_l2_on_restore(void) { }
#endif		/* defined(CONFIG_PM) && defined(CONFIG_ARCH_OMAP3) */

#define PM_OMAP4_ROM_SMP_BOOT_ERRATUM_xxx	(1 << 0)
#define PM_OMAP4_ROM_IVAHD_TESLA_ERRATUM_xxx	(1 << 1)
#define PM_OMAP4_ROM_L3INSTR_ERRATUM_xxx	(1 << 2)
#define PM_OMAP4_ROM_CPU1_BACKUP_ERRATUM_xxx	(1 << 3)

extern u16 pm44xx_errata;
#if defined(CONFIG_ARCH_OMAP4)
#define IS_PM44XX_ERRATUM(id)		(pm44xx_errata & (id))
#else
#define IS_PM44XX_ERRATUM(id)		0
#endif

#ifdef CONFIG_POWER_AVS_OMAP
extern int omap_devinit_smartreflex(void);
extern void omap_enable_smartreflex_on_init(void);
#else
static inline int omap_devinit_smartreflex(void)
{
	return -EINVAL;
}

static inline void omap_enable_smartreflex_on_init(void) {}
#endif

#ifdef CONFIG_TWL4030_CORE
extern int omap_twl_init(void);
extern int omap3_twl_set_sr_bit(bool enable);
#else
static inline int omap_twl_init(void)
{
	return -EINVAL;
}
#endif

#ifdef CONFIG_PM
extern void omap_pm_setup_oscillator(u32 tstart, u32 tshut);
extern void omap_pm_get_oscillator(u32 *tstart, u32 *tshut);
extern void omap_pm_setup_oscillator_voltage_ramp_time(u32 tstart, u32 tshut);
extern void omap_pm_get_oscillator_voltage_ramp_time(u32 *tstart, u32 *tshut);
extern void omap_pm_setup_rsttime_latency(u32 rsttime_latency);
extern u32 omap_pm_get_rsttime_latency(void);
extern int __init omap4_pm_init(void);
extern void __init omap4_set_processor_device_opp(void);
extern void __init omap4_init_cpuidle(void);


#else
static inline void omap_pm_setup_oscillator(u32 tstart, u32 tshut) { }
static inline void omap_pm_get_oscillator(u32 *tstart, u32 *tshut) { }
static inline void omap_pm_setup_oscillator_voltage_ramp_time(
	u32 tstart, u32 tshut) { }
static inline void omap_pm_get_oscillator_voltage_ramp_time(
	u32 *tstart, u32 *tshut) { }
static inline void omap_pm_setup_rsttime_latency(u32 rsttime_latency) {};
static inline u32 omap_pm_get_rsttime_latency(void) { return 0; }
extern inline int omap4_pm_init(void)
{
	return 0;
}

extern inline void omap4_set_processor_device_opp(void) { }
extern inline void omap4_init_cpuidle() { }

#endif

extern int omap_trim_configure(void);

#ifdef CONFIG_PM
extern bool omap_pm_is_ready_status;
/**
 * omap_pm_is_ready() - tells if OMAP pm framework is done it's initialization
 *
 * In few cases, to sequence operations properly, we'd like to know if OMAP's PM
 * framework has completed all it's expected initializations.
 */
static inline bool omap_pm_is_ready(void)
{
	return omap_pm_is_ready_status;
}

extern bool omap_pm_is_prepared_status;
static inline bool omap_pm_is_prepared(void)
{
	return omap_pm_is_prepared_status;
}
#else
static inline bool omap_pm_is_ready(void)
{
	return false;
}

static inline bool omap_pm_is_prepared(void)
{
	return false;
}
#endif

#ifdef CONFIG_PM
extern int omap_sar_save(void);
#else
static inline void omap_sar_save(void)
{
}
#endif

#endif
