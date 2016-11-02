/*
 * drivers/thermal_framework/governor/case_governor.c
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Sebastien Sabatier <s-sabatier1@ti.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/

#include <linux/err.h>
#include <linux/module.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/debugfs.h>
#include <linux/syscalls.h>

#include <linux/thermal_framework.h>

#include <linux/opp.h>
#include <plat/omap_device.h>

#ifdef CONFIG_AMAZON_METRICS_LOG
#include <linux/metricslog.h>
#include <linux/logger.h>
#include <linux/syscalls.h>
#include <linux/unistd.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/file.h>
#include <linux/uaccess.h>
#include <asm/processor.h>
#define THERMO_METRICS_STR_LEN 128
#endif


/* System/Case Thermal thresholds */
#define SYS_THRESHOLD_HOT		78000
#define SYS_THRESHOLD_COLD		74000
#define SYS_THRESHOLD_HOT_INC		1000
#define INIT_COOLING_LEVEL		0
#define CASE_SUBZONES_NUMBER		4
int case_subzone_number = CASE_SUBZONES_NUMBER;
EXPORT_SYMBOL_GPL(case_subzone_number);

static int sys_threshold_hot = SYS_THRESHOLD_HOT;
static int sys_threshold_cold = SYS_THRESHOLD_COLD;
static int sys_threshold_hot_inc = SYS_THRESHOLD_HOT_INC;
static int thot = SYS_THRESHOLD_COLD;
static int tcold = SYS_THRESHOLD_COLD;

struct case_governor {
	struct thermal_dev *temp_sensor;
	int cooling_level;
	int max_cooling_level;
};

static struct thermal_dev *therm_fw;
static struct case_governor *case_gov;
static struct case_policy *extern_policy;

#ifdef CONFIG_AMZN_VITALS
#define       VITAL_STR_LEN  16
static struct timespec       last_thermal_zone_time;
#endif

#ifdef CONFIG_AMAZON_METRICS_LOG

#define CASE_GOVERNOR_THERMAL_SHUTDOWN_TEMP_FILE_NAME \
		"/data/thermal_shutdown_temp_log"
#define CASE_GOVERNOR_THERMAL_SHUTDOWN_TEMP_FILE_MODE \
		(S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH)

/* write log to temp file func */
static void log_to_metrics_by_temp_file(char *buf)
{
	struct file *fp = NULL;
	fp = filp_open(CASE_GOVERNOR_THERMAL_SHUTDOWN_TEMP_FILE_NAME,
		O_RDWR|O_CREAT|O_TRUNC,
		CASE_GOVERNOR_THERMAL_SHUTDOWN_TEMP_FILE_MODE);
	if (IS_ERR(fp)) {
		pr_emerg("create temp file error\n");
	} else {
		int len = strlen(buf);
		mm_segment_t oldfs = get_fs();
		set_fs(KERNEL_DS);
		int writelen = fp->f_op->write(fp, buf, len, &fp->f_pos);
		if (writelen == len)
			pr_emerg("write temp file success\n");
		else
			pr_emerg("write temp file error\n");
		set_fs(oldfs);
		filp_close(fp, NULL);
	}
}
#endif

static void case_reached_max_state(int temp)
{
#ifdef CONFIG_AMAZON_METRICS_LOG
	char *thermal_metric_prefix = "thermzone:def:monitor=1;CT;1";
	char buf[THERMO_METRICS_STR_LEN];

	snprintf(buf, THERMO_METRICS_STR_LEN,
		"%s,thermal_temp=%d;CT;1,thermal_caught_shutdown=1;CT;1:NR",
		thermal_metric_prefix, temp);
	log_to_metrics(ANDROID_LOG_INFO, "ThermalEvent", buf);
	log_to_metrics_by_temp_file(buf);
#endif
	pr_emerg("%s: shutdown due to thermal case policy (temp == %d)\n",
		__func__, temp);

	/* Sync and shutdown. */
	orderly_poweroff(true);
}

/**
 * DOC: Introduction
 * =================
 * The SYSTEM Thermal governor maintains the policy for the SYSTEM
 * temperature (PCB and/or case). The main goal of the governor is to
 * get the temperature from a thermal sensor and calculate the current
 * temperature of the case. When the case temperature is above the hot
 * limit, then the SYSTEM Thermal governor will cool the system by
 * throttling CPU frequency.
 * When the temperature is below the cold limit, then the SYSTEM Thermal
 * governor will remove any constraints about CPU frequency.
 * The SYSTEM Thermal governor may use 3 different sensors :
 * - CPU (OMAP) on-die temperature sensor (if there is no PCB sensor)
 * - PCB sensor located closed to CPU die
 * - Dedicated sensor for case temperature management
 * To take into account the response delay between the case temperature
 * and the temperature from one of these sensors, the sensor temperature
 * should be averaged.
 *
 * @cooling_list: The list of cooling devices available to cool the zone
 * @temp:	Temperature (average on-die CPU temp or PCB temp sensor)
*/

static int case_thermal_manager(struct list_head *cooling_list, int temp)
{
#ifdef CONFIG_AMAZON_METRICS_LOG
	char *thermal_metric_prefix = "thermzone:def:monitor=1;CT;1";
	char buf[THERMO_METRICS_STR_LEN];
#endif

#ifdef CONFIG_AMZN_VITALS
       struct       timespec curr_thermal_zone_time;
       char         buff[VITAL_STR_LEN];
       static int   last_thermal_zone ;
#endif

	pr_debug("%s: temp: %d thot: %d level: %d sys_thot: %d sys_tcold: %d",
		 __func__, temp, thot, case_gov->cooling_level,
		 sys_threshold_hot, sys_threshold_cold);

	if (temp < sys_threshold_cold) {
		/* Do nothing if system is cool and we're already on level 0. */
		if (case_gov->cooling_level == INIT_COOLING_LEVEL)
			return 0;
		case_gov->cooling_level = INIT_COOLING_LEVEL;
		/* We want to be notified on the first subzone */
		thot = sys_threshold_cold;
		tcold = sys_threshold_cold;

		pr_debug("%s: temp: %d < sys_threshold_cold, thot: %d:%d (%d)",
			 __func__, temp, thot, tcold, case_gov->cooling_level);

		goto update;
	}

	/* no need to update here */
	if (tcold <= temp && temp <= thot)
		return 0;

	if (temp >= sys_threshold_hot) {
		if (temp < tcold) {
			case_gov->cooling_level--;
			thot = tcold;
			tcold -= sys_threshold_hot_inc;
		} else { /* temp > thot */
			case_gov->cooling_level++;
			tcold = thot;
			thot += sys_threshold_hot_inc;
		}

		pr_debug("%s: temp: %d >= sys_threshold_hot, thot: %d:%d (%d)",
			 __func__, temp, thot, tcold, case_gov->cooling_level);

		if (case_gov->cooling_level > case_gov->max_cooling_level)
			case_reached_max_state(temp);
	} else if (temp > thot) { /* sys_thot > temp > sys_tcold */
		case_gov->cooling_level++;
		tcold = thot;
		thot = sys_threshold_cold +
			((sys_threshold_hot - sys_threshold_cold) /
			 case_subzone_number) *
			case_gov->cooling_level;

		pr_debug("%s: sys_thot >= temp: %d >= sys_tcold, %d:%d (%d)",
			 __func__, temp, thot, tcold, case_gov->cooling_level);
	} else if (temp <= tcold) {
		case_gov->cooling_level--;
		thot = tcold;
		tcold = sys_threshold_cold +
			((sys_threshold_hot - sys_threshold_cold) /
			case_subzone_number) * (case_gov->cooling_level - 1);
		pr_debug("%s: sys_thot >= temp: %d reseting %d:%d (%d)",
				__func__, temp, thot, tcold,
				case_gov->cooling_level);
	}

update:
	thermal_device_call_all(cooling_list, cool_device,
						case_gov->cooling_level);
	thermal_device_call(case_gov->temp_sensor, set_temp_thresh,
								tcold, thot);
#ifdef CONFIG_AMAZON_METRICS_LOG
	snprintf(buf, THERMO_METRICS_STR_LEN,
		"%s,thermal_zone=%d;CT;1,temp=%d;DV;1:NR",
		thermal_metric_prefix, case_gov->cooling_level, temp);
	log_to_metrics(ANDROID_LOG_INFO, "ThermalEvent", buf);
#endif

#ifdef CONFIG_AMZN_VITALS
        /* Vital for thermal zone duration */
        get_monotonic_boottime(&curr_thermal_zone_time);
        snprintf(buff, VITAL_STR_LEN, "zone%d",  last_thermal_zone);
        log_counter_to_vitals(ANDROID_LOG_INFO, "thermal engine", "thermal",
                "time_in_thermal_bucket", buff, curr_thermal_zone_time.tv_sec - last_thermal_zone_time.tv_sec, "s", false);
        last_thermal_zone_time = curr_thermal_zone_time;
        last_thermal_zone = case_gov->cooling_level;
#endif

        return 0;
}

static int case_process_temp(struct thermal_dev *gov,
				struct list_head *cooling_list,
				struct thermal_dev *temp_sensor,
				int temp)
{
	int ret;

	case_gov->temp_sensor = temp_sensor;
	ret = case_thermal_manager(cooling_list, temp);

	return ret;
}

void set_case_policy(struct case_policy *policy)
{
	if (policy)
		extern_policy = policy;
}

static int option_get(void *data, u64 *val)
{
	u32 *option = data;

	*val = *option;

	return 0;
}

static int option_set(void *data, u64 val)
{
	u32 *option = data;

	*option = val;

	/* reset intermediate thot & tcold as the sys constraint has changed */
	tcold = sys_threshold_cold;
	thot = sys_threshold_cold;

	thermal_device_call(case_gov->temp_sensor, set_temp_thresh,
				tcold, thot);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(case_fops, option_get, option_set, "%llu\n");

static int case_register_debug_entries(struct thermal_dev *gov,
					struct dentry *d)
{
	(void) debugfs_create_file("case_subzone_number",
			S_IRUGO | S_IWUSR, d, &case_subzone_number,
			&case_fops);
	(void) debugfs_create_file("sys_threshold_hot",
			S_IRUGO | S_IWUSR, d, &sys_threshold_hot,
			&case_fops);
	(void) debugfs_create_file("sys_threshold_cold",
			S_IRUGO | S_IWUSR, d, &sys_threshold_cold,
			&case_fops);
	return 0;
}

static struct thermal_dev_ops case_gov_ops = {
	.process_temp = case_process_temp,
#ifdef CONFIG_THERMAL_FRAMEWORK_DEBUG
	.register_debug_entries = case_register_debug_entries,
#endif
};

static int __init case_governor_init(void)
{
	struct thermal_dev *thermal_fw;
	struct device *dev;
	int tmp, opps;

#ifdef CONFIG_AMZN_VITALS
        get_monotonic_boottime(&last_thermal_zone_time);
#endif
        dev = omap_device_get_by_hwmod_name("mpu");
	if (!dev) {
		pr_err("%s: domain does not know the amount of throttling",
			__func__);
		return -ENODEV;
	}
	opps = opp_get_opp_count(dev);

	dev = omap_device_get_by_hwmod_name("gpu");
	if (!dev) {
		pr_err("%s: domain does not know the amount of throttling",
			__func__);
		return -ENODEV;
	}
	tmp = opp_get_opp_count(dev);
	if (tmp > opps)
		opps = tmp;

	case_gov = kzalloc(sizeof(struct case_governor), GFP_KERNEL);
	if (!case_gov) {
		pr_err("%s:Cannot allocate memory\n", __func__);
		return -ENOMEM;
	}

	thermal_fw = kzalloc(sizeof(struct thermal_dev), GFP_KERNEL);
	if (thermal_fw) {
		thermal_fw->name = "case_governor";
		thermal_fw->domain_name = "case";
		thermal_fw->dev_ops = &case_gov_ops;
		thermal_governor_dev_register(thermal_fw);
		therm_fw = thermal_fw;
	} else {
		pr_err("%s: Cannot allocate memory\n", __func__);
		kfree(case_gov);
		return -ENOMEM;
	}

	if (extern_policy) {
		if (extern_policy->sys_threshold_hot)
			sys_threshold_hot = extern_policy->sys_threshold_hot;
		if (extern_policy->sys_threshold_cold) {
			sys_threshold_cold = extern_policy->sys_threshold_cold;
			tcold = sys_threshold_cold;
		}
		if (extern_policy->case_subzone_number)
			case_subzone_number =
				extern_policy->case_subzone_number;
		if (extern_policy->sys_threshold_hot_inc)
			sys_threshold_hot_inc =
				extern_policy->sys_threshold_hot_inc;
	}

	/* Start governor in zone 1, so we can init case_sensor tcold/thot. */
	thot = sys_threshold_cold + (sys_threshold_hot - sys_threshold_cold) /
							case_subzone_number;
	case_gov->cooling_level = INIT_COOLING_LEVEL + 1;
	case_gov->max_cooling_level = case_subzone_number + opps;

	return 0;
}

static void __exit case_governor_exit(void)
{
	thermal_governor_dev_unregister(therm_fw);
	kfree(therm_fw);
	kfree(case_gov);
}

module_init(case_governor_init);
module_exit(case_governor_exit);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("System/Case thermal governor");
MODULE_LICENSE("GPL");
