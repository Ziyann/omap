/*
 * TI LP855x Backlight Driver
 *
 *			Copyright (C) 2011 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/lp855x.h>
#include <linux/gpio.h>
#include <linux/earlysuspend.h>
#include <linux/math64.h>

#define BRIGHTNESS_CTRL	(0x00)
#define DEVICE_CTRL	(0x01)

#define BRIGHTNESS_CALC_MUL	(10000)
#define NO_OF_EXP_MEMBERS	(10)

int f_lut[] = {
	24246246,
	11517471,
	7288654,
	5184774,
	3930809,
	3101741,
	2515406,
	2080713,
	1747050,
	1484040,
	1272349,
	1099082,
	955307,
	834642,
	732403,
	645074,
	569966,
	504985,
	448478,
	399119,
	355838,
	317756,
	284148,
	254410,
	228034,
	204593,
	183721,
	165105,
	148479,
	133609,
	120294,
	108360,
	97653,
	88040,
	79401,
	71632,
	64643,
	58350,
	52683,
	47576 };

#ifdef CONFIG_DEBUG_FS
struct debug_dentry {
	struct dentry *dir;
	struct dentry *reg;
	struct dentry *chip;
	struct dentry *blmode;
};
#endif

struct lp855x {
	const char *chipid;
	struct i2c_client *client;
	struct backlight_device *bl;
	struct device *dev;
	struct mutex xfer_lock;
	struct lp855x_platform_data *pdata;
#ifdef CONFIG_DEBUG_FS
	struct debug_dentry dd;
#endif
	// two stage suspend/resume
	struct early_suspend suspend1;
	struct early_suspend suspend2;
#ifdef CONFIG_HAS_EARLYSUSPEND
	int    power_enabled_state;
#endif
};

static int lp855x_i2c_read(struct lp855x *lp, u8 reg, u8 *data, u8 len)
{
	s32 ret;

	mutex_lock(&lp->xfer_lock);
	ret = i2c_smbus_read_i2c_block_data(lp->client, reg, len, data);
	mutex_unlock(&lp->xfer_lock);

	return (ret != len) ? -EIO : 0;
}

static int lp855x_i2c_write(struct lp855x *lp, u8 reg, u8 *data, u8 len)
{
	s32 ret = 0;

	mutex_lock(&lp->xfer_lock);
	ret = i2c_smbus_write_i2c_block_data(lp->client, reg, len, data);
	mutex_unlock(&lp->xfer_lock);

	return ret;
}

static inline int lp855x_read_byte(struct lp855x *lp, u8 reg, u8 *data)
{
	return lp855x_i2c_read(lp, reg, data, 1);
}

static inline int lp855x_write_byte(struct lp855x *lp, u8 reg, u8 data)
{
	u8 written = data;
	return lp855x_i2c_write(lp, reg, &written, 1);
}

#ifdef CONFIG_DEBUG_FS
static int lp855x_dbg_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t lp855x_help_register(struct file *file, char __user *userbuf,
				    size_t count, loff_t *ppos)
{
	char buf[320];
	unsigned int len;
	const char *help = "\n How to read/write LP855x registers\n\n \
	(example) To read 0x00 register,\n \
	echo 0x00 r > /sys/kernel/debug/lp855x/registers\n \
	To write 0xff into 0x1 address,\n \
	echo 0x00 0xff w > /sys/kernel/debug/lp855x/registers \n \
	To dump values from 0x00 to 0x06 address,\n \
	echo 0x00 0x06 d > /sys/kernel/debug/lp855x/registers\n";

	len = snprintf(buf, sizeof(buf), "%s\n", help);
	if (len > sizeof(buf))
		len = sizeof(buf);

	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static char *lp855x_parse_register_cmd(const char *cmd, u8 *byte)
{
	char tmp[10];
	char *blank;
	unsigned long arg;

	blank = strchr(cmd, ' ');
	memset(tmp, 0x0, sizeof(tmp));
	memcpy(tmp, cmd, blank - cmd);

	if (strict_strtol(tmp, 16, &arg) < 0)
		return NULL;

	*byte = arg;
	return blank;
}

static ssize_t lp855x_ctrl_register(struct file *file,
				    const char __user *userbuf, size_t count,
				    loff_t *ppos)
{
	char mode, buf[20];
	char *pos, *pos2;
	u8 i, arg1, arg2, val;
	struct lp855x *lp = file->private_data;

	if (copy_from_user(buf, userbuf, min(count, sizeof(buf))))
		return -EFAULT;

	mode = buf[count - 2];
	switch (mode) {
	case 'r':
		if (!lp855x_parse_register_cmd(buf, &arg1))
			return -EINVAL;

		lp855x_read_byte(lp, arg1, &val);
		dev_info(lp->dev, "Read [0x%.2x] = 0x%.2x\n", arg1, val);
		break;
	case 'w':
		pos = lp855x_parse_register_cmd(buf, &arg1);
		if (!pos)
			return -EINVAL;
		pos2 = lp855x_parse_register_cmd(pos + 1, &arg2);
		if (!pos2)
			return -EINVAL;

		lp855x_write_byte(lp, arg1, arg2);
		dev_info(lp->dev, "Written [0x%.2x] = 0x%.2x\n", arg1, arg2);
		break;
	case 'd':
		pos = lp855x_parse_register_cmd(buf, &arg1);
		if (!pos)
			return -EINVAL;
		pos2 = lp855x_parse_register_cmd(pos + 1, &arg2);
		if (!pos2)
			return -EINVAL;

		for (i = arg1; i <= arg2; i++) {
			lp855x_read_byte(lp, i, &val);
			dev_info(lp->dev, "Read [0x%.2x] = 0x%.2x\n", i, val);
		}
		break;
	default:
		break;
	}

	return count;
}

static ssize_t lp855x_get_chipid(struct file *file, char __user *userbuf,
				 size_t count, loff_t *ppos)
{
	struct lp855x *lp = file->private_data;
	char buf[10];
	unsigned int len;

	len = snprintf(buf, sizeof(buf), "%s\n", lp->chipid);

	if (len > sizeof(buf))
		len = sizeof(buf);

	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t lp855x_get_bl_mode(struct file *file, char __user *userbuf,
				  size_t count, loff_t *ppos)
{
	char buf[20];
	unsigned int len;
	char *strmode = NULL;
	struct lp855x *lp = file->private_data;
	enum lp855x_brightness_ctrl_mode mode = lp->pdata->mode;

	if (mode == PWM_BASED)
		strmode = "pwm based";
	else if (mode == REGISTER_BASED)
		strmode = "register based";

	len = snprintf(buf, sizeof(buf), "%s\n", strmode);

	if (len > sizeof(buf))
		len = sizeof(buf);

	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

#define LP855X_DBG_ENTRY(name, pread, pwrite) \
static const struct file_operations dbg_##name##_fops = { \
	.open = lp855x_dbg_open, \
	.read = pread, \
	.write = pwrite, \
	.owner = THIS_MODULE, \
	.llseek = default_llseek, \
}

LP855X_DBG_ENTRY(registers, lp855x_help_register, lp855x_ctrl_register);
LP855X_DBG_ENTRY(chip, lp855x_get_chipid, NULL);
LP855X_DBG_ENTRY(blmode, lp855x_get_bl_mode, NULL);

static void lp855x_create_debugfs(struct lp855x *lp)
{
	struct debug_dentry *dd = &lp->dd;

	dd->dir = debugfs_create_dir("lp855x", NULL);

	dd->reg = debugfs_create_file("registers", S_IWUSR | S_IRUGO,
				      dd->dir, lp, &dbg_registers_fops);

	dd->chip = debugfs_create_file("chip_id", S_IRUGO,
				       dd->dir, lp, &dbg_chip_fops);

	dd->blmode = debugfs_create_file("bl_ctl_mode", S_IRUGO,
					 dd->dir, lp, &dbg_blmode_fops);
}

static void lp855x_remove_debugfs(struct lp855x *lp)
{
	struct debug_dentry *dd = &lp->dd;

	debugfs_remove(dd->blmode);
	debugfs_remove(dd->chip);
	debugfs_remove(dd->reg);
	debugfs_remove(dd->dir);
}
#else
static inline void lp855x_create_debugfs(struct lp855x *lp)
{
	return;
}

static inline void lp855x_remove_debugfs(struct lp855x *lp)
{
	return;
}
#endif

static int lp855x_is_valid_rom_area(struct lp855x *lp, u8 addr)
{
	const char *id = lp->chipid;
	u8 start, end;

	if (strstr(id, "lp8550") || strstr(id, "lp8551")
	    || strstr(id, "lp8552") || strstr(id, "lp8553")) {
		start = EEPROM_START;
		end = EEPROM_END;
	} else if (strstr(id, "lp8556")) {
		start = EPROM_START;
		end = EPROM_END;
	}

	return (addr >= start && addr <= end) ? 1 : 0;
}

static void lp855x_init_device(struct lp855x *lp)
{
	u8 val, addr;
	int i, ret;
	struct lp855x_platform_data *pd = lp->pdata;

	val = 0;
	ret = lp855x_write_byte(lp, BRIGHTNESS_CTRL, val);

	val = pd->device_control;
	ret |= lp855x_write_byte(lp, DEVICE_CTRL, val);

	/* only apply on lp8556 */
	if (strstr(lp->chipid, "lp8556")) {
		dev_dbg(lp->dev, "writing CFG5 setting[0x%02X]\n", pd->led_setting);
		ret = lp855x_write_byte(lp, CFG5_REG, pd->led_setting);
	}

	/* only apply on lp8556 */
	if (strstr(lp->chipid, "lp8556")) {
		lp855x_read_byte(lp, CFG6_REG, &val);
		dev_dbg(lp->dev, "boost frequency setting [0x%02X]\n", val);
		if (val != pd->boost_freq) {
			val &= 0x3f;
			val |= pd->boost_freq;
			dev_dbg(lp->dev, "update boost frequency to [0x%02X]\n", val);
			ret = lp855x_write_byte(lp, CFG6_REG, val);
		}
	}

	if (pd->load_new_rom_data && pd->size_program) {
		for (i = 0; i < pd->size_program; i++) {
			addr = pd->rom_data[i].addr;
			val = pd->rom_data[i].val;
			if (!lp855x_is_valid_rom_area(lp, addr))
				continue;

			ret |= lp855x_write_byte(lp, addr, val);
		}
	}

	if (ret)
		dev_err(lp->dev, "i2c write err\n");
}

/** Implement: return = (e^((a/10)*x/255)-1)*(255/(e^(a/10)-1)) **/
u8 calc_exp(int a, u8 x)
{
	u64 sum, temp;
	int i;

	sum = x * a * BRIGHTNESS_CALC_MUL;
	sum = div_u64(sum, 2550);

	temp = sum;

	for (i = 2; i < NO_OF_EXP_MEMBERS; i++) {
		temp = temp * x * a;
		temp = div_u64(temp, 2550 * i);
		sum += temp;
	}

	sum *= f_lut[a - 1];
	sum = div_u64(sum, BRIGHTNESS_CALC_MUL *
				BRIGHTNESS_CALC_MUL);

	if (sum >= 254)
		sum = 255;

	return sum;
}

static int lp855x_bl_update_status(struct backlight_device *bl)
{
	struct lp855x *lp = bl_get_data(bl);
	enum lp855x_brightness_ctrl_mode mode = lp->pdata->mode;

#ifdef CONFIG_HAS_EARLYSUSPEND
	if (!lp->power_enabled_state)
		return bl->props.brightness;
#endif
	if (bl->props.state & BL_CORE_SUSPENDED)
		bl->props.brightness = 0;

	if (mode == PWM_BASED) {
		struct lp855x_pwm_data *pd = &lp->pdata->pwm_data;
		int br = bl->props.brightness;
		int max_br = bl->props.max_brightness;

		if (pd->pwm_set_intensity)
			pd->pwm_set_intensity(br, max_br);

	} else if (mode == REGISTER_BASED) {
		u8 val = bl->props.brightness;

		if (lp->pdata->nonlinearity_factor != 0) {
			u8 exp_val;

			exp_val = calc_exp(lp->pdata->nonlinearity_factor, val);

			lp855x_write_byte(lp, BRIGHTNESS_CTRL, exp_val);
		} else
			lp855x_write_byte(lp, BRIGHTNESS_CTRL, val);
	}

	if (lp->pdata->power_off)
		if (bl->props.state & BL_CORE_SUSPENDED)
			lp->pdata->power_off(&lp->client->dev);

	return (bl->props.brightness);
}

static int lp855x_bl_get_brightness(struct backlight_device *bl)
{
	struct lp855x *lp = bl_get_data(bl);
	enum lp855x_brightness_ctrl_mode mode = lp->pdata->mode;

	if (mode == PWM_BASED) {
		struct lp855x_pwm_data *pd = &lp->pdata->pwm_data;
		int max_br = bl->props.max_brightness;

		if (pd->pwm_get_intensity)
			bl->props.brightness = pd->pwm_get_intensity(max_br);

	} else if (mode == REGISTER_BASED) {
		u8 val;

		lp855x_read_byte(lp, BRIGHTNESS_CTRL, &val);
		bl->props.brightness = val;
	}

	return (bl->props.brightness);
}

static const struct backlight_ops lp855x_bl_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = lp855x_bl_update_status,
	.get_brightness = lp855x_bl_get_brightness,
};

static int lp855x_backlight_register(struct lp855x *lp)
{
	struct backlight_device *bl;
	struct backlight_properties props;
	const char *name = lp->pdata->name;

	if (!name)
		return -ENODEV;

	props.type = BACKLIGHT_RAW;
	props.brightness = lp->pdata->initial_brightness;
	props.max_brightness =
		(lp->pdata->max_brightness < lp->pdata->initial_brightness) ?
		255 : lp->pdata->max_brightness;

	bl = backlight_device_register(name, lp->dev, lp,
				       &lp855x_bl_ops, &props);
	if (IS_ERR(bl))
		return -EIO;

	lp->bl = bl;

	return 0;
}

static void lp855x_backlight_unregister(struct lp855x *lp)
{
	if (lp->bl)
		backlight_device_unregister(lp->bl);
}

static int lp855x_resume(struct i2c_client *client);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void lp855x_late_resume1(struct early_suspend *h)
{
	struct lp855x *lp = container_of(h, struct lp855x, suspend1);
	int result = 0;

	if (lp->pdata->power_on) {
		result = lp->pdata->power_on(&lp->client->dev);
	}

	if (!result)
		lp855x_init_device(lp);
}

static void lp855x_early_suspend1(struct early_suspend *h)
{
	// this is a noop
}

static void lp855x_late_resume2(struct early_suspend *h)
{
	struct lp855x *lp = container_of(h, struct lp855x, suspend2);
	struct lp855x_platform_data *pd = lp->pdata;
	u8 val = 0;

	lp855x_read_byte(lp, CFG5_REG, &val);

	if (pd->led_setting != val) {
		dev_warn(lp->dev, "EPROM reset to defaults, 0x%02x expected 0x%02x\n", val, pd->led_setting);
	}

	lp855x_init_device(lp);
	lp->power_enabled_state = 1;
	lp855x_bl_update_status(lp->bl);
}

static void lp855x_early_suspend2(struct early_suspend *h)
{
	struct lp855x *lp = container_of(h, struct lp855x, suspend2);

	//lp855x_write_byte(lp, BRIGHTNESS_CTRL, 0);
	//lp855x_write_byte(lp, DEVICE_CTRL, 0x4);

	if (lp->pdata->power_off) {
		lp->pdata->power_off(&lp->client->dev);
	}

	lp->power_enabled_state = 0;
}
#endif

static int lp855x_probe(struct i2c_client *cl, const struct i2c_device_id *id)
{
	struct lp855x *lp;
	struct lp855x_platform_data *pdata = cl->dev.platform_data;
	int ret;

	if (pdata->request_resources) {
		pdata->request_resources(&cl->dev);
	}

	if (!i2c_check_functionality(cl->adapter, I2C_FUNC_SMBUS_I2C_BLOCK))
		goto err_io;

	lp = kzalloc(sizeof(struct lp855x), GFP_KERNEL);
	if (!lp)
		goto err_mem;

	lp->client = cl;
	lp->dev = &cl->dev;
	lp->pdata = pdata;
	lp->chipid = id->name;
	i2c_set_clientdata(cl, lp);

	if (lp->pdata->nonlinearity_factor > 40) {
		lp->pdata->nonlinearity_factor = 40;
		printk(KERN_INFO "Backlight (%s): nonlinearity_factor out of range! (1 < X < 40)\n", __func__);
		printk(KERN_INFO "Backlight (%s): nonlinearity_factor limited to 40!\n", __func__);
	} else if (lp->pdata->nonlinearity_factor == 0) {
		printk(KERN_INFO "Backlight (%s): nonlinearity_factor not defined!\n", __func__);
		printk(KERN_INFO "Backlight (%s): Using linear characteristic!\n", __func__);
	}

	mutex_init(&lp->xfer_lock);
	lp855x_resume(cl);
	ret = lp855x_backlight_register(lp);
	if (ret)
		goto err_dev;

#ifdef CONFIG_HAS_EARLYSUSPEND
	lp->power_enabled_state = 1;
#endif
	lp->bl->props.brightness = pdata->initial_brightness;
	lp855x_bl_update_status(lp->bl);

	lp855x_create_debugfs(lp);

#ifdef CONFIG_HAS_EARLYSUSPEND
	// Run the backlight suspend before display
	lp->suspend1.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 4;
	lp->suspend1.suspend = lp855x_early_suspend1;
	lp->suspend1.resume = lp855x_late_resume1;
	register_early_suspend(&lp->suspend1);

	lp->suspend2.level = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1;
	lp->suspend2.suspend = lp855x_early_suspend2;
	lp->suspend2.resume = lp855x_late_resume2;
	register_early_suspend(&lp->suspend2);
#endif

	return ret;

err_io:
	return -EIO;
err_mem:
	return -ENOMEM;
err_dev:
	dev_err(lp->dev, "can not register backlight device. errcode = %d\n",
		ret);
	kfree(lp);
	return ret;
}

static int __devexit lp855x_remove(struct i2c_client *cl)
{
	struct lp855x *lp = i2c_get_clientdata(cl);

	lp->bl->props.brightness = 0;
	backlight_update_status(lp->bl);
	unregister_early_suspend(&lp->suspend1);
	unregister_early_suspend(&lp->suspend2);
	lp855x_remove_debugfs(lp);
	lp855x_backlight_unregister(lp);
	kfree(lp);

	return 0;
}

static void lp855x_shutdown(struct i2c_client *cl)
{
	struct lp855x *lp = i2c_get_clientdata(cl);
	lp->bl->props.brightness = 0;
	backlight_update_status(lp->bl);

	if (lp->pdata->power_off) {
		lp->pdata->power_off(&lp->client->dev);
	}
}

static const struct i2c_device_id lp855x_ids[] = {
	{"lp8550", LP8550},
	{"lp8551", LP8551},
	{"lp8552", LP8552},
	{"lp8553", LP8553},
	{"lp8556", LP8556},
};

static struct i2c_driver lp855x_driver = {
	.driver = {
		   .name = "lp855x",
		   },
	.probe = lp855x_probe,
	.remove = __devexit_p(lp855x_remove),
#ifndef CONFIG_HAS_EARLYSUSPEND
	.resume = lp855x_resume,
#endif
	.id_table = lp855x_ids,
	.shutdown = lp855x_shutdown,
};

static int lp855x_resume(struct i2c_client *client)
{
	struct lp855x_platform_data *pdata = client->dev.platform_data;
	struct lp855x *lp = i2c_get_clientdata(client);
	int result = 0;

	if (pdata->power_on) {
		result = pdata->power_on(&client->dev);
	}

	if (!result)
		lp855x_init_device(lp);

	return 0;
}

static int __init lp855x_init(void)
{
	return i2c_add_driver(&lp855x_driver);
}

static void __exit lp855x_exit(void)
{
	i2c_del_driver(&lp855x_driver);
}

module_init(lp855x_init);
module_exit(lp855x_exit);

MODULE_DESCRIPTION("Texas Instruments LP855x Backlight driver");
MODULE_AUTHOR("Milo Kim <milo.kim@ti.com>, Dainel Jeong <daniel.jeong@ti.com>");
MODULE_LICENSE("GPL");
