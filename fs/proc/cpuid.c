#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

extern unsigned char system_cpuid[20];

static int cpuid_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\n", 
			system_cpuid[0], system_cpuid[1], system_cpuid[2],system_cpuid[3],system_cpuid[4],
			system_cpuid[5], system_cpuid[6], system_cpuid[7],system_cpuid[8],system_cpuid[9],
			system_cpuid[10], system_cpuid[11], system_cpuid[12],system_cpuid[13],system_cpuid[14],
			system_cpuid[15], system_cpuid[16], system_cpuid[17],system_cpuid[18],system_cpuid[19]);

	return 0;
}

static int cpuid_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, cpuid_proc_show, NULL);
}

static const struct file_operations cpuid_proc_fops = {
	.open		= cpuid_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_cpuid_init(void)
{
	proc_create("cpuid", 0, NULL, &cpuid_proc_fops);
	return 0;
}
module_init(proc_cpuid_init);
