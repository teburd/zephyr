#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

static int __init intel_adsp_ztestmod_init(void)
{

	printk(KERN_INFO "Intel ADSP ZTestMod\n");

	return 0;
}

static void __exit intel_adsp_ztestmod_exit(void)
{
	printk(KERN_INFO "Cleaning up Intel ADSP ZTestMod\n");
}


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tom Burdick <thomas.burdick@intel.com>");
MODULE_DESCRIPTION("Provide a host side to perform tests against for Zephyr and Intel ADSP");
MODULE_VERSION("0.1");

module_init(intel_adsp_ztestmod_init);
module_exit(intel_adsp_ztestmod_cleanup);
