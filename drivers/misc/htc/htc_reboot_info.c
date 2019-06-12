#include <linux/atomic.h>
#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/atomic.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <asm-generic/sizes.h>
#include <linux/reboot.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/htc_reboot_info.h>
#include <linux/htc_devices_dtb.h>

typedef struct{
	unsigned int	reboot_reason;
	unsigned char	reboot_string[HTC_REBOOT_INFO_REBOOT_STRING_SIZE];
	unsigned char	reserved[512 - 4 - HTC_REBOOT_INFO_REBOOT_STRING_SIZE];
} htc_reboot_info_struct;

typedef struct{
	unsigned int  is_support;
	unsigned int  offset;
	const char*   partition_path;
} htc_reboot_info_save_emmc_support_struct;

typedef struct{
	int initialized;
	phys_addr_t phys;
	resource_size_t size;
	htc_reboot_info_save_emmc_support_struct save_emmc_support;
	htc_reboot_info_struct *htc_reboot_info_p;
	htc_reboot_info_struct *last_htc_reboot_info_p;
} htc_reboot_info_drv_struct;

static htc_reboot_info_drv_struct htc_reboot_info = {
	.initialized = 0,
	.save_emmc_support = {
		.is_support = 0,
		.offset = 0x0,
		.partition_path = NULL,
	}
};

static int htc_is_valid_stock_reboot_info(void) {
	if (htc_reboot_info.initialized && htc_reboot_info.htc_reboot_info_p) {
		return IS_HTC_STOCK_REBOOT_INFO(htc_reboot_info.htc_reboot_info_p->reboot_reason)?1:0;
	}
	return 0;
}

static int htc_is_valid_oem_reboot_info(void) {
	if (htc_reboot_info.initialized && htc_reboot_info.htc_reboot_info_p) {
		return IS_HTC_OEM_REBOOT_INFO(htc_reboot_info.htc_reboot_info_p->reboot_reason)?1:0;
	}
	return 0;
}

int htc_is_valid_reboot_info(void) {
	if (htc_is_valid_stock_reboot_info() || htc_is_valid_oem_reboot_info() ) {
		return 1;
	}
	return 0;
}
EXPORT_SYMBOL(htc_is_valid_reboot_info);

int htc_is_need_sw_reboot(void) {

	int ret = 0;

	if (!htc_reboot_info.initialized) {
		return 1;
	}

	switch( htc_reboot_info.htc_reboot_info_p->reboot_reason) {
		case HTC_REBOOT_INFO_NORMAL:
		case HTC_REBOOT_INFO_BOOTLOADER:
		case HTC_REBOOT_INFO_REBOOT:
			ret = 0;
			break;
		case HTC_REBOOT_INFO_RAMDUMP:
			if (get_radio_flag() & 0x8) {
				//RAMDUMP Debug Enable, so SW_RESET for RD Debugging
				ret = 1;
			} else {
				//RAMDUMP Debug Disable, so HW_RESET for End User Device
				ret = 0;
			}
			break;
		case HTC_REBOOT_INFO_RECOVERY:
		case HTC_REBOOT_INFO_REBOOT_RUU:
		case HTC_REBOOT_INFO_DOWNLOAD_MODE:
		case HTC_REBOOT_INFO_FTM_1:
		case HTC_REBOOT_INFO_FTM_2:
		case HTC_REBOOT_INFO_FTM_3:
			ret = 1;
			break;
		default:
			ret = 0;
	}

	return ret;
}
EXPORT_SYMBOL(htc_is_need_sw_reboot);

#define HTC_REBOOT_CMD_CMP(htc_cmd, cmd) (strlen(htc_cmd) == strlen(cmd) && 0 == strncmp(htc_cmd, cmd, strlen(htc_cmd)))

int htc_reboot_reason_update_by_cmd(const char* cmd) {
	int ret = -EINVAL;
	pr_crit("%s: cmd:%s\n", __func__, cmd);
	if (htc_reboot_info.initialized) {
		if (cmd == NULL) {
			ret = htc_set_reboot_reason(0, NULL);
		} else if(strlen(cmd) > 4 && !strncmp(cmd, "oem-", 4)){
			unsigned long code = simple_strtoul(cmd + 4, NULL, 16) & 0xff;
			ret = htc_set_reboot_reason(SET_HTC_OEM_REBOOT_REASON(code), cmd);
		} else {
			if (HTC_REBOOT_CMD_CMP("recovery" , cmd)) {
				ret = htc_set_reboot_reason(HTC_REBOOT_INFO_RECOVERY, cmd);
			} else if(HTC_REBOOT_CMD_CMP("bootloader", cmd)) {
				ret = htc_set_reboot_reason(HTC_REBOOT_INFO_BOOTLOADER, cmd);
			} else if(HTC_REBOOT_CMD_CMP("download", cmd)) {
				ret = htc_set_reboot_reason(HTC_REBOOT_INFO_DOWNLOAD_MODE, cmd);
			} else if(HTC_REBOOT_CMD_CMP("meta", cmd)) {
				ret = htc_set_reboot_reason(HTC_REBOOT_INFO_META_MODE, cmd);
			} else if(HTC_REBOOT_CMD_CMP("ftm", cmd)) {
				ret = htc_set_reboot_reason(HTC_REBOOT_INFO_FACTORY_MODE, cmd);
			} else if(HTC_REBOOT_CMD_CMP("force-dog-bark", cmd)) {
				ret = htc_set_reboot_reason(HTC_REBOOT_INFO_RAMDUMP, cmd);
			} else if(HTC_REBOOT_CMD_CMP("force-hard", cmd)) {
				ret = htc_set_reboot_reason(HTC_REBOOT_INFO_RAMDUMP, cmd);
			} else {
				/** non-support reboot command here, just record the reboot cmd **/
				ret = htc_set_reboot_reason(0, cmd);
			}
		}
	}
	mdelay(1000);
	return ret;
}
EXPORT_SYMBOL(htc_reboot_reason_update_by_cmd);

static int htc_reboot_info_save_emmc(char* cmd) {
	struct file *file;
	mm_segment_t old_fs;
	int ret = 0;

	pr_crit("%s: cmd: %s\n", __func__, cmd);

	if (htc_reboot_info.initialized == 0) {
		pr_err("%s: htc_reboot_info is not initialized!\n", __func__);
		ret = -1;
	goto error_exit;
	}

	if (!htc_reboot_info.save_emmc_support.is_support) {
		pr_err("%s: htc_reboot_info is not support save to eMMC!\n", __func__);
		ret = -1;
		goto error_exit;
	}

	htc_reboot_reason_update_by_cmd(cmd);

	file = filp_open(htc_reboot_info.save_emmc_support.partition_path, O_WRONLY, 0);
	if (IS_ERR(file)) {
		pr_err("%s: open %s failed!\n", __func__, htc_reboot_info.save_emmc_support.partition_path);
		ret = -EIO;
		goto error_exit;
	}

	ret = kernel_write(file, (char *)(htc_reboot_info.htc_reboot_info_p), sizeof(htc_reboot_info_struct), htc_reboot_info.save_emmc_support.offset);
	if ( ret < 0 ) {
		pr_err("%s: kernel_write failed! ret=%d\n", __func__, ret);
	}
	old_fs = get_fs();  //Save the current FS segment
	set_fs(get_ds());

	ret = vfs_fsync(file, 0);
	if ( ret < 0 ) {
		pr_err("%s: vfs_fsync failed! ret=%d\n", __func__, ret);
	}

	filp_close(file, NULL);

	set_fs(old_fs); //Reset to save FS

	pr_crit("%s ok!\n", __func__);
	ret = 0;
	return ret;

error_exit:
       return ret;
}

static int htc_reboot_info_notify_sys(struct notifier_block *this, unsigned long event, void *cmd)
{
	pr_crit("%s: event: %ld\n", __func__, event);
	switch (event) {
		case SYS_RESTART:
			htc_reboot_info_save_emmc((char*)cmd);
			return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}

static struct notifier_block htc_reboot_info_reboot_notifier = {
	.notifier_call  = htc_reboot_info_notify_sys,
	.priority = 0,
};

static int htc_reboot_info_register_reboot_notify(void) {
	int err = register_reboot_notifier(&htc_reboot_info_reboot_notifier);
	if (err < 0) {
		pr_err("%s: Register reboot notifier failed(err=%d)\n", __func__, err);
	} else {
		pr_info("%s ok!\n", __func__);
	}
	return err;
}

static int htc_reboot_info_initialize(void) {
	if (htc_reboot_info.initialized != 0) {
		pr_info("%s: htc_reboot_info has been inited already!\n", __func__);
		return htc_reboot_info.initialized;
	}

	htc_reboot_info.htc_reboot_info_p = (htc_reboot_info_struct*) ioremap(htc_reboot_info.phys, sizeof(htc_reboot_info_struct));
	if ( NULL == htc_reboot_info.htc_reboot_info_p ) {
	#ifdef CONFIG_PHYS_ADDR_T_64BIT
		pr_err("%s: htc_reboot_info.htc_reboot_info_p ioremap failed! phys:0x%.16llX size:%lu\n",
					__func__, htc_reboot_info.phys, sizeof(htc_reboot_info_struct));
	#else
		pr_err("%s: htc_reboot_info.htc_reboot_info_p ioremap failed! phys:0x%.8X size:%lu\n",
					__func__, htc_reboot_info.phys, sizeof(htc_reboot_info_struct));
	#endif
		goto error_exit;
	}

	htc_reboot_info.last_htc_reboot_info_p = (htc_reboot_info_struct*) ioremap(htc_reboot_info.phys + sizeof(htc_reboot_info_struct), sizeof(htc_reboot_info_struct));
	if ( NULL == htc_reboot_info.last_htc_reboot_info_p ) {
	#ifdef CONFIG_PHYS_ADDR_T_64BIT
		pr_err("%s: htc_reboot_info.last_htc_reboot_info_p ioremap failed! phys:0x%.16llX size:%lu\n",
					__func__, htc_reboot_info.phys + sizeof(htc_reboot_info_struct), sizeof(htc_reboot_info_struct));
	#else
		pr_err("%s: htc_reboot_info.last_htc_reboot_info_p ioremap failed! phys:0x%.8X size:%lu\n",
					__func__, htc_reboot_info.phys + sizeof(htc_reboot_info_struct), sizeof(htc_reboot_info_struct));
	#endif
		goto error_exit;
	}

	if (htc_reboot_info.save_emmc_support.is_support) {
		htc_reboot_info_register_reboot_notify();
	}

	if (!htc_is_valid_reboot_info()) {
		memset(htc_reboot_info.htc_reboot_info_p, 0, sizeof(htc_reboot_info_struct));
	}
	memset(htc_reboot_info.htc_reboot_info_p, 0, sizeof(htc_reboot_info_struct));
	htc_reboot_info.last_htc_reboot_info_p->reboot_string[HTC_REBOOT_INFO_REBOOT_STRING_SIZE] = 0;

	/* Set UNKNOWN RAMDUMP Reason as Default  */
	htc_set_reboot_reason(HTC_REBOOT_INFO_RAMDUMP, "UNKNOWN");

	htc_reboot_info.initialized = 1;

	return htc_reboot_info.initialized;

error_exit:
	if (NULL != htc_reboot_info.htc_reboot_info_p) {
		iounmap(htc_reboot_info.htc_reboot_info_p);
		htc_reboot_info.htc_reboot_info_p = NULL;
	}

	if (NULL != htc_reboot_info.last_htc_reboot_info_p) {
		iounmap(htc_reboot_info.last_htc_reboot_info_p);
		htc_reboot_info.last_htc_reboot_info_p = NULL;
	}

	htc_reboot_info.initialized = 0;

	return htc_reboot_info.initialized;
}

int htc_set_reboot_reason(unsigned int reboot_reason, const unsigned char* reboot_string) {
	if (htc_reboot_info.initialized && htc_reboot_info.htc_reboot_info_p) {
		memset(htc_reboot_info.htc_reboot_info_p, 0, sizeof(htc_reboot_info_struct));
		htc_reboot_info.htc_reboot_info_p->reboot_reason = reboot_reason;
		if (reboot_string != NULL) {
			strncpy(htc_reboot_info.htc_reboot_info_p->reboot_string, reboot_string,
				sizeof(htc_reboot_info.htc_reboot_info_p->reboot_string)-1);
		}
		return 0;
	}
	return -1;
}
EXPORT_SYMBOL(htc_set_reboot_reason);

int htc_set_reboot_reason_ramdump(const unsigned char* msg)
{
	htc_set_reboot_reason(HTC_REBOOT_INFO_RAMDUMP, msg);
	return 0;
}
EXPORT_SYMBOL(htc_set_reboot_reason_ramdump);

int htc_set_reboot_reason_ramdump_by_panic(const unsigned char* msg)
{
	unsigned char panic_msg[HTC_REBOOT_INFO_REBOOT_STRING_SIZE] = "Kernel Panic";

	if (msg != NULL) {
		snprintf(panic_msg, sizeof(panic_msg), "KP: %s", msg);
	}
	htc_set_reboot_reason(HTC_REBOOT_INFO_RAMDUMP, panic_msg);
	return 0;
}
EXPORT_SYMBOL(htc_set_reboot_reason_ramdump_by_panic);

unsigned int htc_get_reboot_reason(void) {
	if (!htc_reboot_info.initialized) {
		return 0;
	}
	return htc_reboot_info.last_htc_reboot_info_p->reboot_reason;

}
EXPORT_SYMBOL(htc_get_reboot_reason);

unsigned char* htc_get_reboot_string(void) {
	if (!htc_reboot_info.initialized) {
		return NULL;
	}
	return htc_reboot_info.last_htc_reboot_info_p->reboot_string;
}
EXPORT_SYMBOL(htc_get_reboot_string);

static int __init htc_reboot_info_init(void) {
	struct device_node *np = NULL;
	struct resource res = {0};
	int ret;

	if (0 != htc_reboot_info.initialized) {
		pr_err("%s: htc_reboot_info has been initialized already!\n", __func__);
		return -EINVAL;
	}

	if(!(np = of_find_compatible_node( NULL, NULL, "htc,reboot-info"))) {
		pr_err("%s: of_find_compatible_node failed! name:%s\n", __func__, "htc,reboot-info");
		return -EINVAL;
	}

	if (0 != (ret = of_address_to_resource(np, 0, &res)) ) {
		pr_err("%s: of_address_to_resource Failed! ret=%d\n", __func__, ret);
		return ret;
	}

	htc_reboot_info.size = resource_size(&res);
	htc_reboot_info.phys = res.start;

	pr_notice("htc_reboot_info size: 0x%.8X\n", (unsigned int)htc_reboot_info.size);

	if (sizeof(htc_reboot_info_struct)*2 < htc_reboot_info.size ) {
		pr_err("%s: htc_reboot_info size is not correct! it shall be >= %lu\n", __func__, sizeof(htc_reboot_info_struct)*2);
		return -EINVAL;
	}
	#ifdef CONFIG_PHYS_ADDR_T_64BIT
	pr_notice("htc_reboot_info set ok: phys: 0x%.16llX, size: 0x%.8X\n", htc_reboot_info.phys, (unsigned int)htc_reboot_info.size);
	#else
	pr_notice("htc_reboot_info set ok: phys: 0x%.8X, size: 0x%.8X\n", htc_reboot_info.phys, (unsigned int)htc_reboot_info.size);
	#endif


	if ( 0 != of_property_read_u32(np, "save-emmc-feature", &htc_reboot_info.save_emmc_support.is_support ) ) {
		pr_err("%s: save-emmc-feature read failed!\n", __func__);
		htc_reboot_info.save_emmc_support.is_support = 0;
	}

	if (htc_reboot_info.save_emmc_support.is_support) {
		if ( 0 != of_property_read_u32(np, "save-emmc-offset", &htc_reboot_info.save_emmc_support.offset ) ) {
			pr_err("%s: save-emmc-offset read failed!\n", __func__);
			htc_reboot_info.save_emmc_support.is_support = 0;
			htc_reboot_info.save_emmc_support.offset = 0x0;
		}
	}

	if (htc_reboot_info.save_emmc_support.is_support) {
		if ( 0 != of_property_read_string(np, "save-emmc-partition", &htc_reboot_info.save_emmc_support.partition_path ) ) {
			pr_err("%s: save-emmc-partition read failed!\n", __func__);
			htc_reboot_info.save_emmc_support.is_support = 0;
			htc_reboot_info.save_emmc_support.partition_path = NULL;
		}
	}

	if (htc_reboot_info.save_emmc_support.is_support) {
		pr_crit("%s: save emmc is supported\n", __func__);
		pr_crit("%s: save emmc offset: 0x%.8X\n", __func__, htc_reboot_info.save_emmc_support.offset);
		pr_crit("%s: save emmc partition: %s\n", __func__, htc_reboot_info.save_emmc_support.partition_path);
	} else {
		pr_crit("%s: save emmc is not supported\n", __func__);
	}

	htc_reboot_info_initialize();

	if (0 == htc_reboot_info.initialized) {
		pr_err("%s: htc_reboot_info init failed!\n", __func__);
		return -EINVAL;
	} else {
		pr_crit("%s: htc_reboot_info init OK!\n", __func__);
		pr_crit("%s: last reboot_reason: 0x%.8X\n", __func__, htc_get_reboot_reason());
		pr_crit("%s: last reboot_string: %s\n", __func__, htc_get_reboot_string());
	}

	return 0;

}

arch_initcall(htc_reboot_info_init);
