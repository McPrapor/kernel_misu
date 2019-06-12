#include <linux/htc_devices_dtb.h>
#include <linux/of.h>

#define CONFIG_DATA_PATH   "/htc,config-data"
#define SKU_DATA_PATH   "/htc,sku-data"
#define BOARD_INFO_PATH    "/htc_board_info"
#define SKU_DATA_NUM       12
#define MODULE_NAME        "[HTC DTB]"

#define LOG_INF(fmt, ...) pr_info("%s " fmt, MODULE_NAME, ##__VA_ARGS__)
#define LOG_ERR(fmt, ...) pr_err("%s " fmt, MODULE_NAME, ##__VA_ARGS__)

static unsigned int cfg_flag_index[NUM_FLAG_INDEX];
static unsigned int sku_flag_index[SKU_DATA_NUM];

static bool has_config_data = false;
static bool has_sku_data = false;

static int config_data_init(void);
static int sku_data_init(void);

unsigned int get_debug_flag(void)
{
	if (!has_config_data)
		config_data_init();
	if (!has_sku_data) 
		sku_data_init();
	return cfg_flag_index[DEBUG_FLAG_INDEX];
}
EXPORT_SYMBOL(get_debug_flag);

unsigned int get_kernel_flag(void)
{
	if (!has_config_data)
		config_data_init();
	return cfg_flag_index[KERNEL_FLAG_INDEX];
}
EXPORT_SYMBOL(get_kernel_flag);

unsigned int get_bootloader_flag(void)
{
	if (!has_config_data)
		config_data_init();
	return cfg_flag_index[BOOTLOADER_FLAG_INDEX];
}
EXPORT_SYMBOL(get_bootloader_flag);

unsigned int get_radio_flag(void)
{
	if (!has_config_data)
		config_data_init();
	return cfg_flag_index[RADIO_FLAG_INDEX];
}
EXPORT_SYMBOL(get_radio_flag);

unsigned int get_radio_ex1_flag(void)
{
	if (!has_config_data)
		config_data_init();
	return cfg_flag_index[RADIO_FLAG_EX1_INDEX];
}
EXPORT_SYMBOL(get_radio_ex1_flag);

unsigned int get_radio_ex2_flag(void)
{
	if (!has_config_data)
		config_data_init();
	return cfg_flag_index[RADIO_FLAG_EX2_INDEX];
}
EXPORT_SYMBOL(get_radio_ex2_flag);

unsigned int get_sku_data(int index)
{
	if (!has_sku_data)
		sku_data_init();
	return sku_flag_index[index];
}
EXPORT_SYMBOL(get_sku_data);


static int config_data_init(void)
{
	struct device_node *of_config_data = of_find_node_by_path(CONFIG_DATA_PATH);
	u32 config_data[6];
	int ret = 1;

	if (of_config_data) {
		LOG_INF("CONFIG DATA:\n");
		ret = of_property_read_u32_array(of_config_data, "config-data", config_data, 6);
		if(ret < 0){
			LOG_ERR("!!! COULDN'T READ CONFIG DATA !!!\n");
			return ret;
		}
		cfg_flag_index[DEBUG_FLAG_INDEX] = config_data[0];
		LOG_INF("DEBUG_FLAG_INDEX: 0x%.8X\n", cfg_flag_index[DEBUG_FLAG_INDEX]);
		cfg_flag_index[KERNEL_FLAG_INDEX] = config_data[1];
		LOG_INF("KERNEL_FLAG_INDEX: 0x%.8X\n", cfg_flag_index[KERNEL_FLAG_INDEX]);
		cfg_flag_index[BOOTLOADER_FLAG_INDEX] = config_data[2];
		LOG_INF("BOOTLOADER_FLAG_INDEX: 0x%.8X\n", cfg_flag_index[BOOTLOADER_FLAG_INDEX]);
		cfg_flag_index[RADIO_FLAG_INDEX] = config_data[3];
		LOG_INF("RADIO_FLAG_INDEX: 0x%.8X\n", cfg_flag_index[RADIO_FLAG_INDEX]);
		cfg_flag_index[RADIO_FLAG_EX1_INDEX] = config_data[4];
		LOG_INF("RADIO_FLAG_EX1_INDEX: 0x%.8X\n", cfg_flag_index[RADIO_FLAG_EX1_INDEX]);
		cfg_flag_index[RADIO_FLAG_EX2_INDEX] = config_data[5];
		LOG_INF("RADIO_FLAG_EX2_INDEX: 0x%.8X\n", cfg_flag_index[RADIO_FLAG_EX2_INDEX]);
		ret = 0;
		has_config_data = true;
	} else {
		LOG_ERR("!!! COULDN'T FIND CONFIG DATA node !!!\n");
		/* we may need kernel log to debug */
		cfg_flag_index[KERNEL_FLAG_INDEX] = 0x2;
	}

	return ret;
}

static int sku_data_init(void)
{
	struct device_node *of_sku_data = of_find_node_by_path(SKU_DATA_PATH);
	u32 sku_data[12];
	int ret = 1;
	int index = 0;

	if (of_sku_data) {
		LOG_INF("SKU DATA:\n");
		ret = of_property_read_u32_array(of_sku_data, "sku-data", sku_data, SKU_DATA_NUM);
		if(ret < 0){
			LOG_ERR("!!! COULDN'T READ SKU DATA !!!\n");
			return ret;
		}

		for(index = 0; index < SKU_DATA_NUM; index++)
		{
			sku_flag_index[index] = sku_data[index];
			LOG_INF("sku (index:%08X, value:%08X) added\n", index, sku_flag_index[index]);
		}
		ret = 0;
		has_sku_data = true;
	} else {
		LOG_ERR("!!! COULDN'T FIND SKU DATA node !!!\n");
	}

	return ret;
}

int of_machine_projectid(void)
{
	struct device_node *board_info_node;
	static int isRead = 0;
	static u32 array[2];
	int ret = 0;

	if (!isRead) {
		board_info_node = of_find_node_by_path(BOARD_INFO_PATH);
		if (board_info_node) {
			ret = of_property_read_u32_array(board_info_node, "htc_pid,htc_sku1", array, 2);
			if (ret < 0)
				return ret;
			isRead = 1;
		}
	}

	return array[0];
}
EXPORT_SYMBOL(of_machine_projectid);

int of_machine_hwid(void)
{
	struct device_node *board_info_node;
	static int isRead = 0;
	static u32 array[2];
	int ret = 0;

	if (!isRead) {
		board_info_node = of_find_node_by_path(BOARD_INFO_PATH);
		if (board_info_node) {
			ret = of_property_read_u32_array(board_info_node, "htc_pid,htc_sku1", array, 2);
			if (ret < 0)
				return ret;
			isRead = 1;
			}
	}

	return (array[1]>>24);
}
EXPORT_SYMBOL(of_machine_hwid);


int of_machine_bomid(void)
{
       struct device_node *board_info_node;
       static int isRead = 0;
       static u32 array[2];
       int ret = 0;

       if (!isRead) {
               board_info_node = of_find_node_by_path(BOARD_INFO_PATH);
               if (board_info_node) {
                       ret = of_property_read_u32_array(board_info_node, "htc_pid,htc_sku1", array, 2);
                       if (ret < 0)
                               return ret;
                       isRead = 1;
                       }
       }

       return ((array[1]>>8) & 0xff);
}
EXPORT_SYMBOL(of_machine_bomid);

int of_machine_sku1(void)
{
       struct device_node *board_info_node;
       static int isRead = 0;
       static u32 array[2];
       int ret = 0;

       if (!isRead) {
               board_info_node = of_find_node_by_path(BOARD_INFO_PATH);
               if (board_info_node) {
                       ret = of_property_read_u32_array(board_info_node, "htc_pid,htc_sku1", array, 2);
                       if (ret < 0)
                               return ret;
                       isRead = 1;
                       }
       }

       return array[1];
}
EXPORT_SYMBOL(of_machine_sku1);

static char bootmode[64] = "";
static int __init cmdline_bootmode_read(char *s)
{
	strncpy(bootmode,s,sizeof(bootmode)-1);
	return 1;
}
__setup("androidboot.mode=", cmdline_bootmode_read);

char *htc_get_bootmode(void)
{
	return bootmode;
}
EXPORT_SYMBOL(htc_get_bootmode);

/*++ 2015/02/09, USB Team, PCN00034 ++*/
static char cid[64] = "";
static int __init cmdline_cid_read(char *s)
{
	strncpy(cid,s,sizeof(cid)-1);
	return 1;
}
__setup("androidboot.cid=", cmdline_cid_read);

char *htc_get_cid(void)
{
	return cid;
}
EXPORT_SYMBOL(htc_get_cid);

static int __init htc_devices_dtb_init(void) {
	if (!has_config_data) config_data_init();
	if (!has_sku_data) sku_data_init();
	LOG_INF("hwid: %d\n", of_machine_hwid());
	LOG_INF("bomid: %d\n", of_machine_bomid());
	LOG_INF("sku1: 0x%08X\n", of_machine_sku1());
	LOG_INF("boot mode: %s\n", htc_get_bootmode());
}
arch_initcall(htc_devices_dtb_init);

