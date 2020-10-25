#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/proc_fs.h>

#include "hardware_info.h"

char Lcd_info[40] = { "NO THIS DEVICE" };
char Camera_F_info[40] = { "NO THIS DEVICE" };
char Camera_B_info[40] = { "NO THIS DEVICE" };
char Camera_F_other_info[40] = { "NO THIS DEVICE" };
char TP_info[40] = { "NO THIS DEVICE" };
char G_sensor_info[40] = { "ICM-20608D" };
char M_sensor_info[40] = { "ST480" };
char L_sensor_info[40] = { "MN66233TKDN" };
char P_sensor_info[40] = { "MN66233TKDN" };
char Gyro_sensor_info[40] = { "ICM-20608D" };
char Battery_info[40] = { "FEIMAOTUI-CARINA-SDI" };
char EMMC_info[40] = { "NO THIS DEVICE" };
char WIFI_info[40] = { "WCN3615" };
char GPS_info[40] = { "RF-U2200" };
char FM_info[40] = { "WCN3615" };
char BT_info[40] = { "WCN3615" };

static long hardwareinfo_ioctl(struct file *filp, unsigned int cmd,
							   unsigned long arg)
{
	int ret = 0;
	void __user *argp = (void __user *)arg;

	switch (cmd) {
	case HARDWARE_LCD_GET:
		ret = copy_to_user(argp, Lcd_info, sizeof(Lcd_info));
		break;
	case HARDWARE_TP_GET:
		ret = copy_to_user(argp, TP_info, sizeof(TP_info));
		break;
	case HARDWARE_FLASH_GET:
		ret = copy_to_user(argp, EMMC_info, sizeof(EMMC_info));
		break;
	case HARDWARE_FRONT_CAM_GET:
		ret = copy_to_user(argp, Camera_F_info, sizeof(Camera_F_info));
		break;
	case HARDWARE_FRONT_OTHER_CAM_GET:
		ret = copy_to_user(argp, Camera_F_other_info, sizeof(Camera_F_other_info));
		break;
	case HARDWARE_BACK_CAM_GET:
		ret = copy_to_user(argp, Camera_B_info, sizeof(Camera_B_info));
		break;
	case HARDWARE_ACCELEROMETER_GET:
		ret = copy_to_user(argp, G_sensor_info, sizeof(G_sensor_info));
		break;
	case HARDWARE_ALSPS_GET:
		ret = copy_to_user(argp, L_sensor_info, sizeof(L_sensor_info));
		break;
	case HARDWARE_GYROSCOPE_GET:
		ret = copy_to_user(argp, Gyro_sensor_info, sizeof(Gyro_sensor_info));
		break;
	case HARDWARE_MAGNETOMETER_GET:
		ret = copy_to_user(argp, M_sensor_info, sizeof(M_sensor_info));
		break;
	case HARDWARE_BT_GET:
		ret = copy_to_user(argp, BT_info, sizeof(BT_info));
		break;
	case HARDWARE_WIFI_GET:
		ret = copy_to_user(argp, WIFI_info, sizeof(WIFI_info));
		break;
	case HARDWARE_GPS_GET:
		ret = copy_to_user(argp, GPS_info, sizeof(GPS_info));
		break;
	case HARDWARE_FM_GET:
		ret = copy_to_user(argp, FM_info, sizeof(FM_info));
		break;
	case HARDWARE_BATTERY_ID_GET:
		ret = copy_to_user(argp, Battery_info, sizeof(Battery_info));
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long hardwareinfo_compat_ioctl(struct file *filp, unsigned int cmd,
									  unsigned long arg)
{
	return hardwareinfo_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static int hardwareinfo_open(struct inode *inode, struct file *filp)
{
	nonseekable_open(inode, filp);

	return 0;
}

static struct file_operations hardwareinfo_fops = {
	.owner = THIS_MODULE,
	.open = hardwareinfo_open,
	.unlocked_ioctl = hardwareinfo_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = hardwareinfo_compat_ioctl,
#endif
};

static struct miscdevice hardwareinfo_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "hardwareinfo",
	.fops = &hardwareinfo_fops,
};

static int __init hardwareinfo_init(void)
{
	int ret;

	ret = misc_register(&hardwareinfo_device);
	if (ret < 0) {
		printk(KERN_ERR "hardware info misc register failed!\n");
		return -ENODEV;
	}
	printk("hardware info misc_register successfully!\n");
	return 0;
}

static void __exit hardwareinfo_exit(void)
{
	misc_deregister(&hardwareinfo_device);
}

module_init(hardwareinfo_init);
module_exit(hardwareinfo_exit);

MODULE_DESCRIPTION("User mode device interface");
MODULE_LICENSE("GPL");
