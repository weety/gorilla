#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <dfs_fs.h>

int check_udisk_ready(void)
{
	rt_device_t udisk_dev;
	const char mount_path = NULL;

	udisk_dev = rt_device_find(UDISK_DEV_NAME);
	if (!udisk_dev)
		return -1;
	mount_path = dfs_filesystem_get_mounted_path();
	if (!mount_path)
		return -1;

	return 0;
}

int logger_create_file(const char *name)
{
	int fd = -1;
	char file_name[64];

	snprintf(file_name, 64, "%s%s", LOGGER_FILE_PATH, name);

	fd = open(file_name, O_CREAT | O_RDWR);

	return fd;
}

