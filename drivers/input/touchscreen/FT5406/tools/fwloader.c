/*
 * ft5x06 tp firmware loader
 */

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

#define SYSCALL(call) while (((call) == -1) && (errno == EINTR))
extern long hexdec(unsigned const char *hex);

const static char *ctrl_dev = "/dev/ft5x06";
const static char *SEPARATOR = ", \r\n";

enum {
	FT5X06_IOCTL_FW_UPDATE = 0xA1,
	FT5X06_IOCTL_FW_UPDATE_FORCE,
	FT5X06_IOCTL_FW_DEFAULT_LOAD,
	FT5X06_IOCTL_GET_VERSION,
	FT5X06_IOCTL_GET_VENDOR_ID,
};
#define IOCTL_FW_UPDATE		_IO('F', FT5X06_IOCTL_FW_UPDATE)
#define IOCTL_FW_FUPDATE	_IO('F', FT5X06_IOCTL_FW_UPDATE_FORCE)
#define IOCTL_FW_DEFAULT	_IO('F', FT5X06_IOCTL_FW_DEFAULT_LOAD)
#define IOCTL_GET_VERSION	_IO('F', FT5X06_IOCTL_GET_VERSION)
#define IOCTL_GET_VENDOR_ID	_IO('F', FT5X06_IOCTL_GET_VENDOR_ID)

static int vendor_version[2];
static int *get_vendor_version()
{
	int fd;
	int *ret = vendor_version;

	fd = open(ctrl_dev, O_RDWR);
	if (fd < 0) {
		perror("open dev");
		fprintf(stderr, "%s open failed, ctp_ft5x06.ko loaded?\n", ctrl_dev);
		return NULL;
	}
	ret[0] = ioctl(fd, IOCTL_GET_VENDOR_ID);
	if (ret[0] < 0) {
		perror("ioctl get vendor");
		close(fd);
		return NULL;
	}
	ret[1] = ioctl(fd, IOCTL_GET_VERSION);
	if (ret[1] < 0) {
		perror("ioctl get version");
		close(fd);
		return NULL;
	}
	close(fd);
	return ret;
}

static void show_version()
{
	int fd;
	int *ret;

	ret = get_vendor_version();
	if (ret != NULL)
		fprintf(stdout, "0x%02x(%d)\n", ret[1], ret[1]);
}

static void show_vendor_version()
{
	int fd;
	int *ret;

	ret = get_vendor_version();
	if (ret != NULL)
		fprintf(stdout, "Vendor:0x%02x Version:0x%02x(%d)\n",
				ret[0], ret[1], ret[1]);
}

static void fw_default_load()
{
	int fd;
	int ret, *vv;

	fprintf(stdout, "ft5x06 default firmware force load\n");
	fd = open(ctrl_dev, O_RDWR);
	if (fd < 0) {
		perror("open dev");
		fprintf(stderr, "%s open failed, ctp_ft5x06.ko loaded?\n", ctrl_dev);
		return;
	}
	ret = ioctl(fd, IOCTL_FW_DEFAULT);
	close(fd);
	if (ret < 0) {
		perror("ioctl fw default load");
		return;
	}
	fprintf(stdout, "ft5x06 default firmware load done\n");
	show_vendor_version();
}

static void usage(const char *pname)
{
	fprintf(stderr,
		"Usage: %s --input=<app.i file> [--default] [--force] [--help] [--version] [--vendor]\n\n",
		pname);
}

int main(int argc, char *argv[])
{
	int fd = -1, fw = -1;
	int ret;
	struct stat st;
	size_t file_size;
	u_char *file_data = NULL;
	size_t fw_size;
	u_char *fw_data = NULL;
	ssize_t readn, written;
	int force = 0;
	const char *firmware = NULL;
	char *msg;
	int getver = 0;
	int getvendor = 0;
	int default_load = 0;
	char *p;
	long lch;

	static const struct option options[] = {
		{ "force", no_argument, NULL, 'f' },
		{ "input", required_argument, NULL, 'i' },
		{ "version", no_argument, NULL, 'v' },
		{ "vendor", no_argument, NULL, 'b' },
		{ "help", no_argument, NULL, 'h' },
		{ "default", no_argument, NULL, 'd' },
		{}
	};

	for (;;) {
		int option;

		option = getopt_long(argc, argv, "fhi:vbd", options, NULL);
		if (option == -1)
			break;

		switch (option) {
		case 'f':
			force = 1;
			break;
		case 'i':
			firmware = optarg;
			break;
		case 'v':
			getver = 1;
			break;
		case 'b':
			getvendor = 1;
			break;
		case 'd':
			default_load = 1;
			break;
		case 'h':
		default:
			usage(argv[0]);
			return 1;
		}
	}

	/* show version */
	if (getver) {
		show_version();
		return 0;
	}

	/* show vendor */
	if (getvendor) {
		show_vendor_version();
		return 0;
	}

	/* fw default load */
	if (default_load) {
		fw_default_load();
		return 0;
	}

	if (!firmware) {
		fprintf(stderr, "Need app.i file\n");
		usage(argv[0]);
		return 1;
	}

	/*
	 * app.i firmware file open & read
	 */
	fprintf(stdout, "ft5x06 firmware %supdate for %s\n",
		   force ? "force " : "", firmware);
	fw = open(firmware, O_RDONLY);
	if (fw < 0) {
		perror("open fw");
		return -1;
	}
	ret = fstat(fw, &st);
	if (ret < 0) {
		perror("stat fw");
		goto finish;
	}
	file_size = st.st_size;
	fprintf(stdout, "file size %ld bytes\n", file_size);
	file_data = (u_char *)malloc(file_size);
	if (!file_data) {
		perror("out of memory");
		ret = -1;
		goto finish;
	}
	SYSCALL(readn = read(fw, file_data, file_size));
	if (readn != file_size) {
		perror("read fw");
		ret = -1;
		goto finish;
	}
	close(fw); fw = -1;

	/* estimate of max buffer size */
	/* hex string "0xX," is minimum 4 bytes length */
	fw_size = file_size / 4;	/* enough for buffer size */
	fw_data = (u_char *)malloc(fw_size);
	if (!fw_data) {
		perror("out of memory");
		ret = -1;
		goto finish;
	}

	/*
	 * convert hex string to u_char array
	 */
	fw_size = 0;
	p = strtok(file_data, SEPARATOR);
	while (p != NULL) {
		lch = hexdec(&p[2]);
		if (lch == -1) {
			fprintf(stderr, "index %d could not convert\n", fw_size);
			fprintf(stderr, "please check app.i file\n");
			ret = -1;
			goto finish;
		}
		fw_data[fw_size] = (u_char)lch;
		fw_size++;
		p = strtok(NULL, SEPARATOR);
	}
	fprintf(stdout, "fw size %ld bytes\n", fw_size);
	free(file_data); file_data = NULL;
//	return 0;

	fd = open(ctrl_dev, O_RDWR);
	if (fd < 0) {
		perror("open dev");
		fprintf(stderr, "%s open failed, ctp_ft5x06.ko loaded?\n", ctrl_dev);
		ret = -1;
		goto finish;
	}
	SYSCALL(written = write(fd, fw_data, fw_size));
	if (written != fw_size) {
		perror("write fw");
		ret = -1;
		goto finish;
	}
	if (force == 1)
		ret = ioctl(fd, IOCTL_FW_FUPDATE);
	else
		ret = ioctl(fd, IOCTL_FW_UPDATE);
	close(fd); fd = -1;
	if (ret == 0)
		fprintf(stdout, "ft5x06 firmware update to ver.0x%02x successfully\n",
				fw_data[fw_size - 2]);
	else if (ret == -1)
		fprintf(stderr, "ft5x06 firmware update vendor(0x%02x) not match\n",
				fw_data[fw_size - 1]);
	else
		fprintf(stderr, "ft5x06 firmware update to ver.0x%02x failed\n",
				fw_data[fw_size - 2]);

finish:
	if (file_data) free(file_data);
	if (fw_data) free(fw_data);
	if (fw >= 0) close(fw);
	if (fd >= 0) close(fd);
	return ret;
}
