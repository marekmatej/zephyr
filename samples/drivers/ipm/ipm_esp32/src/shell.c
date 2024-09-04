/* Helper shell
 */

#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/dt-bindings/gpio/nordic-npm6001-gpio.h>
#include <zephyr/dt-bindings/regulator/npm6001.h>
#include <zephyr/posix/unistd.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/printk.h>

#include <getopt.h>

#define PR_SHELL(shell, fmt, ...)				\
	shell_fprintf(shell, SHELL_NORMAL, fmt, ##__VA_ARGS__)
#define PR_ERROR(shell, fmt, ...)				\
	shell_fprintf(shell, SHELL_ERROR, fmt, ##__VA_ARGS__)
#define PR_INFO(shell, fmt, ...)				\
	shell_fprintf(shell, SHELL_INFO, fmt, ##__VA_ARGS__)
#define PR_WARNING(shell, fmt, ...)				\
	shell_fprintf(shell, SHELL_WARNING, fmt, ##__VA_ARGS__)

/* Command usage info. */
#define START_HELP \
	("<cmd>\n\n" \
	 "Start the APPCPU")

void esp_appcpu_image_start(int img_index, int slot, unsigned int hdr_offset);

static int cmd_appcpu_start(const struct shell *shell, size_t argc, char **argv)
{
	printk("start appcpu\n");

	//start_cpu1_image(1, 0, 0x20);
	esp_appcpu_image_start(1, 0, 0x20);

	return 0;
}


SHELL_STATIC_SUBCMD_SET_CREATE(sub_amp,
	/* Alphabetically sorted to ensure correct Tab autocompletion. */
	SHELL_CMD_ARG(appstart,	NULL,	START_HELP,	cmd_appcpu_start, 1, 0),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_CMD_REGISTER(amp, &sub_amp, "AMP debug commands.", NULL);
