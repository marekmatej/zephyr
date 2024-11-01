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

#define STOP_HELP \
	("<cmd>\n\n" \
	 "Stop the APPCPU")

void esp_appcpu_image_start(unsigned int hdr_offset);
void esp_appcpu_image_stop(void);

static int cmd_appcpu_start(const struct shell *shell, size_t argc, char **argv)
{
	printk("start appcpu\n");

	esp_appcpu_image_start(0x20);

	return 0;
}

static int cmd_appcpu_stop(const struct shell *shell, size_t argc, char **argv)
{
	printk("stop appcpu\n");

	esp_appcpu_image_stop();

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_amp,
	/* Alphabetically sorted to ensure correct Tab autocompletion. */
	SHELL_CMD_ARG(appstart,	NULL,	START_HELP,	cmd_appcpu_start, 1, 0),
	SHELL_CMD_ARG(appstop,	NULL,	STOP_HELP,	cmd_appcpu_stop, 1, 0),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_CMD_REGISTER(amp, &sub_amp, "AMP debug commands.", NULL);
