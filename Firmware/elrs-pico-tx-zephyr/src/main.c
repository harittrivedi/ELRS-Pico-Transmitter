/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * ELRS Pico TX - Zephyr RTOS Port
 * Phase 1: LED Blink Test
 * 
 * This validates the basic project setup and GPIO functionality.
 * LED Pattern:
 * - 3 fast blinks on boot (initialization complete)
 * - Slow blink (1s on / 1s off) in main loop
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(elrs_tx, LOG_LEVEL_INF);

/* LED configuration from devicetree */
#define LED0_NODE DT_ALIAS(led0)

#if !DT_NODE_HAS_STATUS(LED0_NODE, okay)
#error "LED0 node is not available in devicetree"
#endif

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/* Boot indication: 3 fast blinks */
static void boot_blink(void)
{
    for (int i = 0; i < 3; i++) {
        gpio_pin_set_dt(&led, 1);
        k_msleep(100);
        gpio_pin_set_dt(&led, 0);
        k_msleep(100);
    }
    k_msleep(500);
}

int main(void)
{
    int ret;

    LOG_INF("ELRS Pico TX - Zephyr Port");
    LOG_INF("Phase 1: LED Blink Test");

    /* Initialize LED GPIO */
    if (!gpio_is_ready_dt(&led)) {
        LOG_ERR("LED device not ready");
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure LED: %d", ret);
        return ret;
    }

    LOG_INF("LED initialized on GPIO %d", led.pin);

    /* Boot indication */
    boot_blink();
    LOG_INF("Boot sequence complete");

    /* Main loop - slow blink */
    while (1) {
        gpio_pin_toggle_dt(&led);
        k_msleep(1000);
    }

    return 0;
}
