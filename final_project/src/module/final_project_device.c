/*
ELEC 424/553
Final Project
Authors: Eric Lin(el38), Shaun Lin(hl116), Yen-Yu Chien (yc185), Saif Khan (sbk7)
*/

#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/timekeeping.h>
struct gpio_desc *userbtn, *blue;
unsigned int irq_number;

int state = 0; // Counter Value
int flag = 0; // Flag for button press
long curent_time = 0; // Current time
long last_time = 0; // Last time
long time_diff = 0; // Time difference
long speed; // Current Speed

module_param(speed,long,S_IRUGO);

// interrupt function
static irq_handler_t gpio_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs) {
    
    curent_time = ktime_get_real_ns();
    time_diff = curent_time - last_time;
    last_time = curent_time;
    speed = time_diff;
    
    // return
    return (irq_handler_t) IRQ_HANDLED;
}

// probe function
static int car_probe(struct platform_device *pdev)
{
    userbtn = devm_gpiod_get(&pdev->dev, "userbtn", GPIOD_IN);
    gpiod_set_debounce(userbtn, 1000000);
    /* Setup the interrupt */
    irq_number = gpiod_to_irq(userbtn);
    if(request_irq(irq_number, (irq_handler_t) gpio_irq_handler,IRQF_TRIGGER_FALLING, "my_gpio_irq", NULL) != 0)
    {
        printk("Error!\nCan not request interrupt nr.: %d\n", irq_number);
        gpiod_put(userbtn);
        return -1;
    }
    return(0);
}
// remove function
static int car_remove(struct platform_device *pdev)
{
    printk("GPIO example exit\n");
    gpiod_put(userbtn);
    free_irq(irq_number, NULL);
    return(0);
}
static struct of_device_id matchy_match[] = {
    {.compatible = "ELEC553,Final_Project"},
    {/* end node */}
};
// platform driver object
static struct platform_driver rc_car_driver = {
    .probe = car_probe,
    .remove = car_remove,
    .driver = {
        .name = "rc_car_driver",
        .owner = THIS_MODULE,
        .of_match_table = matchy_match,
    }
};
module_platform_driver(rc_car_driver);
MODULE_DESCRIPTION("ELEC553 Final Project");
MODULE_AUTHOR("GOAT");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:rc_car_driver");