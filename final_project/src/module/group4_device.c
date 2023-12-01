/*
ELEC 424/553
Final Project
Authors: Eric Lin(el38), Shaun Lin(hl116), Yen-Yu Chien (yc185), Saif Khan (sbk7)
*/

#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/kernel.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/ktime.h>

//Inturupt handler number
int irq_number;

//refers to button pin P8_14
struct gpio_desc *button;

ktime_t curr_time, previous_time;
int speed;
// save the speed as a module parameter
module_param(speed, int, S_IRUGO);

// function that handles the interrupt
static irq_handler_t gpio_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs) {
    int temp;
    printk("Speed encoder triggered.\n");
	curr_time = ktime_get();
    // int temp = ktime_to_ns(curr_time - previous_time) / 1000000;
    temp = ktime_to_ns(curr_time - previous_time);
    temp = temp / 1000000;
    if(temp > 1) { speed = temp; }
    printk("Difference between triggers: %d\n", speed);
    previous_time = curr_time;
	return (irq_handler_t) IRQ_HANDLED; 
}

// probe function, takes in the platform device that allows us to get the references to the pins
static int encoder_probe(struct platform_device *pdev)
{
	printk("gpiod_driver has been inserted.\n");
    //Get the pins based on what we called them in the device tree file:  "name"-gpios
	//led = devm_gpiod_get(&pdev->dev, "led", GPIOD_OUT_LOW);
	button = devm_gpiod_get(&pdev->dev, "userbtn", GPIOD_IN);

    //Set waiting period before next input
	gpiod_set_debounce(button, 1000000);


    //Pair the button pin with an irq number
	irq_number = gpiod_to_irq(button);

    //Pair the number with the function that handles it
	if(request_irq(irq_number, (irq_handler_t) gpio_irq_handler, IRQF_TRIGGER_RISING, "my_gpio_irq", NULL) != 0){
		printk("Error!\nCan not request interrupt nr.: %d\n", irq_number);
		free_irq(irq_number, NULL);
		return -1;
	}
	return 0;
}

// remove function ,takes in the platform device that allows us to get the references to the pins
static int encoder_remove(struct platform_device *pdev)
{
	printk("Custom gpiod_driver module has been removed and irq has been freed\n");
	irq_number = gpiod_to_irq(button);
	//TBH not sure what goes in the second argument
    free_irq(irq_number, NULL);
	return 0;
}
static struct of_device_id matchy_match[] = {
    {.compatible = "elec553,finalproject"},
    {/* end node */}
};
// platform driver object
static struct platform_driver encoder_driver = {
    .probe = encoder_probe,
    .remove = encoder_remove,
    .driver = {
        .name = "encoder_driver",
        .owner = THIS_MODULE,
        .of_match_table = matchy_match,
    }
};
module_platform_driver(encoder_driver);
MODULE_DESCRIPTION("ELEC553 Final Project Group 4");
MODULE_AUTHOR("Shaun, Eric, Yen-Yu, Saif");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:encoder_driver");