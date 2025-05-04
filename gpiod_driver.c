#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/kernel.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/interrupt.h>
/* YOU WILL NEED OTHER HEADER FILES */


// Timer variables
static struct hrtimer my_timer;
static ktime_t interval;
ktime_t press_times[2];
int press_num = 0;
ktime_t now;


/* YOU WILL HAVE TO DECLARE SOME VARIABLES HERE */
static struct gpio_desc *led_gpio;
static struct gpio_desc *button_gpio;
static int isr_num;


// Debouncing variables
static struct hrtimer debounce_time;
static bool debouncing = false;


static long nl = 0;
module_param(nl, long, 0644);


// Uses this timer to clear the debouncing state every 300ms
enum hrtimer_restart debounce_callback(struct hrtimer *timer)
{
   debouncing = false;
   return HRTIMER_NORESTART;
}


/* ADD INTERRUPT SERVICE ROUTINE HERE */
static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
   // Does nothing if this is a debouncing press
   if (debouncing){
       return IRQ_HANDLED;
   }
   // Arms our debouncing variables
   debouncing = 1;
   hrtimer_start(&debounce_time, ktime_set(0, 300 * 1000000), HRTIMER_MODE_REL); // 300ms
   now = ktime_get();
   // Records the press time
   press_times[press_num] = now;
   // Sets the new interval when ready
   if(press_num){
       interval = ktime_sub(press_times[1], press_times[0]);
       s64 ns = ktime_to_ns(interval);
       nl = (long)ns;
       pr_info("%lld\n", (long long)ns);
   }else{
      
   }
   press_num = !press_num;


   return IRQ_HANDLED;
}


// probe function
static int led_probe(struct platform_device *pdev)
{
   // LHS is name of function header and RHS slide 35 of lecture 16
   struct device *dev = &pdev->dev;


   // Initialize the GPIO pins
   button_gpio = devm_gpiod_get(dev, "my_input", GPIOD_IN);


   // Initialize the ISR
   isr_num = gpiod_to_irq(button_gpio);
   devm_request_irq(dev, isr_num, gpio_irq_handler, IRQF_TRIGGER_FALLING, "my_isr", NULL);


   // Interval init
   interval = ktime_set(0, 500 * 1000000); // 500ms


   // Debounce timer init
   hrtimer_init(&debounce_time, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
   debounce_time.function = debounce_callback;


   return 0;
}


// remove function
static int led_remove(struct platform_device *pdev)
{
   /* INSERT: Free the irq and print a message */
   devm_free_irq(&pdev->dev, isr_num, NULL);
  
   // Cancel the timers; pr_info is just printk with KERN_INFO
   int ret = hrtimer_cancel(&debounce_time);
   if (ret)
      pr_info("ISR Timer was active and has been cancelled\n");
   else
      pr_info("ISR Timer was not active\n");


   return 0;
}


static struct of_device_id matchy_match[] = {
   {.compatible = "mine"},
   {/* leave alone - keep this here (end node) */},
};


// platform driver object
static struct platform_driver adam_driver = {
   .probe   = led_probe,
   .remove  = led_remove,
   .driver  = {
          .name  = "The Rock: this name doesn't even matter",
          .owner = THIS_MODULE,
          .of_match_table = matchy_match,
   },
};


module_platform_driver(adam_driver);


MODULE_DESCRIPTION("424\'s finest");
MODULE_AUTHOR("Noah");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:adam_driver");

