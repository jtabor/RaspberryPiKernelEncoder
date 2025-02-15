#include <linux/fs.h> 	     /* file stuff */
#include <linux/kernel.h>    /* printk() */
#include <linux/errno.h>     /* error codes */
#include <linux/module.h>  /* THIS_MODULE */
#include <linux/cdev.h>      /* char device stuff */
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/gpio/consumer.h>

//Mount char device with: sudo mknod /dev/encoder-driver c 240 0
//You can change these if you want different pins.
//Be careful which ones you use..  they don't always work
//sometimes different Interrupt channels conflict.
//Update 2/14/2025 - Allegadly, all RPi pins are valid for interrupts,
//so all pins can be used.  


#define QUADRATURE 1
#define NUM_ENCODERS 2

// Need this because the GPIO numbers are different in the kernel
// CAUTION - this might be different on different RPi versions. 
// This was tested on RPi 4B
#define PIN_OFFSET 512
#if QUADRATURE
#define PIN_1A 17
#define PIN_1B 27
#define PIN_2A 23
#define PIN_2B 24
#define PIN_3A 5
#define PIN_3B 6
#define PIN_4A 16
#define PIN_4B 26

volatile static int encoder_counts[NUM_ENCODERS] = {0};
volatile static int encoder_pins[8] = {PIN_1A,PIN_1B,PIN_2A,PIN_2B,PIN_3A,PIN_3B,PIN_4A,PIN_4B}; 

volatile static u8 pin_state = 0;
volatile static unsigned int gpioIRQs[2*NUM_ENCODERS] = {0};

static u8 next_state = 0;
static int next_val = 0;
static u8 prevPins = 0;
static u8 nextPins = 0;
static u8 encoder_id = 0;

//lookup table to see what to add to encoder value based on where you are.
static int stateChangeTable[4][4] = {{ 0,-1,1,0},{1,0,0,-1},{-1,0,0,1},{0,1,-1,0}};

#else
#define PIN_1 17
#define PIN_2 27
#define PIN_3 5
#define PIN_4 6
#define PIN_5 23
#define PIN_6 24
#define PIN_7 16
#define PIN_8 26

volatile static int encoder_counts[NUM_ENCODERS] = {0};
volatile static int encoder_pins[8] = {PIN_1,PIN_2,PIN_3,PIN_4,PIN_5,PIN_6,PIN_7,PIN_8}; 

volatile static u8 pin_state = 0;
volatile static unsigned int gpioIRQs[NUM_ENCODERS] = {0};

#endif //QUADRATURE
#define NUM_PINS (QUADRATURE + 1)*NUM_ENCODERS

static struct gpio_desc *gpios[NUM_PINS];

static u8 isRunning = 0;

//counter for all interrupts for debugging, not really needed.
volatile unsigned int interruptCount = 0;

//interrupt handler.
static irqreturn_t irq_handler(int irq, void *dev_id){   

   static int cur_irq = -1;
   interruptCount++;
#if QUADRATURE
   if (isRunning){
  	int i = 0;
	for (i = 0; i < 2*NUM_ENCODERS; i++){
		if(irq == gpioIRQs[i]){
			cur_irq = i;
			break;
		}
	}

	next_val = gpiod_get_value(gpios[cur_irq]);
	next_state = (pin_state & ~(0x01 << cur_irq)) | (next_val << cur_irq);
	encoder_id = cur_irq >> 1;
	prevPins = pin_state;
	nextPins = next_state;

	if (encoder_id ==0){
		prevPins = prevPins & 0x03;
		nextPins = nextPins & 0x03;
	}
	else if (encoder_id == 1){
		prevPins = (prevPins >> 2) & 0x03;
		nextPins = (nextPins >> 2) & 0x03;
	}
	else if (encoder_id == 2){
		prevPins = (prevPins >> 4) & 0x03;
		nextPins = (nextPins >> 4) & 0x03;
	}
	else{
		prevPins = (prevPins >> 6) & 0x03;
		nextPins = (nextPins >> 6) & 0x03;
	}

	encoder_counts[encoder_id] = encoder_counts[encoder_id] + stateChangeTable[prevPins][nextPins];
	pin_state = next_state;

#else //NOT QUADRATURE
   if (isRunning){
  	int i = 0;
	for (i = 0; i < NUM_ENCODERS; i++){
		if(irq == gpioIRQs[i]){
			cur_irq = i;
			break;
		}
	}
   encoder_counts[cur_irq] ++;

#endif //QUADRATURE
   }
   return IRQ_HANDLED;
}


//any writes to device file reset counts
static ssize_t device_file_write(
                           struct file *file_ptr
                        ,const char __user *user_buffer
                        , size_t count
                        , loff_t *position){

	int i; 
    for (i = 0; i < NUM_ENCODERS; i++){
        encoder_counts[i] = 0;
    }

    return count;
}
//any reads return NUM_ENCODERS ints with encoder counts as binary
static ssize_t device_file_read(
                           struct file *file_ptr
                        , char __user *user_buffer
                        , size_t count
                        , loff_t *position)
{

   printk(KERN_NOTICE "Interrupt Count: %d\n",interruptCount);
   int not_copied = 0;
   not_copied = copy_to_user(user_buffer,(const int*)encoder_counts,sizeof(int)*NUM_ENCODERS);
   if (not_copied){
      printk(KERN_WARNING "ENCODER - Copy to user failed");
      return -EFAULT; // Return a bad address error
   }
   return sizeof(int)*NUM_ENCODERS;
}
/*===============================================================================================*/
static struct file_operations encoder_driver_fops = 
{
   .owner   = THIS_MODULE,
   .read    = device_file_read,
   .write   = device_file_write,
};

static int device_file_major_number = 0;

static const char device_name[] = "encoder-driver";

/*===============================================================================================*/
int register_device(void)
{

   int result = 0;

   result = register_chrdev( 0, device_name, &encoder_driver_fops );
   if( result < 0 )
   {
      printk( KERN_WARNING "encoder-driver:  can\'t register character device with errorcode = %i", result );
      return result;
   }

   device_file_major_number = result;

   int i = 0;
   int status = 0;
   // Go through all the pins and register them, set them as inputs, and request interrupts on them.
   // If this fails - you might have to restart because it won't clean up.
   // but it also shouldn't fail.
   for (i = 0; i < NUM_PINS; i++){
      status = gpio_request(encoder_pins[i] + PIN_OFFSET, "encoder-gpio");
      if(status){
         printk(KERN_ERR "ENCODER - Error requesting gpio %d\n", encoder_pins[i]);
         return status;
      } 
      gpios[i] = gpio_to_desc(encoder_pins[i] + PIN_OFFSET);
      if (!gpios[i]){
         printk(KERN_ERR "ENCODER - Error setting up gpio %d\n", encoder_pins[i]);
         return -1;
      }
    
      status = gpiod_direction_input(gpios[i]);
      if (status){
         printk(KERN_ERR "ENCODER - Error setting up direction of gpio %d\n", encoder_pins[i]);
         return status;
      }
      
      gpioIRQs[i] = gpiod_to_irq(gpios[i]);
      if (gpioIRQs[i] < 0){
         printk(KERN_ERR "ENCODER - Error finding IRQ for gpio %d\n", encoder_pins[i]);
         return gpioIRQs[i];
      }

      status = request_irq(gpioIRQs[i], irq_handler, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "encoder_irq", NULL);
      if(status){
         printk(KERN_ERR "ENCODER - Error requesting IRQ for gpio %d\n", encoder_pins[i]);
         return status;
      }
   }

   isRunning = 1;
   return 0;
}
/*-----------------------------------------------------------------------------------------------*/
// free all the irq and gpio resources.
void unregister_device(void)
{
   isRunning = 0;
   printk( KERN_NOTICE "encoder-driver: unregister_device() is called" );
   int i = 0;


   for (i = 0; i < NUM_PINS; i++){
      free_irq(gpioIRQs[i],NULL);
      gpiod_put(gpios[i]);
   }
   
   if(device_file_major_number != 0){
      unregister_chrdev(device_file_major_number, device_name);
   }

}
