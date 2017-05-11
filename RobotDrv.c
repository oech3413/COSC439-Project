// A device driver to control a robot gripper arm to
// reproduce the hand motion of the remote operator
//
// Written by Ryan Oechsler and David Rogers

  // ******** Include the necessary Linux headers ********

  #include <linux/module.h>         // Header for loading LKMs into the kernel
  #include <linux/kernel.h          // Various types defs, macros, functions for the kernel
  #include <linux/init.h>           // Contains the module_init, module_exit macros
  #include <linux/device.h>         // Header to support the kernel Driver Model
  #include <linux/fs.h>             // Defines the Linux file system support structure
  #include <linux/gpio.h>           // Defines the GPIO-related functions
  #include <asm/uaccess.h>          // Defines the copy_to_user, copy_from_user functions
  #include <linux/delay.h>          // Contains the udelay function

  // GPIO Port Definitions
  #define gpioPortSpiClock   22     // GPIO Port #22 connects to the ADC Clock
  #define gpioPortSpiDataOut 17     // GPIO Port #17 connects to the ADC Data Out
  #define gpioPortSpiDataIn  27     // GPIO Port #27 connects to the ADC Data In
  #define gpioPortChipSelect 4      // GPIO Port #4  connects to the ADC Chip Select
  #define gpioPortServoData  18     // GPIO Port #18 connects to the Servo Data In
   
// ******** Provide Linux with information about the driver ********

  MODULE_DESCRIPTION("A Linux LKM driver to control a robot gripper arm");
  MODULE_AUTHOR("Ryan Oechsler and David Rogers");
  MODULE_VERSION("1.0");
  MODULE_LICENSE("GPL"); // "GPL" indicates free open-source, "Proprietary" indicates pay-for-use

  // the module will be placed at /sys/class/<CLASS_NAME>/<DEVICE_NAME>/device/driver/module

  #define CLASS_NAME  "RaspPi"
  #define DEVICE_NAME "Robot"

// ******** Declare variables used in this device driver module ********

  static int    myDeviceNumber;               // holds the number assigned to the device
  static char   kernelData[256];              // holds data to be passed to/from user memory space
  static struct class*  myClass;              // pointer to the device-driver class struct
  static struct device* myDevice;             // pointer to the device-driver device struct
  static int    myDeviceNumber;               // holds the number assigned to the device
   
// ******** Prototypes for the functions contained in this module ********

  static int      devOpen(struct inode *, struct file *);
  static int      devRelease(struct inode *, struct file *);
  static ssize_t  devRead(struct file *, char *, size_t, loff_t *);
  static ssize_t  devWrite(struct file *, const char *, size_t, loff_t *);
  static int      getHandAngle(void);
  static void     setGripperAngle(int);
  static void     sendToServo(int);
 
  // Next, we associate each of the driver's file function overrides
  // with their corresponding file operation defined in /linux/fs.h

  static struct file_operations fops = {
    .open    = devOpen,
    .read    = devRead,
    .write   = devWrite,
    .release = devRelease,
  };

// ******** All LKMs require contructor/destructor functions ********

  int devConstructor(void) {

    // Request a number for the device
    myDeviceNumber = register_chrdev(0, DEVICE_NAME, &fops);
    if (myDevice < 0) {
      return -1;
    }

    // Register the device class
    myClass = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(myClass)) {
      printk("Registering the device class failed\n");
      unregister_chrdev(myDeviceNumber, DEVICE_NAME);
      return -2;
    }

    // Register the device
    myDevice = device_create(myClass, NULL, MKDEV(myDeviceNumber, 0), NULL, DEVICE_NAME);
    if (IS_ERR(myDevice)) {
      printk("Registering the device failed\n");
      class_destroy(myClass);
      unregister_chrdev(myDeviceNumber, DEVICE_NAME);
      return -3;
    }

   // Request access to the GPIO ports connected to the device
   if ( (gpio_request(gpioPortSpiClock,   "spiClock")   < 0)
     || (gpio_request(gpioPortChipSelect, "chipSelect") < 0) 
     || (gpio_request(gpioPortSpiDataOut, "spiDataOut") < 0)
     || (gpio_request(gpioPortSpiDataIn,  "spiDataIn")  < 0)   
     || (gpio_request(gpioPortServoData,  "servoData")  < 0) ) {   
     printk("Request to access GPIO failed\n");
     device_destroy(myClass, MKDEV(myDeviceNumber, 0)); // remove the device
     class_unregister(myClass);                         // unregister the device class
     class_destroy(myClass);                            // remove the device class
     unregister_chrdev(myDeviceNumber, DEVICE_NAME);    // unregister the device number
     return -4;
    }
        
   // Configure the direction of the GPIO ports
    gpio_direction_output(gpioPortSpiClock, 0);
    gpio_direction_output(gpioPortChipSelect, 1);
    gpio_direction_output(gpioPortSpiDataOut, 0);
    gpio_direction_input(gpioPortSpiDataIn);
    gpio_direction_output(gpioPortServoData, 1);
       
    printk("Driver installed succesfully.\n");
    return 0;
  }

  void devDestructor(void) {
   
    device_destroy(myClass, MKDEV(myDeviceNumber, 0)); // remove the device
    class_unregister(myClass);                         // unregister the device class
    class_destroy(myClass);                            // remove the device class
    unregister_chrdev(myDeviceNumber, DEVICE_NAME);    // relinquish the device number
    gpio_free(gpioPortSpiClock);  
    gpio_free(gpioPortSpiDataOut);
    gpio_free(gpioPortSpiDataIn);                      // release the GPIO pins
    gpio_free(gpioPortChipSelect);  
    gpio_free(gpioPortServoData);  
}

  // pass the names of the driver's constructor/destructor to the
  // module_init, module_exit macros (defined in /linux/init.h)

  module_init(devConstructor);
  module_exit(devDestructor);

// ******** The driver's read, write, open, remove file function overrides ********

  static int devOpen(struct inode *inodep, struct file *filep) {
    return 0;
  }

  static int devRelease(struct inode *inodep, struct file *filep) {
    return 0;
  }

  static ssize_t devRead(struct file *filep, char *userData, size_t len, loff_t *offset) {

    int handAngle;
    
    // read the flex sensor    
    handAngle = getHandAngle();
    
    // populate the kernel data buffer with the latest hand angle reading
    kernelData[0] = (unsigned char)(handAngle);
 
    // copy the data from kernel memory space to user memory space
    if (copy_to_user(userData, kernelData, len) == 0)
      return len;
    else
      return -1;
  }  	
        
  static ssize_t devWrite(struct file *filep, const char *userData, size_t len, loff_t *offset) {

    if (copy_from_user(kernelData, userData, len) == 0) {
     	setGripperAngle((int) kernelData[0]);
    }
    
    return len;
  }    
      
  static int getHandAngle(void) {
  	
     static int i, flexSensorValue, prevFlexSensorValue;
  	  	
     // Read from the flex sensor voltage value via the SPI interface
     // See the data sheet for the MCP3002 for the timing diagram

     gpio_set_value(gpioPortChipSelect, 0);  // activate the chip select signal
         
     gpio_set_value(gpioPortSpiDataOut, 1);  // shift out the 4 bits of control
     for (i = 0; i < 4; i++) { 
       udelay(1);
       gpio_set_value(gpioPortSpiClock, 1);
       udelay(1);
       gpio_set_value(gpioPortSpiClock, 0);
     }  
     
     gpio_set_value(gpioPortSpiDataOut, 0);  // shift in the null bit
     udelay(1);
     gpio_set_value(gpioPortSpiClock, 1);
     udelay(1);
     gpio_set_value(gpioPortSpiClock, 0);

     flexSensorValue = 0;                    // shift in the 10 bit value from the ADC
     for (i = 0; i < 10; i++) { 
       udelay(1);
       gpio_set_value(gpioPortSpiClock, 1);
       flexSensorValue = (flexSensorValue << 1) + gpio_get_value(gpioPortSpiDataIn);
       udelay(1);
       gpio_set_value(gpioPortSpiClock, 0);
     }  
     
     gpio_set_value(gpioPortChipSelect, 1);   // deactivate the chip select signal

     // min/max values returned by the flex sensor (when the hand is fully closed and fully open)
     #define handFullyOpenValue   475
     #define handFullyClosedValue 700

     // normalize to a value between 0 (hand fully closed) and 100 (hand fully open)
     
     if (flexSensorValue > handFullyClosedValue) return (char) prevFlexSensorValue;
     if (flexSensorValue < handFullyOpenValue)   return (char) prevFlexSensorValue;

     flexSensorValue = 100 * (flexSensorValue - handFullyOpenValue) / (handFullyClosedValue - handFullyOpenValue);
     flexSensorValue = 100 - flexSensorValue;
     prevFlexSensorValue = flexSensorValue;
     
     return (char) flexSensorValue;
   }

 static void setGripperAngle(int gripperAngle) {
	
   int servoCommand, i;
	
   if (gripperAngle < 0)   gripperAngle = 0;
   if (gripperAngle > 100) gripperAngle = 100;

   // servo settings are 4200 when fully closed and 7200 when fully open		
   servoCommand = (4200 + 30*gripperAngle);
		
   for (i = 0; i < 3; i++) {
     sendToServo(0xAA);
     sendToServo(0x0C);
     sendToServo(0x04);
     sendToServo(0x00);
     sendToServo(servoCommand & 0x7F);
     sendToServo(servoCommand >> 7);
   }
}  
   
static void sendToServo(int txByte) {
	
 // shift out a byte of data to the servo

  int bit0, bit1, bit2, bit3, bit4, bit5, bit6, bit7;
  
  bit0 = txByte & 1;
  txByte >>= 1;
  bit1 = txByte & 1;
  txByte >>= 1;
  bit2 = txByte & 1;
  txByte >>= 1;
  bit3 = txByte & 1;
  txByte >>= 1;
  bit4 = txByte & 1;
  txByte >>= 1;
  bit5 = txByte & 1;
  txByte >>= 1;
  bit6 = txByte & 1;
  txByte >>= 1;
  bit7 = txByte & 1;
  
  // the data rate is 9600 baud, which equates to 104.1667 usec per bit
  // closest integer value is 104, so bit 3's width is 105 usec to keep the error from accumulating
  
  gpio_set_value(gpioPortServoData, 0);   -- start bit
  udelay(104);
  gpio_set_value(gpioPortServoData, bit0);
  udelay(104);
  gpio_set_value(gpioPortServoData, bit1);
  udelay(104);
  gpio_set_value(gpioPortServoData, bit2);
  udelay(104);
  gpio_set_value(gpioPortServoData, bit3);
  udelay(105);
  gpio_set_value(gpioPortServoData, bit4);
  udelay(104);
  gpio_set_value(gpioPortServoData, bit5);
  udelay(104);
  gpio_set_value(gpioPortServoData, bit6);
  udelay(104);
  gpio_set_value(gpioPortServoData, bit7);
  udelay(104);
  gpio_set_value(gpioPortServoData, 1);  -- stop bit
  udelay(104);
}   