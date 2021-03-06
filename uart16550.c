#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>

#include "uart16550.h"

#include <asm/ioctl.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <asm/io.h>
#include <linux/kfifo.h>
#include <linux/uaccess.h>
#include <linux/ioport.h>
#include <linux/fs.h>

#define COM1_BASEPORT 0x3f8
#define COM2_BASEPORT 0x2f8
#define COM1_IRQ_NO 4
#define COM2_IRQ_NO 3
#define NR_PORTS 8
#define IER					0x01
#define IIR					0x02
#define FCR					0x02
#define LCR					0x03
#define LSR					0x05
#define KFIFO_SIZE 128
#define CHECK_BIT(var, pos) ((var) & (1<<(pos)))
#define SET_BIT_1(var, pos) ((var) | (1<<(pos)))
#define SET_BIT_0(var, pos) ((var) & ~(1<<(pos)))
#define DEFAULT_MAJOR 42
#define DEFAULT_OPTION OPTION_BOTH
#define MINOR_COM1 0
#define MINOR_COM2 1


MODULE_LICENSE("GPL");

struct my_device_data {
	struct cdev dev;
	unsigned long baseport;
	unsigned int minor;
	unsigned int irq_no;
	DECLARE_KFIFO(read_kfifo, unsigned char, KFIFO_SIZE);
	DECLARE_KFIFO(write_kfifo, unsigned char, KFIFO_SIZE);
	wait_queue_head_t read_wq;
	wait_queue_head_t write_wq;
	atomic_t read_kfifo_ready;
	atomic_t write_kfifo_ready;
};

struct my_device_data devs[MAX_NUMBER_DEVICES];

static int major = DEFAULT_MAJOR;
static int option = DEFAULT_OPTION;

module_param(major, int, 0000);
module_param(option, int, 0000);

static int my_open(struct inode *inode, struct file *file)
{
	file->private_data = (struct my_device_data *)
	container_of(inode->i_cdev, struct my_device_data, dev);
	return 0;
}

static int my_read(struct file *file, char __user *user_buffer, size_t size,
	loff_t *offset)
{
	unsigned int len = 0;
	int err;
	struct my_device_data *dev;

	dev = (struct my_device_data *) file->private_data;

	outb(SET_BIT_1(inb(dev->baseport + IER), 0), dev->baseport + IER);
	wait_event_interruptible(dev->read_wq, atomic_cmpxchg
		(&dev->read_kfifo_ready, 0, 0) == 1);

	err = kfifo_to_user(&dev->read_kfifo, user_buffer, size, &len);
	if (err == -EFAULT)
		return 0;

	atomic_set(&dev->read_kfifo_ready, 0);
	return len;
}

static int my_write(struct file *file, const char __user *user_buffer,
	size_t size, loff_t *offset)
{
	unsigned int len;
	int err;
	struct my_device_data *dev;

	dev = (struct my_device_data *) file->private_data;

	err = kfifo_from_user(&dev->write_kfifo, user_buffer, size, &len);
	if (err == -EFAULT)
		return -EFAULT;

	atomic_set(&dev->write_kfifo_ready, 1);
	outb(SET_BIT_1(inb(dev->baseport + IER), 1), dev->baseport + IER);
	wait_event_interruptible(dev->write_wq,
		atomic_cmpxchg(&dev->write_kfifo_ready, 1, 1) == 0);

	return len;
}

static int my_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long my_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct my_device_data *dev;
	struct uart16550_line_info new_settings;
	int err;

	dev = (struct my_device_data *)file->private_data;

	switch (cmd) {
	case UART16550_IOCTL_SET_LINE:
		err = copy_from_user(&new_settings,
		(struct uart16550_line_info *)arg,
		sizeof(struct uart16550_line_info));
		if (err)
			return -ENOTTY;
		outb(0x80, dev->baseport + LCR);
		outb(new_settings.baud, dev->baseport);
		outb(0x00, dev->baseport + IER);
		outb(new_settings.len | new_settings.par | new_settings.stop,
		  dev->baseport + LCR);
		return 0;
	default:
		return -ENOTTY;
	}
}

irqreturn_t uart_handler(int irq_no, void *dev_id)
{
	struct my_device_data *dev;
	char data, lsr_data, iir_data;
	int res;

	dev = (struct my_device_data *)dev_id;
	iir_data = inb(dev->baseport + IIR);

	if (CHECK_BIT(iir_data, 2)) {
	while (true) {
		lsr_data = inb(dev->baseport + LSR);

		if (!(CHECK_BIT(lsr_data, 0)) ||
		atomic_read(&dev->read_kfifo_ready) == 1)
			break;
		data = inb(dev->baseport);
		kfifo_in(&dev->read_kfifo, &data, 1);
	}

	atomic_set(&dev->read_kfifo_ready, 1);
	wake_up(&dev->read_wq);
	outb(SET_BIT_0(inb(dev->baseport + IER), 0), dev->baseport + IER);

	return IRQ_HANDLED;
	} else if (CHECK_BIT(iir_data, 1)) {
		while (true) {
			lsr_data = inb(dev->baseport + LSR);
			if (!(CHECK_BIT(lsr_data, 5)) ||
			atomic_read(&dev->write_kfifo_ready) == 0)
				break;
			res = kfifo_out(&dev->write_kfifo, &data, 1);
			if (!res)
				break;
			outb(data, dev->baseport);
		}
		atomic_set(&dev->write_kfifo_ready, 0);
		wake_up(&dev->write_wq);
		outb(SET_BIT_0(inb(dev->baseport + IER), 1),
		dev->baseport + IER);

		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}

const struct file_operations my_fops = {
	.owner = THIS_MODULE,
	.open = my_open,
	.read = my_read,
	.write = my_write,
	.release = my_release,
	.unlocked_ioctl = my_ioctl
};

static int register_port(struct my_device_data *device)
{
	int err;

	err = register_chrdev_region(MKDEV(major, device->minor), 1,
	"uart16550");
	if (err != 0)
		return err;

	cdev_init(&device->dev, &my_fops);
	cdev_add(&device->dev, MKDEV(major, device->minor), 1);

	if (!request_region(device->baseport, NR_PORTS, "uart16550"))
		return -ENODEV;

	err = request_irq(device->irq_no, uart_handler, IRQF_SHARED,
	"uart16550", device);

	if (err < 0)
		return err;

	outb(0x01, device->baseport + FCR);

	INIT_KFIFO(device->read_kfifo);
	INIT_KFIFO(device->write_kfifo);

	init_waitqueue_head(&device->read_wq);
	init_waitqueue_head(&device->write_wq);

	atomic_set(&device->read_kfifo_ready, 0);
	atomic_set(&device->write_kfifo_ready, 0);

	return 0;
}

static int release_port(struct my_device_data *device)
{
	free_irq(device->irq_no, device);
	release_region(device->baseport, NR_PORTS);
	cdev_del(&device->dev);
	unregister_chrdev_region(MKDEV(major, device->minor), 1);
	return 0;
}

static void declare_devices(void)
{
	devs[0].baseport = COM1_BASEPORT;
	devs[0].minor = MINOR_COM1;
	devs[0].irq_no = COM1_IRQ_NO;

	devs[1].baseport = COM2_BASEPORT;
	devs[1].minor = MINOR_COM2;
	devs[1].irq_no = COM2_IRQ_NO;
}

static int initialize_devices(void)
{
	switch (option) {
	case OPTION_COM1:
		register_port(&devs[MINOR_COM1]);
		break;
	case OPTION_COM2:
		register_port(&devs[MINOR_COM2]);
		break;
	case OPTION_BOTH:
		register_port(&devs[MINOR_COM1]);
		register_port(&devs[MINOR_COM2]);
		break;
	default:
		return -1;
	}
	return 0;
}

static void release_devices(void)
{
	switch (option) {
	case OPTION_COM1:
		release_port(&devs[MINOR_COM1]);
		break;
	case OPTION_COM2:
		release_port(&devs[MINOR_COM2]);
		break;
	default:
		release_port(&devs[MINOR_COM1]);
		release_port(&devs[MINOR_COM2]);
	}
}

static int uart16550_init(void)
{
	declare_devices();
	return initialize_devices();
}

static void uart16550_exit(void)
{
	release_devices();
}

module_init(uart16550_init);
module_exit(uart16550_exit);
