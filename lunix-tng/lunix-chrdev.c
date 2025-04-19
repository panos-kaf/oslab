/*
 * lunix-chrdev.c
 *
 * Implementation of character devices
 * for Lunix:TNG
 *
 * Nikolaos Angelitsis 
 *
 */

#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mmzone.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>

#include "lunix.h"
#include "lunix-chrdev.h"
#include "lunix-lookup.h"

/*
 * Global data
 */
struct cdev lunix_chrdev_cdev;

static int lunix_chrdev_state_needs_refresh(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor = state->sensor;
	
	WARN_ON(sensor == NULL);
	if(!sensor) {
		return 0;
	}
	if (sensor->msr_data[state->type]->last_update != state->buf_timestamp) {
		debug("Needs refresh\n");
		return 1;
	}

	return 0;
}

/*
 * Updates the cached state of a character device
 * based on sensor data. Must be called with the
 * character device state lock held.
 */
static int lunix_chrdev_state_update(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor = state->sensor;
	if(!sensor) {
		debug("Sensor not found\n");
		return -ENODEV;
	}
	int ret = 0;

	if(down_interruptible(&state->lock)) {
		debug("Semaphore down_interruptible failed\n");
		return -ERESTARTSYS;
	}
	/*
	 * Grab the raw data quickly, hold the
	 * spinlock for as little as possible.
	 */

	/* Why use spinlocks? See LDD3, p. 119 */
	/*
	 * Any new data available?
	 */
	uint32_t raw_data;
	if(lunix_chrdev_state_needs_refresh(state)) {
		spin_lock(&sensor->lock);
		state->buf_timestamp = sensor->msr_data[state->type]->last_update;
		state->buf_lim = LUNIX_CHRDEV_BUFSZ;
		raw_data = sensor->msr_data[state->type]->values[0];
		spin_unlock(&sensor->lock);
	}
	else{
		ret = -EAGAIN;
		up(&state->lock);
		return ret;
	}
	debug("Updated\n");
	/*
	 * Now we can take our time to format them,
	 * holding only the private state semaphore
	 */
	long voltage = lookup_voltage[raw_data];
	long temperature = lookup_temperature[raw_data];
	long light = lookup_light[raw_data];
	switch (state->type) {
		case BATT:
			if(state->mode == RAW) {
				state->buf_lim = snprintf(state->buf_data, state->buf_lim, "0x%04x\n", raw_data);
			}
			else {
				state->buf_lim = snprintf(state->buf_data, state->buf_lim, "%ld.%03ld\n", voltage / 1000, voltage % 1000);
			}
			break;
		case TEMP:
			if(state->mode == RAW) {
				state->buf_lim = snprintf(state->buf_data, state->buf_lim, "0x%04x\n", raw_data);
			}
			else {
				state->buf_lim = snprintf(state->buf_data, state->buf_lim, "%ld.%03ld\n", temperature / 1000, temperature % 1000);
			}
			break;
		case LIGHT:
			if(state->mode == RAW) {
				state->buf_lim = snprintf(state->buf_data, state->buf_lim, "0x%04x\n", raw_data);
			}
			else {
				state->buf_lim = snprintf(state->buf_data, state->buf_lim, "%ld.%03ld\n", light / 1000, light % 1000);
			}
			break;
		default:
			debug("Invalid type\n");
			ret = -EINVAL;
			break;
	}

	debug("leaving state_update\n");
	up(&state->lock);
	return ret;
}

/*************************************
 * Implementation of file operations
 * for the Lunix character device
 *************************************/

static int lunix_chrdev_open(struct inode *inode, struct file *filp)
{
	/* Declarations */
	int minor; // Minor number of the lunix device opened.
	enum lunix_msr_enum type; // Type of measurement that the device is set to provide
	int sensor_num; 
	struct lunix_sensor_struct *sensor;
	struct lunix_chrdev_state_struct *state;

	int ret;

	debug("Entering open syscall for lunix device %d\n", iminor(inode));
	if ((ret = nonseekable_open(inode, filp)) < 0){
		// This is obsolete nonseekable_open return only 0.
		ret = -ENODEV;
		goto out;
	}

	/*
	 * Associate this open file with the relevant sensor based on
	 * the minor number of the device node [/dev/sensor<NO>-<TYPE>]
	 */
	
	minor = iminor(inode);
	type = minor & 0x7; // Lower 3 bits contain the type of the sensor (0-7)
	sensor_num = minor >> 3; //Divide by 8 to get the sensor number

	if (sensor_num >= lunix_sensor_cnt) {
		debug("Invalid sensor number\n");
		ret = -ENODEV;
		goto out;
	}

	sensor = &lunix_sensors[sensor_num];
	if (!sensor) {
		debug("Sensor not found\n");
		ret = -ENODEV;
		goto out;
	}

	if(type >= N_LUNIX_MSR) {
		debug("Invalid type\n");
		ret = -ENODEV;
		goto out;
	}
	
	/* Allocate a new Lunix character device private state structure */

	// This state struct allocation should be necesarilly deallocated in the release function
	state = kmalloc(sizeof(struct lunix_chrdev_state_struct), GFP_KERNEL);
	if (!state) {
		debug("kmalloc for lunix_chrdev_state_struct failed\n");
		ret = -ENOMEM;
		goto out;
	}

	sema_init(&state->lock, 1); // Initialize the semaphore to 1, meaning that only one process can access the state at a time
	if(down_interruptible(&state->lock)) {
		ret = -ERESTARTSYS;
		goto out;
	}
	//Initialisation of the state struct
	state->type = type;
	state->sensor = sensor;
	state->buf_lim = 0;
	state->buf_timestamp = 0;
	state->mode = COOKED;
	up(&state->lock);
	filp->private_data = state;
	filp->f_pos = 0;

	ret = 0;
out:
	debug("leaving from lunix_chrdev_open, with ret = %d\n", ret);
	return ret;
}

static int lunix_chrdev_release(struct inode *inode, struct file *filp)
{
	struct lunix_chrdev_state_struct *state;
	state = filp->private_data;
	if(state) {
		kfree(state);
	}
	debug("Someone is closing a lunix device\n");
	return 0;
}

static long lunix_chrdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct lunix_chrdev_state_struct *state = filp->private_data;
	printk("Someone is doing an ioctl on a lunix device\n");
	WARN_ON(!state);
	if(_IOC_TYPE(cmd) != LUNIX_IOC_MAGIC) {
		return -ENOTTY;
	}
	switch(cmd){
		case LUNIX_IOC_COOKED:
			state->mode = COOKED;
			return 0;
		case LUNIX_IOC_RAW:
			state->mode = RAW;
			return 0;
		default:
			return -ENOTTY;
	}
	return 0;
}

static ssize_t lunix_chrdev_read(struct file *filp, char __user *usrbuf, size_t cnt, loff_t *f_pos)
{
	ssize_t ret;

	struct lunix_chrdev_state_struct *state = filp->private_data;
	struct lunix_sensor_struct *sensor =  state->sensor;
	printk("Someone is reading from a lunix device\n");
	WARN_ON(!state);
	WARN_ON(!sensor);
	if(!sensor) {
		return -ENODEV;
	}
	
	/*
	 * If the cached character device state needs to be
	 * updated by actual sensor data (i.e. we need to report
	 * on a "fresh" measurement, do so
	 */

	if (*f_pos == 0) {
		debug("The process goes to sleep\n");
		while (lunix_chrdev_state_update(state) == -EAGAIN) {
			// The process needs to sleep 
			// See LDD3, page 153 for a hint
			if(wait_event_interruptible(sensor->wq, lunix_chrdev_state_needs_refresh(state))) {
				ret = -ERESTARTSYS;
				goto out;
			}
		}
	}
	
	/* End of file */
	if(*f_pos >= state->buf_lim) {
		debug("EOF\n");
		ret = 0;
		goto out;
	}
	
	/* Determine the number of cached bytes to copy to userspace */
	if(cnt > state->buf_lim - *f_pos) {
		cnt = state->buf_lim - *f_pos;
	}
	
	/* Copy to user */
	if(copy_to_user(usrbuf, state->buf_data + *f_pos, cnt) != 0) {
		debug("Failed to copy to userspace\n");
		ret = -EFAULT;
		goto out;
	}
	printk("Copied %ld bytes to userspace\n", cnt);

	/* Auto-rewind on EOF mode? */
	*f_pos += cnt;
	if(*f_pos >= state->buf_lim) {
		*f_pos = 0;
	}
	ret = cnt;
out:
	printk("return from read with ret = %ld\n", ret);
	return ret;
}

static int lunix_chrdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	return -EINVAL;
}

static struct file_operations lunix_chrdev_fops = 
{
	.owner          = THIS_MODULE,
	.open           = lunix_chrdev_open,
	.release        = lunix_chrdev_release,
	.read           = lunix_chrdev_read,
	.unlocked_ioctl = lunix_chrdev_ioctl,
	.mmap           = lunix_chrdev_mmap
};

int lunix_chrdev_init(void)
{
	/*
	 * Register the character device with the kernel, asking for
	 * a range of minor numbers (number of sensors * 8 measurements / sensor)
	 * beginning with LINUX_CHRDEV_MAJOR:0
	 */
	int ret;
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;

	debug("initializing character device\n");
	cdev_init(&lunix_chrdev_cdev, &lunix_chrdev_fops);
	lunix_chrdev_cdev.owner = THIS_MODULE;
	lunix_chrdev_cdev.ops = &lunix_chrdev_fops;
	
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	ret = register_chrdev_region(dev_no, lunix_minor_cnt, "lunix");
	debug("register_chrdev_region returned %d\n", ret);
	/* register_chrdev_region*/

	if (ret < 0) {
		debug("failed to register region, ret = %d\n", ret);
		goto out;
	}
	/* cdev_add */
	// -- initialize the char device
	ret = cdev_add(&lunix_chrdev_cdev, dev_no, lunix_minor_cnt);

	if (ret < 0) {
		debug("failed to add character device\n");
		goto out_with_chrdev_region;
	}
	debug("Added character device\n");
	return 0;

out_with_chrdev_region:
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
out:
	return ret;
}

void lunix_chrdev_destroy(void)
{
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;

	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	cdev_del(&lunix_chrdev_cdev);
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
	debug("Just destroyed all lunix devices\n");
}
