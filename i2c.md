## How to write a linux i2c driver (in kernel or userspace)

This article will tell you how to write a i2c driver for an external device like PCA9685 or some sensors using i2c interface. This tutorial is based on the latest Linux kernel 5.x.x. However, the general structure is the same even in kenel 2.6.0, just some new features (`regmap pm_ops`) were not supported.

This article only delivers the main concept of implementing i2c driver in Linux, please remember since Linux is fast developing every day, the structures and code in this article are not latest, please refer to the daily mainline Linux kernel [i2c source code](https://github.com/torvalds/linux/blob/master/include/linux/i2c.h) for reference.

There are two ways of implementing a i2c driver in Linux, we will talk about the kernel driver first, then the user space driver.

### Build the driver structure

First we need to provider the i2c driver with necessary information about the device, `probe` and `remove` routines, etc. For example:

```c
/* The device names to be registered into i2c table */
static struct i2c_device_id foo_idtable[] = {
	{ "foo", my_id_for_foo },
	{ "bar", my_id_for_bar },
	{ }
};

/* Tell user space about which devices the module supports */
MODULE_DEVICE_TABLE(i2c, foo_idtable);
// MODULE_ALIAS("foo_alias");

/* Driver itself provides general methods */
static struct i2c_driver foo_driver = {
	.driver = {
    /* Matched i2c device name, must not contain spaces */
		.name	= "foo",
    /* Power manage ops */
		.pm	= &foo_pm_ops,	/* optional */
	},
  
	.id_table	= foo_idtable,
	.probe		= foo_probe, /* Callback for probe routine */
	.remove		= foo_remove, /* Callback for remove routine */
  
	/* if device autodetection is needed: (talk later)*/
	.class		= I2C_CLASS_SOMETHING,
	.detect		= foo_detect,
	.address_list	= normal_i2c,
  
	.shutdown	= foo_shutdown,	/* shutdown or reboot */
	.command	= foo_command,	/* optional, deprecated */
}
```

If we [instantiate a i2c device in userspace](https://www.kernel.org/doc/Documentation/i2c/instantiating-devices) with:

```sh
# Instantiate an i2c device foo at address 0x50
echo foo 0x50 > /sys/bus/i2c/devices/i2c-3/new_device
```

The kernel will find `foo_driver` according to the name `foo` and use contained methods for common routines.



### Understand the client structure

If you read the code in the driver part you will notice there is no operation methods like read and write. This is because a driver structure only contains general access routines (through some callback functions). It is used for the kernel to manage the device at certain events like shutdown, sleep, or device is removed. 

When a i2c device is connected on the bus, a `i2c_client` structure needs to be instantiated. The client structure contains all the information of the slave device, like shown below, code is cut from the linux mainline [i2c driver source code](https://github.com/torvalds/linux/blob/master/include/linux/i2c.h#L298):

```c
/**
 * struct i2c_client - represent an I2C slave device
 * @flags: see I2C_CLIENT_* for possible flags
 * @addr: Address used on the I2C bus connected to the parent adapter.
 * @name: Indicates the type of the device, usually a chip name that's
 *	generic enough to hide second-sourcing and compatible revisions.
 * @adapter: manages the bus segment hosting this I2C device
 * @dev: Driver model device node for the slave.
 * @init_irq: IRQ that was set at initialization
 * @irq: indicates the IRQ generated by this device (if any)
 * @detected: member of an i2c_driver.clients list or i2c-core's
 *	userspace_devices list
 * @slave_cb: Callback when I2C slave mode of an adapter is used. The adapter
 *	calls it to pass on slave events to the slave driver.
 * @devres_group_id: id of the devres group that will be created for resources
 *	acquired when probing this device.
 *
 * An i2c_client identifies a single device (i.e. chip) connected to an
 * i2c bus. The behaviour exposed to Linux is defined by the driver
 * managing the device.
 */
struct i2c_client {
	unsigned short flags;		/* div., see below		*/
#define I2C_CLIENT_PEC		0x04	/* Use Packet Error Checking */
#define I2C_CLIENT_TEN		0x10	/* we have a ten bit chip address */
					/* Must equal I2C_M_TEN below */
#define I2C_CLIENT_SLAVE	0x20	/* we are the slave */
#define I2C_CLIENT_HOST_NOTIFY	0x40	/* We want to use I2C host notify */
#define I2C_CLIENT_WAKE		0x80	/* for board_info; true iff can wake */
#define I2C_CLIENT_SCCB		0x9000	/* Use Omnivision SCCB protocol */
					/* Must match I2C_M_STOP|IGNORE_NAK */

	unsigned short addr;		/* chip address - NOTE: 7bit	*/
					/* addresses are stored in the	*/
					/* _LOWER_ 7 bits		*/
	char name[I2C_NAME_SIZE];
	struct i2c_adapter *adapter;	/* the adapter we sit on	*/
	struct device dev;		/* the device structure		*/
	int init_irq;			/* irq set at initialization	*/
	int irq;			/* irq issued by device		*/
	struct list_head detected;
#if IS_ENABLED(CONFIG_I2C_SLAVE)
	i2c_slave_cb_t slave_cb;	/* callback for slave mode	*/
#endif
	void *devres_group_id;		/* ID of probe devres group	*/
};
```

Unlike other device driver, i2c client does not contain the ops for the device. Ops are stored in a `i2c_algorithm` structure.


#### Extra client data

Each client structure has a special data field that can point to any structure at all. You should use this to keep device-specific data.

```c
/* store the value */
void i2c_set_clientdata(struct i2c_client *client, void *data);

/* retrieve the value */
void *i2c_get_clientdata(const struct i2c_client *client);
```

Please be care that i2c client structure DOES NOT HAVE any ops assiociated with the device, instead, linux iteself had i2c stack implemented already (in `struct i2c_algorithm`) and we just need to call from these methods to communicate, these ops are:

#### Plain i2c communication

```c
int i2c_master_send(struct i2c_client *client, const char *buf,
			    int count);
int i2c_master_recv(struct i2c_client *client, char *buf, int count);

int i2c_transfer(struct i2c_adapter *adap, struct i2c_msg *msg,
			 int num);
```

These methods are not prefered if the device supports the following `SMBus` ops.

#### SMBus communication

```c
s32 i2c_smbus_xfer(struct i2c_adapter *adapter, u16 addr,
			unsigned short flags, char read_write, u8 command,
			int size, union i2c_smbus_data *data);
```

This is the generic SMBus function. All functions below are implemented
in terms of it. Never use this function directly!

```c
s32 i2c_smbus_read_byte(struct i2c_client *client);
s32 i2c_smbus_write_byte(struct i2c_client *client, u8 value);
s32 i2c_smbus_read_byte_data(struct i2c_client *client, u8 command);
s32 i2c_smbus_write_byte_data(struct i2c_client *client,
				u8 command, u8 value);
s32 i2c_smbus_read_word_data(struct i2c_client *client, u8 command);
s32 i2c_smbus_write_word_data(struct i2c_client *client,
				u8 command, u16 value);
s32 i2c_smbus_read_block_data(struct i2c_client *client,
				u8 command, u8 *values);
s32 i2c_smbus_write_block_data(struct i2c_client *client,
				u8 command, u8 length, const u8 *values);
s32 i2c_smbus_read_i2c_block_data(struct i2c_client *client,
				u8 command, u8 length, u8 *values);
s32 i2c_smbus_write_i2c_block_data(struct i2c_client *client,
				u8 command, u8 length,
				const u8 *values);
```

These ones were removed from i2c-core because they had no users, but could
be added back later if needed:

```c
s32 i2c_smbus_write_quick(struct i2c_client *client, u8 value);
s32 i2c_smbus_process_call(struct i2c_client *client,
			u8 command, u16 value);
s32 i2c_smbus_block_process_call(struct i2c_client *client,
			u8 command, u8 length, u8 *values);
```

All these transactions return a negative errno value on failure. The 'write'
transactions return 0 on success; the 'read' transactions return the read
value, except for block transactions, which return the number of values
read. The block buffers need not be longer than 32 bytes.

You can read the file `smbus-protocol' for more information about the
actual SMBus protocol.

### Other important structures

So now you see the i2c client and the i2c driver structures, there are other 2 structures in Linux kernel that are very important. They are, at most time, irrelavent in the code design because they take care of the i2c infrastures like the adapters. We will talk about it here briefly:

#### `i2c_adapter` structure

```c
/*
 * i2c_adapter is the structure used to identify a physical i2c bus along
 * with the access algorithms necessary to access it.
 */
struct i2c_adapter {
	struct module *owner;
	unsigned int class;		  /* classes to allow probing for */
	const struct i2c_algorithm *algo; /* the algorithm to access the bus */
	void *algo_data;

	/* data fields that are valid for all devices	*/
	const struct i2c_lock_operations *lock_ops;
	struct rt_mutex bus_lock;
	struct rt_mutex mux_lock;

	int timeout;			/* in jiffies */
	int retries;
	struct device dev;		/* the adapter device */
	unsigned long locked_flags;	/* owned by the I2C core */
#define I2C_ALF_IS_SUSPENDED		0
#define I2C_ALF_SUSPEND_REPORTED	1

	int nr;
	char name[48];
	struct completion dev_released;

	struct mutex userspace_clients_lock;
	struct list_head userspace_clients;

	struct i2c_bus_recovery_info *bus_recovery_info;
	const struct i2c_adapter_quirks *quirks;

	struct irq_domain *host_notify_domain;
};
#define to_i2c_adapter(d) container_of(d, struct i2c_adapter, dev)
```

Further information can be found at [linux/i2c.h](https://github.com/torvalds/linux/blob/master/include/linux/i2c.h)

#### `i2c_algorithm` structure

```c
struct i2c_algorithm {
	/*
	 * If an adapter algorithm can't do I2C-level access, set master_xfer
	 * to NULL. If an adapter algorithm can do SMBus access, set
	 * smbus_xfer. If set to NULL, the SMBus protocol is simulated
	 * using common I2C messages.
	 *
	 * master_xfer should return the number of messages successfully
	 * processed, or a negative value on error
	 */
	int (*master_xfer)(struct i2c_adapter *adap, struct i2c_msg *msgs,
			   int num);
	int (*master_xfer_atomic)(struct i2c_adapter *adap,
				   struct i2c_msg *msgs, int num);
	int (*smbus_xfer)(struct i2c_adapter *adap, u16 addr,
			  unsigned short flags, char read_write,
			  u8 command, int size, union i2c_smbus_data *data);
	int (*smbus_xfer_atomic)(struct i2c_adapter *adap, u16 addr,
				 unsigned short flags, char read_write,
				 u8 command, int size, union i2c_smbus_data *data);

	/* To determine what the adapter supports */
	u32 (*functionality)(struct i2c_adapter *adap);

#if IS_ENABLED(CONFIG_I2C_SLAVE)
	int (*reg_slave)(struct i2c_client *client);
	int (*unreg_slave)(struct i2c_client *client);
#endif
};
```

Further information can be found at [linux/i2c.h](https://github.com/torvalds/linux/blob/master/include/linux/i2c.h)

### How to instantiate a `i2c_client` structure

After understanding all the structures above, you may have a faint concept of their relationship and now we are going to write some code. In order to communicate a i2c device on the bus, you need to instantiate the `i2c_client` structure first. This step however can be done in 4 different ways. Reference: [How to instantiate I2C devices](https://www.kernel.org/doc/Documentation/i2c/instantiating-devices)

* Method 1a: Declare the I2C devices by bus number
* Method 1b: Declare the I2C devices via devicetree
* Method 1c: Declare the I2C devices via ACPI
* Method 2: Instantiate the devices explicitly
* Method 3: Probe an I2C bus for certain devices (should be avoided)
* Method 4: Instantiate from user-space (should be avoided)

After you instantiate the `i2c_client`, you are free to use the i2c/smbus communication methods above. If a device supports smbus protocol, it is always better than using the plain i2c communication methods.



## Using `regmap` to access i2c device

If you are familiar with the i2c protocol you may notice that a lot of the i2c operations are based on the register reading and writing. So if you are dealing with such a device, there is a new feature after Linux kernel version 3.1 called `regmap`. It abstracts the registers on the i2c device, so you can read and write via memory address like operating on a normal memory segments. 

`regmap` is very useful in i2c and spi communication methods, it provides a high level abstraction and data buffering mechanism for i2c communication. 

### `regmap_config` Structure

```c
/**
 * struct regmap_config - Configuration for the register map of a device.
 *
 * @name: Optional name of the regmap. Useful when a device has multiple
 *        register regions.
 *
 * @reg_bits: Number of bits in a register address, mandatory.
 * @reg_stride: The register address stride. Valid register addresses are a
 *              multiple of this value. If set to 0, a value of 1 will be
 *              used.
 * @pad_bits: Number of bits of padding between register and value.
 * @val_bits: Number of bits in a register value, mandatory.
 *
 * @writeable_reg: Optional callback returning true if the register
 *		   can be written to. If this field is NULL but wr_table
 *		   (see below) is not, the check is performed on such table
 *                 (a register is writeable if it belongs to one of the ranges
 *                  specified by wr_table).
 * @readable_reg: Optional callback returning true if the register
 *		  can be read from. If this field is NULL but rd_table
 *		   (see below) is not, the check is performed on such table
 *                 (a register is readable if it belongs to one of the ranges
 *                  specified by rd_table).
 * @volatile_reg: Optional callback returning true if the register
 *		  value can't be cached. If this field is NULL but
 *		  volatile_table (see below) is not, the check is performed on
 *                such table (a register is volatile if it belongs to one of
 *                the ranges specified by volatile_table).
 * @precious_reg: Optional callback returning true if the register
 *		  should not be read outside of a call from the driver
 *		  (e.g., a clear on read interrupt status register). If this
 *                field is NULL but precious_table (see below) is not, the
 *                check is performed on such table (a register is precious if
 *                it belongs to one of the ranges specified by precious_table).
 * @writeable_noinc_reg: Optional callback returning true if the register
 *			supports multiple write operations without incrementing
 *			the register number. If this field is NULL but
 *			wr_noinc_table (see below) is not, the check is
 *			performed on such table (a register is no increment
 *			writeable if it belongs to one of the ranges specified
 *			by wr_noinc_table).
 * @readable_noinc_reg: Optional callback returning true if the register
 *			supports multiple read operations without incrementing
 *			the register number. If this field is NULL but
 *			rd_noinc_table (see below) is not, the check is
 *			performed on such table (a register is no increment
 *			readable if it belongs to one of the ranges specified
 *			by rd_noinc_table).
 * @disable_locking: This regmap is either protected by external means or
 *                   is guaranteed not to be accessed from multiple threads.
 *                   Don't use any locking mechanisms.
 * @lock:	  Optional lock callback (overrides regmap's default lock
 *		  function, based on spinlock or mutex).
 * @unlock:	  As above for unlocking.
 * @lock_arg:	  this field is passed as the only argument of lock/unlock
 *		  functions (ignored in case regular lock/unlock functions
 *		  are not overridden).
 * @reg_read:	  Optional callback that if filled will be used to perform
 *           	  all the reads from the registers. Should only be provided for
 *		  devices whose read operation cannot be represented as a simple
 *		  read operation on a bus such as SPI, I2C, etc. Most of the
 *		  devices do not need this.
 * @reg_write:	  Same as above for writing.
 * @fast_io:	  Register IO is fast. Use a spinlock instead of a mutex
 *	     	  to perform locking. This field is ignored if custom lock/unlock
 *	     	  functions are used (see fields lock/unlock of struct regmap_config).
 *		  This field is a duplicate of a similar file in
 *		  'struct regmap_bus' and serves exact same purpose.
 *		   Use it only for "no-bus" cases.
 * @max_register: Optional, specifies the maximum valid register address.
 * @wr_table:     Optional, points to a struct regmap_access_table specifying
 *                valid ranges for write access.
 * @rd_table:     As above, for read access.
 * @volatile_table: As above, for volatile registers.
 * @precious_table: As above, for precious registers.
 * @wr_noinc_table: As above, for no increment writeable registers.
 * @rd_noinc_table: As above, for no increment readable registers.
 * @reg_defaults: Power on reset values for registers (for use with
 *                register cache support).
 * @num_reg_defaults: Number of elements in reg_defaults.
 *
 * @read_flag_mask: Mask to be set in the top bytes of the register when doing
 *                  a read.
 * @write_flag_mask: Mask to be set in the top bytes of the register when doing
 *                   a write. If both read_flag_mask and write_flag_mask are
 *                   empty and zero_flag_mask is not set the regmap_bus default
 *                   masks are used.
 * @zero_flag_mask: If set, read_flag_mask and write_flag_mask are used even
 *                   if they are both empty.
 * @use_relaxed_mmio: If set, MMIO R/W operations will not use memory barriers.
 *                    This can avoid load on devices which don't require strict
 *                    orderings, but drivers should carefully add any explicit
 *                    memory barriers when they may require them.
 * @use_single_read: If set, converts the bulk read operation into a series of
 *                   single read operations. This is useful for a device that
 *                   does not support  bulk read.
 * @use_single_write: If set, converts the bulk write operation into a series of
 *                    single write operations. This is useful for a device that
 *                    does not support bulk write.
 * @can_multi_write: If set, the device supports the multi write mode of bulk
 *                   write operations, if clear multi write requests will be
 *                   split into individual write operations
 *
 * @cache_type: The actual cache type.
 * @reg_defaults_raw: Power on reset values for registers (for use with
 *                    register cache support).
 * @num_reg_defaults_raw: Number of elements in reg_defaults_raw.
 * @reg_format_endian: Endianness for formatted register addresses. If this is
 *                     DEFAULT, the @reg_format_endian_default value from the
 *                     regmap bus is used.
 * @val_format_endian: Endianness for formatted register values. If this is
 *                     DEFAULT, the @reg_format_endian_default value from the
 *                     regmap bus is used.
 *
 * @ranges: Array of configuration entries for virtual address ranges.
 * @num_ranges: Number of range configuration entries.
 * @use_hwlock: Indicate if a hardware spinlock should be used.
 * @hwlock_id: Specify the hardware spinlock id.
 * @hwlock_mode: The hardware spinlock mode, should be HWLOCK_IRQSTATE,
 *		 HWLOCK_IRQ or 0.
 * @can_sleep: Optional, specifies whether regmap operations can sleep.
 */
struct regmap_config {
	const char *name;

	int reg_bits;
	int reg_stride;
	int pad_bits;
	int val_bits;

	bool (*writeable_reg)(struct device *dev, unsigned int reg);
	bool (*readable_reg)(struct device *dev, unsigned int reg);
	bool (*volatile_reg)(struct device *dev, unsigned int reg);
	bool (*precious_reg)(struct device *dev, unsigned int reg);
	bool (*writeable_noinc_reg)(struct device *dev, unsigned int reg);
	bool (*readable_noinc_reg)(struct device *dev, unsigned int reg);

	bool disable_locking;
	regmap_lock lock;
	regmap_unlock unlock;
	void *lock_arg;

	int (*reg_read)(void *context, unsigned int reg, unsigned int *val);
	int (*reg_write)(void *context, unsigned int reg, unsigned int val);

	bool fast_io;

	unsigned int max_register;
	const struct regmap_access_table *wr_table;
	const struct regmap_access_table *rd_table;
	const struct regmap_access_table *volatile_table;
	const struct regmap_access_table *precious_table;
	const struct regmap_access_table *wr_noinc_table;
	const struct regmap_access_table *rd_noinc_table;
	const struct reg_default *reg_defaults;
	unsigned int num_reg_defaults;
	enum regcache_type cache_type;
	const void *reg_defaults_raw;
	unsigned int num_reg_defaults_raw;

	unsigned long read_flag_mask;
	unsigned long write_flag_mask;
	bool zero_flag_mask;

	bool use_single_read;
	bool use_single_write;
	bool use_relaxed_mmio;
	bool can_multi_write;

	enum regmap_endian reg_format_endian;
	enum regmap_endian val_format_endian;

	const struct regmap_range_cfg *ranges;
	unsigned int num_ranges;

	bool use_hwlock;
	unsigned int hwlock_id;
	unsigned int hwlock_mode;

	bool can_sleep;
};
```

You may say WOW! That's too much. However, we do not need to fill this structure ourself for the most case, we can use `devm_regmap_init_i2c` marco, such as in [`pwm-pca9685.c`](https://github.com/torvalds/linux/blob/master/drivers/pwm/pwm-pca9685.c#L457), we only need to fill part of it and the rest will be filled using these marcos.

```c
/* Code segment from pwm-pca9685.c */
static const struct regmap_config pca9685_regmap_i2c_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = PCA9685_NUMREGS,
	.cache_type = REGCACHE_NONE,
};

/* Some code is skipped */
/* Fill the rest fields with `devm_regmap_init_i2c` marco */
pca->regmap = devm_regmap_init_i2c(client, &pca9685_regmap_i2c_config);
```

### `devm_regmap_init_i2c` macro

```c
/**
 * devm_regmap_init_i2c() - Initialise managed register map
 *
 * @i2c: Device that will be interacted with. Here will be our i2c_client.
 * @config: Configuration for register map
 *
 * The return value will be an ERR_PTR() on error or a valid pointer
 * to a struct regmap.  The regmap will be automatically freed by the
 * device management code.
 */
#define devm_regmap_init_i2c(i2c, config)				\
...
```

### Read and write `regmap`

After the we create the regmap, we can read/write the regmap, regmap which will send a `smbus` read/write command to the i2c slave and fetch the data. [regmap.c](https://github.com/torvalds/linux/blob/master/drivers/base/regmap/regmap.c)

```c
/**
 * regmap_read() - Read a value from a single register
 *
 * @map: Register map to read from
 * @reg: Register to be read from
 * @val: Pointer to store read value
 *
 * A value of zero will be returned on success, a negative errno will
 * be returned in error cases.
 */
int regmap_read(struct regmap *map, unsigned int reg, unsigned int *val)
{
	int ret;

	if (!IS_ALIGNED(reg, map->reg_stride))
		return -EINVAL;

	map->lock(map->lock_arg);

	ret = _regmap_read(map, reg, val);

	map->unlock(map->lock_arg);

	return ret;
}

/**
 * regmap_write() - Write a value to a single register
 *
 * @map: Register map to write to
 * @reg: Register to write to
 * @val: Value to be written
 *
 * A value of zero will be returned on success, a negative errno will
 * be returned in error cases.
 */
int regmap_write(struct regmap *map, unsigned int reg, unsigned int val)
{
	int ret;

	if (!IS_ALIGNED(reg, map->reg_stride))
		return -EINVAL;

	map->lock(map->lock_arg);

	ret = _regmap_write(map, reg, val);

	map->unlock(map->lock_arg);

	return ret;
}
```

## Accessing i2c from user space

You may want to write an user space i2c driver code, however it is not prefered. If you want to understand what is the difference between a kernel driver and a user space driver, check LDD3 [Chapter 2](https://static.lwn.net/images/pdf/LDD3/ch02.pdf) at page 37.

In general, a kernel driver always performs better than a user space driver. However, in case some people want it, here is a brief introduction of [how to access i2c device from userspace](https://www.kernel.org/doc/Documentation/i2c/dev-interface). It is pretty straight forward with a very good example.


Reference list:

1. https://www.kernel.org/doc/Documentation/i2c/writing-clients
2. https://www.kernel.org/doc/Documentation/i2c/instantiating-devices
3. https://www.kernel.org/doc/Documentation/i2c/dev-interface
4. https://github.com/torvalds/linux/blob/master/include/linux/i2c.h
5. https://github.com/torvalds/linux/blob/master/drivers/pwm/pwm-pca9685.c
6. https://github.com/torvalds/linux/blob/master/drivers/base/regmap/regmap.c
7. https://github.com/torvalds/linux/blob/master/include/linux/regmap.h
