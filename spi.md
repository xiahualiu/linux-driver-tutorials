## How to write a SPI driver in Linux (kernel and user space)

In this section I will go over the detail of implementing a SPI driver in Linux kernel and user space. 

## Fill the `spi_driver` structure

`spi_driver` contains the informaiton of general access routine of the SPI device. Things like `probe` and `remove`. However, 

```c
/**
 * struct spi_driver - Host side "protocol" driver
 * @id_table: List of SPI devices supported by this driver
 * @probe: Binds this driver to the spi device.  Drivers can verify
 *	that the device is actually present, and may need to configure
 *	characteristics (such as bits_per_word) which weren't needed for
 *	the initial configuration done during system setup.
 * @remove: Unbinds this driver from the spi device
 * @shutdown: Standard shutdown callback used during system state
 *	transitions such as powerdown/halt and kexec
 * @driver: SPI device drivers should initialize the name and owner
 *	field of this structure.
 *
 * This represents the kind of device driver that uses SPI messages to
 * interact with the hardware at the other end of a SPI link.  It's called
 * a "protocol" driver because it works through messages rather than talking
 * directly to SPI hardware (which is what the underlying SPI controller
 * driver does to pass those messages).  These protocols are defined in the
 * specification for the device(s) supported by the driver.
 *
 * As a rule, those device protocols represent the lowest level interface
 * supported by a driver, and it will support upper level interfaces too.
 * Examples of such upper levels include frameworks like MTD, networking,
 * MMC, RTC, filesystem character device nodes, and hardware monitoring.
 */
struct spi_driver {
	const struct spi_device_id *id_table;
	int			(*probe)(struct spi_device *spi);
	int			(*remove)(struct spi_device *spi);
	void			(*shutdown)(struct spi_device *spi);
	struct device_driver	driver;
};
```

## Understand `spi_device` structure

This is the main structure of a SPI slave device on bus. It contains configuration of the spi protocol. 

```c
/**
 * struct spi_device - Controller side proxy for an SPI slave device
 * @dev: Driver model representation of the device.
 * @controller: SPI controller used with the device.
 * @master: Copy of controller, for backwards compatibility.
 * @max_speed_hz: Maximum clock rate to be used with this chip
 *	(on this board); may be changed by the device's driver.
 *	The spi_transfer.speed_hz can override this for each transfer.
 * @chip_select: Chipselect, distinguishing chips handled by @controller.
 * @mode: The spi mode defines how data is clocked out and in.
 *	This may be changed by the device's driver.
 *	The "active low" default for chipselect mode can be overridden
 *	(by specifying SPI_CS_HIGH) as can the "MSB first" default for
 *	each word in a transfer (by specifying SPI_LSB_FIRST).
 * @bits_per_word: Data transfers involve one or more words; word sizes
 *	like eight or 12 bits are common.  In-memory wordsizes are
 *	powers of two bytes (e.g. 20 bit samples use 32 bits).
 *	This may be changed by the device's driver, or left at the
 *	default (0) indicating protocol words are eight bit bytes.
 *	The spi_transfer.bits_per_word can override this for each transfer.
 * @rt: Make the pump thread real time priority.
 * @irq: Negative, or the number passed to request_irq() to receive
 *	interrupts from this device.
 * @controller_state: Controller's runtime state
 * @controller_data: Board-specific definitions for controller, such as
 *	FIFO initialization parameters; from board_info.controller_data
 * @modalias: Name of the driver to use with this device, or an alias
 *	for that name.  This appears in the sysfs "modalias" attribute
 *	for driver coldplugging, and in uevents used for hotplugging
 * @driver_override: If the name of a driver is written to this attribute, then
 *	the device will bind to the named driver and only the named driver.
 * @cs_gpio: LEGACY: gpio number of the chipselect line (optional, -ENOENT when
 *	not using a GPIO line) use cs_gpiod in new drivers by opting in on
 *	the spi_master.
 * @cs_gpiod: gpio descriptor of the chipselect line (optional, NULL when
 *	not using a GPIO line)
 * @word_delay: delay to be inserted between consecutive
 *	words of a transfer
 *
 * @statistics: statistics for the spi_device
 *
 * A @spi_device is used to interchange data between an SPI slave
 * (usually a discrete chip) and CPU memory.
 *
 * In @dev, the platform_data is used to hold information about this
 * device that's meaningful to the device's protocol driver, but not
 * to its controller.  One example might be an identifier for a chip
 * variant with slightly different functionality; another might be
 * information about how this particular board wires the chip's pins.
 */
struct spi_device {
	struct device		dev;
	struct spi_controller	*controller;
	struct spi_controller	*master;	/* compatibility layer */
	u32			max_speed_hz;
	u8			chip_select;
	u8			bits_per_word;
	bool			rt;
#define SPI_NO_TX	BIT(31)		/* no transmit wire */
#define SPI_NO_RX	BIT(30)		/* no receive wire */
	/*
	 * All bits defined above should be covered by SPI_MODE_KERNEL_MASK.
	 * The SPI_MODE_KERNEL_MASK has the SPI_MODE_USER_MASK counterpart,
	 * which is defined in 'include/uapi/linux/spi/spi.h'.
	 * The bits defined here are from bit 31 downwards, while in
	 * SPI_MODE_USER_MASK are from 0 upwards.
	 * These bits must not overlap. A static assert check should make sure of that.
	 * If adding extra bits, make sure to decrease the bit index below as well.
	 */
#define SPI_MODE_KERNEL_MASK	(~(BIT(30) - 1))
	u32			mode;
	int			irq;
	void			*controller_state;
	void			*controller_data;
	char			modalias[SPI_NAME_SIZE];
	const char		*driver_override;
	int			cs_gpio;	/* LEGACY: chip select gpio */
	struct gpio_desc	*cs_gpiod;	/* chip select gpio desc */
	struct spi_delay	word_delay; /* inter-word delay */

	/* the statistics */
	struct spi_statistics	statistics;

	/*
	 * likely need more hooks for more protocol options affecting how
	 * the controller talks to each chip, like:
	 *  - memory packing (12 bit samples into low bits, others zeroed)
	 *  - priority
	 *  - chipselect delays
	 *  - ...
	 */
};
```

## Understand `spi_controller` structure

This is the Linux side SPI controller structure. Sometimes it is called "master" SPI node. Each SPI controller can communicate with one or more spi_device children. These make a small bus, sharing MOSI, MISO and SCK signals but not chip select signals. Each device may be configured to use a different clock rate, since those shared signals are ignored unless the chip is selected.

```c
/**
 * struct spi_controller - interface to SPI master or slave controller
 * @dev: device interface to this driver
 * @list: link with the global spi_controller list
 * @bus_num: board-specific (and often SOC-specific) identifier for a
 *	given SPI controller.
 * @num_chipselect: chipselects are used to distinguish individual
 *	SPI slaves, and are numbered from zero to num_chipselects.
 *	each slave has a chipselect signal, but it's common that not
 *	every chipselect is connected to a slave.
 * @dma_alignment: SPI controller constraint on DMA buffers alignment.
 * @mode_bits: flags understood by this controller driver
 * @buswidth_override_bits: flags to override for this controller driver
 * @bits_per_word_mask: A mask indicating which values of bits_per_word are
 *	supported by the driver. Bit n indicates that a bits_per_word n+1 is
 *	supported. If set, the SPI core will reject any transfer with an
 *	unsupported bits_per_word. If not set, this value is simply ignored,
 *	and it's up to the individual driver to perform any validation.
 * @min_speed_hz: Lowest supported transfer speed
 * @max_speed_hz: Highest supported transfer speed
 * @flags: other constraints relevant to this driver
 * @slave: indicates that this is an SPI slave controller
 * @max_transfer_size: function that returns the max transfer size for
 *	a &spi_device; may be %NULL, so the default %SIZE_MAX will be used.
 * @max_message_size: function that returns the max message size for
 *	a &spi_device; may be %NULL, so the default %SIZE_MAX will be used.
 * @io_mutex: mutex for physical bus access
 * @bus_lock_spinlock: spinlock for SPI bus locking
 * @bus_lock_mutex: mutex for exclusion of multiple callers
 * @bus_lock_flag: indicates that the SPI bus is locked for exclusive use
 * @setup: updates the device mode and clocking records used by a
 *	device's SPI controller; protocol code may call this.  This
 *	must fail if an unrecognized or unsupported mode is requested.
 *	It's always safe to call this unless transfers are pending on
 *	the device whose settings are being modified.
 * @set_cs_timing: optional hook for SPI devices to request SPI master
 * controller for configuring specific CS setup time, hold time and inactive
 * delay interms of clock counts
 * @transfer: adds a message to the controller's transfer queue.
 * @cleanup: frees controller-specific state
 * @can_dma: determine whether this controller supports DMA
 * @queued: whether this controller is providing an internal message queue
 * @kworker: pointer to thread struct for message pump
 * @pump_messages: work struct for scheduling work to the message pump
 * @queue_lock: spinlock to syncronise access to message queue
 * @queue: message queue
 * @idling: the device is entering idle state
 * @cur_msg: the currently in-flight message
 * @cur_msg_prepared: spi_prepare_message was called for the currently
 *                    in-flight message
 * @cur_msg_mapped: message has been mapped for DMA
 * @last_cs_enable: was enable true on the last call to set_cs.
 * @last_cs_mode_high: was (mode & SPI_CS_HIGH) true on the last call to set_cs.
 * @xfer_completion: used by core transfer_one_message()
 * @busy: message pump is busy
 * @running: message pump is running
 * @rt: whether this queue is set to run as a realtime task
 * @auto_runtime_pm: the core should ensure a runtime PM reference is held
 *                   while the hardware is prepared, using the parent
 *                   device for the spidev
 * @max_dma_len: Maximum length of a DMA transfer for the device.
 * @prepare_transfer_hardware: a message will soon arrive from the queue
 *	so the subsystem requests the driver to prepare the transfer hardware
 *	by issuing this call
 * @transfer_one_message: the subsystem calls the driver to transfer a single
 *	message while queuing transfers that arrive in the meantime. When the
 *	driver is finished with this message, it must call
 *	spi_finalize_current_message() so the subsystem can issue the next
 *	message
 * @unprepare_transfer_hardware: there are currently no more messages on the
 *	queue so the subsystem notifies the driver that it may relax the
 *	hardware by issuing this call
 *
 * @set_cs: set the logic level of the chip select line.  May be called
 *          from interrupt context.
 * @prepare_message: set up the controller to transfer a single message,
 *                   for example doing DMA mapping.  Called from threaded
 *                   context.
 * @transfer_one: transfer a single spi_transfer.
 *
 *                  - return 0 if the transfer is finished,
 *                  - return 1 if the transfer is still in progress. When
 *                    the driver is finished with this transfer it must
 *                    call spi_finalize_current_transfer() so the subsystem
 *                    can issue the next transfer. Note: transfer_one and
 *                    transfer_one_message are mutually exclusive; when both
 *                    are set, the generic subsystem does not call your
 *                    transfer_one callback.
 * @handle_err: the subsystem calls the driver to handle an error that occurs
 *		in the generic implementation of transfer_one_message().
 * @mem_ops: optimized/dedicated operations for interactions with SPI memory.
 *	     This field is optional and should only be implemented if the
 *	     controller has native support for memory like operations.
 * @unprepare_message: undo any work done by prepare_message().
 * @slave_abort: abort the ongoing transfer request on an SPI slave controller
 * @cs_setup: delay to be introduced by the controller after CS is asserted
 * @cs_hold: delay to be introduced by the controller before CS is deasserted
 * @cs_inactive: delay to be introduced by the controller after CS is
 *	deasserted. If @cs_change_delay is used from @spi_transfer, then the
 *	two delays will be added up.
 * @cs_gpios: LEGACY: array of GPIO descs to use as chip select lines; one per
 *	CS number. Any individual value may be -ENOENT for CS lines that
 *	are not GPIOs (driven by the SPI controller itself). Use the cs_gpiods
 *	in new drivers.
 * @cs_gpiods: Array of GPIO descs to use as chip select lines; one per CS
 *	number. Any individual value may be NULL for CS lines that
 *	are not GPIOs (driven by the SPI controller itself).
 * @use_gpio_descriptors: Turns on the code in the SPI core to parse and grab
 *	GPIO descriptors rather than using global GPIO numbers grabbed by the
 *	driver. This will fill in @cs_gpiods and @cs_gpios should not be used,
 *	and SPI devices will have the cs_gpiod assigned rather than cs_gpio.
 * @unused_native_cs: When cs_gpiods is used, spi_register_controller() will
 *	fill in this field with the first unused native CS, to be used by SPI
 *	controller drivers that need to drive a native CS when using GPIO CS.
 * @max_native_cs: When cs_gpiods is used, and this field is filled in,
 *	spi_register_controller() will validate all native CS (including the
 *	unused native CS) against this value.
 * @statistics: statistics for the spi_controller
 * @dma_tx: DMA transmit channel
 * @dma_rx: DMA receive channel
 * @dummy_rx: dummy receive buffer for full-duplex devices
 * @dummy_tx: dummy transmit buffer for full-duplex devices
 * @fw_translate_cs: If the boot firmware uses different numbering scheme
 *	what Linux expects, this optional hook can be used to translate
 *	between the two.
 * @ptp_sts_supported: If the driver sets this to true, it must provide a
 *	time snapshot in @spi_transfer->ptp_sts as close as possible to the
 *	moment in time when @spi_transfer->ptp_sts_word_pre and
 *	@spi_transfer->ptp_sts_word_post were transmitted.
 *	If the driver does not set this, the SPI core takes the snapshot as
 *	close to the driver hand-over as possible.
 * @irq_flags: Interrupt enable state during PTP system timestamping
 * @fallback: fallback to pio if dma transfer return failure with
 *	SPI_TRANS_FAIL_NO_START.
 *
 * Each SPI controller can communicate with one or more @spi_device
 * children.  These make a small bus, sharing MOSI, MISO and SCK signals
 * but not chip select signals.  Each device may be configured to use a
 * different clock rate, since those shared signals are ignored unless
 * the chip is selected.
 *
 * The driver for an SPI controller manages access to those devices through
 * a queue of spi_message transactions, copying data between CPU memory and
 * an SPI slave device.  For each such message it queues, it calls the
 * message's completion function when the transaction completes.
 */
struct spi_controller {
	struct device	dev;

	struct list_head list;

	/* other than negative (== assign one dynamically), bus_num is fully
	 * board-specific.  usually that simplifies to being SOC-specific.
	 * example:  one SOC has three SPI controllers, numbered 0..2,
	 * and one board's schematics might show it using SPI-2.  software
	 * would normally use bus_num=2 for that controller.
	 */
	s16			bus_num;

	/* chipselects will be integral to many controllers; some others
	 * might use board-specific GPIOs.
	 */
	u16			num_chipselect;

	/* some SPI controllers pose alignment requirements on DMAable
	 * buffers; let protocol drivers know about these requirements.
	 */
	u16			dma_alignment;

	/* spi_device.mode flags understood by this controller driver */
	u32			mode_bits;

	/* spi_device.mode flags override flags for this controller */
	u32			buswidth_override_bits;

	/* bitmask of supported bits_per_word for transfers */
	u32			bits_per_word_mask;
#define SPI_BPW_MASK(bits) BIT((bits) - 1)
#define SPI_BPW_RANGE_MASK(min, max) GENMASK((max) - 1, (min) - 1)

	/* limits on transfer speed */
	u32			min_speed_hz;
	u32			max_speed_hz;

	/* other constraints relevant to this driver */
	u16			flags;
#define SPI_CONTROLLER_HALF_DUPLEX	BIT(0)	/* can't do full duplex */
#define SPI_CONTROLLER_NO_RX		BIT(1)	/* can't do buffer read */
#define SPI_CONTROLLER_NO_TX		BIT(2)	/* can't do buffer write */
#define SPI_CONTROLLER_MUST_RX		BIT(3)	/* requires rx */
#define SPI_CONTROLLER_MUST_TX		BIT(4)	/* requires tx */

#define SPI_MASTER_GPIO_SS		BIT(5)	/* GPIO CS must select slave */

	/* flag indicating this is a non-devres managed controller */
	bool			devm_allocated;

	/* flag indicating this is an SPI slave controller */
	bool			slave;

	/*
	 * on some hardware transfer / message size may be constrained
	 * the limit may depend on device transfer settings
	 */
	size_t (*max_transfer_size)(struct spi_device *spi);
	size_t (*max_message_size)(struct spi_device *spi);

	/* I/O mutex */
	struct mutex		io_mutex;

	/* lock and mutex for SPI bus locking */
	spinlock_t		bus_lock_spinlock;
	struct mutex		bus_lock_mutex;

	/* flag indicating that the SPI bus is locked for exclusive use */
	bool			bus_lock_flag;

	/* Setup mode and clock, etc (spi driver may call many times).
	 *
	 * IMPORTANT:  this may be called when transfers to another
	 * device are active.  DO NOT UPDATE SHARED REGISTERS in ways
	 * which could break those transfers.
	 */
	int			(*setup)(struct spi_device *spi);

	/*
	 * set_cs_timing() method is for SPI controllers that supports
	 * configuring CS timing.
	 *
	 * This hook allows SPI client drivers to request SPI controllers
	 * to configure specific CS timing through spi_set_cs_timing() after
	 * spi_setup().
	 */
	int (*set_cs_timing)(struct spi_device *spi, struct spi_delay *setup,
			     struct spi_delay *hold, struct spi_delay *inactive);

	/* bidirectional bulk transfers
	 *
	 * + The transfer() method may not sleep; its main role is
	 *   just to add the message to the queue.
	 * + For now there's no remove-from-queue operation, or
	 *   any other request management
	 * + To a given spi_device, message queueing is pure fifo
	 *
	 * + The controller's main job is to process its message queue,
	 *   selecting a chip (for masters), then transferring data
	 * + If there are multiple spi_device children, the i/o queue
	 *   arbitration algorithm is unspecified (round robin, fifo,
	 *   priority, reservations, preemption, etc)
	 *
	 * + Chipselect stays active during the entire message
	 *   (unless modified by spi_transfer.cs_change != 0).
	 * + The message transfers use clock and SPI mode parameters
	 *   previously established by setup() for this device
	 */
	int			(*transfer)(struct spi_device *spi,
						struct spi_message *mesg);

	/* called on release() to free memory provided by spi_controller */
	void			(*cleanup)(struct spi_device *spi);

	/*
	 * Used to enable core support for DMA handling, if can_dma()
	 * exists and returns true then the transfer will be mapped
	 * prior to transfer_one() being called.  The driver should
	 * not modify or store xfer and dma_tx and dma_rx must be set
	 * while the device is prepared.
	 */
	bool			(*can_dma)(struct spi_controller *ctlr,
					   struct spi_device *spi,
					   struct spi_transfer *xfer);

	/*
	 * These hooks are for drivers that want to use the generic
	 * controller transfer queueing mechanism. If these are used, the
	 * transfer() function above must NOT be specified by the driver.
	 * Over time we expect SPI drivers to be phased over to this API.
	 */
	bool				queued;
	struct kthread_worker		*kworker;
	struct kthread_work		pump_messages;
	spinlock_t			queue_lock;
	struct list_head		queue;
	struct spi_message		*cur_msg;
	bool				idling;
	bool				busy;
	bool				running;
	bool				rt;
	bool				auto_runtime_pm;
	bool                            cur_msg_prepared;
	bool				cur_msg_mapped;
	bool				last_cs_enable;
	bool				last_cs_mode_high;
	bool                            fallback;
	struct completion               xfer_completion;
	size_t				max_dma_len;

	int (*prepare_transfer_hardware)(struct spi_controller *ctlr);
	int (*transfer_one_message)(struct spi_controller *ctlr,
				    struct spi_message *mesg);
	int (*unprepare_transfer_hardware)(struct spi_controller *ctlr);
	int (*prepare_message)(struct spi_controller *ctlr,
			       struct spi_message *message);
	int (*unprepare_message)(struct spi_controller *ctlr,
				 struct spi_message *message);
	int (*slave_abort)(struct spi_controller *ctlr);

	/*
	 * These hooks are for drivers that use a generic implementation
	 * of transfer_one_message() provided by the core.
	 */
	void (*set_cs)(struct spi_device *spi, bool enable);
	int (*transfer_one)(struct spi_controller *ctlr, struct spi_device *spi,
			    struct spi_transfer *transfer);
	void (*handle_err)(struct spi_controller *ctlr,
			   struct spi_message *message);

	/* Optimized handlers for SPI memory-like operations. */
	const struct spi_controller_mem_ops *mem_ops;

	/* CS delays */
	struct spi_delay	cs_setup;
	struct spi_delay	cs_hold;
	struct spi_delay	cs_inactive;

	/* gpio chip select */
	int			*cs_gpios;
	struct gpio_desc	**cs_gpiods;
	bool			use_gpio_descriptors;
	u8			unused_native_cs;
	u8			max_native_cs;

	/* statistics */
	struct spi_statistics	statistics;

	/* DMA channels for use with core dmaengine helpers */
	struct dma_chan		*dma_tx;
	struct dma_chan		*dma_rx;

	/* dummy data for full duplex devices */
	void			*dummy_rx;
	void			*dummy_tx;

	int (*fw_translate_cs)(struct spi_controller *ctlr, unsigned cs);

	/*
	 * Driver sets this field to indicate it is able to snapshot SPI
	 * transfers (needed e.g. for reading the time of POSIX clocks)
	 */
	bool			ptp_sts_supported;

	/* Interrupt enable state during PTP system timestamping */
	unsigned long		irq_flags;
};
```

How do we 

## Reference list:

1. https://www.kernel.org/doc/Documentation/spi/spi-summary
2. 
