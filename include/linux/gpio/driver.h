/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __LINUX_GPIO_DRIVER_H
#define __LINUX_GPIO_DRIVER_H

#include <linux/device.h>
#include <linux/types.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/lockdep.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinconf-generic.h>

struct gpio_desc;
struct of_phandle_args;
struct device_node;
struct seq_file;
struct gpio_device;
struct module;

#ifdef CONFIG_GPIOLIB




/**
 * struct gpio_irq_chip - GPIO interrupt controller
 */
struct gpio_irq_chip {
	/**
	 * @chip:
	 *
	 * GPIO IRQ chip implementation, provided by GPIO driver.
	 */
	struct irq_chip *chip;

	/**
	 * @domain:
	 *
	 * Interrupt translation domain; responsible for mapping between GPIO
	 * hwirq number and Linux IRQ number.
	 */
	struct irq_domain *domain;

	/**
	 * @domain_ops:
	 *
	 * Table of interrupt domain operations for this IRQ chip.
	 */
	const struct irq_domain_ops *domain_ops;

#ifdef CONFIG_IRQ_DOMAIN_HIERARCHY
	/**
	 * @fwnode:
	 *
	 * Firmware node corresponding to this gpiochip/irqchip, necessary
	 * for hierarchical irqdomain support.
	 */
	struct fwnode_handle *fwnode;

	/**
	 * @parent_domain:
	 *
	 * If non-NULL, will be set as the parent of this GPIO interrupt
	 * controller's IRQ domain to establish a hierarchical interrupt
	 * domain. The presence of this will activate the hierarchical
	 * interrupt support.
	 */
	struct irq_domain *parent_domain;

	/**
	 * @child_to_parent_hwirq:
	 *
	 * This callback translates a child hardware IRQ offset to a parent
	 * hardware IRQ offset on a hierarchical interrupt chip. The child
	 * hardware IRQs correspond to the GPIO index 0..ngpio-1 (see the
	 * ngpio field of struct gpio_chip) and the corresponding parent
	 * hardware IRQ and type (such as IRQ_TYPE_*) shall be returned by
	 * the driver. The driver can calculate this from an offset or using
	 * a lookup table or whatever method is best for this chip. Return
	 * 0 on successful translation in the driver.
	 *
	 * If some ranges of hardware IRQs do not have a corresponding parent
	 * HWIRQ, return -EINVAL, but also make sure to fill in @valid_mask and
	 * @need_valid_mask to make these GPIO lines unavailable for
	 * translation.
	 */
	int (*child_to_parent_hwirq)(struct gpio_chip *gc,
				     unsigned int child_hwirq,
				     unsigned int child_type,
				     unsigned int *parent_hwirq,
				     unsigned int *parent_type);

	/**
	 * @populate_parent_alloc_arg :
	 *
	 * This optional callback allocates and populates the specific struct
	 * for the parent's IRQ domain. If this is not specified, then
	 * &gpiochip_populate_parent_fwspec_twocell will be used. A four-cell
	 * variant named &gpiochip_populate_parent_fwspec_fourcell is also
	 * available.
	 */
	void *(*populate_parent_alloc_arg)(struct gpio_chip *gc,
				       unsigned int parent_hwirq,
				       unsigned int parent_type);

	/**
	 * @child_offset_to_irq:
	 *
	 * This optional callback is used to translate the child's GPIO line
	 * offset on the GPIO chip to an IRQ number for the GPIO to_irq()
	 * callback. If this is not specified, then a default callback will be
	 * provided that returns the line offset.
	 */
	unsigned int (*child_offset_to_irq)(struct gpio_chip *gc,
					    unsigned int pin);

	/**
	 * @child_irq_domain_ops:
	 *
	 * The IRQ domain operations that will be used for this GPIO IRQ
	 * chip. If no operations are provided, then default callbacks will
	 * be populated to setup the IRQ hierarchy. Some drivers need to
	 * supply their own translate function.
	 */
	struct irq_domain_ops child_irq_domain_ops;
#endif

	/**
	 * @handler:
	 *
	 * The IRQ handler to use (often a predefined IRQ core function) for
	 * GPIO IRQs, provided by GPIO driver.
	 */
	irq_flow_handler_t handler;

	/**
	 * @default_type:
	 *
	 * Default IRQ triggering type applied during GPIO driver
	 * initialization, provided by GPIO driver.
	 */
	unsigned int default_type;

	/**
	 * @lock_key:
	 *
	 * Per GPIO IRQ chip lockdep class for IRQ lock.
	 */
	struct lock_class_key *lock_key;

	/**
	 * @request_key:
	 *
	 * Per GPIO IRQ chip lockdep class for IRQ request.
	 */
	struct lock_class_key *request_key;

	/**
	 * @parent_handler:
	 *
	 * The interrupt handler for the GPIO chip's parent interrupts, may be
	 * NULL if the parent interrupts are nested rather than cascaded.
	 */
	irq_flow_handler_t parent_handler;

	/**
	 * @parent_handler_data:
	 *
	 * Data associated, and passed to, the handler for the parent
	 * interrupt.
	 */
	void *parent_handler_data;

	/**
	 * @num_parents:
	 *
	 * The number of interrupt parents of a GPIO chip.
	 */
	unsigned int num_parents;

	/**
	 * @parents:
	 *
	 * A list of interrupt parents of a GPIO chip. This is owned by the
	 * driver, so the core will only reference this list, not modify it.
	 */
	unsigned int *parents;

	/**
	 * @map:
	 *
	 * A list of interrupt parents for each line of a GPIO chip.
	 */
	unsigned int *map;

	/**
	 * @threaded:
	 *
	 * True if set the interrupt handling uses nested threads.
	 */
	bool threaded;

	/**
	 * @init_hw: optional routine to initialize hardware before
	 * an IRQ chip will be added. This is quite useful when
	 * a particular driver wants to clear IRQ related registers
	 * in order to avoid undesired events.
	 */
	int (*init_hw)(struct gpio_chip *gc);

	/**
	 * @init_valid_mask: optional routine to initialize @valid_mask, to be
	 * used if not all GPIO lines are valid interrupts. Sometimes some
	 * lines just cannot fire interrupts, and this routine, when defined,
	 * is passed a bitmap in "valid_mask" and it will have ngpios
	 * bits from 0..(ngpios-1) set to "1" as in valid. The callback can
	 * then directly set some bits to "0" if they cannot be used for
	 * interrupts.
	 */
	void (*init_valid_mask)(struct gpio_chip *gc,
				unsigned long *valid_mask,
				unsigned int ngpios);

	/**
	 * @valid_mask:
	 *
	 * If not %NULL holds bitmask of GPIOs which are valid to be included
	 * in IRQ domain of the chip.
	 */
	unsigned long *valid_mask;

	/**
	 * @first:
	 *
	 * Required for static IRQ allocation. If set, irq_domain_add_simple()
	 * will allocate and map all IRQs during initialization.
	 */
	unsigned int first;

	/**
	 * @irq_enable:
	 *
	 * Store old irq_chip irq_enable callback
	 */
	void		(*irq_enable)(struct irq_data *data);

	/**
	 * @irq_disable:
	 *
	 * Store old irq_chip irq_disable callback
	 */
	void		(*irq_disable)(struct irq_data *data);
	/**
	 * @irq_unmask:
	 *
	 * Store old irq_chip irq_unmask callback
	 */
	void		(*irq_unmask)(struct irq_data *data);

	/**
	 * @irq_mask:
	 *
	 * Store old irq_chip irq_mask callback
	 */
	void		(*irq_mask)(struct irq_data *data);
};


/**
 * struct gpio_chip - abstract a GPIO controller
 * @label: a functional name for the GPIO device, such as a part
 *	number or the name of the SoC IP-block implementing it.
 * @gpiodev: the internal state holder, opaque struct
 * @parent: optional parent device providing the GPIOs
 * @owner: helps prevent removal of modules exporting active GPIOs
 * @request: optional hook for chip-specific activation, such as
 *	enabling module power and clock; may sleep
 * @free: optional hook for chip-specific deactivation, such as
 *	disabling module power and clock; may sleep
 * @get_direction: returns direction for signal "offset", 0=out, 1=in,
 *	(same as GPIOF_DIR_XXX), or negative error
 * @direction_input: configures signal "offset" as input, or returns error
 * @direction_output: configures signal "offset" as output, or returns error
 * @get: returns value for signal "offset", 0=low, 1=high, or negative error
 * @set: assigns output value for signal "offset"
 * @set_multiple: assigns output values for multiple signals defined by "mask"
 * @set_config: optional hook for all kinds of settings. Uses the same
 *	packed config format as generic pinconf.
 * @to_irq: optional hook supporting non-static gpio_to_irq() mappings;
 *	implementation may not sleep
 * @dbg_show: optional routine to show contents in debugfs; default code
 *	will be used when this is omitted, but custom code can show extra
 *	state (such as pullup/pulldown configuration).
 * @base: identifies the first GPIO number handled by this chip;
 *	or, if negative during registration, requests dynamic ID allocation.
 *	DEPRECATION: providing anything non-negative and nailing the base
 *	offset of GPIO chips is deprecated. Please pass -1 as base to
 *	let gpiolib select the chip base in all possible cases. We want to
 *	get rid of the static GPIO number space in the long run.
 * @ngpio: the number of GPIOs handled by this controller; the last GPIO
 *	handled is (base + ngpio - 1).
 * @names: if set, must be an array of strings to use as alternative
 *      names for the GPIOs in this chip. Any entry in the array
 *      may be NULL if there is no alias for the GPIO, however the
 *      array must be @ngpio entries long.  A name can include a single printk
 *      format specifier for an unsigned int.  It is substituted by the actual
 *      number of the gpio.
 * @can_sleep: flag must be set iff get()/set() methods sleep, as they
 *	must while accessing GPIO expander chips over I2C or SPI. This
 *	implies that if the chip supports IRQs, these IRQs need to be threaded
 *	as the chip access may sleep when e.g. reading out the IRQ status
 *	registers.
 * @read_reg: reader function for generic GPIO
 * @write_reg: writer function for generic GPIO
 * @pin2mask: some generic GPIO controllers work with the big-endian bits
 *	notation, e.g. in a 8-bits register, GPIO7 is the least significant
 *	bit. This callback assigns the right bit mask.
 * @reg_dat: data (in) register for generic GPIO
 * @reg_set: output set register (out=high) for generic GPIO
 * @reg_clr: output clear register (out=low) for generic GPIO
 * @reg_dir: direction setting register for generic GPIO
 * @bgpio_bits: number of register bits used for a generic GPIO i.e.
 *	<register width> * 8
 * @bgpio_lock: used to lock chip->bgpio_data. Also, this is needed to keep
 *	shadowed and real data registers writes together.
 * @bgpio_data:	shadowed data register for generic GPIO to clear/set bits
 *	safely.
 * @bgpio_dir: shadowed direction register for generic GPIO to clear/set
 *	direction safely.
 * @irqchip: GPIO IRQ chip impl, provided by GPIO driver
 * @irqdomain: Interrupt translation domain; responsible for mapping
 *	between GPIO hwirq number and linux irq number
 * @irq_base: first linux IRQ number assigned to GPIO IRQ chip (deprecated)
 * @irq_handler: the irq handler to use (often a predefined irq core function)
 *	for GPIO IRQs, provided by GPIO driver
 * @irq_default_type: default IRQ triggering type applied during GPIO driver
 *	initialization, provided by GPIO driver
 * @irq_chained_parent: GPIO IRQ chip parent/bank linux irq number,
 *	provided by GPIO driver for chained interrupt (not for nested
 *	interrupts).
 * @irq_nested: True if set the interrupt handling is nested.
 * @irq_need_valid_mask: If set core allocates @irq_valid_mask with all
 *	bits set to one
 * @irq_valid_mask: If not %NULL holds bitmask of GPIOs which are valid to
 *	be included in IRQ domain of the chip
 * @lock_key: per GPIO IRQ chip lockdep class
 *
 * A gpio_chip can help platforms abstract various sources of GPIOs so
 * they can all be accessed through a common programing interface.
 * Example sources would be SOC controllers, FPGAs, multifunction
 * chips, dedicated GPIO expanders, and so on.
 *
 * Each chip controls a number of signals, identified in method calls
 * by "offset" values in the range 0..(@ngpio - 1).  When those signals
 * are referenced through calls like gpio_get_value(gpio), the offset
 * is calculated by subtracting @base from the gpio number.
 */
struct gpio_chip {
	const char		*label;
	struct gpio_device	*gpiodev;
	struct device		*parent;
	struct module		*owner;

	int			(*request)(struct gpio_chip *chip,
						unsigned offset);
	void			(*free)(struct gpio_chip *chip,
						unsigned offset);
	int			(*get_direction)(struct gpio_chip *chip,
						unsigned offset);
	int			(*direction_input)(struct gpio_chip *chip,
						unsigned offset);
	int			(*direction_output)(struct gpio_chip *chip,
						unsigned offset, int value);
	int			(*get)(struct gpio_chip *chip,
						unsigned offset);
	void			(*set)(struct gpio_chip *chip,
						unsigned offset, int value);
	void			(*set_multiple)(struct gpio_chip *chip,
						unsigned long *mask,
						unsigned long *bits);
	int			(*set_config)(struct gpio_chip *chip,
					      unsigned offset,
					      unsigned long config);
	int			(*to_irq)(struct gpio_chip *chip,
						unsigned offset);

	void			(*dbg_show)(struct seq_file *s,
						struct gpio_chip *chip);
	int			base;
	u16			ngpio;
	const char		*const *names;
	bool			can_sleep;

#if IS_ENABLED(CONFIG_GPIO_GENERIC)
	unsigned long (*read_reg)(void __iomem *reg);
	void (*write_reg)(void __iomem *reg, unsigned long data);
	unsigned long (*pin2mask)(struct gpio_chip *gc, unsigned int pin);
	void __iomem *reg_dat;
	void __iomem *reg_set;
	void __iomem *reg_clr;
	void __iomem *reg_dir;
	int bgpio_bits;
	ipipe_spinlock_t bgpio_lock;
	unsigned long bgpio_data;
	unsigned long bgpio_dir;
#endif

#ifdef CONFIG_GPIOLIB_IRQCHIP
	/*
	 * With CONFIG_GPIOLIB_IRQCHIP we get an irqchip inside the gpiolib
	 * to handle IRQs for most practical cases.
	 */
	struct gpio_irq_chip 	irq;
	struct irq_chip		*irqchip;
	struct irq_domain	*irqdomain;
	unsigned int		irq_base;
	irq_flow_handler_t	irq_handler;
	unsigned int		irq_default_type;
	unsigned int		irq_chained_parent;
	bool			irq_nested;
	bool			irq_need_valid_mask;
	unsigned long		*irq_valid_mask;
	struct lock_class_key	*lock_key;
#endif

#if defined(CONFIG_OF_GPIO)
	/*
	 * If CONFIG_OF is enabled, then all GPIO controllers described in the
	 * device tree automatically may have an OF translation
	 */

	/**
	 * @of_node:
	 *
	 * Pointer to a device tree node representing this GPIO controller.
	 */
	struct device_node *of_node;

	/**
	 * @of_gpio_n_cells:
	 *
	 * Number of cells used to form the GPIO specifier.
	 */
	unsigned int of_gpio_n_cells;

	/**
	 * @of_xlate:
	 *
	 * Callback to translate a device tree GPIO specifier into a chip-
	 * relative GPIO number and flags.
	 */
	int (*of_xlate)(struct gpio_chip *gc,
			const struct of_phandle_args *gpiospec, u32 *flags);
#endif
};

extern const char *gpiochip_is_requested(struct gpio_chip *chip,
			unsigned offset);

/* add/remove chips */
extern int gpiochip_add_data(struct gpio_chip *chip, void *data);
static inline int gpiochip_add(struct gpio_chip *chip)
{
	return gpiochip_add_data(chip, NULL);
}
extern void gpiochip_remove(struct gpio_chip *chip);
extern int devm_gpiochip_add_data(struct device *dev, struct gpio_chip *chip,
				  void *data);
extern void devm_gpiochip_remove(struct device *dev, struct gpio_chip *chip);

extern struct gpio_chip *gpiochip_find(void *data,
			      int (*match)(struct gpio_chip *chip, void *data));

/* lock/unlock as IRQ */
int gpiochip_lock_as_irq(struct gpio_chip *chip, unsigned int offset);
void gpiochip_unlock_as_irq(struct gpio_chip *chip, unsigned int offset);
bool gpiochip_line_is_irq(struct gpio_chip *chip, unsigned int offset);

/* Line status inquiry for drivers */
bool gpiochip_line_is_open_drain(struct gpio_chip *chip, unsigned int offset);
bool gpiochip_line_is_open_source(struct gpio_chip *chip, unsigned int offset);

/* Sleep persistence inquiry for drivers */
bool gpiochip_line_is_persistent(struct gpio_chip *chip, unsigned int offset);

/* get driver data */
void *gpiochip_get_data(struct gpio_chip *chip);

struct gpio_chip *gpiod_to_chip(const struct gpio_desc *desc);

struct bgpio_pdata {
	const char *label;
	int base;
	int ngpio;
};

#if IS_ENABLED(CONFIG_GPIO_GENERIC)

int bgpio_init(struct gpio_chip *gc, struct device *dev,
	       unsigned long sz, void __iomem *dat, void __iomem *set,
	       void __iomem *clr, void __iomem *dirout, void __iomem *dirin,
	       unsigned long flags);

#define BGPIOF_BIG_ENDIAN		BIT(0)
#define BGPIOF_UNREADABLE_REG_SET	BIT(1) /* reg_set is unreadable */
#define BGPIOF_UNREADABLE_REG_DIR	BIT(2) /* reg_dir is unreadable */
#define BGPIOF_BIG_ENDIAN_BYTE_ORDER	BIT(3)
#define BGPIOF_READ_OUTPUT_REG_SET	BIT(4) /* reg_set stores output value */
#define BGPIOF_NO_OUTPUT		BIT(5) /* only input */

#endif

#ifdef CONFIG_GPIOLIB_IRQCHIP

void gpiochip_set_chained_irqchip(struct gpio_chip *gpiochip,
		struct irq_chip *irqchip,
		unsigned int parent_irq,
		irq_flow_handler_t parent_handler);

void gpiochip_set_nested_irqchip(struct gpio_chip *gpiochip,
		struct irq_chip *irqchip,
		unsigned int parent_irq);

int gpiochip_irqchip_add_key(struct gpio_chip *gpiochip,
			     struct irq_chip *irqchip,
			     unsigned int first_irq,
			     irq_flow_handler_t handler,
			     unsigned int type,
			     bool nested,
			     struct lock_class_key *lock_key);

#ifdef CONFIG_LOCKDEP

/*
 * Lockdep requires that each irqchip instance be created with a
 * unique key so as to avoid unnecessary warnings. This upfront
 * boilerplate static inlines provides such a key for each
 * unique instance.
 */
static inline int gpiochip_irqchip_add(struct gpio_chip *gpiochip,
				       struct irq_chip *irqchip,
				       unsigned int first_irq,
				       irq_flow_handler_t handler,
				       unsigned int type)
{
	static struct lock_class_key key;

	return gpiochip_irqchip_add_key(gpiochip, irqchip, first_irq,
					handler, type, false, &key);
}

static inline int gpiochip_irqchip_add_nested(struct gpio_chip *gpiochip,
			  struct irq_chip *irqchip,
			  unsigned int first_irq,
			  irq_flow_handler_t handler,
			  unsigned int type)
{

	static struct lock_class_key key;

	return gpiochip_irqchip_add_key(gpiochip, irqchip, first_irq,
					handler, type, true, &key);
}
#else
static inline int gpiochip_irqchip_add(struct gpio_chip *gpiochip,
				       struct irq_chip *irqchip,
				       unsigned int first_irq,
				       irq_flow_handler_t handler,
				       unsigned int type)
{
	return gpiochip_irqchip_add_key(gpiochip, irqchip, first_irq,
					handler, type, false, NULL);
}

static inline int gpiochip_irqchip_add_nested(struct gpio_chip *gpiochip,
			  struct irq_chip *irqchip,
			  unsigned int first_irq,
			  irq_flow_handler_t handler,
			  unsigned int type)
{
	return gpiochip_irqchip_add_key(gpiochip, irqchip, first_irq,
					handler, type, true, NULL);
}
#endif /* CONFIG_LOCKDEP */

#endif /* CONFIG_GPIOLIB_IRQCHIP */

int gpiochip_generic_request(struct gpio_chip *chip, unsigned offset);
void gpiochip_generic_free(struct gpio_chip *chip, unsigned offset);
int gpiochip_generic_config(struct gpio_chip *chip, unsigned offset,
			    unsigned long config);

#ifdef CONFIG_PINCTRL

/**
 * struct gpio_pin_range - pin range controlled by a gpio chip
 * @node: list for maintaining set of pin ranges, used internally
 * @pctldev: pinctrl device which handles corresponding pins
 * @range: actual range of pins controlled by a gpio controller
 */
struct gpio_pin_range {
	struct list_head node;
	struct pinctrl_dev *pctldev;
	struct pinctrl_gpio_range range;
};

int gpiochip_add_pin_range(struct gpio_chip *chip, const char *pinctl_name,
			   unsigned int gpio_offset, unsigned int pin_offset,
			   unsigned int npins);
int gpiochip_add_pingroup_range(struct gpio_chip *chip,
			struct pinctrl_dev *pctldev,
			unsigned int gpio_offset, const char *pin_group);
void gpiochip_remove_pin_ranges(struct gpio_chip *chip);

#else

static inline int
gpiochip_add_pin_range(struct gpio_chip *chip, const char *pinctl_name,
		       unsigned int gpio_offset, unsigned int pin_offset,
		       unsigned int npins)
{
	return 0;
}
static inline int
gpiochip_add_pingroup_range(struct gpio_chip *chip,
			struct pinctrl_dev *pctldev,
			unsigned int gpio_offset, const char *pin_group)
{
	return 0;
}

static inline void
gpiochip_remove_pin_ranges(struct gpio_chip *chip)
{
}

#endif /* CONFIG_PINCTRL */

struct gpio_desc *gpiochip_request_own_desc(struct gpio_chip *chip, u16 hwnum,
					    const char *label);
void gpiochip_free_own_desc(struct gpio_desc *desc);

#else /* CONFIG_GPIOLIB */

static inline struct gpio_chip *gpiod_to_chip(const struct gpio_desc *desc)
{
	/* GPIO can never have been requested */
	WARN_ON(1);
	return ERR_PTR(-ENODEV);
}

#endif /* CONFIG_GPIOLIB */

#endif
