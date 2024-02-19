/*
 * In this driver, we do these things:
 * 	 1. implment io cycle r/w and export their api to
 * 		replace inb/outb in asmio.h;
 * 	 2. implement serial irq.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/irqchip/chained_irq.h>

#include <asm/io.h>

#if 1
#define LPC_DBG(msg, arg...)	printk(KERN_DEBUG	"LPC: " msg, ## arg)
#else
#define LPC_DBG(msg, arg...)
#endif

#define LPC_INFO(msg, arg...)	printk(KERN_INFO	"LPC: " msg, ## arg)
#define LPC_WARN(msg, arg...)	printk(KERN_WARNING	"LPC: " msg, ## arg)
#define LPC_ERR(msg, arg...)	printk(KERN_ERR		"LPC: " msg, ## arg)

#define MASK(start, num) (((1 << num) - 1) << start)
#define REG(value, start, num) ((value & MASK(start, num)) >> start)

#define LPC_CLK	20000000

#define	SDVT_LPC_HOST_CONTROL			0x0
#define	SDVT_LPC_HOST_SERIRQ_CONTROL	0x4
#define SDVT_LPC_HOST_SERIRQ_IRQ_STAT	0x8
// #define SDVT_LPC_HOST_SERIRQ_IRQ_STAT	0x2C
#define	SDVT_LPC_HOST_COMMAND			0xC
#define	SDVT_LPC_HOST_IDSEL_MADDR		0x10
#define	SDVT_LPC_HOST_IRQ_ENABLE		0x28
#define	SDVT_LPC_HOST_IRQ_STATUS		0x2C
#define	SDVT_LPC_HOST_WR_FIFO			0x34
#define	SDVT_LPC_HOST_RD_FIFO			0x38

#define SDVT_LPC_HOST_CONTROL_RESET			0x4202020
#define SDVT_LPC_HOST_SERIRQ_CONTROL_RESET	0xFBE14

#define	SDVT_LPC_HOST_IO			0

#define	SDVT_LPC_HOST_READ			0
#define	SDVT_LPC_HOST_WRITE			1

#define SDVT_LPC_HOST_CONTINUOUS_MODE	0
#define SDVT_LPC_HOST_QUIET_MODE		1

#define SDVT_LPC_HOST_IRQ_SERIRQ_OFFSET 11
// #define SDVT_LPC_HOST_IRQ_SERIRQ_OFFSET 12
#define SERIRQ_NUMBER 16

struct lpc_data {
	struct platform_device *pdev;

	void __iomem *base;

	int parent_irq;
	struct irq_chip *chip;
	struct irq_domain *lpc_domain;
	spinlock_t lock;
};

static struct lpc_data *lpc_priv;

struct sdvt_lpc_host_config {
	unsigned char	enable;
	unsigned char	sync_timeout;
	unsigned char	prescaler;
	unsigned char	serirq_enable;
	unsigned char	serirq_mode;
};

struct sdvt_lpc_host_command {
	unsigned char	type;
	unsigned char	dir;
};

unsigned int read_reg(unsigned char offset) {
	unsigned int data = *(unsigned int *)(lpc_priv->base + offset);
	LPC_DBG("read 0x%x <<< 0x%lx", data, lpc_priv->base + offset);
	return data;
}

void write_reg(unsigned char offset, unsigned int data) {
	LPC_DBG("write 0x%x >>> 0x%lx", data, lpc_priv->base + offset);
	*(unsigned int *)(lpc_priv->base + offset) = data;
}

unsigned int read_command(void) {
	return read_reg(SDVT_LPC_HOST_COMMAND);
}

int is_command_pending(void) {
	return REG(read_command(), 0, 1);
}

int is_command_error(void) {
	return REG(read_command(), 7, 2);
}

int wait_command_done(void) {
	while (is_command_pending()) {
		if (is_command_error()) {
			LPC_ERR("send command error, reg value: 0x%x", read_command());
			return 1;
		}
	}
	return 0;
}

int send_command(struct sdvt_lpc_host_command *cmd, unsigned int offset) {
	write_reg(SDVT_LPC_HOST_IDSEL_MADDR, offset);

	unsigned int cmd_reg = 1;
	cmd_reg |= (cmd->type	<<  2);
	cmd_reg |= (cmd->dir	<<  6);

	write_reg(SDVT_LPC_HOST_COMMAND, cmd_reg);

	return wait_command_done();
}

void lpc_host_init(struct sdvt_lpc_host_config *config) {
	// init host control reg
	unsigned int data = SDVT_LPC_HOST_CONTROL_RESET;
	data |= (config->enable);
	data &= ~MASK(1, 1);
	data |= (config->sync_timeout	<< 1);
	data &= ~MASK(3, 8);
	data |= (config->prescaler		<< 3);
	write_reg(SDVT_LPC_HOST_CONTROL, data);
	LPC_DBG("control reg: 0x%x", read_reg(SDVT_LPC_HOST_CONTROL));

	// init serirq control reg
	data = SDVT_LPC_HOST_SERIRQ_CONTROL_RESET;
	data |= (config->serirq_enable);
	data &= ~MASK(1, 1);
	data |= (config->serirq_mode << 1);
	write_reg(SDVT_LPC_HOST_SERIRQ_CONTROL, data);
	write_reg(SDVT_LPC_HOST_SERIRQ_IRQ_STAT, 0);
	LPC_DBG("serirq control reg: 0x%x", read_reg(SDVT_LPC_HOST_SERIRQ_CONTROL));

	// init irq control reg - only unmask serirq
	write_reg(SDVT_LPC_HOST_IRQ_STATUS, 0xffffffff);
	write_reg(SDVT_LPC_HOST_IRQ_ENABLE, BIT(SDVT_LPC_HOST_IRQ_SERIRQ_OFFSET));
}

unsigned char lpc_read(unsigned char cycle_type, unsigned int port) {
	struct sdvt_lpc_host_command command = {
		.type	= cycle_type,
		.dir	= SDVT_LPC_HOST_READ,
	};

	send_command(&command, port);

	return REG(read_reg(SDVT_LPC_HOST_RD_FIFO), 24, 8);
}

int lpc_write(unsigned char cycle_type, unsigned char data, unsigned long port) {
	struct sdvt_lpc_host_command command = {
		.type	= cycle_type,
		.dir	= SDVT_LPC_HOST_WRITE,
	};

	write_reg(SDVT_LPC_HOST_WR_FIFO, data << 24);
	return send_command(&command, port);
}

unsigned char io_cycle_read(unsigned int port) {
	return lpc_read(SDVT_LPC_HOST_IO, port);
}

int io_cycle_write(unsigned char data, unsigned int port) {
	return lpc_write(SDVT_LPC_HOST_IO, data, port);
}

unsigned char lpc_inb(unsigned int port) {
	return io_cycle_read(port);
}

void lpc_outb(unsigned char b, unsigned int port) {
	io_cycle_write(b, port);
}

EXPORT_SYMBOL(lpc_inb);
EXPORT_SYMBOL(lpc_outb);

static void lpc_serirq_ack(struct irq_data *d)
{
	unsigned long flags;
	struct lpc_data *priv = d->domain->host_data;

	spin_lock_irqsave(&priv->lock, flags);

	LPC_DBG("%s, hwirq: %d, virq: %d", __func__, d->hwirq, d->irq);
	write_reg(SDVT_LPC_HOST_SERIRQ_IRQ_STAT, 0);
    write_reg(SDVT_LPC_HOST_IRQ_STATUS, BIT(SDVT_LPC_HOST_IRQ_SERIRQ_OFFSET));
	spin_unlock_irqrestore(&priv->lock, flags);
}

// static void lpc_serirq_mask(struct irq_data *d)
// {
// 	unsigned long flags;
// 	struct lpc_data *priv = d->domain->host_data;

// 	spin_lock_irqsave(&priv->lock, flags);

// 	LPC_DBG("%s: hwirq: %d, virq: %d", __func__, d->hwirq, d->irq);
// 	write_reg(SDVT_LPC_HOST_IRQ_ENABLE, 0);
// 	spin_unlock_irqrestore(&priv->lock, flags);
// }

// static void lpc_serirq_unmask(struct irq_data *d)
// {
// 	unsigned long flags;
// 	struct lpc_data *priv = d->domain->host_data;
// 	spin_lock_irqsave(&priv->lock, flags);

// 	write_reg(SDVT_LPC_HOST_IRQ_ENABLE, BIT(SDVT_LPC_HOST_IRQ_SERIRQ_OFFSET));
// 	LPC_DBG("%s: hwirq: %d, virq: %d", __func__, d->hwirq, d->irq);

// 	spin_unlock_irqrestore(&priv->lock, flags);
// }

static const struct irq_chip lpc_serirq_chip = {
	.name			= "lpc",
	.irq_ack		= lpc_serirq_ack,
	// .irq_mask		= lpc_serirq_mask,
	// .irq_unmask		= lpc_serirq_unmask,
};

static void lpc_serirq_dispatch(struct irq_desc *desc) {
	unsigned int pending, hwirq, ret;
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct lpc_data *priv = irq_desc_get_handler_data(desc);

	chained_irq_enter(chip, desc);

	LPC_DBG("%s, irqchip name: %s, irq: %d", __func__, chip->name, priv->parent_irq);

    pending = read_reg(SDVT_LPC_HOST_SERIRQ_IRQ_STAT) & 0xffff;

	if (!pending)
		LPC_ERR("get spurious interrupt");

	while (pending) {
		hwirq = __ffs(pending);
		pending &= ~BIT(hwirq);
		ret = generic_handle_domain_irq(priv->lpc_domain, hwirq);
		LPC_DBG("%s, handled hwirq: %d, irq %d, ret: %d, pending: %x", __func__, hwirq, irq_find_mapping(priv->lpc_domain, hwirq), ret, pending);
	}
	chained_irq_exit(chip, desc);
}

static int lpc_serirq_map(struct irq_domain *d, unsigned int irq, irq_hw_number_t hw) {
	struct lpc_data *priv = d->host_data;
	LPC_DBG("%s, mapping irq: %d", __func__, irq);
	irq_set_chip_and_handler(irq, priv->chip, handle_level_irq);
	return 0;
}

static const struct irq_domain_ops lpc_domain_ops = {
	.map		= lpc_serirq_map
};

static const struct of_device_id lpc_dt_table[] = {
	{ .compatible = "smart-dv,lpc" },
	{},
};

static int __init sdvt_lpc_probe(struct platform_device *pdev) {
	unsigned int clk_freq;
	unsigned char clk_prescaler;
	struct clk *lpc_clk;
	struct resource *res;
	struct lpc_data *data;
	struct fwnode_handle *fwnode = of_node_to_fwnode(pdev->dev.of_node);

	data = kzalloc(sizeof(struct lpc_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	lpc_priv = data;

	platform_set_drvdata(pdev, lpc_priv);
	lpc_priv->pdev = pdev;

	lpc_clk = devm_clk_get(&pdev->dev, "clk_gate_ahb_lpc");
	if (IS_ERR(lpc_clk))	{
		LPC_ERR("Cannot get lpc gating clock!\n");
		goto out;
	}
	clk_prepare_enable(lpc_clk);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	LPC_DBG("res start: 0x%lx, size: 0x%x", res->start, resource_size(res));
	lpc_priv->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(lpc_priv->base)) {
		LPC_ERR("failed to map registers\n");
		goto out;
	}
	LPC_DBG("virt addr: 0x%lx", lpc_priv->base);

	device_property_read_u32(&pdev->dev, "clock-frequency", &clk_freq);
	clk_prescaler = clk_freq / (LPC_CLK * 2);
	LPC_INFO("src clk: %d, lpc clk prescaler: %d", clk_freq, clk_prescaler);
	struct sdvt_lpc_host_config cfg = {
		.enable			= 1,
		.sync_timeout	= 1,
		.prescaler		= clk_prescaler,
		.serirq_enable	= 1,
		.serirq_mode	= SDVT_LPC_HOST_CONTINUOUS_MODE,
	};

	lpc_host_init(&cfg);

	lpc_priv->parent_irq = platform_get_irq_byname(pdev, "lpcirq");
	lpc_priv->chip = &lpc_serirq_chip;
	LPC_DBG("lpc intc irq: %d, irqchip name: %s", lpc_priv->parent_irq, lpc_priv->chip->name);

	// lpc_priv->lpc_domain = irq_domain_create_linear(fwnode, lpc_priv->irq_num, &lpc_domain_ops, lpc_priv);
	int irq_base = irq_alloc_descs(-1, 0, SERIRQ_NUMBER, 0);
	LPC_DBG("irq_base = %d", irq_base);
	lpc_priv->lpc_domain = irq_domain_add_legacy(pdev->dev.of_node, SERIRQ_NUMBER, irq_base, 0, &lpc_domain_ops, lpc_priv);
	if (!lpc_priv->lpc_domain) {
		LPC_ERR("create linear irq doamin failed");
		goto out;
	}
	LPC_DBG("lpc_domain name: %s", lpc_priv->lpc_domain->name);

	irq_set_chained_handler_and_data(lpc_priv->parent_irq, lpc_serirq_dispatch, lpc_priv);

	return 0;

out:
	if (lpc_priv->base)
		iounmap(lpc_priv->base);
	kfree(data);
	return -1;
}

static int __exit sdvt_lpc_remove(struct platform_device *pdev) {
	free_irq(lpc_priv->parent_irq, NULL);
	iounmap(lpc_priv->base);
	kfree(lpc_priv);
	return 0;
}

static struct platform_driver sdvt_lpc_driver = {
	.probe	= sdvt_lpc_probe,
	.remove	= sdvt_lpc_remove,
	.driver = {
		.name 			= "smart-dv,lpc",
		.of_match_table	= of_match_ptr(lpc_dt_table),
	},
};

static int __init sdvt_lpc_init(void) {
	return platform_driver_register(&sdvt_lpc_driver);
}

arch_initcall(sdvt_lpc_init);
