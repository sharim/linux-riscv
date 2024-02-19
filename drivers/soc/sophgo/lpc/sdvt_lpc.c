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

#include <asm/io.h>

#if 0
#define LPC_DBG(msg, arg...) printk(KERN_DEBUG "LPC: " msg, ## arg)
#else
#define LPC_DBG(msg, arg...)
#endif

#define LPC_INFO(msg, arg...)	printk(KERN_INFO "LPC: " msg, ## arg)
#define LPC_WARN(msg, arg...)	printk(KERN_WARNING "LPC: " msg, ## arg)
#define LPC_ERR(msg, arg...)	printk(KERN_ERR "LPC: " msg, ## arg)

#define MASK(start, num) (((1 << num) - 1) << start)
#define REG(value, start, num) ((value & MASK(start, num)) >> start)

#define LPC_CLK	20000000

#define	SDVT_LPC_HOST_CONTROL			0x0
#define	SDVT_LPC_HOST_SERIRQ_CONTROL	0x0
#define	SDVT_LPC_HOST_COMMAND			0xC
#define	SDVT_LPC_HOST_IDSEL_MADDR		0x10
#define	SDVT_LPC_HOST_IRQ_ENABLE		0x28
#define	SDVT_LPC_HOST_WR_FIFO			0x34
#define	SDVT_LPC_HOST_RD_FIFO			0x38

#define SDVT_LPC_HOST_CONTROL_RESET			0x4202020
#define SDVT_LPC_HOST_SERIRQ_CONTROL_RESET	0xFBE14

#define	SDVT_LPC_HOST_IO			0

#define	SDVT_LPC_HOST_READ			0
#define	SDVT_LPC_HOST_WRITE			1

#define SDVT_LPC_HOST_CONTINUOUS_MODE	0
#define SDVT_LPC_HOST_QUIET_MODE		1


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

void __iomem *lpc_reg_addr __read_mostly;

unsigned int read_reg(unsigned char offset) {
	return (*(unsigned int *)(lpc_reg_addr + offset));
}

void write_reg(unsigned char offset, unsigned int data) {
	*(unsigned int *)(lpc_reg_addr + offset) = data;
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

	// init serirq control reg
	data = SDVT_LPC_HOST_SERIRQ_CONTROL_RESET;
	data |= (config->serirq_enable);
	data &= ~MASK(1, 1);
	data |= (config->serirq_mode 	<< 1);
	write_reg(SDVT_LPC_HOST_IRQ_ENABLE, data);
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

static const struct of_device_id lpc_dt_table[] = {
	{ .compatible = "smart-dv,lpc" },
	{},
};

static int __init sdvt_lpc_probe(struct platform_device *pdev) {
	struct resource *res;
	struct clk *lpc_clk;
	unsigned int clk_freq;
	unsigned char clk_prescaler;

	lpc_clk = devm_clk_get(&pdev->dev, "clk_gate_ahb_lpc");
	if (IS_ERR(lpc_clk))	{
		LPC_ERR("Cannot get lpc gating clock!\n");
		return PTR_ERR(lpc_clk);
	}
	clk_prepare_enable(lpc_clk);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	lpc_reg_addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(lpc_reg_addr)) {
		LPC_ERR("failed map registers\n");
		goto out;
	}

	device_property_read_u32(&pdev->dev, "clock-frequency", &clk_freq);
	clk_prescaler = clk_freq / (LPC_CLK * 2);
	LPC_DBG("src clk is %d, lpc clk prescaler is", clk_freq, clk_prescaler);
	struct sdvt_lpc_host_config cfg = {
		.enable			= 1,
		.sync_timeout	= 1,
		.prescaler		= clk_prescaler,
		.serirq_enable	= 1,
		.serirq_mode	= SDVT_LPC_HOST_CONTINUOUS_MODE,
	};

	lpc_host_init(&cfg);
	return 0;

out:
	if (lpc_reg_addr)
		iounmap(lpc_reg_addr);
	return -1;
}

static struct platform_driver sdvt_lpc_driver = {
	.driver = {
		.name = "smart-dv,lpc",
		.of_match_table = of_match_ptr(lpc_dt_table),
	},
	.probe = sdvt_lpc_probe,
};

static int __init sdvt_lpc_init(void) {
	return platform_driver_register(&sdvt_lpc_driver);
}

arch_initcall(sdvt_lpc_init);