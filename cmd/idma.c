// cmd/idma.c

#include <command.h>
#include <cpu_func.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <vsprintf.h>
#include <malloc.h>
#include <linux/dma-mapping.h>

#define IDMA_CONF              0x000
#define IDMA_STATUS_0          0x004
#define IDMA_NEXT_ID_0         0x044
#define IDMA_DONE_ID_0         0x084

#define IDMA_DST_ADDR_LOW      0x0d0
#define IDMA_DST_ADDR_HIGH     0x0d4
#define IDMA_SRC_ADDR_LOW      0x0d8
#define IDMA_SRC_ADDR_HIGH     0x0dc
#define IDMA_LENGTH_LOW        0x0e0
#define IDMA_LENGTH_HIGH       0x0e4


#define IDMA_IRQ_STATUS        0x1000
#define IDMA_IRQ_ENABLE        0x1004


#define IDMA_TIMEOUT_US        1000000

static inline unsigned long align_down_cache(unsigned long addr)
{
	return addr & ~(ARCH_DMA_MINALIGN - 1);
}

static inline unsigned long align_up_cache(unsigned long addr)
{
	return (addr + ARCH_DMA_MINALIGN - 1) & ~(ARCH_DMA_MINALIGN - 1);
}

static void idma_write64(void __iomem *base, u32 low_off, u32 high_off, u64 val)
{
	writel((u32)val, base + low_off);
	writel((u32)(val >> 32), base + high_off);
}

static int idma_run_copy(ulong base_addr, ulong src_addr, ulong dst_addr, ulong len)
{
    //ulong src_addr, dst_addr;
    void __iomem *base;
	u8 *src, *dst;
	u32 tid, done;
	ulong start, end;
	int i;
	ulong timeout;



	base = (void __iomem *)base_addr;
	src = (u8 *)src_addr;
	dst = (u8 *)dst_addr;

	printf("iDMA test: base=%pa src=%pa dst=%pa len=0x%lx\n",
	       &base_addr, &src_addr, &dst_addr, len);

	for (i = 0; i < len; i++) {
		src[i] = (u8)(i ^ 0xa5);
		dst[i] = 0x5a;
	}

	/*
	 * Non-coherent DMA:
	 * - source must be written back before DMA reads it
	 * - destination dirty lines must not later overwrite DMA output
	 */
	start = align_down_cache(src_addr);
	end   = align_up_cache(src_addr + len);
	flush_dcache_range(start, end);

	start = align_down_cache(dst_addr);
	end   = align_up_cache(dst_addr + len);
	flush_dcache_range(start, end);
	invalidate_dcache_range(start, end);

	/* conservative config first */
	writel(0x00000000, base + IDMA_CONF);

	idma_write64(base, IDMA_SRC_ADDR_LOW, IDMA_SRC_ADDR_HIGH, src_addr);
	idma_write64(base, IDMA_DST_ADDR_LOW, IDMA_DST_ADDR_HIGH, dst_addr);
	idma_write64(base, IDMA_LENGTH_LOW,   IDMA_LENGTH_HIGH,   len);

	/*
	 * iDMA register frontend launch:
	 * reading NEXT_ID launches the transfer.
	 */
	tid = readl(base + IDMA_NEXT_ID_0);
	if (!tid) {
		printf("iDMA: launch failed, next_id returned 0\n");
		return CMD_RET_FAILURE;
	}

	printf("iDMA: launched transfer id %u\n", tid);

	timeout = IDMA_TIMEOUT_US;
	while (timeout--) {
		done = readl(base + IDMA_DONE_ID_0);
		if (done >= tid)
			break;
		udelay(1);
	}

	if (!timeout) {
		printf("iDMA: timeout, status=0x%08x done_id=%u expected=%u\n",
		       readl(base + IDMA_STATUS_0), done, tid);
		return CMD_RET_FAILURE;
	}

	start = align_down_cache(dst_addr);
	end   = align_up_cache(dst_addr + len);
	invalidate_dcache_range(start, end);

	for (i = 0; i < len; i++) {
		if (src[i] != dst[i]) {
			printf("iDMA: mismatch at +0x%x: src=0x%02x dst=0x%02x\n",
			       i, src[i], dst[i]);
			return CMD_RET_FAILURE;
		}
	}

    return CMD_RET_SUCCESS;

}


static void idma_enable_interrupts(ulong base_addr, int enable)
{
    void __iomem *base;
    u32 irq_status;


	base = (void __iomem *)base_addr;

    writel(0x00000001, base + IDMA_IRQ_STATUS); // clear
    if (enable)
        writel(0x00000001, base + IDMA_IRQ_ENABLE);
    else
        writel(0x00000000, base + IDMA_IRQ_ENABLE);
}

static int idma_irqcheck_and_clear(ulong base_addr)
{
    //ulong src_addr, dst_addr;
    void __iomem *base;
    u32 irq_status;

    base = (void __iomem *)base_addr;

    irq_status = readl(base + IDMA_IRQ_STATUS);
    if (irq_status != 0x1) {
        printf("iDMA: IRQ not detected\n");
        return CMD_RET_FAILURE;
    }
    else
    {
        // Clear status
        writel(0x00000001, base + IDMA_IRQ_STATUS);
    }

    //printf("iDMA: PASS, id=%u len=0x%lx\n", tid, len);    
    return CMD_RET_SUCCESS;

}

static int do_idma_test(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	ulong base_addr, src_addr, dst_addr, len;
	int ret;

    if (argc != 5)
        return CMD_RET_USAGE;

    //if (strcmp(argv[1], "test"))
    if (strcmp(argv[0], "test"))
        return CMD_RET_USAGE;

    base_addr = hextoul(argv[1], NULL);
    src_addr  = hextoul(argv[2], NULL);
    dst_addr  = hextoul(argv[3], NULL);
    len       = hextoul(argv[4], NULL);

    ret = idma_run_copy(base_addr, src_addr, dst_addr, len);
    if (ret != CMD_RET_SUCCESS)
        return CMD_RET_FAILURE;

    printf("iDMA: PASS, len=0x%lx\n", len);
    return CMD_RET_SUCCESS;
}

static int do_idma_loop(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
    //u32 i, len, lenCopy;
	ulong base_addr, src_addr, dst_addr, len, lenCopy;
	//void __iomem *base;
    int i;

    if (argc != 6)
        return CMD_RET_USAGE;

    if (strcmp(argv[0], "loop"))
        return CMD_RET_USAGE;

    len       = hextoul(argv[1], NULL);

	base_addr = hextoul(argv[2], NULL);
	src_addr  = hextoul(argv[3], NULL);
	dst_addr  = hextoul(argv[4], NULL);
	lenCopy       = hextoul(argv[5], NULL);

    for (i = 0; i < len; i++) {
        printf("iDMA loop test iteration %u\n", i);
        if (idma_run_copy(base_addr, src_addr, dst_addr, lenCopy) != CMD_RET_SUCCESS)
            return CMD_RET_FAILURE;
    }

    return CMD_RET_SUCCESS;
}



static int do_idma_irqloop(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	ulong base_addr, src_addr, dst_addr, len, lenCopy;
	//void __iomem *base;
    int i;

    if (argc != 6)
        return CMD_RET_USAGE;

    if (strcmp(argv[0], "irqloop"))
        return CMD_RET_USAGE;

    len       = hextoul(argv[1], NULL);

	base_addr = hextoul(argv[2], NULL);
	src_addr  = hextoul(argv[3], NULL);
	dst_addr  = hextoul(argv[4], NULL);
	lenCopy       = hextoul(argv[5], NULL);

    idma_enable_interrupts(base_addr, 1);
    for (i = 0; i < len; i++) {
        printf("iDMA loop test iteration %u\n", i);
        // Do the copy
        if (idma_run_copy(base_addr, src_addr, dst_addr, lenCopy) != CMD_RET_SUCCESS)
            return CMD_RET_FAILURE;
        // Wait for interrupt
        if (idma_irqcheck_and_clear(base_addr) != CMD_RET_SUCCESS)
            return CMD_RET_FAILURE;
    }
    idma_enable_interrupts(base_addr, 0);

    return CMD_RET_SUCCESS;
}


static int do_idma_regs(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
    ulong base_addr;
    void __iomem *base;
    u32 reg;


    if (argc != 2)
        return CMD_RET_USAGE;

    if (strcmp(argv[0], "regs"))
        return CMD_RET_USAGE;

    base_addr = hextoul(argv[1], NULL);
    base = (void __iomem *)base_addr;


    reg = IDMA_CONF;
    printf("iDMA: reg, id=%x val=0x%08x\n", reg, readl(base + reg));

    reg = IDMA_STATUS_0;
	printf("iDMA: reg, id=%x val=0x%08x\n", reg, readl(base + reg));

    reg = IDMA_DONE_ID_0;
	printf("iDMA: reg, id=%x val=0x%08x\n", reg, readl(base + reg));

    reg = IDMA_DST_ADDR_LOW;
	printf("iDMA: reg, id=%x val=0x%08x\n", reg, readl(base + reg));

    reg = IDMA_DST_ADDR_HIGH;
	printf("iDMA: reg, id=%x val=0x%08x\n", reg, readl(base + reg));

    reg = IDMA_SRC_ADDR_LOW;
	printf("iDMA: reg, id=%x val=0x%08x\n", reg, readl(base + reg));

    reg = IDMA_SRC_ADDR_HIGH;
	printf("iDMA: reg, id=%x val=0x%08x\n", reg, readl(base + reg));

    reg = IDMA_LENGTH_LOW;
	printf("iDMA: reg, id=%x val=0x%08x\n", reg, readl(base + reg));

    reg = IDMA_LENGTH_HIGH;
	printf("iDMA: reg, id=%x val=0x%08x\n", reg, readl(base + reg));

    return CMD_RET_SUCCESS;
}

// The arguments and argc value received by functions has the first parameter removed
static struct cmd_tbl cmd_idma[] = {
	U_BOOT_CMD_MKENT(test, 6, 0, do_idma_test, "", ""),
	U_BOOT_CMD_MKENT(loop, 7, 1, do_idma_loop, "", ""),
	U_BOOT_CMD_MKENT(irqloop, 7, 1, do_idma_irqloop, "", ""),
	U_BOOT_CMD_MKENT(regs, 3, 0, do_idma_regs, "", ""),
};


static int do_idmaops(struct cmd_tbl *cmdtp, int flag, int argc,
		     char *const argv[])
{
	struct cmd_tbl *cp;

	cp = find_cmd_tbl(argv[1], cmd_idma, ARRAY_SIZE(cmd_idma));

	/* Drop the mmc command */
	argc--;
	argv++;

	if (cp == NULL || argc > cp->maxargs)
		return CMD_RET_USAGE;
	if (flag == CMD_FLAG_REPEAT && !cmd_is_repeatable(cp))
		return CMD_RET_SUCCESS;

	return cp->cmd(cmdtp, flag, argc, argv);
}


U_BOOT_CMD(
    idma, 8, 1, do_idmaops,
	"iDMA test commmands",
	"test <base> <src> <dst> <len>\n"
	"    - fill src, run iDMA copy, compare dst\n"
	"loop <count> <base> <src> <dst> <len>\n"
	"    - Repeat <count> times fill src, run iDMA copy, compare dst\n"
	"irqloop <count> <base> <src> <dst> <len>\n"
	"    - Repeat <count> times fill src, run iDMA copy, compare dst, waiting for IRQ\n"
	"regs <base>\n"
	"    - print all iDMA registers\n"
);

