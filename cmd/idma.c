// cmd/idma.c

#include <command.h>
#include <cpu_func.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <vsprintf.h>
#include <malloc.h>
#include <mapmem.h>
#include <linux/dma-mapping.h>

#define IDMA_REG_CONF              0x000
#define IDMA_REG_STATUS_0          0x004
#define IDMA_REG_NEXT_ID_0         0x044
#define IDMA_REG_DONE_ID_0         0x084

#define IDMA_REG_DST_ADDR_LOW      0x0d0
#define IDMA_REG_DST_ADDR_HIGH     0x0d4
#define IDMA_REG_SRC_ADDR_LOW      0x0d8
#define IDMA_REG_SRC_ADDR_HIGH     0x0dc
#define IDMA_REG_LENGTH_LOW        0x0e0
#define IDMA_REG_LENGTH_HIGH       0x0e4


#define IDMA_REG_IRQ_STATUS        0x1000
#define IDMA_REG_IRQ_ENABLE        0x1004


#define IDMA_TIMEOUT_US        1000000

#define IDMA_DESC64_DESC_ADDR   0x000
#define IDMA_DESC64_STATUS      0x008
#define IDMA_DESC64_IRQ_STATUS  0x100
#define IDMA_DESC64_IRQ_ENABLE  0x104
#define IDMA_DESC64_STATUS_BUSY (1U << 0)
#define IDMA_DESC64_STATUS_FULL (1U << 1)
#define IDMA_DESC64_NEXT_END    (~0ULL)
#define IDMA_DESC64_FLAG_IRQ_ON_DONE (1U << 0)
#define IDMA_DESC64_FLAG_SRC_INCR    (1U << 1)
#define IDMA_DESC64_FLAG_DST_INCR    (1U << 3)
#define IDMA_DESC64_FLAG_SERIALIZE   (1U << 6)

#define IDMA_DESC64_FLAGS_NOIRQ       (IDMA_DESC64_FLAG_SRC_INCR | \
				 IDMA_DESC64_FLAG_DST_INCR | \
				 IDMA_DESC64_FLAG_SERIALIZE)

// There's apparently no maximum length, so this value is arbitrary
#define IDMA_DESC64_MAX_CHAIN_LENGTH        (0xFFFF)

#define IDMA_DESC64_FLAGS_IRQ       (IDMA_DESC64_FLAG_IRQ_ON_DONE | IDMA_DESC64_FLAGS_NOIRQ)

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


struct idma_desc64_desc {
	u32 length;
	u32 flags;
	u64 next;
	u64 src;
	u64 dst;
} __packed __aligned(8);

static int idma_desc64_run_copy(ulong base_addr, ulong src_addr, ulong dst_addr,
                                       ulong len, ulong chain_length)
{
    void __iomem *base = (void __iomem *)base_addr;
    struct idma_desc64_desc *desc;
    u8 *src_cur;
    u8 *dst_cur;
    ulong start, end;
    ulong desc_addr_head;
    ulong timeout;
    u32 status;
    u32 irq_status;
    int i;
    ulong idx_desc;
    int ret = CMD_RET_FAILURE;

    if (!len) {
        printf("iDMA desc64: zero length is not valid\n");
        return CMD_RET_FAILURE;
    }

    if (!chain_length) {
        printf("iDMA desc64: chain length provided not valid.\n");
        return CMD_RET_FAILURE;
    }

    if (chain_length > IDMA_DESC64_MAX_CHAIN_LENGTH) {
        printf("iDMA desc64: unsupported chain length value.\n");
        return CMD_RET_FAILURE;
    }

    desc = memalign(ARCH_DMA_MINALIGN, sizeof(*desc) * chain_length);
    if (!desc) {
        printf("iDMA desc64: descriptor allocation failed\n");
        return CMD_RET_FAILURE;
    }

    desc_addr_head = map_to_sysmem(desc);
    printf("iDMA desc64 test: base=0x%lx desc=%p desc_addr=0x%lx src=0x%lx dst=0x%lx len=0x%lx chain=0x%lx\n",
           base_addr, desc, desc_addr_head, src_addr, dst_addr, len, chain_length);

    for (idx_desc = 0; idx_desc < chain_length; idx_desc++) {
        u64 src_i = src_addr + (idx_desc * len);
        u64 dst_i = dst_addr + (idx_desc * len);
        ulong desc_i = map_to_sysmem(&desc[idx_desc]);
        u64 next_i = (idx_desc + 1 == chain_length) ?
                    IDMA_DESC64_NEXT_END : (u64)map_to_sysmem(&desc[idx_desc + 1]);

        src_cur = (u8 *)src_i;
        dst_cur = (u8 *)dst_i;

        for (i = 0; i < len; i++) {
            src_cur[i] = (u8)(i ^ 0xa5);
            dst_cur[i] = 0x5a;
        }

        desc[idx_desc].length = (u32)len;
        desc[idx_desc].flags = (idx_desc + 1 == chain_length) ?
                               IDMA_DESC64_FLAGS_IRQ : IDMA_DESC64_FLAGS_NOIRQ;
        desc[idx_desc].next = next_i;
        desc[idx_desc].src = src_i;
        desc[idx_desc].dst = dst_i;

        start = align_down_cache((ulong)&desc[idx_desc]);
        end = align_up_cache((ulong)&desc[idx_desc] + sizeof(desc[idx_desc]));
        flush_dcache_range(start, end);

        start = align_down_cache(src_i);
        end = align_up_cache(src_i + len);
        flush_dcache_range(start, end);

        start = align_down_cache(dst_i);
        end = align_up_cache(dst_i + len);
        flush_dcache_range(start, end);
        invalidate_dcache_range(start, end);

        printf("iDMA desc64[%lu]: desc_addr=0x%lx next=0x%llx src=0x%llx dst=0x%llx len=0x%lx flags=0x%x\n",
               idx_desc, desc_i, desc[idx_desc].next, src_i, dst_i, len,
               desc[idx_desc].flags);
    }

    writel(0x00000001, base + IDMA_DESC64_IRQ_STATUS);
    writel(0x00000001, base + IDMA_DESC64_IRQ_ENABLE);

    status = readl(base + IDMA_DESC64_STATUS);
    if (status & IDMA_DESC64_STATUS_FULL) {
        printf("iDMA desc64: descriptor FIFO full before submit, status=0x%08x\n",
               status);
        goto freemem;
    }

    idma_write64(base, IDMA_DESC64_DESC_ADDR, IDMA_DESC64_DESC_ADDR + 4,
                 (u64)desc_addr_head);
    udelay(1);

    timeout = IDMA_TIMEOUT_US;
    while (timeout--) {
        status = readl(base + IDMA_DESC64_STATUS);
        if (!(status & IDMA_DESC64_STATUS_BUSY))
            break;
        udelay(1);
    }

    if (!timeout) {
        printf("iDMA desc64: timeout, status=0x%08x\n",
               readl(base + IDMA_DESC64_STATUS));
        goto freemem;
    }

    irq_status = readl(base + IDMA_DESC64_IRQ_STATUS);
    if (!(irq_status & 0x1)) {
        printf("iDMA desc64: IRQ not detected, irq_status=0x%08x\n",
               irq_status);
        goto freemem;
    }
    writel(0x00000001, base + IDMA_DESC64_IRQ_STATUS);

    for (idx_desc = 0; idx_desc < chain_length; idx_desc++) {
        ulong src_i = src_addr + (idx_desc * len);
        ulong dst_i = dst_addr + (idx_desc * len);

        src_cur = (u8 *)src_i;
        dst_cur = (u8 *)dst_i;

        start = align_down_cache(dst_i);
        end = align_up_cache(dst_i + len);
        invalidate_dcache_range(start, end);

        for (i = 0; i < len; i++) {
            if (src_cur[i] != dst_cur[i]) {
                printf("iDMA desc64[%lu]: mismatch at +0x%x: src=0x%02x dst=0x%02x\n",
                       idx_desc, i, src_cur[i], dst_cur[i]);
                goto freemem;
            }
        }
    }

    ret = CMD_RET_SUCCESS;

freemem:
    free(desc);
    return ret;
}

static int idma_reg_run_copy(ulong base_addr, ulong src_addr, ulong dst_addr, ulong len)
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
	writel(0x00000000, base + IDMA_REG_CONF);

	idma_write64(base, IDMA_REG_SRC_ADDR_LOW, IDMA_REG_SRC_ADDR_HIGH, src_addr);
	idma_write64(base, IDMA_REG_DST_ADDR_LOW, IDMA_REG_DST_ADDR_HIGH, dst_addr);
	idma_write64(base, IDMA_REG_LENGTH_LOW,   IDMA_REG_LENGTH_HIGH,   len);

	/*
	 * iDMA register frontend launch:
	 * reading NEXT_ID launches the transfer.
	 */
	tid = readl(base + IDMA_REG_NEXT_ID_0);
	if (!tid) {
		printf("iDMA: launch failed, next_id returned 0\n");
		return CMD_RET_FAILURE;
	}

	printf("iDMA: launched transfer id %u\n", tid);

	timeout = IDMA_TIMEOUT_US;
	while (timeout--) {
		done = readl(base + IDMA_REG_DONE_ID_0);
		if (done >= tid)
			break;
		udelay(1);
	}

	if (!timeout) {
		printf("iDMA: timeout, status=0x%08x done_id=%u expected=%u\n",
		       readl(base + IDMA_REG_STATUS_0), done, tid);
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


static void idma_reg_enable_interrupts(ulong base_addr, int enable)
{
    void __iomem *base;
    u32 irq_status;


	base = (void __iomem *)base_addr;

    writel(0x00000001, base + IDMA_REG_IRQ_STATUS); // clear
    if (enable)
        writel(0x00000001, base + IDMA_REG_IRQ_ENABLE);
    else
        writel(0x00000000, base + IDMA_REG_IRQ_ENABLE);
}

static int idma_reg_irqcheck_and_clear(ulong base_addr)
{
    //ulong src_addr, dst_addr;
    void __iomem *base;
    u32 irq_status;

    base = (void __iomem *)base_addr;

    irq_status = readl(base + IDMA_REG_IRQ_STATUS);
    if (irq_status != 0x1) {
        printf("iDMA: IRQ not detected\n");
        return CMD_RET_FAILURE;
    }
    else
    {
        // Clear status
        writel(0x00000001, base + IDMA_REG_IRQ_STATUS);
    }

    //printf("iDMA: PASS, id=%u len=0x%lx\n", tid, len);    
    return CMD_RET_SUCCESS;

}

static int do_idma_reg_test(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
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

    ret = idma_reg_run_copy(base_addr, src_addr, dst_addr, len);
    if (ret != CMD_RET_SUCCESS)
        return CMD_RET_FAILURE;

    printf("iDMA: PASS, len=0x%lx\n", len);
    return CMD_RET_SUCCESS;
}

static int do_idma_reg_loop(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
    ulong base_addr, src_addr, dst_addr, len, lenCopy;
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
        if (idma_reg_run_copy(base_addr, src_addr, dst_addr, lenCopy) != CMD_RET_SUCCESS)
            return CMD_RET_FAILURE;
    }

    return CMD_RET_SUCCESS;
}


static int do_idma_desc_loop(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
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
        if (idma_desc64_run_copy(base_addr, src_addr, dst_addr, lenCopy, 1) != CMD_RET_SUCCESS)
            return CMD_RET_FAILURE;
    }

    return CMD_RET_SUCCESS;
}



static int do_idma_reg_irqloop(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
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

    idma_reg_enable_interrupts(base_addr, 1);
    for (i = 0; i < len; i++) {
        printf("iDMA loop test iteration %u\n", i);
        // Do the copy
        if (idma_reg_run_copy(base_addr, src_addr, dst_addr, lenCopy) != CMD_RET_SUCCESS)
            return CMD_RET_FAILURE;
        // Wait for interrupt
        if (idma_reg_irqcheck_and_clear(base_addr) != CMD_RET_SUCCESS)
            return CMD_RET_FAILURE;
    }
    idma_reg_enable_interrupts(base_addr, 0);

    return CMD_RET_SUCCESS;
}



static int do_idma_desc_test(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
    ulong base_addr, src_addr, dst_addr, len;
    int ret;

    if (argc != 5)
        return CMD_RET_USAGE;

    if (strcmp(argv[0], "test"))
        return CMD_RET_USAGE;

    base_addr = hextoul(argv[1], NULL);
    src_addr  = hextoul(argv[2], NULL);
    dst_addr  = hextoul(argv[3], NULL);
    len       = hextoul(argv[4], NULL);

    ret = idma_desc64_run_copy(base_addr, src_addr, dst_addr, len, 1);
    if (ret != CMD_RET_SUCCESS)
        return CMD_RET_FAILURE;

    printf("iDMA desc64: PASS, len=0x%lx\n", len);
    return CMD_RET_SUCCESS;
}


static int do_idma_desc_test_chain(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
    ulong base_addr, src_addr, dst_addr, len, chain_len;
    int ret;

    if (argc != 6)
        return CMD_RET_USAGE;

    if (strcmp(argv[0], "chain"))
        return CMD_RET_USAGE;

    chain_len       = hextoul(argv[1], NULL);

    base_addr = hextoul(argv[2], NULL);
    src_addr  = hextoul(argv[3], NULL);
    dst_addr  = hextoul(argv[4], NULL);
    len       = hextoul(argv[5], NULL);

    ret = idma_desc64_run_copy(base_addr, src_addr, dst_addr, len, chain_len);
    if (ret != CMD_RET_SUCCESS)
        return CMD_RET_FAILURE;

    printf("iDMA desc64 chain: PASS, len=0x%lx\n", len);
    return CMD_RET_SUCCESS;
}

static int do_idma_reg_regs(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
    ulong base_addr;
    void __iomem *base;
    u32 reg;


    if (argc != 1)
        return CMD_RET_USAGE;

    if (strcmp(argv[0], "regs"))
        return CMD_RET_USAGE;

    base_addr = hextoul(argv[1], NULL);
    base = (void __iomem *)base_addr;


    reg = IDMA_REG_CONF;
    printf("iDMA: reg, id=%x val=0x%08x\n", reg, readl(base + reg));

    reg = IDMA_REG_STATUS_0;
    printf("iDMA: reg, id=%x val=0x%08x\n", reg, readl(base + reg));

    reg = IDMA_REG_DONE_ID_0;
    printf("iDMA: reg, id=%x val=0x%08x\n", reg, readl(base + reg));

    reg = IDMA_REG_DST_ADDR_LOW;
    printf("iDMA: reg, id=%x val=0x%08x\n", reg, readl(base + reg));

    reg = IDMA_REG_DST_ADDR_HIGH;
    printf("iDMA: reg, id=%x val=0x%08x\n", reg, readl(base + reg));

    reg = IDMA_REG_SRC_ADDR_LOW;
    printf("iDMA: reg, id=%x val=0x%08x\n", reg, readl(base + reg));

    reg = IDMA_REG_SRC_ADDR_HIGH;
    printf("iDMA: reg, id=%x val=0x%08x\n", reg, readl(base + reg));

    reg = IDMA_REG_LENGTH_LOW;
    printf("iDMA: reg, id=%x val=0x%08x\n", reg, readl(base + reg));

    reg = IDMA_REG_LENGTH_HIGH;
    printf("iDMA: reg, id=%x val=0x%08x\n", reg, readl(base + reg));

    return CMD_RET_SUCCESS;
}

// The arguments and argc value received by functions has the first parameter removed
static struct cmd_tbl cmd_idma_reg[] = {
    U_BOOT_CMD_MKENT(test, 6, 0, do_idma_reg_test, "", ""),
    U_BOOT_CMD_MKENT(loop, 7, 1, do_idma_reg_loop, "", ""),
    U_BOOT_CMD_MKENT(irqloop, 7, 1, do_idma_reg_irqloop, "", ""),
    U_BOOT_CMD_MKENT(regs, 3, 0, do_idma_reg_regs, "", ""),
};


static struct cmd_tbl cmd_idma_desc[] = {
    U_BOOT_CMD_MKENT(test, 6, 0, do_idma_desc_test, "", ""),
    U_BOOT_CMD_MKENT(loop, 7, 1, do_idma_desc_loop, "", ""),
    U_BOOT_CMD_MKENT(chain, 7, 1, do_idma_desc_test_chain, "", ""),
};


static int do_idma_descops(struct cmd_tbl *cmdtp, int flag, int argc,
                char *const argv[])
{
    struct cmd_tbl *cp;

    if (argc < 2)
        return CMD_RET_USAGE;

    cp = find_cmd_tbl(argv[1], cmd_idma_desc, ARRAY_SIZE(cmd_idma_desc));

    argc--;
    argv++;

    if (cp == NULL || argc > cp->maxargs)
        return CMD_RET_USAGE;
    if (flag == CMD_FLAG_REPEAT && !cmd_is_repeatable(cp))
        return CMD_RET_SUCCESS;

    return cp->cmd(cmdtp, flag, argc, argv);
}


static int do_idma_regops(struct cmd_tbl *cmdtp, int flag, int argc,
                char *const argv[])
{
    struct cmd_tbl *cp;

    cp = find_cmd_tbl(argv[1], cmd_idma_reg, ARRAY_SIZE(cmd_idma_reg));

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
    idma_reg, 8, 1, do_idma_regops,
    "iDMA reg64 test commmands",
    "test <base> <src> <dst> <len>\n"
    "    - fill src, run iDMA copy, compare dst\n"
    "idma_reg loop <count> <base> <src> <dst> <len>\n"
    "    - Repeat <count> times fill src, run iDMA copy, compare dst\n"
    "idma_reg irqloop <count> <base> <src> <dst> <len>\n"
    "    - Repeat <count> times fill src, run iDMA copy, compare dst, waiting for IRQ\n"
    "idma_reg regs <base>\n"
    "    - print all iDMA registers\n"
);



U_BOOT_CMD(
    idma_desc, 7, 1, do_idma_descops,
    "iDMA desc64 test commands",
    "test <base> <src> <dst> <len>\n"
    "    - fill src, submit desc64 descriptor, compare dst\n"
    "idma_desc loop <count> <base> <src> <dst> <len>\n"
    "    - repeat <count> times desc64 descriptor copy test\n"
    "idma_desc chain <count> <base> <src> <dst> <len>\n"
    "    - request <count> chained descriptor copies (address offsets based on len increments)\n"
);
