// cmd/idma.c

#include <command.h>
#include <cpu_func.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <vsprintf.h>
#include <malloc.h>
#include <mapmem.h>
#include <linux/dma-mapping.h>
#include <div64.h>

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
#define IDMA_DESC64_FLAG_PRESERVE    (1U << 30)
#define IDMA_DESC64_FLAG_PROT_SRC_BIT    24
#define IDMA_DESC64_FLAG_PROT_DST_BIT    27

#define IDMA_DESC64_PROT_AXI            (0)
#define IDMA_DESC64_PROT_AXIS           (5)

#define IDMA_DESC64_FLAGS_NOIRQ       (IDMA_DESC64_FLAG_SRC_INCR | \
				 IDMA_DESC64_FLAG_DST_INCR | \
				 IDMA_DESC64_FLAG_SERIALIZE)

// There's apparently no maximum length, so this value is arbitrary
#define IDMA_DESC64_MAX_CHAIN_LENGTH        (0xFFFF)

#define IDMA_DESC64_FLAGS_IRQ               (IDMA_DESC64_FLAG_IRQ_ON_DONE | IDMA_DESC64_FLAGS_NOIRQ)
#define IDMA_DESC64_FLAGS_AXI_TO_AXIS       ((IDMA_DESC64_PROT_AXI << IDMA_DESC64_FLAG_PROT_SRC_BIT) | \
                                            (IDMA_DESC64_PROT_AXIS << IDMA_DESC64_FLAG_PROT_DST_BIT))

#define IDMA_DESC64_AXIS_TIMEOUT_US 10000UL
#define IDMA_DESC64_CYCLIC_CTRL      0x110
#define IDMA_DESC64_CYCLIC_STOP      BIT(0)
#define IDMA_DESC64_CYCLIC_RUNNING   BIT(1)
#define IDMA_DESC64_CYCLIC_STOPPED   BIT(2)
#define IDMA_AUDIO_SAMPLE_RATE      44100U
#define IDMA_AUDIO_FRAME_BYTES      8U
#define IDMA_AUDIO_MAX_DURATION_MS  60000UL

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

static struct idma_desc64_desc *idma_cyclic_desc;

/* One quadrant of a 256-entry signed Q23 sine table. */
static const s32 idma_sine_quarter_q23[65] = {
          0,  205867,  411609,  617104,  822227, 1026855, 1230864, 1434132,
    1636536, 1837954, 2038265, 2237349, 2435084, 2631353, 2826037, 3019018,
    3210181, 3399410, 3586592, 3771613, 3954362, 4134729, 4312606, 4487885,
    4660460, 4830229, 4997087, 5160936, 5321676, 5479210, 5633444, 5784285,
    5931641, 6075424, 6215548, 6351927, 6484481, 6613128, 6737792, 6858398,
    6974872, 7087145, 7195148, 7298818, 7398091, 7492908, 7583211, 7668946,
    7750062, 7826510, 7898243, 7965219, 8027396, 8084739, 8137211, 8184782,
    8227422, 8265107, 8297813, 8325521, 8348214, 8365878, 8378503, 8386081,
    8388607
};

static s32 idma_sine_sample_q23(u32 phase)
{
    u32 index = phase >> 24;
    u32 offset = index & 0x3f;

    switch (index >> 6) {
    case 0:
        return idma_sine_quarter_q23[offset];
    case 1:
        return idma_sine_quarter_q23[64 - offset];
    case 2:
        return -idma_sine_quarter_q23[offset];
    default:
        return -idma_sine_quarter_q23[64 - offset];
    }
}

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

static int idma_desc64_submit_axis(ulong base_addr, ulong src_addr, ulong len,
                                   ulong timeout_us, const char *name)
{
    void __iomem *base = (void __iomem *)base_addr;
    struct idma_desc64_desc *desc;
    ulong desc_addr;
    ulong start, end;
    ulong timeout;
    u32 status;
    int ret = CMD_RET_FAILURE;

    if (!len || len > 0xffffffffUL) {
        printf("%s: length must be in range 1..0xffffffff\n", name);
        return CMD_RET_FAILURE;
    }

    desc = memalign(ARCH_DMA_MINALIGN, sizeof(*desc));
    if (!desc) {
        printf("%s: descriptor allocation failed\n", name);
        return CMD_RET_FAILURE;
    }

    desc->length = (u32)len;
    desc->flags = IDMA_DESC64_FLAGS_IRQ | IDMA_DESC64_FLAGS_AXI_TO_AXIS;
    desc->next = IDMA_DESC64_NEXT_END;
    desc->src = src_addr;
    desc->dst = 0;

    start = align_down_cache((ulong)desc);
    end = align_up_cache((ulong)desc + sizeof(*desc));
    flush_dcache_range(start, end);
    start = align_down_cache(src_addr);
    end = align_up_cache(src_addr + len);
    flush_dcache_range(start, end);

    status = readl(base + IDMA_DESC64_STATUS);
    if (status & IDMA_DESC64_STATUS_FULL) {
        printf("%s: descriptor FIFO full, status=0x%08x\n", name, status);
        goto out;
    }

    desc_addr = map_to_sysmem(desc);
    printf("%s: base=0x%lx desc=0x%lx src=0x%lx len=0x%lx\n",
           name, base_addr, desc_addr, src_addr, len);
    idma_write64(base, IDMA_DESC64_DESC_ADDR, IDMA_DESC64_DESC_ADDR + 4,
                 (u64)desc_addr);
    udelay(1);

    timeout = timeout_us;
    do {
        status = readl(base + IDMA_DESC64_STATUS);
        if (!(status & IDMA_DESC64_STATUS_BUSY))
            break;
        udelay(1);
    } while (--timeout);

    if (status & IDMA_DESC64_STATUS_BUSY) {
        printf("%s: timeout, status=0x%08x\n", name,
               readl(base + IDMA_DESC64_STATUS));
        goto out;
    }

    printf("%s: PASS, len=0x%lx\n", name, len);
    ret = CMD_RET_SUCCESS;

out:
    free(desc);
    return ret;
}

static int idma_desc64_run_axis(ulong base_addr, ulong src_addr, ulong len)
{
    u8 *src = (u8 *)src_addr;
    ulong i;

    if (!len || len > 0xffffffffUL) {
        printf("iDMA desc64 AXIS: length must be in range 1..0xffffffff\n");
        return CMD_RET_FAILURE;
    }

    for (i = 0; i < len; i++)
        src[i] = (u8)(i ^ 0xa5);

    return idma_desc64_submit_axis(base_addr, src_addr, len,
                                   IDMA_DESC64_AXIS_TIMEOUT_US,
                                   "iDMA desc64 AXIS");
}

static int do_idma_desc_axis(struct cmd_tbl *cmdtp, int flag, int argc,
                             char *const argv[])
{
    ulong base_addr;
    ulong src_addr;
    ulong len;

    if (argc != 4)
        return CMD_RET_USAGE;

    base_addr = hextoul(argv[1], NULL);
    src_addr = hextoul(argv[2], NULL);
    len = hextoul(argv[3], NULL);
    return idma_desc64_run_axis(base_addr, src_addr, len);
}

static int do_idma_sine(struct cmd_tbl *cmdtp, int flag, int argc,
                        char *const argv[])
{
    ulong base_addr;
    ulong src_addr;
    ulong frequency;
    ulong duration_ms;
    ulong amplitude = 25;
    ulong frames;
    ulong len;
    ulong timeout_us;
    u64 value;
    u32 phase = 0;
    u32 phase_step;
    u32 *samples;
    ulong i;

    if (argc < 5 || argc > 6)
        return CMD_RET_USAGE;

    base_addr = hextoul(argv[1], NULL);
    src_addr = hextoul(argv[2], NULL);
    frequency = dectoul(argv[3], NULL);
    duration_ms = dectoul(argv[4], NULL);
    if (argc == 6)
        amplitude = dectoul(argv[5], NULL);

    if (!frequency || frequency >= IDMA_AUDIO_SAMPLE_RATE / 2) {
        printf("idma_sine: frequency must be 1..22049 Hz\n");
        return CMD_RET_FAILURE;
    }
    if (!duration_ms || duration_ms > IDMA_AUDIO_MAX_DURATION_MS) {
        printf("idma_sine: duration must be 1..60000 ms\n");
        return CMD_RET_FAILURE;
    }
    if (!amplitude || amplitude > 100) {
        printf("idma_sine: amplitude must be 1..100 percent\n");
        return CMD_RET_FAILURE;
    }

    value = (u64)IDMA_AUDIO_SAMPLE_RATE * duration_ms + 999;
    do_div(value, 1000);
    if (value > 0xffffffffULL / IDMA_AUDIO_FRAME_BYTES) {
        printf("idma_sine: generated buffer is too large\n");
        return CMD_RET_FAILURE;
    }
    frames = (ulong)value;
    len = frames * IDMA_AUDIO_FRAME_BYTES;

    value = (u64)frequency << 32;
    do_div(value, IDMA_AUDIO_SAMPLE_RATE);
    phase_step = (u32)value;
    samples = (u32 *)src_addr;

    for (i = 0; i < frames; i++) {
        s32 sample = idma_sine_sample_q23(phase);

        sample = sample * (s32)amplitude / 100;
        samples[2 * i] = (u32)sample;
        samples[2 * i + 1] = (u32)sample;
        phase += phase_step;
    }

    timeout_us = duration_ms * 1000 + 100000;
    printf("idma_sine: %lu Hz, %lu frames, %lu ms, %lu%% amplitude\n",
           frequency, frames, duration_ms, amplitude);
    return idma_desc64_submit_axis(base_addr, src_addr, len, timeout_us,
                                   "iDMA sine");
}


#define IDMA_AXIS_REG_IRQ_STATUS        0x100
#define IDMA_AXIS_REG_IRQ_ENABLE        0x104

static void idma_axis_enable_interrupts(ulong base_addr, int enable)
{
    void __iomem *base;
    u32 irq_status;


	base = (void __iomem *)base_addr;

    writel(0x00000001, base + IDMA_AXIS_REG_IRQ_STATUS); // clear
    if (enable)
        writel(0x00000001, base + IDMA_AXIS_REG_IRQ_ENABLE);
    else
        writel(0x00000000, base + IDMA_AXIS_REG_IRQ_ENABLE);
}

static int do_idma_sine_cyclic(struct cmd_tbl *cmdtp, int flag, int argc,
                               char *const argv[])
{
    void __iomem *base;
    ulong base_addr, src_addr, frequency, period_ms, amplitude = 25;
    ulong frames, len, desc_addr, i;
    u64 value;
    u32 phase = 0, phase_step;
    u32 *samples;

    if (argc < 5 || argc > 6)
        return CMD_RET_USAGE;
    if (idma_cyclic_desc) {
        printf("idma_sine_cyclic: a cyclic transfer is already active\n");
        return CMD_RET_FAILURE;
    }

    base_addr = hextoul(argv[1], NULL);
    src_addr = hextoul(argv[2], NULL);
    frequency = dectoul(argv[3], NULL);
    period_ms = dectoul(argv[4], NULL);
    if (argc == 6)
        amplitude = dectoul(argv[5], NULL);
    if (!frequency || frequency >= IDMA_AUDIO_SAMPLE_RATE / 2 ||
        !period_ms || period_ms > IDMA_AUDIO_MAX_DURATION_MS ||
        !amplitude || amplitude > 100)
        return CMD_RET_USAGE;

    value = (u64)IDMA_AUDIO_SAMPLE_RATE * period_ms + 999;
    do_div(value, 1000);
    frames = (ulong)value;
    len = frames * IDMA_AUDIO_FRAME_BYTES;
    if (!len || len > 0xffffffffUL)
        return CMD_RET_FAILURE;

    value = (u64)frequency << 32;
    do_div(value, IDMA_AUDIO_SAMPLE_RATE);
    phase_step = (u32)value;
    samples = (u32 *)src_addr;
    for (i = 0; i < frames; i++) {
        s32 sample = idma_sine_sample_q23(phase);

        sample = sample * (s32)amplitude / 100;
        samples[2 * i] = (u32)sample;
        samples[2 * i + 1] = (u32)sample;
        phase += phase_step;
    }

    idma_cyclic_desc = memalign(ARCH_DMA_MINALIGN, sizeof(*idma_cyclic_desc));
    if (!idma_cyclic_desc)
        return CMD_RET_FAILURE;
    desc_addr = map_to_sysmem(idma_cyclic_desc);
    idma_cyclic_desc->length = (u32)len;
    idma_cyclic_desc->flags = IDMA_DESC64_FLAGS_IRQ |
                              IDMA_DESC64_FLAGS_AXI_TO_AXIS |
                              IDMA_DESC64_FLAG_PRESERVE;
    idma_cyclic_desc->next = (u64)desc_addr;
    idma_cyclic_desc->src = src_addr;
    idma_cyclic_desc->dst = 0;
    flush_dcache_range(align_down_cache((ulong)idma_cyclic_desc),
                       align_up_cache((ulong)idma_cyclic_desc +
                                      sizeof(*idma_cyclic_desc)));
    flush_dcache_range(align_down_cache(src_addr),
                       align_up_cache(src_addr + len));

    base = (void __iomem *)base_addr;
    if (readl(base + IDMA_DESC64_STATUS) & IDMA_DESC64_STATUS_FULL) {
        free(idma_cyclic_desc);
        idma_cyclic_desc = NULL;
        return CMD_RET_FAILURE;
    }

    printf("idma_sine_cyclic: enabling interrupts.\n");
    idma_axis_enable_interrupts(base_addr, 1);
    printf("idma_sine_cyclic: enabled.\n");

    idma_write64(base, IDMA_DESC64_DESC_ADDR, IDMA_DESC64_DESC_ADDR + 4,
                 (u64)desc_addr);

    u32 before, after;

    printf("idma_sine_cyclic: waiting for IRQ status.\n");
    /* Wait until one cyclic-period completion latches the wrapper IRQ. */
    while (!(readl( base + IDMA_AXIS_REG_IRQ_STATUS) & 1));

    printf("idma_sine_cyclic: IRQ status detected.\n");
    before = readl(base + IDMA_AXIS_REG_IRQ_STATUS);
    // acknowledge
    writel(1, base + IDMA_AXIS_REG_IRQ_STATUS);
    after = readl(base + IDMA_AXIS_REG_IRQ_STATUS);


    printf("idma_sine_cyclic: running desc=0x%lx, %lu Hz, %lu ms period. IRQ clear: before: %d, after: %d\n",
           desc_addr, frequency, period_ms, before, after);
    return CMD_RET_SUCCESS;
}

static int do_idma_cyclic_stop(struct cmd_tbl *cmdtp, int flag, int argc,
                               char *const argv[])
{
    void __iomem *base;
    ulong timeout = IDMA_TIMEOUT_US;
    u32 ctrl;

    if (argc < 2 || argc > 3)
        return CMD_RET_USAGE;
    base = (void __iomem *)hextoul(argv[1], NULL);
    if (argc == 3)
        timeout = dectoul(argv[2], NULL);
    writel(IDMA_DESC64_CYCLIC_STOP, base + IDMA_DESC64_CYCLIC_CTRL);
    do {
        ctrl = readl(base + IDMA_DESC64_CYCLIC_CTRL);
        if (ctrl & IDMA_DESC64_CYCLIC_STOPPED)
            break;
        udelay(1);
    } while (--timeout);
    if (!(ctrl & IDMA_DESC64_CYCLIC_STOPPED)) {
        printf("idma_cyclic_stop: timeout, CYCLIC_CTRL=0x%08x\n", ctrl);
        return CMD_RET_FAILURE;
    }
    free(idma_cyclic_desc);
    idma_cyclic_desc = NULL;
    printf("idma_cyclic_stop: stopped, CYCLIC_CTRL=0x%08x\n", ctrl);
    return CMD_RET_SUCCESS;
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

U_BOOT_CMD(
    idma_desc_axis, 4, 1, do_idma_desc_axis,
    "play DDR bytes through the dedicated iDMA AXI-Stream frontend",
    "<base> <src> <len>\n"
    "    - fill src with i^0xa5 and submit one descriptor\n"
);

U_BOOT_CMD(
    idma_sine, 6, 1, do_idma_sine,
    "generate and play a stereo sine wave through iDMA AXI-Stream",
    "<base> <src> <frequency_hz> <duration_ms> [amplitude_percent]\n"
    "    - generate signed 24-bit, 44.1 kHz stereo samples and submit one descriptor\n"
);

U_BOOT_CMD(
    idma_sine_cyclic, 6, 1, do_idma_sine_cyclic,
    "start cyclic stereo sine playback through iDMA AXI-Stream",
    "<base> <src> <frequency_hz> <period_ms> [amplitude_percent]\n"
    "    - create a preserved circular descriptor and return while it plays\n"
);

U_BOOT_CMD(
    idma_cyclic_stop, 3, 1, do_idma_cyclic_stop,
    "gracefully stop cyclic iDMA AXI-Stream playback",
    "<base> [timeout_us]\n"
    "    - issue CYCLIC_CTRL.STOP and wait for STOPPED\n"
);
