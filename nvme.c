/*
 * NVM Express device driver
 * Copyright (c) 2011, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "nvme.h"
#include <linux/bio.h>
#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/blkdev.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/genhd.h>
#include <linux/idr.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kdev_t.h>
#include <linux/kthread.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pci.h>
#include <linux/poison.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/version.h>

/* In v2.6.35, the REQ and BIO_RW flags were unified.  The FUA flag
 * doesn't have the same meaning as barrier, so don't ship this code
 * into production on earlier kernels.
 */
#ifndef REQ_RAHEAD
#define REQ_RAHEAD	(1 << BIO_RW_AHEAD)
#define REQ_FLUSH	(1 << BIO_RW_BARRIER)
#undef REQ_FUA
#define REQ_FUA		(1 << BIO_RW_BARRIER)
#endif

#ifndef RHEL_RELEASE_CODE
#define RHEL_RELEASE_CODE 0
#define RHEL_RELEASE_VERSION(x, y)	1
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 35)

#define dma_set_coherent_mask(dev, mask)	\
				pci_set_consistent_dma_mask(pdev, mask)
#endif

static int nvme_submit_bio_queue(struct nvme_queue *nvmeq, struct nvme_ns *ns,
				struct bio *bio, struct request *req,
				int start_sg_index, int start_sg_offset,
				int start_len_completed);
static int nvme_reset_controller(struct nvme_dev *dev);

#define num_possible_cpus num_online_cpus

/* 2.6.34 introduced this offset */
#ifndef POISON_POINTER_DELTA
#define POISON_POINTER_DELTA	0
#endif

/* 2.6.34 renamed this function */
#ifndef for_each_set_bit
#define for_each_set_bit for_each_bit
#endif

#define NVME_Q_DEPTH 1024
#define SQ_SIZE(depth)		(depth * sizeof(struct nvme_command))
#define CQ_SIZE(depth)		(depth * sizeof(struct nvme_completion))
#define NVME_MINORS 16
#define NVME_MAX_DEVS 1024
#define ADMIN_TIMEOUT	(60 * HZ)

static DEFINE_SPINLOCK(dev_list_lock);
static LIST_HEAD(dev_list);
static struct task_struct *nvme_thread;

static inline __attribute__((const))
unsigned long __rounddown_pow_of_two(unsigned long n)
{
	return 1UL << (fls_long(n) - 1);
}

#define rounddown_pow_of_two(n) \
(                               \
	__builtin_constant_p(n) ? (  \
	   (n == 1) ? 0 :            \
	   (1UL << ilog2(n))) :      \
	__rounddown_pow_of_two(n)    \
)

/*
 * An NVM Express queue.  Each device has at least two (one for admin
 * commands and one for I/O commands).
 */
struct nvme_queue {
	struct device *q_dmadev;
	struct nvme_dev *dev;
	spinlock_t q_lock;
	struct nvme_command *sq_cmds;
	volatile struct nvme_completion *cqes;
	dma_addr_t sq_dma_addr;
	dma_addr_t cq_dma_addr;
	u32 __iomem *q_db;
	u16 q_depth;
	u16 cq_vector;
	u16 sq_head;
	u16 sq_tail;
	u16 cq_head;
	u16 qid;
	u8 cq_phase;
	u8 cqe_seen;
	unsigned long cmdid_data[];
};

/*
 * Check we didn't inadvertently grow the command struct
 */
static inline void _nvme_check_size(void)
{
	BUILD_BUG_ON(sizeof(struct nvme_common_command) != 64);
	BUILD_BUG_ON(sizeof(struct nvme_rw_command) != 64);
	BUILD_BUG_ON(sizeof(struct nvme_identify) != 64);
	BUILD_BUG_ON(sizeof(struct nvme_create_cq) != 64);
	BUILD_BUG_ON(sizeof(struct nvme_create_sq) != 64);
	BUILD_BUG_ON(sizeof(struct nvme_delete_queue) != 64);
	BUILD_BUG_ON(sizeof(struct nvme_download_firmware) != 64);
	BUILD_BUG_ON(sizeof(struct nvme_features) != 64);
	BUILD_BUG_ON(sizeof(struct nvme_command) != 64);
	BUILD_BUG_ON(sizeof(struct nvme_abort_cmd) != 64);
	BUILD_BUG_ON(sizeof(struct nvme_id_ctrl) != 4096);
	BUILD_BUG_ON(sizeof(struct nvme_id_ns) != 4096);
	BUILD_BUG_ON(sizeof(struct nvme_lba_range_type) != 64);
	BUILD_BUG_ON(sizeof(struct nvme_smart_log) != 512);
	BUILD_BUG_ON(sizeof(struct nvme_completion) != 16);
	BUILD_BUG_ON(sizeof(struct nvme_bar) != 56);
}
/*
typedef void (*nvme_completion_fn)(struct nvme_queue *, void *,
						struct nvme_completion *);
*/
struct nvme_cmd_info {
	nvme_completion_fn fn;
	void *ctx;
	unsigned long timeout;
};

struct _devmajors {
	int char_major;
	int block_major;
	int block_minor;
	struct gendisk *bd_disk;
};

static struct _devmajors devmajors[4][16]; /* hold the major numbers of the char
                                              and block interfaces; per adapter */
static struct nvme_cmd_info *nvme_cmd_info(struct nvme_queue *nvmeq)
{
	return (void *)&nvmeq->cmdid_data[BITS_TO_LONGS(nvmeq->q_depth)];
}

/**
 * alloc_cmdid() - Allocate a Command ID
 * @nvmeq: The queue that will be used for this command
 * @ctx: A pointer that will be passed to the handler
 * @handler: The function to call on completion
 *
 * Allocate a Command ID for a queue.  The data passed in will
 * be passed to the completion handler.  This is implemented by using
 * the bottom two bits of the ctx pointer to store the handler ID.
 * Passing in a pointer that's not 4-byte aligned will cause a BUG.
 * We can change this if it becomes a problem.
 *
 * May be called with local interrupts disabled and the q_lock held,
 * or with interrupts enabled and no locks held.
 */
static int alloc_cmdid(struct nvme_queue *nvmeq, void *ctx,
				nvme_completion_fn handler, unsigned timeout)
{
	int depth = nvmeq->q_depth - 1;
	struct nvme_cmd_info *info = nvme_cmd_info(nvmeq);
	int cmdid;

	do {
		cmdid = find_first_zero_bit(nvmeq->cmdid_data, depth);
		if (cmdid >= depth)
			return -EBUSY;
	} while (test_and_set_bit(cmdid, nvmeq->cmdid_data));

	info[cmdid].fn = handler;
	info[cmdid].ctx = ctx;
	info[cmdid].timeout = jiffies + timeout;
	return cmdid;
}

static int alloc_cmdid_killable(struct nvme_queue *nvmeq, void *ctx,
				nvme_completion_fn handler, unsigned timeout)
{
	int cmdid;
	cmdid = alloc_cmdid(nvmeq, ctx, handler, timeout);
	return (cmdid < 0) ? -EINTR : cmdid;
}

/* Special values must be less than 0x1000 */
#define CMD_CTX_BASE		((void *)POISON_POINTER_DELTA)
#define CMD_CTX_CANCELLED	(0x30C + CMD_CTX_BASE)
#define CMD_CTX_COMPLETED	(0x310 + CMD_CTX_BASE)
#define CMD_CTX_INVALID		(0x314 + CMD_CTX_BASE)
#define CMD_CTX_FLUSH		(0x318 + CMD_CTX_BASE)

static void special_completion(struct nvme_queue *nvmeq, void *ctx,
						struct nvme_completion *cqe)
{
	struct nvme_dev *dev = nvmeq->dev;

	if (ctx == CMD_CTX_CANCELLED)
		return;
	if (ctx == CMD_CTX_FLUSH)
		return;
	if (ctx == CMD_CTX_COMPLETED) {
		dev_warn(&dev->pci_dev->dev,
				"completed id %d twice on queue %d\n",
				cqe->command_id, le16_to_cpup(&cqe->sq_id));
		return;
	}
	if (ctx == CMD_CTX_INVALID) {
		dev_warn(&dev->pci_dev->dev,
				"invalid id %d completed on queue %d\n",
				cqe->command_id, le16_to_cpup(&cqe->sq_id));
		return;
	}

	dev_warn(&dev->pci_dev->dev, "Unknown special completion %p\n", ctx);
}

/*
 * Called with local interrupts disabled and the q_lock held.  May not sleep.
 */
static void *free_cmdid(struct nvme_queue *nvmeq, int cmdid,
						nvme_completion_fn *fn)
{
	void *ctx;
	struct nvme_cmd_info *info = nvme_cmd_info(nvmeq);

	if (cmdid >= nvmeq->q_depth) {
		*fn = special_completion;
		return CMD_CTX_INVALID;
	}
	if (fn)
		*fn = info[cmdid].fn;
	ctx = info[cmdid].ctx;
	info[cmdid].fn = special_completion;
	info[cmdid].ctx = CMD_CTX_COMPLETED;
	clear_bit(cmdid, nvmeq->cmdid_data);
	return ctx;
}

static void *cancel_cmdid(struct nvme_queue *nvmeq, int cmdid,
						nvme_completion_fn *fn)
{
	void *ctx;
	struct nvme_cmd_info *info = nvme_cmd_info(nvmeq);
	if (fn)
		*fn = info[cmdid].fn;
	ctx = info[cmdid].ctx;
	info[cmdid].fn = special_completion;
	info[cmdid].ctx = CMD_CTX_CANCELLED;
	return ctx;
}

struct nvme_queue *get_nvmeq(struct nvme_dev *dev)
{
	return dev->queues[get_cpu() + 1];
}

static void put_nvmeq(struct nvme_queue *nvmeq)
{
	put_cpu();
}

/**
 * nvme_submit_cmd() - Copy a command into a queue and ring the doorbell
 * @nvmeq: The queue to use
 * @cmd: The command to send
 *
 * Safe to use from interrupt context
 */
static int nvme_submit_cmd(struct nvme_queue *nvmeq, struct nvme_command *cmd)
{
	unsigned long flags;
	u16 tail;
	spin_lock_irqsave(&nvmeq->q_lock, flags);
	tail = nvmeq->sq_tail;
	memcpy(&nvmeq->sq_cmds[tail], cmd, sizeof(*cmd));
	if (++tail == nvmeq->q_depth)
		tail = 0;
	writel(tail, nvmeq->q_db);
	nvmeq->sq_tail = tail;
	spin_unlock_irqrestore(&nvmeq->q_lock, flags);

	return 0;
}

static __le64 **iod_list(struct nvme_iod *iod)
{
	return ((void *)iod) + iod->offset;
}

/*
 * Will slightly overestimate the number of pages needed.  This is OK
 * as it only leads to a small amount of wasted memory for the lifetime of
 * the I/O.
 */
static int nvme_npages(unsigned size)
{
	unsigned nprps = DIV_ROUND_UP(size + PAGE_SIZE, PAGE_SIZE);
	return DIV_ROUND_UP(8 * nprps, PAGE_SIZE - 8);
}

struct nvme_iod *
nvme_alloc_iod(unsigned nbytes, gfp_t gfp)
{
	struct nvme_iod *iod = kmalloc(sizeof(struct nvme_iod) +
				sizeof(__le64 *) * nvme_npages(nbytes), gfp);

	if (iod) {
		iod->offset = sizeof(struct nvme_iod);
		iod->npages = -1;
		iod->length = nbytes;
		iod->nents = 0;
	}

	return iod;
}

void nvme_free_iod(struct nvme_dev *dev, struct nvme_iod *iod)
{
	const int last_prp = PAGE_SIZE / 8 - 1;
	int i;
	__le64 **list = iod_list(iod);
	dma_addr_t prp_dma = iod->first_dma;

	if (iod->npages == 0)
		dma_pool_free(dev->prp_small_pool, list[0], prp_dma);
	for (i = 0; i < iod->npages; i++) {
		__le64 *prp_list = list[i];
		dma_addr_t next_prp_dma = le64_to_cpu(prp_list[last_prp]);
		dma_pool_free(dev->prp_page_pool, prp_list, prp_dma);
		prp_dma = next_prp_dma;
	}
	kfree(iod);
}

static void bio_completion(struct nvme_queue *nvmeq, void *ctx,
						struct nvme_completion *cqe)
{
	struct nvme_iod *iod = ctx;
	struct bio *bio = iod->private;
	struct request *req = iod->req;
	struct nvme_dev *dev = nvmeq->dev;

	int start_sg_index = iod->start_sg_index;
	int start_sg_offset = iod->start_sg_offset;
	int start_len_completed = iod->start_len_completed;

	int end_sg_index = iod->end_sg_index;
	int end_sg_offset = iod->end_sg_offset;
	int end_len_completed = iod->end_len_completed;

	struct nvme_ns *ns = req->rq_disk->private_data;
	u16 status = le16_to_cpup(&cqe->status) >> 1;

	nvme_free_iod(dev, iod);
	if(status) {
		if(!bio_failfast(bio) && !(status & 0x4000)) {
			bio->bi_rw |= BIO_RW_FAILFAST;
			if (!nvme_submit_bio_queue(nvmeq, ns, bio, req,
						start_sg_index, start_sg_offset,
						start_len_completed))
				return;
		}
	}
	if (end_len_completed != bio->bi_size) {
		if (!(nvme_submit_bio_queue(nvmeq, ns, bio, req,
						end_sg_index, end_sg_offset,
						end_len_completed)))
			return;
		status = -EIO;
	}
	req->errors = status;
	blk_complete_request(req);
}

static void bio_completion_softirq(struct request *req)
{
	if (req->errors)
		bio_endio(req->bio, 0, -EIO);
	else
		bio_endio(req->bio, bio_sectors(req->bio), 0);
	end_that_request_last(req, req->errors);

	if (blk_queue_stopped(req->q))
		blk_start_queue(req->q);
}

/* length is in bytes.  gfp flags indicates whether we may sleep. */
static int nvme_setup_prps(struct nvme_dev *dev,
			struct nvme_common_command *cmd, struct nvme_iod *iod,
			int total_len, gfp_t gfp, int lba_addr_off)
{
	struct dma_pool *pool;
	int length = total_len;
	struct scatterlist *sg = iod->sg;
	int dma_len, prev_dma_len, prp_len, total_prp_len = 0;
	u64 dma_addr;
	int offset;
	__le64 *prp_list;
	__le64 **list = iod_list(iod);
	dma_addr_t prp_dma;
	int nprps, i;

	sg = nth_sg(sg, iod->end_sg_index);
	dma_len = sg_dma_len(sg) - iod->end_sg_offset;
	dma_addr = sg_dma_address(sg) + iod->end_sg_offset;
	offset = offset_in_page(dma_addr);

	prev_dma_len = dma_len + offset;

	prp_len = min(PAGE_SIZE - offset, dma_len);
	cmd->prp1 = cpu_to_le64(dma_addr);
	length -= prp_len;
	if (length == 0) {
		iod->end_sg_offset = 0;
		return total_len;
	}

	total_prp_len += prp_len;
	lba_addr_off += prp_len;
	dma_len -= prp_len;
	if (dma_len) {
		iod->end_sg_offset = prp_len;
		dma_addr += (PAGE_SIZE - offset);
	} else {
		iod->end_sg_offset = 0;
		iod->end_sg_index++;
		sg = sg_next(sg);
		dma_addr = sg_dma_address(sg);
		dma_len = sg_dma_len(sg);
		if (offset_in_page(dma_addr) || (prev_dma_len % PAGE_SIZE))
			return total_prp_len;
	}

	prp_len = min(PAGE_SIZE, dma_len);
	if (length <= PAGE_SIZE) {
		cmd->prp2 = cpu_to_le64(dma_addr);
		iod->end_sg_offset = 0;
		return total_len;
	}

	nprps = DIV_ROUND_UP(length, PAGE_SIZE);
	if (nprps <= (256 / 8)) {
		pool = dev->prp_small_pool;
		iod->npages = 0;
	} else {
		pool = dev->prp_page_pool;
		iod->npages = 1;
	}

	prp_list = dma_pool_alloc(pool, gfp, &prp_dma);
	if (!prp_list) {
		cmd->prp2 = cpu_to_le64(dma_addr);
		iod->npages = -1;
		return (total_len - length) + PAGE_SIZE;
	}
	list[0] = prp_list;
	iod->first_dma = prp_dma;
	cmd->prp2 = cpu_to_le64(prp_dma);
	i = 0;
	for (;;) {
		if (i == PAGE_SIZE / 8) {
			__le64 *old_prp_list = prp_list;
			prp_list = dma_pool_alloc(pool, gfp, &prp_dma);
			if (!prp_list)
				return total_len - length;
			list[iod->npages++] = prp_list;
			prp_list[0] = old_prp_list[i - 1];
			old_prp_list[i - 1] = cpu_to_le64(prp_dma);
			i = 1;
		}
		prp_list[i++] = cpu_to_le64(dma_addr);
		prp_len = min(PAGE_SIZE, dma_len);
		iod->end_sg_offset += prp_len;
		lba_addr_off += prp_len;
		total_prp_len += prp_len;
		prev_dma_len = dma_len;

		dma_len -= PAGE_SIZE;
		dma_addr += PAGE_SIZE;
		length -= PAGE_SIZE;
		if (length <= 0)
			break;
		if (dma_len > 0)
			continue;

		iod->end_sg_offset = 0;
		iod->end_sg_index++;
		sg = sg_next(sg);
		dma_addr = sg_dma_address(sg);
		dma_len = sg_dma_len(sg);

		if (offset_in_page(dma_addr) || (prev_dma_len % PAGE_SIZE))
			return total_prp_len;
	}

	return total_len;
}

static int nvme_submit_flush(struct nvme_queue *nvmeq, struct nvme_ns *ns,
								int cmdid)
{
	struct nvme_command *cmnd = &nvmeq->sq_cmds[nvmeq->sq_tail];

	memset(cmnd, 0, sizeof(*cmnd));
	cmnd->common.opcode = nvme_cmd_flush;
	cmnd->common.command_id = cmdid;
	cmnd->common.nsid = cpu_to_le32(ns->ns_id);

	if (++nvmeq->sq_tail == nvmeq->q_depth)
		nvmeq->sq_tail = 0;
	writel(nvmeq->sq_tail, nvmeq->q_db);

	return 0;
}

int nvme_submit_flush_data(struct nvme_queue *nvmeq, struct nvme_ns *ns)
{
	int cmdid = alloc_cmdid(nvmeq, (void *)CMD_CTX_FLUSH,
					special_completion, NVME_IO_TIMEOUT);
	if (unlikely(cmdid < 0))
		return cmdid;

	return nvme_submit_flush(nvmeq, ns, cmdid);
}

/*
 * Called with local interrupts disabled and the q_lock held.  May not sleep.
 */
static int nvme_submit_bio_queue(struct nvme_queue *nvmeq, struct nvme_ns *ns,
				struct bio *bio, struct request *req, int start_sg_index,
				int start_sg_offset, int start_len_completed)
{
	struct nvme_command *cmnd;
	struct nvme_iod *iod;
	int cmdid, length, result = -ENOMEM;
	u16 control;
	u32 dsmgmt;
	struct scatterlist *sglist, *sg;
	unsigned long lba_addr_offset = (bio->bi_sector << 9) + start_len_completed;
	sector_t sector = lba_addr_offset >> 9;
	length = bio->bi_size - start_len_completed;

	vmklnx_blk_rq_map_sg(req, &sglist);
	iod = nvme_alloc_iod(bio->bi_size, GFP_ATOMIC);
	if (!iod)
		goto nomem;
	iod->private = bio;
	iod->req = req;
	iod->sg = sglist;
	iod->start_sg_index = start_sg_index;
	iod->start_sg_offset = start_sg_offset;
	iod->start_len_completed = start_len_completed;
	iod->end_sg_index = start_sg_index;
	iod->end_sg_offset = start_sg_offset;
	iod->end_len_completed = start_len_completed;

	result = -EBUSY;
	cmdid = alloc_cmdid(nvmeq, iod, bio_completion, NVME_IO_TIMEOUT);
	if (unlikely(cmdid < 0))
		goto free_iod;

	if ((bio->bi_rw & REQ_FLUSH))
		return nvme_submit_flush(nvmeq, ns, cmdid);

	control = 0;
	if (bio->bi_rw & REQ_FUA)
		control |= NVME_RW_FUA;

	dsmgmt = 0;
	cmnd = &nvmeq->sq_cmds[nvmeq->sq_tail];

	memset(cmnd, 0, sizeof(*cmnd));
	if (bio_data_dir(bio))
		cmnd->rw.opcode = nvme_cmd_write;
	else
		cmnd->rw.opcode = nvme_cmd_read;

	cmnd->rw.command_id = cmdid;
	cmnd->rw.nsid = cpu_to_le32(ns->ns_id);
	length = nvme_setup_prps(nvmeq->dev, &cmnd->common, iod, length,
						GFP_ATOMIC, lba_addr_offset);

	cmnd->rw.slba = cpu_to_le64(sector >> (ns->lba_shift - 9));
	cmnd->rw.length = cpu_to_le16((length >> ns->lba_shift) - 1);
	cmnd->rw.control = cpu_to_le16(control);
	cmnd->rw.dsmgmt = cpu_to_le32(dsmgmt);

	sg_reset(sglist);
	iod->end_len_completed += length;

	if (++nvmeq->sq_tail == nvmeq->q_depth)
		nvmeq->sq_tail = 0;
	writel(nvmeq->sq_tail, nvmeq->q_db);

	return 0;

 free_iod:
	nvme_free_iod(nvmeq->dev, iod);
 nomem:
	return result;
}

static const enum dma_data_direction nvme_to_direction[] = {
	DMA_NONE, DMA_TO_DEVICE, DMA_FROM_DEVICE, DMA_BIDIRECTIONAL
};

/*
 * NB: return value of non-zero would mean that we were a stacking driver.
 * make_request must always succeed.
 */
static int nvme_make_request(struct request_queue *q, struct request *req)
{
	struct nvme_ns *ns = q->queuedata;
	struct nvme_queue *nvmeq = get_nvmeq(ns->dev);
	struct bio *bio = req->bio;
	int result;

	spin_lock_irq(&nvmeq->q_lock);
	result = nvme_submit_bio_queue(nvmeq, ns, bio, req, 0, 0, 0);
	spin_unlock_irq(&nvmeq->q_lock);
	put_nvmeq(nvmeq);

	return result;
}

static void nvme_request(struct request_queue *q)
{
	struct request *req;
	while((req = elv_next_request(q)) != NULL) {
		if (nvme_make_request(q, req)) {
			blk_stop_queue(q);
			return;
		}
		blkdev_dequeue_request(req);
	}
}

static int nvme_process_cq(struct nvme_queue *nvmeq)
{
	u16 head, phase;

	head = nvmeq->cq_head;
	phase = nvmeq->cq_phase;

	for (;;) {
		void *ctx;
		nvme_completion_fn fn;
		struct nvme_completion cqe = nvmeq->cqes[head];
		if ((le16_to_cpu(cqe.status) & 1) != phase)
			break;
		nvmeq->sq_head = le16_to_cpu(cqe.sq_head);
		if (++head == nvmeq->q_depth) {
			head = 0;
			phase = !phase;
		}

		ctx = free_cmdid(nvmeq, cqe.command_id, &fn);
		fn(nvmeq, ctx, &cqe);
	}

	/* If the controller ignores the cq head doorbell and continuously
	 * writes to the queue, it is theoretically possible to wrap around
	 * the queue twice and mistakenly return IRQ_NONE.  Linux only
	 * requires that 0.1% of your interrupts are handled, so this isn't
	 * a big problem.
	 */
	if (head == nvmeq->cq_head && phase == nvmeq->cq_phase)
		return 0;

	writel(head, nvmeq->q_db + (1 << nvmeq->dev->db_stride));
	nvmeq->cq_head = head;
	nvmeq->cq_phase = phase;

	nvmeq->cqe_seen = 1;
	return 1;
}

static irqreturn_t nvme_irq(int irq, void *data)
{
	irqreturn_t result;
	struct nvme_queue *nvmeq = data;
	spin_lock(&nvmeq->q_lock);
	nvme_process_cq(nvmeq);
	result = nvmeq->cqe_seen ? IRQ_HANDLED : IRQ_NONE;
	nvmeq->cqe_seen = 0;
	spin_unlock(&nvmeq->q_lock);
	return result;
}

static void nvme_abort_command(struct nvme_queue *nvmeq, int cmdid)
{
	spin_lock_irq(&nvmeq->q_lock);
	cancel_cmdid(nvmeq, cmdid, NULL);
	spin_unlock_irq(&nvmeq->q_lock);
}

struct sync_cmd_info {
	struct task_struct *task;
	u32 result;
	int status;
};

static void sync_completion(struct nvme_queue *queue, void *ctx,
						struct nvme_completion *cqe)
{
	struct sync_cmd_info *cmdinfo = ctx;
	cmdinfo->result = le32_to_cpup(&cqe->result);
	cmdinfo->status = le16_to_cpup(&cqe->status) >> 1;
	wake_up_process(cmdinfo->task);
}

/*
 * Returns 0 on success.  If the result is negative, it's a Linux error code;
 * if the result is positive, it's an NVM Express status code
 */
static int nvme_submit_sync_cmd(struct nvme_queue *nvmeq,
			struct nvme_command *cmd, u32 *result, unsigned timeout)
{
	int cmdid;
	struct sync_cmd_info cmdinfo;

	cmdinfo.task = current;
	cmdinfo.status = -EINTR;

	cmdid = alloc_cmdid_killable(nvmeq, &cmdinfo, sync_completion,
								timeout);
	if (cmdid < 0)
		return cmdid;
	cmd->common.command_id = cmdid;

	set_current_state(TASK_UNINTERRUPTIBLE);
	nvme_submit_cmd(nvmeq, cmd);
	schedule_timeout(timeout);

	if (cmdinfo.status == -EINTR) {
		nvme_abort_command(nvmeq, cmdid);
		return -EINTR;
	}

	if (result)
		*result = cmdinfo.result;

	return cmdinfo.status;
}

int nvme_submit_admin_cmd(struct nvme_dev *dev, struct nvme_command *cmd,
								u32 *result)
{
	return nvme_submit_sync_cmd(dev->queues[0], cmd, result, ADMIN_TIMEOUT);
}

static int adapter_delete_queue(struct nvme_dev *dev, u8 opcode, u16 id)
{
	int status;
	struct nvme_command c;

	memset(&c, 0, sizeof(c));
	c.delete_queue.opcode = opcode;
	c.delete_queue.qid = cpu_to_le16(id);

	status = nvme_submit_admin_cmd(dev, &c, NULL);
	if (status)
		return -EIO;
	return 0;
}

static int adapter_alloc_cq(struct nvme_dev *dev, u16 qid,
						struct nvme_queue *nvmeq)
{
	int status;
	struct nvme_command c;
	int flags = NVME_QUEUE_PHYS_CONTIG | NVME_CQ_IRQ_ENABLED;

	memset(&c, 0, sizeof(c));
	c.create_cq.opcode = nvme_admin_create_cq;
	c.create_cq.prp1 = cpu_to_le64(nvmeq->cq_dma_addr);
	c.create_cq.cqid = cpu_to_le16(qid);
	c.create_cq.qsize = cpu_to_le16(nvmeq->q_depth - 1);
	c.create_cq.cq_flags = cpu_to_le16(flags);
	c.create_cq.irq_vector = cpu_to_le16(nvmeq->cq_vector);

	status = nvme_submit_admin_cmd(dev, &c, NULL);
	if (status)
		return -EIO;
	return 0;
}

static int adapter_alloc_sq(struct nvme_dev *dev, u16 qid,
						struct nvme_queue *nvmeq)
{
	int status;
	struct nvme_command c;
	int flags = NVME_QUEUE_PHYS_CONTIG | NVME_SQ_PRIO_MEDIUM;

	memset(&c, 0, sizeof(c));
	c.create_sq.opcode = nvme_admin_create_sq;
	c.create_sq.prp1 = cpu_to_le64(nvmeq->sq_dma_addr);
	c.create_sq.sqid = cpu_to_le16(qid);
	c.create_sq.qsize = cpu_to_le16(nvmeq->q_depth - 1);
	c.create_sq.sq_flags = cpu_to_le16(flags);
	c.create_sq.cqid = cpu_to_le16(qid);

	status = nvme_submit_admin_cmd(dev, &c, NULL);
	if (status)
		return -EIO;
	return 0;
}

static int adapter_delete_cq(struct nvme_dev *dev, u16 cqid)
{
	return adapter_delete_queue(dev, nvme_admin_delete_cq, cqid);
}

static int adapter_delete_sq(struct nvme_dev *dev, u16 sqid)
{
	return adapter_delete_queue(dev, nvme_admin_delete_sq, sqid);
}

int nvme_identify(struct nvme_dev *dev, unsigned nsid, unsigned cns,
							dma_addr_t dma_addr)
{
	struct nvme_command c;

	memset(&c, 0, sizeof(c));
	c.identify.opcode = nvme_admin_identify;
	c.identify.nsid = cpu_to_le32(nsid);
	c.identify.prp1 = cpu_to_le64(dma_addr);
	c.identify.cns = cpu_to_le32(cns);

	return nvme_submit_admin_cmd(dev, &c, NULL);
}

int nvme_get_features(struct nvme_dev *dev, unsigned fid, unsigned nsid,
			dma_addr_t dma_addr, u32 *result)
{
	struct nvme_command c;

	memset(&c, 0, sizeof(c));
	c.features.opcode = nvme_admin_get_features;
	c.features.nsid = cpu_to_le32(nsid);
	c.features.prp1 = cpu_to_le64(dma_addr);
	c.features.fid = cpu_to_le32(fid);

	return nvme_submit_admin_cmd(dev, &c, result);
}

int nvme_set_features(struct nvme_dev *dev, unsigned fid,
			unsigned dword11, dma_addr_t dma_addr, u32 *result)
{
	struct nvme_command c;

	memset(&c, 0, sizeof(c));
	c.features.opcode = nvme_admin_set_features;
	c.features.prp1 = cpu_to_le64(dma_addr);
	c.features.fid = cpu_to_le32(fid);
	c.features.dword11 = cpu_to_le32(dword11);

	return nvme_submit_admin_cmd(dev, &c, result);
}

/**
 * nvme_cancel_ios - Cancel outstanding I/Os
 * @queue: The queue to cancel I/Os on
 * @timeout: True to only cancel I/Os which have timed out
 */
static void nvme_cancel_ios(struct nvme_queue *nvmeq, bool timeout)
{
	int depth = nvmeq->q_depth - 1;
	struct nvme_cmd_info *info = nvme_cmd_info(nvmeq);
	unsigned long now = jiffies;
	int cmdid;

	for_each_set_bit(cmdid, nvmeq->cmdid_data, depth) {
		void *ctx;
		nvme_completion_fn fn;
		static struct nvme_completion cqe = {
			.status = cpu_to_le16(NVME_SC_ABORT_REQ << 1),
		};

		if (timeout && !time_after(now, info[cmdid].timeout))
			continue;
		if (info[cmdid].ctx == CMD_CTX_CANCELLED)
			continue;
		dev_warn(nvmeq->q_dmadev, "Cancelling I/O %d\n", cmdid);
		ctx = cancel_cmdid(nvmeq, cmdid, &fn);
		fn(nvmeq->dev, ctx, &cqe);
	}
}

static void nvme_free_queue_mem(struct nvme_queue *nvmeq)
{
	dma_free_coherent(nvmeq->q_dmadev, CQ_SIZE(nvmeq->q_depth),
				(void *)nvmeq->cqes, nvmeq->cq_dma_addr);
	dma_free_coherent(nvmeq->q_dmadev, SQ_SIZE(nvmeq->q_depth),
					nvmeq->sq_cmds, nvmeq->sq_dma_addr);
	kfree(nvmeq);
}

static void nvme_free_queue(struct nvme_dev *dev, int qid)
{
	struct nvme_queue *nvmeq = dev->queues[qid];
	int vector = dev->entry[nvmeq->cq_vector].vector;

	spin_lock_irq(&nvmeq->q_lock);
	nvme_cancel_ios(nvmeq, false);
	spin_unlock_irq(&nvmeq->q_lock);

	free_irq(vector, nvmeq);

	/* Don't tell the adapter to delete the admin queue */
	if (qid) {
		adapter_delete_sq(dev, qid);
		adapter_delete_cq(dev, qid);
	}

	nvme_free_queue_mem(nvmeq);
}

static struct nvme_queue *nvme_alloc_queue(struct nvme_dev *dev, int qid,
							int depth, int vector)
{
	struct device *dmadev = &dev->pci_dev->dev;
	unsigned extra = DIV_ROUND_UP(depth, 8) + (depth *
						sizeof(struct nvme_cmd_info));
	struct nvme_queue *nvmeq = kzalloc(sizeof(*nvmeq) + extra, GFP_KERNEL);
	if (!nvmeq)
		return NULL;

	nvmeq->cqes = dma_alloc_coherent(dmadev, CQ_SIZE(depth),
					&nvmeq->cq_dma_addr, GFP_KERNEL);
	if (!nvmeq->cqes)
		goto free_nvmeq;
	memset((void *)nvmeq->cqes, 0, CQ_SIZE(depth));

	nvmeq->sq_cmds = dma_alloc_coherent(dmadev, SQ_SIZE(depth),
					&nvmeq->sq_dma_addr, GFP_KERNEL);
	if (!nvmeq->sq_cmds)
		goto free_cqdma;

	nvmeq->q_dmadev = dmadev;
	nvmeq->dev = dev;
	spin_lock_init(&nvmeq->q_lock);
	nvmeq->cq_head = 0;
	nvmeq->cq_phase = 1;
	nvmeq->q_db = &dev->dbs[qid << (dev->db_stride + 1)];
	nvmeq->q_depth = depth;
	nvmeq->cq_vector = vector;
	nvmeq->qid = qid;

	return nvmeq;

 free_cqdma:
	dma_free_coherent(dmadev, CQ_SIZE(depth), (void *)nvmeq->cqes,
							nvmeq->cq_dma_addr);
 free_nvmeq:
	kfree(nvmeq);
	return NULL;
}

static int queue_request_irq(struct nvme_dev *dev, struct nvme_queue *nvmeq,
							const char *name)
{
	return request_irq(dev->entry[nvmeq->cq_vector].vector, nvme_irq,
				IRQF_DISABLED | IRQF_SHARED, name, nvmeq);
}

static __devinit struct nvme_queue *nvme_create_queue(struct nvme_dev *dev,
					int qid, int cq_size, int vector)
{
	int result;
	struct nvme_queue *nvmeq = nvme_alloc_queue(dev, qid, cq_size, vector);

	if (!nvmeq)
		return ERR_PTR(-ENOMEM);

	result = adapter_alloc_cq(dev, qid, nvmeq);
	if (result < 0)
		goto free_nvmeq;

	result = adapter_alloc_sq(dev, qid, nvmeq);
	if (result < 0)
		goto release_cq;

	result = queue_request_irq(dev, nvmeq, "nvme");
	if (result < 0)
		goto release_sq;

	return nvmeq;

 release_sq:
	adapter_delete_sq(dev, qid);
 release_cq:
	adapter_delete_cq(dev, qid);
 free_nvmeq:
	dma_free_coherent(nvmeq->q_dmadev, CQ_SIZE(nvmeq->q_depth),
				(void *)nvmeq->cqes, nvmeq->cq_dma_addr);
	dma_free_coherent(nvmeq->q_dmadev, SQ_SIZE(nvmeq->q_depth),
					nvmeq->sq_cmds, nvmeq->sq_dma_addr);
	kfree(nvmeq);
	return ERR_PTR(result);
}

static int nvme_wait_ready(struct nvme_dev *dev, u64 cap, bool enabled)
{
	unsigned long timeout;
	u32 bit = enabled ? NVME_CSTS_RDY : 0;

	timeout = ((NVME_CAP_TIMEOUT(cap) + 1) * HZ / 2) + jiffies;

	while ((readl(&dev->bar->csts) & NVME_CSTS_RDY) != bit) {
		msleep(100);
		if (signal_pending(current))
			flush_signals(current);
		if (time_after(jiffies, timeout)) {
			dev_err(&dev->pci_dev->dev,
				"Device not ready; aborting initialisation\n");
			return -ENODEV;
		}
	}

	return 0;
}

/*
 * If the device has been passed off to us in an enabled state, just clear
 * the enabled bit.  The spec says we should set the 'shutdown notification
 * bits', but doing so may cause the device to complete commands to the
 * admin queue ... and we don't know what memory that might be pointing at!
 */
static int nvme_disable_ctrl(struct nvme_dev *dev, u64 cap)
{
	u32 cc = readl(&dev->bar->cc);

	if (cc & NVME_CC_ENABLE)
		writel(cc & ~NVME_CC_ENABLE, &dev->bar->cc);
	return nvme_wait_ready(dev, cap, false);
}

static int nvme_enable_ctrl(struct nvme_dev *dev, u64 cap)
{
	return nvme_wait_ready(dev, cap, true);
}

static int __devinit nvme_configure_admin_queue(struct nvme_dev *dev)
{
	int result;
	u32 aqa;
	u64 cap = readq(&dev->bar->cap);
	struct nvme_queue *nvmeq;

	dev->dbs = ((void __iomem *)dev->bar) + 4096;
	dev->db_stride = NVME_CAP_STRIDE(cap);

	if (((PAGE_SHIFT - 12) < NVME_CAP_MPSMIN(cap)) ||
			((PAGE_SHIFT - 12) > NVME_CAP_MPSMAX(cap))) {
		printk(KERN_WARNING "nvme: This device does not support a"
				"page size of %d\n", (1 << PAGE_SHIFT));
		return -EINVAL;
	}

	result = nvme_disable_ctrl(dev, cap);
	if (result < 0)
		return result;

	nvmeq = nvme_alloc_queue(dev, 0, 64, 0);
	if (!nvmeq)
		return -ENOMEM;

	aqa = nvmeq->q_depth - 1;
	aqa |= aqa << 16;

	dev->ctrl_config = NVME_CC_ENABLE | NVME_CC_CSS_NVM;
	dev->ctrl_config |= (PAGE_SHIFT - 12) << NVME_CC_MPS_SHIFT;
	dev->ctrl_config |= NVME_CC_ARB_RR | NVME_CC_SHN_NONE;
	dev->ctrl_config |= NVME_CC_IOSQES | NVME_CC_IOCQES;

	writel(aqa, &dev->bar->aqa);
	writeq(nvmeq->sq_dma_addr, &dev->bar->asq);
	writeq(nvmeq->cq_dma_addr, &dev->bar->acq);
	writel(dev->ctrl_config, &dev->bar->cc);

	result = nvme_enable_ctrl(dev, cap);
	if (result)
		goto free_q;

	result = queue_request_irq(dev, nvmeq, "nvme admin");
	if (result)
		goto free_q;

	dev->queues[0] = nvmeq;
	return result;

 free_q:
	nvme_free_queue_mem(nvmeq);
	return result;
}

static int nvme_submit_io(struct nvme_ns *ns, struct nvme_user_io __user *uio)
{
	struct nvme_dev *dev = ns->dev;
	struct nvme_queue *nvmeq;
	struct nvme_user_io io;
	struct nvme_command c;
	unsigned length;
	int status;
	void* mem;
	dma_addr_t dma_addr;

	if (copy_from_user(&io, uio, sizeof(io)))
		return -EFAULT;
	length = (io.nblocks + 1) << ns->lba_shift;

	switch (io.opcode) {
	case nvme_cmd_write:
	case nvme_cmd_read:
	case nvme_cmd_compare:
		break;
	default:
		return -EINVAL;
	}

	if (io.addr & 3)
		return -EINVAL;
	if (!length || length != PAGE_SIZE)
		return -EINVAL;
	mem = dma_pool_alloc(dev->prp_page_pool, GFP_KERNEL, &dma_addr);
	if(!mem)
		return -EINVAL;

	memset(&c, 0, sizeof(c));
	c.rw.opcode = io.opcode;
	c.rw.flags = io.flags;
	c.rw.nsid = cpu_to_le32(ns->ns_id);
	c.rw.slba = cpu_to_le64(io.slba);
	c.rw.length = cpu_to_le16(io.nblocks);
	c.rw.control = cpu_to_le16(io.control);
	c.rw.dsmgmt = cpu_to_le16(io.dsmgmt);
	c.rw.reftag = io.reftag;
	c.rw.apptag = io.apptag;
	c.rw.appmask = io.appmask;
	c.rw.prp1 = dma_addr;
	/* XXX: metadata */
	if(io.opcode & 1)
		copy_from_user(mem, io.addr, PAGE_SIZE);

	nvmeq = get_nvmeq(dev);
	/*
	 * Since nvme_submit_sync_cmd sleeps, we can't keep preemption
	 * disabled.  We may be preempted at any point, and be rescheduled
	 * to a different CPU.  That will cause cacheline bouncing, but no
	 * additional races since q_lock already protects against other CPUs.
	 */
	put_nvmeq(nvmeq);

	if (length != (io.nblocks + 1) << ns->lba_shift)
		status = -ENOMEM;
	else
		status = nvme_submit_sync_cmd(nvmeq, &c, NULL, NVME_IO_TIMEOUT);

	if(!(io.opcode & 1))
		copy_to_user(io.addr, mem, PAGE_SIZE);
	dma_pool_free(dev->prp_page_pool, mem, dma_addr);
	return status;
}

static int nvme_user_admin_cmd(struct nvme_dev *dev,
					struct nvme_admin_cmd __user *ucmd)
{
	struct nvme_admin_cmd cmd;
	struct nvme_command c;
	int status;
	unsigned timeout;
	void* mem;
	dma_addr_t dma_addr;

	if (!capable(CAP_SYS_ADMIN))
		return -EACCES;
	if (copy_from_user(&cmd, ucmd, sizeof(cmd)))
		return -EFAULT;

	memset(&c, 0, sizeof(c));
	c.common.opcode = cmd.opcode;
	c.common.flags = cmd.flags;
	c.common.nsid = cpu_to_le32(cmd.nsid);
	c.common.cdw2[0] = cpu_to_le32(cmd.cdw2);
	c.common.cdw2[1] = cpu_to_le32(cmd.cdw3);
	c.common.cdw10[0] = cpu_to_le32(cmd.cdw10);
	c.common.cdw10[1] = cpu_to_le32(cmd.cdw11);
	c.common.cdw10[2] = cpu_to_le32(cmd.cdw12);
	c.common.cdw10[3] = cpu_to_le32(cmd.cdw13);
	c.common.cdw10[4] = cpu_to_le32(cmd.cdw14);
	c.common.cdw10[5] = cpu_to_le32(cmd.cdw15);

	if (cmd.data_len) {
		mem = dma_pool_alloc(dev->prp_page_pool, GFP_KERNEL, &dma_addr);
		c.common.prp1 = dma_addr;
	}

	status = nvme_submit_admin_cmd(dev, &c, &cmd.result);
	if (cmd.data_len) {
		copy_to_user(cmd.addr, mem, cmd.data_len);
		dma_pool_free(dev->prp_page_pool, mem, dma_addr);
	}
	if (copy_to_user(&ucmd->result, &cmd.result, sizeof(cmd.result)))
		return -EFAULT;

	return status;
}

static int nvme_process_ioctl(struct nvme_dev *dev, unsigned int cmd,
							unsigned long arg)
{
	switch (cmd) {
	case NVME_IOCTL_ADMIN_CMD:
		return nvme_user_admin_cmd(dev, (void __user *)arg);
	case NVME_IOCTL_SPEC_VERSION:
		return readl(&dev->bar->vs);
	case NVME_IOCTL_COMMAND_SET:
		return NVME_CC_CSS(readl(&dev->bar->cc));
	case NVME_IOCTL_ARBITRATION_SELECTED:
		return NVME_CC_AMS(readl(&dev->bar->cc));
	case NVME_IOCTL_READY_TIMEOUT:
		return (int)NVME_CAP_TIMEOUT(readq(&dev->bar->cap));
	case NVME_IOCTL_DEVICE_INSTANCE:
		return dev->instance;
	case NVME_IOCTL_RESET_CONTROLLER:
		return nvme_reset_controller(dev);
	default:
		return -ENOTTY;
	}
}

static int nvme_ioctl(struct inode *inode, struct file* f, unsigned cmd, unsigned long arg)
{
	struct block_device *bdev = inode->i_bdev;
	struct nvme_ns *ns = bdev->bd_disk->private_data;
	struct nvme_dev *dev = ns->dev;

	switch (cmd) {
	case NVME_IOCTL_ID:
		return ns->ns_id;
	case NVME_IOCTL_SUBMIT_IO:
		return nvme_submit_io(ns, (void __user *)arg);
	default:
		return nvme_process_ioctl(dev, cmd, arg);
	}
}

int nvme_dev_char_ioctl(struct inode *inode, struct file *f, unsigned int cmd, unsigned long arg);
int nvme_char_ioctl(struct inode *inode, struct file *f, unsigned int cmd, unsigned long arg);

int nvme_char_open(struct inode *inode, struct file *f)
{
	return 0;
}

static struct file_operations nvme_char_fops = {
	.owner = THIS_MODULE,
	.open = nvme_char_open,
	.ioctl = nvme_char_ioctl
};

static struct file_operations nvme_dev_char_fops = {
	.owner = THIS_MODULE,
	.open = nvme_char_open,
	.ioctl = nvme_dev_char_ioctl
};

static const struct block_device_operations nvme_fops = {
	.owner		= THIS_MODULE,
	.ioctl		= nvme_ioctl,
	.compat_ioctl	= nvme_ioctl,
};

static int nvme_kthread(void *data)
{
	struct nvme_dev *dev;

	while (!kthread_should_stop()) {
		set_current_state(TASK_INTERRUPTIBLE);
		spin_lock(&dev_list_lock);
		list_for_each_entry(dev, &dev_list, node) {
			int i;
			for (i = 0; i < dev->queue_count; i++) {
				struct nvme_queue *nvmeq = dev->queues[i];
				if (!nvmeq)
					continue;
				spin_lock_irq(&nvmeq->q_lock);
				nvme_process_cq(nvmeq);
				nvme_cancel_ios(nvmeq, true);
				spin_unlock_irq(&nvmeq->q_lock);
			}
		}
		spin_unlock(&dev_list_lock);
		schedule_timeout(round_jiffies_relative(HZ));
	}
	return 0;
}

static struct nvme_ns *nvme_alloc_ns(struct nvme_dev *dev, int nsid,
			struct nvme_id_ns *id, struct nvme_lba_range_type *rt)
{
	struct nvme_ns *ns;
	struct gendisk *disk;
	int lbaf;

	if (rt->attributes & NVME_LBART_ATTRIB_HIDE)
		return NULL;

	ns = kzalloc(sizeof(*ns), GFP_KERNEL);
	if (!ns)
		return NULL;
	spin_lock_init(&ns->ns_lock);
	ns->queue = blk_init_queue(nvme_request, &ns->ns_lock);
	if (!ns->queue)
		goto out_free_ns;

	ns->dev = dev;
	ns->queue->queuedata = ns;

	disk = alloc_disk(NVME_MINORS);
	if (!disk)
		goto out_free_queue;
	ns->ns_id = nsid;
	ns->disk = disk;
	lbaf = id->flbas & 0xf;
	ns->lba_shift = id->lbaf[lbaf].ds;

	disk->major = dev->nvme_major;
	disk->minors = NVME_MINORS;
	disk->first_minor = NVME_MINORS * (nsid - 1);
	if (dev->max_hw_sectors)
		disk->maxXfer = dev->max_hw_sectors * VMK_SECTOR_SIZE;
	disk->fops = &nvme_fops;
	disk->private_data = ns;
	disk->queue = ns->queue;
	disk->driverfs_dev = &dev->pci_dev->dev;
	sprintf(disk->disk_name, "nvme%dn%d", dev->instance, nsid);
	set_capacity(disk, le64_to_cpup(&id->nsze) << (ns->lba_shift - 9));

	blk_queue_hardsect_size(ns->queue, 1 << ns->lba_shift);

	/* char device for management interface */
	char charname[15];
	memset(charname, 0x0, 15);
	snprintf(charname, 14, "nvme%dn%d", dev->instance, ns->ns_id);
	devmajors[dev->instance][ns->ns_id].char_major = register_chrdev(0, charname, &nvme_char_fops);
	if (devmajors[dev->instance][ns->ns_id].char_major < 0) {
		printk(KERN_WARNING "nvme: failed to register char device %s\n",
		charname);
	}
	devmajors[dev->instance][ns->ns_id].block_major = dev->nvme_major;
	devmajors[dev->instance][ns->ns_id].block_minor = disk->first_minor;
	devmajors[dev->instance][ns->ns_id].bd_disk = disk;

	blk_queue_softirq_done(disk->queue, bio_completion_softirq);
	return ns;

 out_free_queue:
	blk_cleanup_queue(ns->queue);
 out_free_ns:
	kfree(ns);
	return NULL;
}

static void nvme_ns_free(struct nvme_ns *ns)
{
	int index = ns->disk->first_minor / NVME_MINORS;
	put_disk(ns->disk);
	blk_cleanup_queue(ns->queue);
	kfree(ns);
}

static int set_queue_count(struct nvme_dev *dev, int count)
{
	int status;
	u32 result;
	u32 q_count = (count - 1) | ((count - 1) << 16);

	status = nvme_set_features(dev, NVME_FEAT_NUM_QUEUES, q_count, 0,
								&result);
	if (status)
		return -EIO;
	return min(result & 0xffff, result >> 16) + 1;
}

static void nvme_free_queues(struct nvme_dev *dev)
{
	int i;

	for (i = dev->queue_count - 1; i >= 0; i--)
		nvme_free_queue(dev, i);
	dev->queue_count = 0;
}

static int __devinit nvme_setup_io_queues(struct nvme_dev *dev)
{
	struct pci_dev *pdev = dev->pci_dev;
	int result, cpu, i, vecs, nr_io_queues, db_bar_size, q_depth;

	nr_io_queues = num_online_cpus();
	result = set_queue_count(dev, nr_io_queues);
	if (result < 0)
		return result;
	if (result < nr_io_queues)
		nr_io_queues = result;

	/* Deregister the admin queue's interrupt */
	free_irq(dev->entry[0].vector, dev->queues[0]);

	db_bar_size = 4096 + ((nr_io_queues + 1) << (dev->db_stride + 3));
	if (db_bar_size > 8192) {
		iounmap(dev->bar);
		dev->bar = ioremap(pci_resource_start(pdev, 0), db_bar_size);
		if (!dev->bar)
			return -ENOMEM;
		dev->dbs = ((void __iomem *)dev->bar) + 4096;
		dev->queues[0]->q_db = dev->dbs;
	}

	vecs = nr_io_queues;
	for (i = 0; i < vecs; i++)
		dev->entry[i].entry = i;
	for (;;) {
		result = pci_enable_msix(pdev, dev->entry, vecs);
		if (result <= 0)
			break;
		vecs = result;
	}

	if (result < 0)
		vecs = 1;

	/*
	 * Should investigate if there's a performance win from allocating
	 * more queues than interrupt vectors; it might allow the submission
	 * path to scale better, even if the receive path is limited by the
	 * number of interrupts.
	 */
	nr_io_queues = vecs;

	result = queue_request_irq(dev, dev->queues[0], "nvme admin");
	/* XXX: handle failure here */

	q_depth = min_t(int, NVME_CAP_MQES(readq(&dev->bar->cap)) + 1,
								NVME_Q_DEPTH);
	for (i = 0; i < nr_io_queues; i++) {
		dev->queues[i + 1] = nvme_create_queue(dev, i + 1, q_depth, i);
		if (IS_ERR(dev->queues[i + 1])) {
			result = PTR_ERR(dev->queues[i + 1]);
			goto free_queues;
		}
		dev->queue_count++;
	}

	for (; i < num_possible_cpus(); i++) {
		int target = i % rounddown_pow_of_two(dev->queue_count - 1);
		dev->queues[i + 1] = dev->queues[target + 1];
	}

	return 0;
 free_queues:
	nvme_free_queues(dev);
	return result;
}

static int __devinit nvme_dev_add(struct nvme_dev *dev)
{
	int res, nn, i;
	struct nvme_ns *ns, *next;
	struct nvme_id_ctrl *ctrl;
	struct nvme_id_ns *id_ns;
	void *mem;
	dma_addr_t dma_addr;
	int shift = NVME_CAP_MPSMIN(readq(&dev->bar->cap)) + 12;


	spin_lock(&dev_list_lock);
	list_add(&dev->node, &dev_list);
	spin_unlock(&dev_list_lock);

	res = nvme_setup_io_queues(dev);
	if (res)
		goto del_node;

	mem = dma_alloc_coherent(&dev->pci_dev->dev, 8192, &dma_addr,
								GFP_KERNEL);
	if (!mem)
		return -ENOMEM;

	res = nvme_identify(dev, 0, 1, dma_addr);
	if (res) {
		res = -EIO;
		goto out_free;
	}

	ctrl = mem;
	nn = le32_to_cpup(&ctrl->nn);
	memcpy(dev->serial, ctrl->sn, sizeof(ctrl->sn));
	memcpy(dev->model, ctrl->mn, sizeof(ctrl->mn));
	memcpy(dev->firmware_rev, ctrl->fr, sizeof(ctrl->fr));
	dev->vendor_id = le16_to_cpup(&ctrl->vid);

	if (ctrl->mdts)
		dev->max_hw_sectors = 1 << (ctrl->mdts + shift - 9);
	id_ns = mem;
	for (i = 1; i <= nn; i++) {
		res = nvme_identify(dev, i, 0, dma_addr);
		if (res)
			continue;

		if (id_ns->ncap == 0)
			continue;

		res = nvme_get_features(dev, NVME_FEAT_LBA_RANGE, i,
							dma_addr + 4096, NULL);
		if (res)
			memset(mem + 4096, 0, 4096);

		ns = nvme_alloc_ns(dev, i, mem, mem + 4096);
		if (ns)
			list_add_tail(&ns->list, &dev->namespaces);
	}
	i = 0;
	list_for_each_entry(ns, &dev->namespaces, list) {
		add_disk(ns->disk);
		vmklnx_block_register_ssd(dev->nvme_major, i++);
	}

	dma_free_coherent(&dev->pci_dev->dev, 8192, mem, dma_addr);
	return 0;

 out_free:
	dma_free_coherent(&dev->pci_dev->dev, 8192, mem, dma_addr);
	nvme_free_queues(dev);
 del_node:
	spin_lock(&dev_list_lock);
	list_del(&dev->node);
	spin_unlock(&dev_list_lock);
	return res;
}

static int nvme_dev_remove(struct nvme_dev *dev)
{
	struct nvme_ns *ns, *next;

	spin_lock(&dev_list_lock);
	list_del(&dev->node);
	spin_unlock(&dev_list_lock);

	/* TODO: wait all I/O finished or cancel them */

	list_for_each_entry_safe(ns, next, &dev->namespaces, list) {
		list_del(&ns->list);
		del_gendisk(ns->disk);
		nvme_ns_free(ns);
	}

	nvme_free_queues(dev);
	return 0;
}

static int nvme_reset_controller(struct nvme_dev *dev)
{
	nvme_dev_remove(dev);
	pci_disable_msix(dev->pci_dev);

	dev->entry[0].vector = dev->pci_dev->irq;
	if (nvme_configure_admin_queue(dev))
		return -1;

	dev->queue_count++;
	if (nvme_dev_add(dev)) {
		nvme_free_queues(dev);
		return -1;
	}
	return 0;
}

static int nvme_setup_prp_pools(struct nvme_dev *dev)
{
	struct device *dmadev = &dev->pci_dev->dev;
	dev->prp_page_pool = dma_pool_create("prp list page", dmadev,
						PAGE_SIZE, PAGE_SIZE, 0);
	if (!dev->prp_page_pool)
		return -ENOMEM;

	/* Optimisation for I/Os between 4k and 128k */
	dev->prp_small_pool = dma_pool_create("prp list 256", dmadev,
						256, 256, 0);
	if (!dev->prp_small_pool) {
		dma_pool_destroy(dev->prp_page_pool);
		return -ENOMEM;
	}
	return 0;
}

static void nvme_release_prp_pools(struct nvme_dev *dev)
{
	dma_pool_destroy(dev->prp_page_pool);
	dma_pool_destroy(dev->prp_small_pool);
}

static void nvme_set_instance(struct nvme_dev *dev)
{
	static int instance;
	dev->instance = instance++;
}

static void nvme_release_instance(struct nvme_dev *dev)
{
}

static int __devinit nvme_probe(struct pci_dev *pdev,
						const struct pci_device_id *id)
{
	int bars, result = -ENOMEM;
	struct nvme_dev *dev;

	printk(KERN_DEBUG "999 enter nvme_probe func!\n");
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
	dev->entry = kcalloc(num_possible_cpus(), sizeof(*dev->entry),
								GFP_KERNEL);
	if (!dev->entry)
		goto free;
	dev->queues = kcalloc(num_possible_cpus() + 1, sizeof(void *),
								GFP_KERNEL);
	if (!dev->queues)
		goto free;

	if (pci_enable_device(pdev))
		goto free;
	pci_set_master(pdev);
	if (pci_request_regions(pdev, "nvme"))
		goto disable;
	nvme_set_instance(dev);
	dev->nvme_major = MAX_BLKDEV - dev->instance - 1;
	vmklnx_register_blkdev(dev->nvme_major, "nvme", pci_domain_nr(pdev->bus),
		pdev->bus->number, pdev->devfn, dev);

	INIT_LIST_HEAD(&dev->namespaces);
	dev->pci_dev = pdev;
	pci_set_drvdata(pdev, dev);

	if (!dma_set_mask(&pdev->dev, DMA_BIT_MASK(64)))
		dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(64));
	else if (!dma_set_mask(&pdev->dev, DMA_BIT_MASK(32)))
		dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
	else
		goto disable;

	dev->entry[0].vector = pdev->irq;

	result = nvme_setup_prp_pools(dev);
	if (result)
		goto disable_msix;
	dev->bar = ioremap(pci_resource_start(pdev, 0), 8192);
	if (!dev->bar) {
		result = -ENOMEM;
		goto disable_msix;
	}

	result = nvme_configure_admin_queue(dev);
	if (result)
		goto unmap;
	dev->queue_count++;

	result = nvme_dev_add(dev);
	if (result)
		goto unmap;

	nvme_set_features(dev, NVME_FEAT_SW_PROGRESS, 0, 0, NULL);

	vmklnx_block_register_sglimit(dev->nvme_major, 16);
	vmklnx_block_init_done(dev->nvme_major);

	char charname[15];
	memset(charname, 0x0, 15);
	snprintf(charname, 14, "nvme%d", dev->instance);
	dev->char_major = register_chrdev(0, charname, &nvme_dev_char_fops);
	printk(KERN_DEBUG "999 leave nvme_probe func!\n");
	return 0;

 unmap:
	iounmap(dev->bar);
 disable_msix:
	pci_disable_msix(pdev);
	nvme_release_instance(dev);
	nvme_release_prp_pools(dev);
 disable:
	pci_disable_device(pdev);
	pci_release_regions(pdev);
 free:
	kfree(dev->queues);
	kfree(dev->entry);
	kfree(dev);
	printk(KERN_DEBUG "999 leave nvme_probe func!\n");
	return result;
}

static void __devexit nvme_remove(struct pci_dev *pdev)
{
	struct nvme_dev *dev = pci_get_drvdata(pdev);
	nvme_dev_remove(dev);
	pci_disable_msix(pdev);
	iounmap(dev->bar);
	nvme_release_instance(dev);
	nvme_release_prp_pools(dev);
	pci_disable_device(pdev);
	pci_release_regions(pdev);
	kfree(dev->queues);
	kfree(dev->entry);
	kfree(dev);
}

/* These functions are yet to be implemented */
#define nvme_error_detected NULL
#define nvme_dump_registers NULL
#define nvme_link_reset NULL
#define nvme_slot_reset NULL
#define nvme_error_resume NULL
#define nvme_suspend NULL
#define nvme_resume NULL

static struct pci_error_handlers nvme_err_handler = {
	.error_detected	= nvme_error_detected,
	.mmio_enabled	= nvme_dump_registers,
	.link_reset	= nvme_link_reset,
	.slot_reset	= nvme_slot_reset,
	.resume		= nvme_error_resume,
};

#define PCI_CLASS_STORAGE_EXPRESS	0x010802

const struct pci_device_id nvme_id_table[] = {
	{ PCI_DEVICE_CLASS(PCI_CLASS_STORAGE_EXPRESS, 0xffffff) },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, nvme_id_table);

static struct pci_driver nvme_driver = {
	.name		= "nvme",
	.id_table	= nvme_id_table,
	.probe		= nvme_probe,
	.remove		= __devexit_p(nvme_remove),
	.suspend	= nvme_suspend,
	.resume		= nvme_resume,
	.err_handler	= &nvme_err_handler,
};

int nvme_dev_char_ioctl(struct inode *inode, struct file *f, unsigned int cmd,
							unsigned long arg)
{
	int blkmajor = 0, blkminor = 0, chrmajor = 0;
	struct gendisk *bd_disk;
	struct nvme_dev *dev;

	chrmajor = MAJOR(inode->i_rdev);
	if ( !chrmajor) {
		printk(KERN_WARNING "Invalid Char Device. major: %d\n",
							MAJOR(inode->i_rdev));
		return -ENODEV;
	}

	spin_lock(&dev_list_lock);
	list_for_each_entry(dev, &dev_list, node) {
		if(chrmajor == dev->char_major) {
			break;
		}
	}
	spin_unlock(&dev_list_lock);

	if (&dev->node == &dev_list)
		return -ENOTTY;

	return nvme_process_ioctl(dev, cmd, arg);
}

int nvme_char_ioctl(struct inode *inode, struct file *f, unsigned int cmd,
							unsigned long arg)
{
	int blkmajor = 0, blkminor = 0, chrmajor = 0;
	int i, j;
	struct gendisk *bd_disk;

	chrmajor = MAJOR(inode->i_rdev);
	if ( !chrmajor) {
		printk(KERN_WARNING "Invalid Char Device. major: %d\n",
							MAJOR(inode->i_rdev));
		return -ENODEV;
	}

	for (i = 0; i < 4; i++) {
		for (j = 0; j < 16; j++) {
			if ( devmajors[i][j].char_major == chrmajor ) {
				blkmajor = devmajors[i][j].block_major;
				blkminor = devmajors[i][j].block_minor;
				bd_disk = devmajors[i][j].bd_disk;
			}
		}
	}
	if (!blkmajor) {
		printk(KERN_WARNING "No block device corresponding to this char device: %d\n",
			MAJOR(inode->i_rdev));
		return -ENODEV;
	}

	inode->i_bdev = bdget(MKDEV(blkmajor, blkminor));
	if (inode->i_bdev == NULL) {
		printk(KERN_WARNING "bdget failed on this char device: major=%d minor=%d\n",
			blkmajor, blkminor);
		return -ENOMEM;
	}

	if (!inode->i_bdev->bd_disk) {
		inode->i_bdev->bd_disk = bd_disk;
	}

	return nvme_ioctl(inode, f, cmd, arg);
}

static int __init nvme_init(void)
{
	int result;

	printk(KERN_DEBUG "999 enter nvme_init func!\n");
	nvme_thread = kthread_run(nvme_kthread, NULL, "nvme");
	if (IS_ERR(nvme_thread))
		return PTR_ERR(nvme_thread);

	result = pci_register_driver(&nvme_driver);
	if (result)
		goto kill_kthread;
	printk(KERN_DEBUG "999 leave nvme_init func!\n");
	return 0;

 kill_kthread:
	printk(KERN_DEBUG "999 leave nvme_init func!\n");
	kthread_stop(nvme_thread);
	return result;
}

static void __exit nvme_exit(void)
{
	printk(KERN_DEBUG "999 enter nvme_exit func!\n");
	pci_unregister_driver(&nvme_driver);
	kthread_stop(nvme_thread);
	printk(KERN_DEBUG "999 enter nvme_exit func!\n");
}

MODULE_AUTHOR("Matthew Wilcox <willy@linux.intel.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.8");
module_init(nvme_init);
module_exit(nvme_exit);
