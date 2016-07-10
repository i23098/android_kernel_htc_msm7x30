/*
 * BFQ: CGROUPS support.
 *
 * Based on ideas and code from CFQ:
 * Copyright (C) 2003 Jens Axboe <axboe@kernel.dk>
 *
 * Copyright (C) 2008 Fabio Checconi <fabio@gandalf.sssup.it>
 *		      Paolo Valente <paolo.valente@unimore.it>
 *
 * Copyright (C) 2010 Paolo Valente <paolo.valente@unimore.it>
 *
 * Licensed under the GPL-2 as detailed in the accompanying COPYING.BFQ
 * file.
 */

static inline void bfq_init_entity(struct bfq_entity *entity,
				   struct bfq_group *bfqg)
{
	entity->weight = entity->new_weight;
	entity->orig_weight = entity->new_weight;
	entity->ioprio = entity->new_ioprio;
	entity->ioprio_class = entity->new_ioprio_class;
	entity->sched_data = &bfqg->sched_data;
}

static inline struct bfq_group *
bfq_bic_update_cgroup(struct bfq_io_cq *bic)
{
	struct bfq_data *bfqd = bic_to_bfqd(bic);
	return bfqd->root_group;
}

static inline void bfq_bfqq_move(struct bfq_data *bfqd,
				 struct bfq_queue *bfqq,
				 struct bfq_entity *entity,
				 struct bfq_group *bfqg)
{
}

static void bfq_end_wr_async(struct bfq_data *bfqd)
{
	bfq_end_wr_async_queues(bfqd, bfqd->root_group);
}

static inline void bfq_disconnect_groups(struct bfq_data *bfqd)
{
	bfq_put_async_queues(bfqd, bfqd->root_group);
}

static inline void bfq_free_root_group(struct bfq_data *bfqd)
{
	kfree(bfqd->root_group);
}

static struct bfq_group *bfq_alloc_root_group(struct bfq_data *bfqd, int node)
{
	struct bfq_group *bfqg;
	int i;

	bfqg = kmalloc_node(sizeof(*bfqg), GFP_KERNEL | __GFP_ZERO, node);
	if (bfqg == NULL)
		return NULL;

	for (i = 0; i < BFQ_IOPRIO_CLASSES; i++)
		bfqg->sched_data.service_tree[i] = BFQ_SERVICE_TREE_INIT;

	return bfqg;
}
