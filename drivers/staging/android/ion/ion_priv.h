/*
 * drivers/staging/android/ion/ion_priv.h
 *
 * Copyright (C) 2011 Google, Inc.
 * Copyright (c) 2011-2012, The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _ION_PRIV_H
#define _ION_PRIV_H

#include <linux/device.h>
#include <linux/dma-direction.h>
#include <linux/kref.h>
#include <linux/mm_types.h>
#include <linux/mutex.h>
#include <linux/rbtree.h>
#include <linux/sched.h>
#include <linux/shrinker.h>
#include <linux/types.h>

#include <linux/ion.h>
#include <linux/iommu.h>
#include <linux/seq_file.h>

enum {
	DI_PARTITION_NUM = 0,
	DI_DOMAIN_NUM = 1,
	DI_MAX,
};

/**
 * struct ion_iommu_map - represents a mapping of an ion buffer to an iommu
 * @iova_addr - iommu virtual address
 * @node - rb node to exist in the buffer's tree of iommu mappings
 * @domain_info - contains the partition number and domain number
 *		domain_info[1] = domain number
 *		domain_info[0] = partition number
 * @ref - for reference counting this mapping
 * @mapped_size - size of the iova space mapped
 *		(may not be the same as the buffer size)
 * @flags - iommu domain/partition specific flags.
 *
 * Represents a mapping of one ion buffer to a particular iommu domain
 * and address range. There may exist other mappings of this buffer in
 * different domains or address ranges. All mappings will have the same
 * cacheability and security.
 */
struct ion_iommu_map {
	unsigned long iova_addr;
	struct rb_node node;
	union {
		int domain_info[DI_MAX];
		uint64_t key;
	};
	struct ion_buffer *buffer;
	struct kref ref;
	int mapped_size;
	unsigned long flags;
};

struct ion_buffer *ion_handle_buffer(struct ion_handle *handle);

/**
 * struct ion_buffer - metadata for a particular buffer
 * @ref:		refernce count
 * @node:		node in the ion_device buffers tree
 * @dev:		back pointer to the ion_device
 * @heap:		back pointer to the heap the buffer came from
 * @flags:		buffer specific flags
 * @size:		size of the buffer
 * @priv_virt:		private data to the buffer representable as
 *			a void *
 * @priv_phys:		private data to the buffer representable as
 *			an ion_phys_addr_t (and someday a phys_addr_t)
 * @lock:		protects the buffers cnt fields
 * @kmap_cnt:		number of times the buffer is mapped to the kernel
 * @vaddr:		the kenrel mapping if kmap_cnt is not zero
 * @dmap_cnt:		number of times the buffer is mapped for dma
 * @sg_table:		the sg table for the buffer if dmap_cnt is not zero
*/
struct ion_buffer {
	struct kref ref;
	union {
		struct rb_node node;
		struct list_head list;
	};
	struct ion_device *dev;
	struct ion_heap *heap;
	unsigned long flags;
	size_t size;
	union {
		void *priv_virt;
		ion_phys_addr_t priv_phys;
	};
	struct mutex lock;
	int kmap_cnt;
	void *vaddr;
	int dmap_cnt;
	struct sg_table *sg_table;
	int umap_cnt;
	unsigned int iommu_map_cnt;
	struct rb_root iommu_maps;
	int marked;
};
void ion_buffer_destroy(struct ion_buffer *buffer);

/**
 * struct ion_heap_ops - ops to operate on a given heap
 * @allocate:		allocate memory
 * @free:		free memory
 * @phys		get physical address of a buffer (only define on
 *			physically contiguous heaps)
 * @map_dma		map the memory for dma to a scatterlist
 * @unmap_dma		unmap the memory for dma
 * @map_kernel		map memory to the kernel
 * @unmap_kernel	unmap memory to the kernel
 * @map_user		map memory to userspace
 *
 * allocate, phys, and map_user return 0 on success, -errno on error.
 * map_dma and map_kernel return pointer on success, ERR_PTR on error.
 */
struct ion_heap_ops {
	int (*allocate)(struct ion_heap *heap,
			struct ion_buffer *buffer, unsigned long len,
			unsigned long align, unsigned long flags);
	void (*free)(struct ion_buffer *buffer);
	int (*phys)(struct ion_heap *heap, struct ion_buffer *buffer,
		    ion_phys_addr_t *addr, size_t *len);
	struct sg_table * (*map_dma)(struct ion_heap *heap,
				     struct ion_buffer *buffer);
	void (*unmap_dma)(struct ion_heap *heap, struct ion_buffer *buffer);
	void * (*map_kernel)(struct ion_heap *heap, struct ion_buffer *buffer);
	void (*unmap_kernel)(struct ion_heap *heap, struct ion_buffer *buffer);
	int (*map_user)(struct ion_heap *mapper, struct ion_buffer *buffer,
			struct vm_area_struct *vma);
	int (*shrink)(struct ion_heap *heap, gfp_t gfp_mask, int nr_to_scan);
	void (*unmap_user) (struct ion_heap *mapper, struct ion_buffer *buffer);
	int (*cache_op)(struct ion_heap *heap, struct ion_buffer *buffer,
			void *vaddr, unsigned int offset,
			unsigned int length, unsigned int cmd);
	int (*map_iommu)(struct ion_buffer *buffer,
				struct ion_iommu_map *map_data,
				unsigned int domain_num,
				unsigned int partition_num,
				unsigned long align,
				unsigned long iova_length,
				unsigned long flags);
	void (*unmap_iommu)(struct ion_iommu_map *data);
	int (*print_debug)(struct ion_heap *heap, struct seq_file *s,
			   const struct rb_root *mem_map);
	int (*secure_heap)(struct ion_heap *heap, int version, void *data);
	int (*unsecure_heap)(struct ion_heap *heap, int version, void *data);
};

/**
 * struct ion_heap - represents a heap in the system
 * @node:		rb node to put the heap on the device's tree of heaps
 * @dev:		back pointer to the ion_device
 * @type:		type of heap
 * @ops:		ops struct as above
 * @id:			id of heap, also indicates priority of this heap when
 *			allocating.  These are specified by platform data and
 *			MUST be unique
 * @name:		used for debugging
 * @priv:		private heap data
 * @shrinker:		a shrinker for the heap
 * @free_list:		free list head if deferred free is used
 * @free_list_size	size of the deferred free list in bytes
 * @lock:		protects the free list
 * @waitqueue:		queue to wait on from deferred free thread
 * @task:		task struct of deferred free thread
 * @debug_show:		called when heap debug file is read to add any
 *			heap specific debug info to output
 *
 * Represents a pool of memory from which buffers can be made.  In some
 * systems the only heap is regular system memory allocated via vmalloc.
 * On others, some blocks might require large physically contiguous buffers
 * that are allocated from a specially reserved heap.
 */
struct ion_heap {
	struct rb_node node;
	struct ion_device *dev;
	enum ion_heap_type type;
	struct ion_heap_ops *ops;
	int id;
	const char *name;
	void *priv;
};

/**
 * struct mem_map_data - represents information about the memory map for a heap
 * @node:		rb node used to store in the tree of mem_map_data
 * @addr:		start address of memory region.
 * @addr:		end address of memory region.
 * @size:		size of memory region
 * @client_name:		name of the client who owns this buffer.
 *
 */
struct mem_map_data {
	struct rb_node node;
	unsigned long addr;
	unsigned long addr_end;
	unsigned long size;
	const char *client_name;
};

#define iommu_map_domain(__m)		((__m)->domain_info[1])
#define iommu_map_partition(__m)	((__m)->domain_info[0])

/**
 * ion_device_create - allocates and returns an ion device
 * @custom_ioctl:	arch specific ioctl function if applicable
 *
 * returns a valid device or -PTR_ERR
 */
struct ion_device *ion_device_create(long (*custom_ioctl)
				     (struct ion_client *client,
				      unsigned int cmd,
				      unsigned long arg));

/**
 * ion_device_destroy - free and device and it's resource
 * @dev:		the device
 */
void ion_device_destroy(struct ion_device *dev);

/**
 * ion_device_add_heap - adds a heap to the ion device
 * @dev:		the device
 * @heap:		the heap to add
 */
void ion_device_add_heap(struct ion_device *dev, struct ion_heap *heap);

/**
 * some helpers for common operations on buffers using the sg_table
 * and vaddr fields
 */
void *ion_heap_map_kernel(struct ion_heap *, struct ion_buffer *);
void ion_heap_unmap_kernel(struct ion_heap *, struct ion_buffer *);
int ion_heap_map_user(struct ion_heap *, struct ion_buffer *,
			struct vm_area_struct *);
int ion_heap_buffer_zero(struct ion_buffer *buffer);
int ion_heap_pages_zero(struct page *page, size_t size, pgprot_t pgprot);

/**
 * ion_heap_init_shrinker
 * @heap:		the heap
 *
 * If a heap sets the ION_HEAP_FLAG_DEFER_FREE flag or defines the shrink op
 * this function will be called to setup a shrinker to shrink the freelists
 * and call the heap's shrink op.
 */
void ion_heap_init_shrinker(struct ion_heap *heap);

/**
 * ion_heap_init_deferred_free -- initialize deferred free functionality
 * @heap:		the heap
 *
 * If a heap sets the ION_HEAP_FLAG_DEFER_FREE flag this function will
 * be called to setup deferred frees. Calls to free the buffer will
 * return immediately and the actual free will occur some time later
 */
int ion_heap_init_deferred_free(struct ion_heap *heap);

/**
 * ion_heap_freelist_add - add a buffer to the deferred free list
 * @heap:		the heap
 * @buffer:		the buffer
 *
 * Adds an item to the deferred freelist.
 */
void ion_heap_freelist_add(struct ion_heap *heap, struct ion_buffer *buffer);

/**
 * ion_heap_freelist_drain - drain the deferred free list
 * @heap:		the heap
 * @size:		ammount of memory to drain in bytes
 *
 * Drains the indicated amount of memory from the deferred freelist immediately.
 * Returns the total amount freed.  The total freed may be higher depending
 * on the size of the items in the list, or lower if there is insufficient
 * total memory on the freelist.
 */
size_t ion_heap_freelist_drain(struct ion_heap *heap, size_t size);

/**
 * ion_heap_freelist_size - returns the size of the freelist in bytes
 * @heap:		the heap
 */
size_t ion_heap_freelist_size(struct ion_heap *heap);


/**
 * functions for creating and destroying the built in ion heaps.
 * architectures can add their own custom architecture specific
 * heaps as appropriate.
 */

struct ion_heap *ion_heap_create(struct ion_platform_heap *);
void ion_heap_destroy(struct ion_heap *);
struct ion_heap *ion_system_heap_create(struct ion_platform_heap *);
void ion_system_heap_destroy(struct ion_heap *);

struct ion_heap *ion_system_contig_heap_create(struct ion_platform_heap *);
void ion_system_contig_heap_destroy(struct ion_heap *);

struct ion_heap *ion_carveout_heap_create(struct ion_platform_heap *);
void ion_carveout_heap_destroy(struct ion_heap *);

struct ion_heap *ion_iommu_heap_create(struct ion_platform_heap *);
void ion_iommu_heap_destroy(struct ion_heap *);

struct ion_heap *ion_cp_heap_create(struct ion_platform_heap *);
void ion_cp_heap_destroy(struct ion_heap *);

struct ion_heap *ion_reusable_heap_create(struct ion_platform_heap *);
void ion_reusable_heap_destroy(struct ion_heap *);

/**
 * kernel api to allocate/free from carveout -- used when carveout is
 * used to back an architecture specific custom heap
 */
ion_phys_addr_t ion_carveout_allocate(struct ion_heap *heap, unsigned long size,
				      unsigned long align);
void ion_carveout_free(struct ion_heap *heap, ion_phys_addr_t addr,
		       unsigned long size);

struct ion_heap *ion_cma_heap_create(struct ion_platform_heap *);
void ion_cma_heap_destroy(struct ion_heap *);

struct ion_heap *msm_get_contiguous_heap(void);
/**
 * The carveout/cp heap returns physical addresses, since 0 may be a valid
 * physical address, this is used to indicate allocation failed
 */
#define ION_CARVEOUT_ALLOCATE_FAIL -1
#define ION_CP_ALLOCATE_FAIL -1

/**
 * The reserved heap returns physical addresses, since 0 may be a valid
 * physical address, this is used to indicate allocation failed
 */
#define ION_RESERVED_ALLOCATE_FAIL -1

/**
 * ion_map_fmem_buffer - map fmem allocated memory into the kernel
 * @buffer - buffer to map
 * @phys_base - physical base of the heap
 * @virt_base - virtual base of the heap
 * @flags - flags for the heap
 * struct ion_page_pool - pagepool struct
 * @high_count:		number of highmem items in the pool
 * @low_count:		number of lowmem items in the pool
 * @high_items:		list of highmem items
 * @low_items:		list of lowmem items
 * @mutex:		lock protecting this struct and especially the count
 *			item list
 * @gfp_mask:		gfp_mask to use from alloc
 * @order:		order of pages in the pool
 * @list:		plist node for list of pools
 *
 * Map fmem allocated memory into the kernel address space. This
 * is designed to be used by other heaps that need fmem behavior.
 * The virtual range must be pre-allocated.
 */
void *ion_map_fmem_buffer(struct ion_buffer *buffer, unsigned long phys_base,
				void *virt_base, unsigned long flags);

/**
 * ion_do_cache_op - do cache operations.
 *
 * @client - pointer to ION client.
 * @handle - pointer to buffer handle.
 * @uaddr -  virtual address to operate on.
 * @offset - offset from physical address.
 * @len - Length of data to do cache operation on.
 * @cmd - Cache operation to perform:
 *		ION_IOC_CLEAN_CACHES
 *		ION_IOC_INV_CACHES
 *		ION_IOC_CLEAN_INV_CACHES
 *
 * Returns 0 on success
 */
int ion_do_cache_op(struct ion_client *client, struct ion_handle *handle,
			void *uaddr, unsigned long offset, unsigned long len,
			unsigned int cmd);

void ion_cp_heap_get_base(struct ion_heap *heap, unsigned long *base,
			unsigned long *size);

void ion_mem_map_show(struct ion_heap *heap);

#endif /* _ION_PRIV_H */
