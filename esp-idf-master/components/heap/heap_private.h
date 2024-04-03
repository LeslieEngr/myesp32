/*
 * SPDX-FileCopyrightText: 2015-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdlib.h>
#include <stdint.h>
#include <soc/soc_memory_layout.h>
#include "multi_heap.h"
#include "multi_heap_platform.h"
#include "sys/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Some common heap registration data structures used
   for heap_caps_init.c to share heap information with heap_caps.c
*/

#define HEAP_SIZE_MAX (SOC_MAX_CONTIGUOUS_RAM_SIZE)

/* Type for describing each registered heap */
typedef struct heap_t_ {
    uint32_t caps[SOC_MEMORY_TYPE_NO_PRIOS]; ///< Capabilities for the type of memory in this heap (as a prioritised set). Copied from soc_memory_types so it's in RAM not flash.
    intptr_t start;
    intptr_t end;
    multi_heap_lock_t heap_mux;
    multi_heap_handle_t heap;
    SLIST_ENTRY(heap_t_) next;
} heap_t;

/* All registered heaps.

   Forms a single linked list, even though most entries are contiguous.
   This means at the expense of 4 bytes per heap, new heaps can be
   added at runtime in a fast & thread-safe way.
*/
extern SLIST_HEAD(registered_heap_ll, heap_t_) registered_heaps;

bool heap_caps_match(const heap_t *heap, uint32_t caps);

/* return all possible capabilities (across all priorities) for a given heap */
inline static uint32_t get_all_caps(const heap_t *heap)
{
    if (heap->heap == NULL) {
        return 0;
    }
    uint32_t all_caps = 0;
    for (int prio = 0; prio < SOC_MEMORY_TYPE_NO_PRIOS; prio++) {
        all_caps |= heap->caps[prio];
    }
    return all_caps;
}

/*
 Because we don't want to add _another_ known allocation method to the stack of functions to trace wrt memory tracing,
 these are declared private. The newlib malloc()/realloc() implementation also calls these, so they are declared
 separately in newlib/syscalls.c.
*/
void *heap_caps_realloc_default(void *p, size_t size);
void *heap_caps_malloc_default(size_t size);
void *heap_caps_aligned_alloc_default(size_t alignment, size_t size);


#ifdef __cplusplus
}
#endif
