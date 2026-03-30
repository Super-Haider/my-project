#include "Lab10_heap_driver.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>

/* ── Heap region parameters ──────────────────────────────────────────────────
 *  HEAP_START_ADDR : 0x20001000  (avoids overwriting .data and .bss)
 *  HEAP_SIZE       : 4 KB
 *  BLOCK_SIZE      : 16 bytes per block
 *  BLOCK_COUNT     : 256 blocks total
 * ──────────────────────────────────────────────────────────────────────────*/
#define HEAP_START_ADDR  ((uint8_t *)0x20001000)
#define HEAP_SIZE        (4 * 1024)
#define BLOCK_SIZE       16
#define BLOCK_COUNT      (HEAP_SIZE / BLOCK_SIZE)   /* 256 */

/* ── Block map ───────────────────────────────────────────────────────────────
 *  One byte per block.
 *    0 = free
 *    1 = in use
 *  Stored in normal .bss — NOT inside the heap region itself.
 * ──────────────────────────────────────────────────────────────────────────*/
static uint8_t block_map[BLOCK_COUNT];

/* ── heap_init ───────────────────────────────────────────────────────────────
 *  Zero the block_map so all 256 blocks are marked free.
 *  Must be called once before any heap_alloc() / heap_free() call.
 * ──────────────────────────────────────────────────────────────────────────*/
void heap_init(void)
{
    memset(block_map, 0, sizeof(block_map));
}

/* ── heap_alloc ──────────────────────────────────────────────────────────────
 *  Allocate at least 'size' bytes from the custom heap.
 *
 *  Steps:
 *    1. Calculate how many blocks are needed (round up to BLOCK_SIZE).
 *    2. Scan block_map for a contiguous run of free blocks.
 *    3. If found: mark them as used (1) and return the SRAM address.
 *    4. If not found: return NULL.
 *
 *  Example:
 *    heap_alloc(32)  →  needs 2 blocks  →  finds blocks 0-1 free  →  marks 0,1 = 1
 *    heap_alloc(48)  →  needs 3 blocks  →  finds blocks 2-4 free  →  marks 2,3,4 = 1
 * ──────────────────────────────────────────────────────────────────────────*/
void *heap_alloc(size_t size)
{
    if (size == 0) return NULL;

    /* Number of blocks needed — round up */
    size_t blocks_needed = (size + BLOCK_SIZE - 1) / BLOCK_SIZE;

    size_t run_start = 0;
    size_t run_len   = 0;

    for (size_t i = 0; i < BLOCK_COUNT; i++)
    {
        if (block_map[i] == 0)
        {
            if (run_len == 0) run_start = i;   /* start tracking a new free run */
            run_len++;

            if (run_len == blocks_needed)
            {
                /* Found a large enough contiguous free region — mark all as used */
                for (size_t j = run_start; j < run_start + blocks_needed; j++)
                {
                    block_map[j] = 1;
                }

                /* Return pointer to the first byte of block run_start in SRAM */
                return (void *)(HEAP_START_ADDR + run_start * BLOCK_SIZE);
            }
        }
        else
        {
            /* Used block — break the current run and start fresh */
            run_len = 0;
        }
    }

    return NULL;   /* Not enough contiguous free space */
}

/* ── heap_free ───────────────────────────────────────────────────────────────
 *  Free the blocks starting at ptr.
 *
 *  Steps:
 *    1. Validate ptr is non-NULL and within the heap region.
 *    2. Confirm ptr is block-aligned.
 *    3. Starting from the block that ptr points to, clear consecutive
 *       used blocks (1 → 0) until a free block (0) is encountered.
 *       This naturally covers the entire allocation made by heap_alloc().
 *
 *  @param  ptr  pointer previously returned by heap_alloc()
 * ──────────────────────────────────────────────────────────────────────────*/
void heap_free(void *ptr)
{
    if (ptr == NULL) return;

    uint8_t *p = (uint8_t *)ptr;

    /* Boundary check — must be inside the heap region */
    if (p < HEAP_START_ADDR || p >= HEAP_START_ADDR + HEAP_SIZE) return;

    /* Alignment check — must be at a block boundary */
    if ((p - HEAP_START_ADDR) % BLOCK_SIZE != 0) return;

    /* Calculate block index */
    size_t start_block = (size_t)(p - HEAP_START_ADDR) / BLOCK_SIZE;

    /* Clear consecutive used blocks — stops at the first free block */
    for (size_t i = start_block; i < BLOCK_COUNT; i++)
    {
        if (block_map[i] == 1)
        {
            block_map[i] = 0;
        }
        else
        {
            break;   /* Hit a free block — end of this allocation */
        }
    }
}