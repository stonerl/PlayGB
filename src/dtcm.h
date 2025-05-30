#pragma once

#include <stdbool.h>
#include <stdio.h>

extern void *dtcm_mempool;

void dtcm_set_mempool(void *addr);
void dtcm_init(void);
bool dtcm_verify(void);
bool dtcm_enabled(void);  // true if dtcm_init called and DTCM_ALLOC enabled

void *dtcm_alloc(size_t size);
void *dtcm_alloc_aligned(size_t size, size_t offset);