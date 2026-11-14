/*
 * lat_fence.h - Inter-core memory ordering fences.
 *
 * Uses RISC-V `fence` on target and `__atomic_thread_fence(__ATOMIC_SEQ_CST)`
 * on host tests.
 */
#ifndef LAT_FENCE_H
#define LAT_FENCE_H

#if defined(__riscv)
/* Memory-to-memory ordering (shared SRAM IPC structures). */
#define LAT_FENCE_RW()   __asm volatile("fence rw, rw" ::: "memory")
/* I/O and memory ordering (before PFIC or V5F startup). */
#define LAT_FENCE_IORW() __asm volatile("fence iorw, iorw" ::: "memory")
#else
#define LAT_FENCE_RW()   __atomic_thread_fence(__ATOMIC_SEQ_CST)
#define LAT_FENCE_IORW() __atomic_thread_fence(__ATOMIC_SEQ_CST)
#endif

#endif /* LAT_FENCE_H */
