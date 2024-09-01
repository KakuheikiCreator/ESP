/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :common memory allocation functions header file
 *
 * CREATED:2022/03/26 11:35:00
 * AUTHOR :Nakanohito
 *
 * DESCRIPTION:動的メモリ割り当て関係の関数群
 *
 * CHANGE HISTORY:
 *
 * LAST MODIFIED BY:
 *
 *******************************************************************************
 *
 * Copyright (c) 2024 Nakanohito
 * Released under the MIT license
 * https://opensource.org/licenses/mit-license.php
 *
 ******************************************************************************/

#ifndef __NTFW_COM_MEM_ALLOC_H__
#define __NTFW_COM_MEM_ALLOC_H__

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
/***      Include files                                                     ***/
/******************************************************************************/
#include <stdio.h>
#include <stdbool.h>
#include <esp_system.h>

/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/
/** memory storage size */
#ifndef MEM_STORAGE_SIZE
    // メモリ貯蔵域のデフォルトサイズ
    #define MEM_STORAGE_SIZE    (32768)
#endif

/******************************************************************************/
/***      Type Definitions                                                  ***/
/******************************************************************************/
/**
 * メモリセグメント情報
 */
typedef struct {
    uint8_t* pu8_address;                       // アドレス
    uint32_t u32_size;                          // サイズ
} ts_mem_segment_info_t;

/******************************************************************************/
/***      Exported Variables                                                ***/
/******************************************************************************/

/******************************************************************************/
/***      Exported Functions                                                ***/
/******************************************************************************/
/** allocate area size */
uint32_t u32_mem_alloc_size();
/** Unused area size */
uint32_t u32_mem_unused_size();
/** Number of unused areas */
uint32_t u32_mem_unused_cnt();
/** Memory segment information */
ts_mem_segment_info_t s_mem_unused_info_addr(uint32_t u32_idx);
/** Memory segment information */
ts_mem_segment_info_t s_mem_unused_info_size(uint32_t u32_idx);
/** Dynamic Memory Assignment with Exclusive Control */
void* pv_mem_malloc(uint32_t u32_size);
/** Dynamic Clear Memory Assignment with Exclusive Control */
void* pv_mem_calloc(uint32_t u32_size);
/** Dynamic Memory reassignment with Exclusive Control */
void* pv_mem_realloc(void* pv_memory, uint32_t u32_size);
/** Data array clone processing */
void* pv_mem_clone(void* pv_data, uint32_t u32_size);
/** Release dynamic memory with exclusive control */
long l_mem_free(void* pv_memory);
/** Get dynamically allocated memory size */
long l_mem_sizeof(void* pv_memory);

#ifdef __cplusplus
}
#endif

#endif /* __NTFW_COM_MEM_ALLOC_H__ */
