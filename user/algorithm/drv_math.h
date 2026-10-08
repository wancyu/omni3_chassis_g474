#ifndef DRV_MATH_H
#define DRV_MATH_H

#ifdef __cplusplus
extern "C" {
#endif


#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <stdbool.h>
#include <stdint.h>

/* Exported macros -----------------------------------------------------------*/
#define RPM_TO_RADPS (2.0f * PI / 60.0f)
#define DEG_TO_RAD (PI / 180.0f)
#define CELSIUS_TO_KELVIN (273.15f)

/* 通用限幅宏 */
#define Math_Constrain(x, Min, Max) \
    do { \
        if (*(x) < (Min)) { *(x) = (Min); } \
        else if (*(x) > (Max)) { *(x) = (Max); } \
    } while (0)

/* 绝对值宏 */
#define Math_Abs(x) ((x) > 0 ? (x) : -(x))

/* 大小端转换（单参数：原地反转指针指向的变量） */
static inline void Math_Endian_Reverse_16_Inplace(void *Address)
{
    uint16_t *p = (uint16_t *)Address;
    *p = __builtin_bswap16(*p);
}

static inline void Math_Endian_Reverse_32_Inplace(void *Address)
{
    uint32_t *p = (uint32_t *)Address;
    *p = __builtin_bswap32(*p);
}

/* 大小端转换（双参数：从源读取反转后返回，可选存入目的地址） */
static inline uint16_t Math_Endian_Reverse_16(void *Source, void *Destination)
{
    uint16_t val = __builtin_bswap16(*(uint16_t *)Source);
    if (Destination != NULL) {
        *(uint16_t *)Destination = val;
    }
    return val;
}

static inline uint32_t Math_Endian_Reverse_32(void *Source, void *Destination)
{
    uint32_t val = __builtin_bswap32(*(uint32_t *)Source);
    if (Destination != NULL) {
        *(uint32_t *)Destination = val;
    }
    return val;
}

/* 导出函数声明 */
void Math_Boolean_Logical_Not(bool *Value);
uint8_t Math_Sum_8(uint8_t *Address, uint32_t Length);
uint16_t Math_Sum_16(uint16_t *Address, uint32_t Length);
uint32_t Math_Sum_32(uint32_t *Address, uint32_t Length);
float Math_Sinc(float x);
int32_t Math_Float_To_Int(float x, float Float_Min, float Float_Max, int32_t Int_Min, int32_t Int_Max);
float Math_Int_To_Float(int32_t x, int32_t Int_Min, int32_t Int_Max, float Float_Min, float Float_Max);

#ifdef __cplusplus
}
#endif

#endif /* DRV_MATH_H */