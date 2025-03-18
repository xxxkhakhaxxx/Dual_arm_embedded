/**
 ********************************************************************************
 ** @file    ApiMacro.h
 ** @author  HH (hunghoang.1806@gmail.com)
 ** @date    Oct 30, 2024 (created)
 ** @brief   
 ********************************************************************************
 **/

#ifndef MIDDLEWARE_APIMACRO_APIMACRO_H_
#define MIDDLEWARE_APIMACRO_APIMACRO_H_


/********************************************************************************
 * INCLUDES
 ********************************************************************************/
//#include "LibraryHHInterface.h"
#include "stdint.h"


/********************************************************************************
 * MACROS AND DEFINES
 ********************************************************************************/
/* ELEMENT TYPE */
#define PRIVATE static
#define GLOBAL

/* BOOLEAN */
#define TRUE  1
#define FALSE 0


/* IDENTIFIER */
#define AND       &&
#define OR        ||
#define NOT       !
#define NOTEQUALS !=
#define EQUALS    ==
#define IS        =


/* TYPES */
//#define BOOL uint8_t
//#define U08  uint8_t
//#define U16  uint16_t
//#define U32  uint32_t
//#define U64  uint64_t
//#define I08  int8_t
//#define I16  int16_t
//#define I32  int32_t
//#define I64  int64_t

#define INT16  short
#define UINT16 unsigned short
#define INT32  int
#define UINT32 unsigned int
#define INT64  long long
#define UINT64 unsigned long long
#define CHAR8 char

#define U32_MAX (65535)

/* MATHS */
#define PI                  (3.14159265)
#define RAD2DEG(x)          ((x)/PI*180)
#define DEG2RAD(x)          ((x)*PI/180)
#define ALIGNB(x, align)    (((x) + ((align) - 1)) & ~((align) - 1))
#define ALIGN(x, align)     ((((x) + ((align) - 1)) / (align)) * (align))
#define FLOORB(x, align)    ((x) & ~((align) - 1))
#define FLOOR(x, align)     (((x) / (align)) * (align))
#define CEILB(x, align)     ALIGNB(x, align)
#define CEIL(x, align)      ALIGN(x, align)
#define CLIP(x, min, max)   (((x) < (min)) ? (min) : \
                            (((x) > (max)) ? (max) : (x)))
#define UCLIP(x, max)       (((x) > (max)) ? (max) : (x))
#define LCLIP(x, min)       (((x) < (min)) ? (min) : (x))
#define MIN(x, y)           (((x) < (y)) ?  (x) : (y))
#define MAX(x, y)           (((x) > (y)) ?  (x) : (y))
#define ABS(x)              (((x) <  0) ? -(x) : (x))
#define DIFF(a,b)           ABS((a)-(b))
#define IS_NAN(x)            ((x) != (x))
#define IMPLIES(x, y)       (!(x) || (y))
#define SWAP(a, b)          do { a ^= b; b ^= a; a ^= b; } while ( 0 )
#define SORT(a, b)          do { if ((a) > (b)) SWAP((a), (b)); } while (0)
#define COMPARE(x, y)       (((x) > (y)) - ((x) < (y)))
#define SIGN(x)             COMPARE(x, 0)
#define IS_ODD( num )       ((num) & 1)
#define IS_EVEN( num )      (!IS_ODD( (num) ))
#define IS_BETWEEN(n,L,H)   ((unsigned char)((n) >= (L) && (n) <= (H)))
#define CONSTRAIN(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))


/* BITS */
#define BIT(x)          (1<<(x))
#define SETBIT(x,p)     ((x)|(1<<(p)))
#define CLEARBIT(x,p)   ((x)&(~(1<<(p))))
#define GETBIT(x,p)     (((x)>>(p))&1)
#define TOGGLEBIT(x,p)  ((x)^(1<<(p)))


/* ARRAYS */
#define ARRAY_SIZE(a)   (sizeof(a) / sizeof(*a))
#define IS_ARRAY(a)   ((void *)&a == (void *)a)

/********************************************************************************
 * TYPEDEFS AND ENUMS
 ********************************************************************************/
typedef uint8_t		BOOL;
typedef uint8_t		U08;
typedef uint16_t	U16;
typedef uint32_t	U32;
typedef uint64_t	U64;
typedef int8_t		I08;
typedef int16_t		I16;
typedef int32_t		I32;
typedef int64_t		I64;

#endif /* MIDDLEWARE_APIMACRO_APIMACRO_H_ */
