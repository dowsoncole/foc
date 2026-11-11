#ifndef FOC_MEMORY_H
#define FOC_MEMORY_H

#include <string.h>

/* A fixed-size clear must not introduce a library call in the carrier ISR.
   ArmClang can emit the stores in place, avoiding saving live FP arguments
   across __aeabi_memclr4. The fallback retains portable C/host behaviour. */
#if defined(__clang__)
#if __has_builtin(__builtin_memset_inline)
#define FOC_CLEAR_OBJECT(object) __builtin_memset_inline((object), 0, sizeof(*(object)))
#endif
#endif
#ifndef FOC_CLEAR_OBJECT
#define FOC_CLEAR_OBJECT(object) memset((object), 0, sizeof(*(object)))
#endif

#endif
