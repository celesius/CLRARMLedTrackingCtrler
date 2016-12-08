/*
 * CLRDebug.h
 *
 *  Created on: 2016年11月30日
 *      Author: clover
 */

#ifndef DEBUG_CLRDEBUG_H_
#define DEBUG_CLRDEBUG_H_

#include "diag/Trace.h"

#ifdef __cplusplus
extern "C"{
#endif

#define CLRDEBUG

#ifdef CLRDEBUG
#define clr_debug_printf trace_printf
#else
#define clr_debug_printf
#endif

#ifdef __cplusplus
}
#endif
#endif /* DEBUG_CLRDEBUG_H_ */
