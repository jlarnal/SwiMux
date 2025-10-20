#ifndef H_DEBUG_MACROS_H
#define H_DEBUG_MACROS_H


#define DEBUG_ENABLED

#ifdef DEBUG_ENABLED
#define DebugInit() DebugPrintfBufferFree()
#else
#define DebugInit()
#endif


#endif // !H_DEBUG_MACROS_H

