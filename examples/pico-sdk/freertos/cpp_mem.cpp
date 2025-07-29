#include "FreeRTOS.h"

void *operator new  (size_t s) { return pvPortMalloc(s); }
void *operator new[](size_t s) { return pvPortMalloc(s); }
void  operator delete  (void *p) noexcept { vPortFree(p); }
void  operator delete[](void *p) noexcept { vPortFree(p); }
