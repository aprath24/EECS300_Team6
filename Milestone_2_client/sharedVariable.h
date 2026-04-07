#ifndef SHARED_VARIABLE_H_
#define SHARED_VARIABLE_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

/*
 * Macro:  INIT_SHARED_VARIABLE
 * --------------------
 * initializes a shared variable
 * 
 * s: an instance of a shared variable struct (e.g., shared_uint32)
 * v: the desired initial value; the type must much the shared type (e.g., you must supply uint32_t for shared_uint32)
 */
#define INIT_SHARED_VARIABLE(s, v)    s.value = v; \
                                      s.sem = xSemaphoreCreateMutex()
/*
 * Macro:  LOCK_SHARED_VARIABLE
 * --------------------
 * locks a shared variable
 * 
 * s: an instance of a shared variable struct (e.g., shared_uint32)
 */                                 
#define LOCK_SHARED_VARIABLE(s)       xSemaphoreTake(s.sem, portMAX_DELAY)
/*
 * Macro:  UNLOCK_SHARED_VARIABLE
 * --------------------
 * unlocks a shared variable
 * 
 * s: an instance of a shared variable struct (e.g., shared_uint32)
 */
#define UNLOCK_SHARED_VARIABLE(s)     xSemaphoreGive(s.sem)
                             
typedef struct shared_uint32
{
  uint32_t value;
  SemaphoreHandle_t sem;
} shared_uint32;

typedef struct shared_double
{
  double value;
  SemaphoreHandle_t sem;
} shared_double;

//add more types (e.g., string) if needed



#endif /* SHARED_VARIABLE_H_ */
