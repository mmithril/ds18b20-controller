/*
 * Error queue logs errors and detects if errors accumulated, i.e. more than QUEUE_LENGTH errors were logged in last ACCUMULATION_PERIOD ms.
 */

// Number of errors to accumulate
#define QUEUE_LENGTH 10
// Accumulation time period
#define ACCUMULATION_PERIOD 15000

#include <Arduino.h>

/*
 * Check returning true if more than m errors were logged in last n minutes
 */
bool errorsAccumulated();

/*
 * Logs an error
 */
void logError();

/*
 * Clear all errors - accumulation starts from beginning.
 */
void clearErrors();