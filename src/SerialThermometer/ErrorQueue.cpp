#include "ErrorQueue.h"

long errorTimestamps[QUEUE_LENGTH];
int headIdx = -1;
int tailIdx = -1;
bool rolloverHappened = false;

bool errorsAccumulated() {
    if (!rolloverHappened) {
        return false;
    }
    long oldestTimestamp = errorTimestamps[tailIdx];
    return oldestTimestamp > millis() - ACCUMULATION_PERIOD;
}

int nextIdx(int idx) {
    idx++;
    if (idx == QUEUE_LENGTH) {
        idx = 0;
    }
    return idx;
}

void logError() {
    if (headIdx == QUEUE_LENGTH - 1) {
        rolloverHappened = true;
    }
    headIdx = nextIdx(headIdx);
    errorTimestamps[headIdx] = millis();

    tailIdx = rolloverHappened ? nextIdx(headIdx) : 0;
}

void clearErrors() {
    headIdx = -1;
    tailIdx = -1;
    rolloverHappened = false;
}