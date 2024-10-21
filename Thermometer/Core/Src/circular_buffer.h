#ifndef SRC_CIRCULAR_BUFFER_H_
#define SRC_CIRCULAR_BUFFER_H_

#include <inttypes.h>

#define CIRCULAR_BUFFER_SIZE 150

typedef struct {
	uint16_t start;
	uint16_t end;

	enum {
		CAPTURE_TYPE_NONE,
		CAPTURE_TYPE_LOW,
		CAPTURE_TYPE_HIGH,
	} level;
} Capture;

typedef struct {
	Capture captures[CIRCULAR_BUFFER_SIZE];
	uint16_t head;
	uint16_t tail;
} CircularBuffer;

uint8_t circularBufferAdd(CircularBuffer* circBuf, Capture* capture);
Capture circularBufferFirst(CircularBuffer* circBuf);
uint8_t circularBufferIsEmpty(CircularBuffer* circBuf);

#endif /* SRC_CIRCULAR_BUFFER_H_ */
