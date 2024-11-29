#include "circular_buffer.h"

uint8_t circularBufferAdd(CircularBuffer* circBuf, Capture* capture)
{
	// Check if full
	if((circBuf->head == circBuf->tail - 1) || (circBuf->tail == 0 && circBuf->head == CIRCULAR_BUFFER_SIZE))
		return 0;

	// Wrap around if at end
	if(circBuf->head == CIRCULAR_BUFFER_SIZE)
		circBuf->head = 0;

	circBuf->captures[circBuf->head] = *capture;
	circBuf->head += 1;
	return 1;
}

Capture circularBufferFirst(CircularBuffer* circBuf)
{
	// Check if empty
	if(circBuf->tail == circBuf->head)
	{
		Capture emptyCapture = {0};
		return emptyCapture;
	}

	// Wrap around if at end
	if(circBuf->tail == CIRCULAR_BUFFER_SIZE)
		circBuf->tail = 0;

	Capture capture = circBuf->captures[circBuf->tail];
	circBuf->tail += 1;
	return capture;
}

uint8_t circularBufferIsEmpty(CircularBuffer* circBuf)
{
	return circBuf->tail == circBuf->head;
}
