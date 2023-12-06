/*
 * syscalls.c
 *
 *  Created on: 5 дек. 2023 г.
 *      Author: laurelindo
 */

#include <stdint.h>
#include <errno.h>
#include <malloc.h>

#define MAX_STACK_SIZE  8192

void local_heap_setup(uint8_t **start, uint8_t **end);

#pragma weak local_heap_setup = __local_ram

/* these are defined by the linker script */
extern uint8_t _ebss, _stack;

static uint8_t *_cur_brk = NULL;
static uint8_t *_heap_end = NULL;

/*
 * If not overridden, this puts the heap into the left
 * over ram between the BSS section and the stack while
 * preserving MAX_STACK_SIZE bytes for the stack itself.
 */
static void __local_ram(uint8_t **start, uint8_t **end) {
    *start = &_ebss;
    *end = (uint8_t*) (&_stack - MAX_STACK_SIZE);
}

/* prototype to make gcc happy */
void* _sbrk_r(struct _reent*, ptrdiff_t);

void* _sbrk_r(struct _reent *reent, ptrdiff_t diff) {
    uint8_t *_old_brk;

    if(_heap_end == NULL) {
        local_heap_setup(&_cur_brk, &_heap_end);
    }

    _old_brk = _cur_brk;
    if(_cur_brk + diff > _heap_end) {
        reent->_errno = ENOMEM;
        return (void*) -1;
    }
    _cur_brk += diff;
    return _old_brk;
}

void _exit() {
    while(1);
}
