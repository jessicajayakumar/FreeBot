#include <stdlib.h>

/* Struct's */
#define NUM_EVENTS 4
#define NUM_SUPERVISORS 1

#define EV_move 0

#define EV_btnStop 1

#define EV_stop 2

#define EV_btnMove 3

void SCT_init();
void SCT_reset();
void SCT_add_callback(unsigned char event, void (*clbk)(void *), unsigned char (*ci)(void *), void *data);
void SCT_run_step();
