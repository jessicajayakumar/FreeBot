#include <stdlib.h>

/* Struct's */
#define NUM_EVENTS 8
#define NUM_SUPERVISORS 4

#define EV_atWork 0

#define EV_charge 1

#define EV_atCharger 2

#define EV_notAtCharger 3

#define EV_work 4

#define EV_moveToWork 5

#define EV_notAtWork 6

#define EV_moveToCharge 7


void SCT_init();
void SCT_reset();
void SCT_add_callback( unsigned char event, void (*clbk)( void* ), unsigned char (*ci)( void* ), void* data );
void SCT_run_step();


//~ void SCT_set_decay_prob_event( unsigned char event, char factor );
//~ void SCT_decay_prob();
void SCT_set_decay_prob_event( unsigned char event, float init_decay, float decay );
void SCT_decay_prob();

