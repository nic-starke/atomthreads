
#include <avr/cpufunc.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdlib.h>

#include "atom.h"
#include "atomport-private.h"

static const uint16_t prescalers[] = {1, 2, 4, 8, 64, 256, 1024};
#define NUM_PRESCALERS (sizeof(prescalers) / sizeof(prescalers[0]))
#define MAX_PER        UINT8_MAX

/**
 * \b avrInitSystemTickTimer
 *
 * Initialise the system tick timer. TCC0 exists on all xmega platforms so it is used here.
 *
 * @return None
 */
void avrInitSystemTickTimer(void)
{
    uint32_t clocks_per_tick = F_CPU / SYSTEM_TICKS_PER_SEC;
    uint32_t per             = 0;
    uint32_t lowest_error    = UINT32_MAX;
    int16_t  best_prescaler  = -1;
    uint16_t best_per        = 0;

    for (int i = 0; i < NUM_PRESCALERS; i++)
    {
        per = clocks_per_tick / prescalers[i];
        per--;
        if (per > MAX_PER)
            continue;

        uint32_t error;
        error = abs((int32_t)clocks_per_tick - (int32_t)((per + 1) * prescalers[i]));
        if (error < lowest_error)
        {
            lowest_error   = error;
            best_prescaler = prescalers[i];
            best_per       = per;
        }
    }

    // Storage status register, and disable interrupts
    volatile uint8_t sreg = SREG;
    cli();

    // Configure the timer with the best period and prescaler values.
    TCC0.PER      = best_per;
    TCC0.CTRLA    = best_prescaler;
    TCC0.CTRLB    = TC_WGMODE_NORMAL_gc; // Use "normal mode"
    TCC0.INTCTRLA = TC_OVFINTLVL_HI_gc;  // Overflow interrupt with high priority
    TCC0.CNT      = 0;                   // Set starting count to 0

    // Restore interrupts
    _MemoryBarrier();
    SREG = sreg;
}

/**
 *
 * System tick ISR.
 *
 * This is responsible for regularly calling the OS system tick handler.
 * The system tick handler checks if any timer callbacks are necessary,
 * and runs the scheduler.
 *
 * The compiler automatically saves all registers necessary before calling
 * out to a C routine. This will be (at least) R0, R1, SREG, R18-R27 and
 * R30/R31.
 *
 * The system may decide to schedule in a new thread during the call to
 * atomTimerTick(), in which case around half of the thread's context will
 * already have been saved here, ready for when we return here when the
 * interrupted thread is scheduled back in. The remaining context will be
 * saved by the context switch routine.
 *
 * As with all interrupts, the ISR should call atomIntEnter() and
 * atomIntExit() on entry and exit. This serves two purposes:
 *
 * a) To notify the OS that it is running in interrupt context
 * b) To defer the scheduler until after the ISR is completed
 *
 * We defer all scheduling decisions until after the ISR has completed
 * in case the interrupt handler makes more than one thread ready.
 *
 * @return None
 */
ISR(TCC0_OVF_vect)
{
    /* Call the interrupt entry routine */
    atomIntEnter();

    /* Call the OS system tick handler */
    atomTimerTick();

    /* Call the interrupt exit routine */
    atomIntExit(TRUE);
}

/**
 *
 * Default (no handler installed) ISR.
 *
 * Installs a default handler to be called if any interrupts occur for
 * which we have not registered an ISR. This is empty and has only been
 * included to handle user-created code which may enable interrupts. The
 * core OS does not enable any interrupts other than the system timer
 * tick interrupt.
 *
 * @return None
 */
ISR(BADISR_vect)
{
    /* Empty */
}
