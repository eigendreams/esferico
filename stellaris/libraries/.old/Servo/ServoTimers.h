#define _useTimer1
#define _useTimer2
#define _useTimer3

// The WTIMER4 is not tied to any pin or used by the lm4f core. However, it may be that
// another library conflicts with the Servo implementation. Changing the associated
// timer is easy, just change the n in WTIMERn to another value (from 0 to 5), or
// use the normal width timer changing WTIMERn to TIMERn, wherever it is. Check the
// microcontroller datasheet
#if defined (_useTimer1)
#define TIM1_BASE			WTIMER4_BASE										// ulBase
#define TIM1_CFG			TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC  		// ulTimerCFG
#define TIM1_TRIGGER		TIMER_TIMA_TIMEOUT									// ulTrigger
#define TIM1_INTERRUPT		INT_WTIMER4A										// ulInterrupt
#define TIM1_TIMER			TIMER_A												// ulTimer
#define TIM1_PERIPH			SYSCTL_PERIPH_WTIMER4								// ulPeriph
#endif
#if defined (_useTimer2)
#define TIM2_BASE			WTIMER4_BASE										// ulBase
#define TIM2_CFG			TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_PERIODIC  		// ulTimerCFG
#define TIM2_TRIGGER		TIMER_TIMB_TIMEOUT									// ulTrigger
#define TIM2_INTERRUPT		INT_WTIMER4B										// ulInterrupt
#define TIM2_TIMER			TIMER_B												// ulTimer
#define TIM2_PERIPH			SYSCTL_PERIPH_WTIMER4								// ulPeriph
#endif
#if defined (_useTimer3)
#define TIM3_BASE			WTIMER5_BASE										// ulBase
#define TIM3_CFG			TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC  		// ulTimerCFG
#define TIM3_TRIGGER		TIMER_TIMA_TIMEOUT									// ulTrigger
#define TIM3_INTERRUPT		INT_WTIMER5A										// ulInterrupt
#define TIM3_TIMER			TIMER_A												// ulTimer
#define TIM3_PERIPH			SYSCTL_PERIPH_WTIMER5								// ulPeriph
#endif

typedef enum {
	_timer1, _timer2, _timer3, _Nbr_16timers
} timer16_Sequence_t;

