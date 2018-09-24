
#ifndef __SYSTEM_GCT301S_H
#define __SYSTEM_GCT301S_H

#ifdef __cplusplus
extern "C" {
#endif

extern uint32_t SystemFrequency;    /*!< System Clock Frequency (Core Clock)  */
extern uint32_t SystemCoreClock;    /*!< Processor Clock Frequency            */
/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System and update the SystemFrequency variable.
 */
extern void SystemInit (void);
/**
 * Update the SystemCoreClock variable
 *
 * @param  none
 * @return none
 *
 * @brief  Update the SystemCoreClock variable after clock setting changed.
 */
extern void SystemCoreClockUpdate (void);

#ifdef __cplusplus
}
#endif

#endif /* __SYSTEM_GCT301S_H */

