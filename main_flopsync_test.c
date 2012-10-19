/*
 ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
 2011 Giovanni Di Sirio.

 This file is part of ChibiOS/RT.

 ChibiOS/RT is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 3 of the License, or
 (at your option) any later version.

 ChibiOS/RT is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>

#include "ch.h"
#include "hal.h"
#include "halconf.h"
#include "test.h"
#include "shell.h"
#include "chprintf.h"

#define WA_SIZE_1K      THD_WA_SIZE(1024)

uint32_t timer_cnt = 0;
uint32_t last_width = 0;

/*===========================================================================*/
/* Local functions.                                                          */
/*===========================================================================*/

inline unsigned long tick_now(void) {
  return (timer_cnt * (&PWM_DRIVER)->config->period) + (&PWM_DRIVER)->tim->CNT;
}

inline unsigned long time_now(void) {
  return (tick_now() / (&PWM_DRIVER)->config->period);
}

/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/

#define SHELL_WA_SIZE   THD_WA_SIZE(4096)
#define TEST_WA_SIZE    THD_WA_SIZE(1024)

static void cmd_mem(BaseSequentialStream *chp, int argc, char *argv[]) {
  size_t n, size;

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: mem\r\n");
    return;
  }
  n = chHeapStatus(NULL, &size);
  chprintf(chp, "core free memory : %u bytes\r\n", chCoreStatus());
  chprintf(chp, "heap fragments   : %u\r\n", n);
  chprintf(chp, "heap free total  : %u bytes\r\n", size);
}

static void cmd_threads(BaseSequentialStream *chp, int argc, char *argv[]) {
  static const char *states[] =
    {THD_STATE_NAMES};
  Thread *tp;

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: threads\r\n");
    return;
  }
  chprintf(chp, "    addr    stack prio refs     state time\r\n");
  tp = chRegFirstThread();
  do {
    chprintf(chp, "%.8lx %.8lx %4lu %4lu %9s %lu\r\n", (uint32_t)tp,
             (uint32_t)tp->p_ctx.r13, (uint32_t)tp->p_prio,
             (uint32_t)(tp->p_refs - 1), states[tp->p_state],
             (uint32_t)tp->p_time);
    tp = chRegNextThread(tp);
  } while (tp != NULL);
}

static void cmd_pwm(BaseSequentialStream *chp, int argc, char *argv[]) {
  pwmcnt_t pwm;

  (void)argv;
  if (argc != 1) {
    chprintf(chp, "Usage: pwm <width>\r\n");
    return;
  }

  pwm = atoi(argv[0]);
  pwmEnableChannel(&PWM_DRIVER, 0, pwm);
}

static void cmd_can_tx(BaseSequentialStream *chp, int argc, char *argv[]) {
	CANTxFrame ctf;

	(void) argv;

	if (argc > 0) {
		chprintf(chp, "Usage: tx\r\n");
		return;
	}

	ctf.DLC = 0;
	ctf.RTR = CAN_RTR_DATA;
	ctf.IDE = CAN_IDE_EXT;
	ctf.EID = 123;

	canTransmit(&CAND1, &ctf);
}

static const ShellCommand commands[] =
  {
    {"mem", cmd_mem},
     {"threads", cmd_threads},
     {"pwm", cmd_pwm},
     {"tx", cmd_can_tx},
     {NULL, NULL}};

static const ShellConfig shell_cfg1 =
  {(BaseSequentialStream *)&SERIAL_DRIVER, commands};

/*===========================================================================*/
/* PWM related.                                                              */
/*===========================================================================*/

/*
 * PWM update event callback.
 */
static void pwmupdatecb(PWMDriver *pwmp) {

  (void)pwmp;

  chSysLockFromIsr();
  timer_cnt++;
  chSysUnlockFromIsr();
}

/*
 * PWM CH1 callback.
 */
static void pwmch1cb(PWMDriver *pwmp) {

  (void)pwmp;

  palTogglePad(LED_GPIO, LED2);
}

/*
 * PWM configuration structure.
 * Update event callback enabled, channels 1 enabled with callback,
 * the active state is a logic one.
 */
static PWMConfig pwmcfg = {
  1000000,                                 /* 1Mhz PWM clock frequency.   */
  1000,                                    /* PWM period 1ms (in ticks).   */
  pwmupdatecb,
  {
    {PWM_OUTPUT_ACTIVE_HIGH, pwmch1cb},
    {PWM_OUTPUT_DISABLED, NULL},
    {PWM_OUTPUT_DISABLED, NULL},
    {PWM_OUTPUT_DISABLED, NULL}
  },
  /* HW dependent part.*/
  0
};

/*===========================================================================*/
/* ICU related.                                                              */
/*===========================================================================*/

/*
 * ICU width measuremenet callback.
 */

static void icuwidthcb(ICUDriver *icup) {

  last_width = icuGetWidth(icup);
}

static ICUConfig icucfg = {
  ICU_INPUT_ACTIVE_HIGH,
  1000000,                                    /* 1MHz ICU clock frequency.   */
  icuwidthcb,
  NULL,
  NULL,
  ICU_CHANNEL_1
};

/*===========================================================================*/
/* CAN related.                                                              */
/*===========================================================================*/

/*
 * CAN transmit complete callback.
 */
static void can_tx_cb(CANDriver *canp, flagsmask_t flags) {

	(void) canp;
	(void) flags;

	palTogglePad(LED_GPIO, LED3);
}

/*
 * CAN receive callback.
 */
static void can_rx_cb(CANDriver *canp) {

	(void) canp;

	palTogglePad(LED_GPIO, LED4);
}

/*
 * CAN configuration.
 */
static const CANConfig can_cfg = { can_tx_cb, can_rx_cb, NULL,
		CAN_MCR_NART | CAN_MCR_TTCM, CAN_BTR_SJW(0) | CAN_BTR_TS2(2)
				| CAN_BTR_TS1(4) | CAN_BTR_BRP(3) };

/*===========================================================================*/
/* Application threads.                                                      */
/*===========================================================================*/

/*
 * Red LED blinker thread, times are in milliseconds.
 */
static WORKING_AREA(waThread1, 128);
static msg_t Thread1(void *arg) {

  (void)arg;

  chRegSetThreadName("blinker");
  while (TRUE) {
    palTogglePad(LED_GPIO, LED1);
    chThdSleepMilliseconds(500);
  }
  return 0;
}

/*
 * Application entry point.
 */
int main(void) {
  Thread *shelltp = NULL;

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
   * Activates the serial driver 1 using the driver default configuration.
   */
  sdStart(&SERIAL_DRIVER, NULL);

  /*
   * Initializes the PWM driver.
   */
  pwmStart(&PWM_DRIVER, &pwmcfg);
  pwmEnableChannel(&PWM_DRIVER, 0, 100);

  /*
   * Initializes the ICU driver.
   */
  icuStart(&ICU_DRIVER, &icucfg);
  icuEnable(&ICU_DRIVER);

  /*
   * Initializes the CAN driver and set a pass-through filter.
   */
  canStart(&CAN_DRIVER, &can_cfg);
  canClearFilters(&CAN_DRIVER);

  /*
   * Shell manager initialization.
   */
  shellInit();

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   */
  while (TRUE) {
    if (!shelltp)
      shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, NORMALPRIO);
    else if (chThdTerminated(shelltp)) {
      chThdRelease(shelltp);
      shelltp = NULL;
    }
    chprintf((BaseSequentialStream *)&SERIAL_DRIVER, "tick: %U ms: %U W: %d\r\n", tick_now(), time_now(), last_width);
    chThdSleepMilliseconds(500);
  }
}
