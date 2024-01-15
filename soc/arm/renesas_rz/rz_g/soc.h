/*
 * Copyright (c) 2024 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef _SOC__H_
#define _SOC__H_

#ifndef _ASMLANGUAGE

/* __CM33_REV                0x0004U   Core revision r0p4 */
#define __SAUREGION_PRESENT	CONFIG_CPU_HAS_ARM_SAU
#define __VTOR_PRESENT		CONFIG_CPU_CORTEX_M_HAS_VTOR
#define __MPU_PRESENT		CONFIG_CPU_HAS_ARM_MPU

/* core0 - no FPU, core1 - FPU */
#define __FPU_PRESENT             0U        /* FPU present */

#endif /* !_ASMLANGUAGE */

#endif /* _SOC__H_ */
