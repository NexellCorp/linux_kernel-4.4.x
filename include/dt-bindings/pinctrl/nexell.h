/*
 * Nexell pinctrl bindings
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd.
 * Author: Jaewon Kim <jaewon02.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __DT_BINDINGS_PINCTRL_NEXELL_H__
#define __DT_BINDINGS_PINCTRL_NEXELL_H__

/* Pin function */
#define NX_PIN_FUNC0			0
#define NX_PIN_FUNC1			1
#define NX_PIN_FUNC2			2
#define NX_PIN_FUNC3			3

/* Pin pull Up/Down */
#define NX_PIN_PULL_DOWN		0
#define NX_PIN_PULL_UP			1
#define NX_PIN_PULL_NONE		2

/* Pin drive strength */
#define NX_PIN_STR0			0
#define NX_PIN_STR1			1
#define NX_PIN_STR2			2
#define NX_PIN_STR3			3

/* Pin direction */
#define NX_GPIO_INPUT			0
#define NX_GPIO_OUTPUT			1

/* Pin value */
#define NX_GPIO_LOW			0
#define NX_GPIO_HIGH			1

#endif /* __DT_BINDINGS_PINCTRL_NEXELL_H__ */
