/*
 * Copyright (c) 2023 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * This very simple hello world C code can be used as a test case for building probably the simplest
 * loadable module. It requires a single symbol be linked, section relocation support, and the ability
 * to export and call out to a function.
 *
 */

/* Various build options should be documented here to generate the test elf for each architecture
 *
 * armv7-thumb: -mlong-call -mthumb -c -o hello_world_armv7_thumb.elf hello_world.c
 * i386: -m32 -march=lakemont -mtune=lakemont -c -o hello_world_386.elf hello_world.c
 * xtensa: -mlongcalls -mtext-section-literals -c -o hello_world_xtensa.elf hello_world.c
 */

extern void printk(char *fmt, ...);

extern void hello_world(void)
{
	printk("hello world\n");
}
