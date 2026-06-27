// SPDX-License-Identifier: GPL-2.0+

#include <command.h>
#include <linux/string.h>
#include <mapmem.h>
#include <vsprintf.h>
#include <console.h>

#define POKERAM_MAGIC 0x55555555

static int do_pokeram(struct cmd_tbl *cmdtp, int flag, int argc,
		      char *const argv[])
{
	ulong start_addr;
	ulong end_addr;
	ulong count;
	void *start;
	void *end;
	void *p;
	void *last_print = NULL;
	u64 old, new;
	volatile u64 *wordp;

	printf("argc=%d\n", argc);
	printf("sizeof(ulong)=%zu sizeof(phys_addr_t)=%zu sizeof(uintptr_t)=%zu\n",
	       sizeof(ulong), sizeof(phys_addr_t), sizeof(uintptr_t));

	if (argc != 3)
		return CMD_RET_USAGE;

	start_addr = simple_strtoull(argv[1], NULL, 16);
	end_addr = simple_strtoull(argv[2], NULL, 16);

	if (!start_addr || !end_addr || start_addr >= end_addr) {
		printf("Invalid arguments: start_addr=0x%lx end_addr=0x%lx\n",
		       start_addr, end_addr);
		return CMD_RET_FAILURE;
	}

	count = end_addr - start_addr;
	start = map_sysmem(start_addr, count);
	end = start + count;
	for (p = start; p < end; p += sizeof(wordp)) {
		if (ctrlc()) {
			printf("interrupted at 0x%p\n", p);

			return CMD_RET_FAILURE;
		}

		if (p - last_print > 0x40000) {
			last_print = p;
			printf("looking at 0x%p\n", p);
		}

		wordp = p;
		old = *wordp;
		*wordp = POKERAM_MAGIC;
		new = *wordp;
		*wordp = old;

		if (new != POKERAM_MAGIC) {
			printf("hole 0x%p old=0x%llx new=0x%llx\n", p, old,
			       new);
		}
	}
	unmap_sysmem(start);

	return 0;
}

U_BOOT_CMD(pokeram, 3, 0, do_pokeram, "poke RAM to find protected regions",
	   "start end\n"
	   "    - all values hex");
