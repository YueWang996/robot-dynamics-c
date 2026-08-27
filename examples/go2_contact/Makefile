# SPDX-License-Identifier: Apache-2.0
RD    := ../../RobotDynamics
MODEL := ../../benchmark/models

CFLAGS := -O2 -std=c11 -Wall -Wextra -I$(RD) -I$(MODEL) \
          -DRD_MAX_LINKS=40 -DRD_MAX_JOINTS=24

go2_contact: go2_contact.c $(RD)/rd_algorithms.c $(RD)/rd_chain.c $(RD)/rd_state.c
	$(CC) $(CFLAGS) $^ -lm -o $@

clean:
	rm -f go2_contact

.PHONY: clean
