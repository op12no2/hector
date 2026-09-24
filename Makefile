CFLAGS ?= -O2 -Wall -Wextra
hector: hector.c
	$(CC) $(CFLAGS) -o $@ $< -lm
clean:
	rm -f hector
.PHONY: clean
