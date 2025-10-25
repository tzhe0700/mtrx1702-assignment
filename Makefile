CC = gcc
CFLAGS = -Wall -Wextra -std=c11 -g
TARGET = rover
SOURCES = src/rover.c src/bitmap.c
HEADERS = include/bitmap.h

$(TARGET): $(SOURCES) $(HEADERS)
	$(CC) $(CFLAGS) -o $(TARGET) $(SOURCES) -Iinclude

clean:
	rm -f $(TARGET)

.PHONY: clean
