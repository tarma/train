SOURCE = $(wildcard *.c)
TARGET = $(patsubst %.c,%.o,$(SOURCE))

exec: $(TARGET)
	gcc -o exec $(TARGET)

%.o: %.c
	gcc -g -c $<

clean:
	rm -f $(TARGET)
	rm -f exec
