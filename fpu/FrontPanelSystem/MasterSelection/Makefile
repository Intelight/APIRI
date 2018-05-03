SOURCES := MasterSelection.c tohex.c xprintf.c

LIBRARIES := -L../libfpui -lfpui -lpthread

INCLUDES := ../FrontPanelDriver/front_panel.h

OBJECTS := $(SOURCES:.c=.o)

.c.o:
	$(CC) -O2 -Wall -I../ -I../FrontPanelDriver -I../libfpui -c $<

## Targets
TARGET := MasterSelection

all: $(TARGET)

$(TARGET) : $(OBJECTS) $(INCLUDES)
	$(CC) -O2 -o $@ $(OBJECTS) $(LIBRARIES)

install:
	cp $(TARGET) $(HOME)/bin
	chmod 755 $(HOME)/bin/$(TARGET)

clean:
	rm -f $(OBJECTS) $(TARGET) $(HOME)/bin/$(TARGET)
