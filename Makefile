#MAKE_OPTS=-C /usr/src/linux SUBDIRS=`pwd` V=1
MAKE_OPTS=-C /usr/src/linux SUBDIRS=`pwd`
PATH_SYSFS :=$(shell find /sys/ -name get_icons | sed 's/\/get_icons//')
#CFLAGS+= -DDEBUG

obj-m += yealink.o

modules:
	make $(MAKE_OPTS) $@

clean:
	make $(MAKE_OPTS) $@

test: modules
	modprobe uhci_hcd ; echo -n
	modprobe evdev ; echo -n
	rmmod yealink ; echo -n
	insmod yealink.ko
	sleep 1
	echo -n RINGTONE > $(PATH_SYSFS)/show_icon
	date +"%m.%e.%k:%M"  | sed 's/^0/ /' > $(PATH_SYSFS)/line1
	date +%a | perl -e 'printf "%.2s", uc(<>);' > $(PATH_SYSFS)/show_icon
	cat $(PATH_SYSFS)/get_icons
	cat $(PATH_SYSFS)/line?
	sleep 6
	echo -n RINGTONE > $(PATH_SYSFS)/hide_icon

tar:
	tar jcvf yealink.tar.bz2 README TODO Makefile *.[ch]
