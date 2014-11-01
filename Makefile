#MAKE_OPTS = -C /usr/src/linux SUBDIRS=`pwd` V=1
#MAKE_OPTS = -C /usr/src/linux SUBDIRS=`pwd`
MAKE_OPTS = -C /usr/src/linux-`uname -r` SUBDIRS=`pwd`
PATH_SYSFS := $(shell find /sys/ -name get_icons | sed 's/\/get_icons//')
#EXTRA_CFLAGS += -DDEBUG -O0 -g -Wall
SHELL := $(shell which bash)

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
	cat $(PATH_SYSFS)/model
	echo -n RINGTONE > $(PATH_SYSFS)/show_icon
	date +"%m.%e.%k:%M"  | sed 's/^0/ /' > $(PATH_SYSFS)/line1
	LANG=C date +%a | perl -e 'printf "%.2s", uc(<>);' > $(PATH_SYSFS)/show_icon
	cat $(PATH_SYSFS)/get_icons
	cat $(PATH_SYSFS)/line?
	sleep 6
	echo -n RINGTONE > $(PATH_SYSFS)/hide_icon
	cnt=0 ; while [ $$cnt != 5000 ] ; do \
		dd if=/dev/urandom bs=1 count=6 2>/dev/null | \
		hexdump -e '"%02x"' > $(PATH_SYSFS)/line3 ;\
		let cnt=cnt+1;\
		done

tar:
	rev=$$(svn info | grep "Revision" | awk '{print $$2}'); \
	tar jcvf yealink-r$${rev}.tar.bz2 README TODO Makefile *.[ch]

dist:
	vers=`grep 'define  *DRIVER_VERSION' yealink.c` ; \
	vers=$${vers#*\"} ; vers=$${vers%\"*} ; \
	echo "creating yealink-module-$${vers}.tar.bz2"; \
	mkdir yealink-module-$${vers}; \
	cp README TODO Makefile *.[ch] yealink-module-$${vers}; \
	tar jcvf yealink-module-$${vers}.tar.bz2 yealink-module-$${vers}; \
	rm -Rf yealink-module-$${vers}
