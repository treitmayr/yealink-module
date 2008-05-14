/*
 * drivers/usb/input/yealink.c
 *
 * Copyright (c) 2005,2006 Henk Vergonet <Henk.Vergonet@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
/*
 * Description:
 *   Driver for the USB-P1K voip usb phone.
 *   This device is produced by Yealink Network Technology Co Ltd
 *   but may be branded under several names:
 *	- Yealink usb-p1k
 *	- Tiptel 115
 *	- ...
 *
 * This driver is based on:
 *   - the usbb2k-api	http://savannah.nongnu.org/projects/usbb2k-api/
 *   - information from	http://memeteau.free.fr/usbb2k
 *   - the xpad-driver	drivers/usb/input/xpad.c
 *
 * Thanks to:
 *   - Olivier Vandorpe, for providing the usbb2k-api.
 *   - Martin Diehl, for spotting my memory allocation bug.
 *   - Steve Underwood, for reverse engineering the P4K.
 *   - Toshiharu Ohno, fixing key event initialization bug and adding
 *		P4K hookflash support.
 *
 * History:
 *   20050527 henk	First version, functional keyboard. Keyboard events
 *			will pop-up on the ../input/eventX bus.
 *   20050531 henk	Added led, LCD, dialtone and sysfs interface.
 *   20050610 henk	Cleanups, make it ready for public consumption.
 *   20050630 henk	Cleanups, fixes in response to comments.
 *   20050701 henk	sysfs write serialisation, fix potential unload races
 *   20050801 henk	Added ringtone, restructure USB
 *   20050816 henk	Merge 2.6.13-rc6
 *   20060220 henk	Experimental P4K support
 *   20060430 Toshiharu	fixing key event initialization bug.
 *   20060503 Toshiharu	Added hookflash support.
 *   20060503 Henk	Added privacy, only allow user and group access to sysfs
 *   20060830 Henk	Proper urb cleanup cycle, thanks Ivan Jensen for
 *   			pointing this out.
 */

#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/rwsem.h>
#include <linux/usb.h>
#include <linux/usb_input.h>

#include "map_to_7segment.h"
#include "yealink.h"

#define DRIVER_VERSION "yld-20060830"
#define DRIVER_AUTHOR "Henk Vergonet"
#define DRIVER_DESC "Yealink phone driver"

#define YEALINK_POLLING_FREQUENCY	10	/* in [Hz] */

struct yld_status {
	u8	lcd[24];
	u8	led;
	u8	dialtone;
	u8	ringtone;
	u8	backlight;
	u8	speaker;
	u8	pstn;
	u8	keynum;
} __attribute__ ((packed));

/*
 * Register the LCD segment and icon map
 */
#define _LOC(k,l)	{ .a = (k), .m = (l) }
#define _SEG(t, a, am, b, bm, c, cm, d, dm, e, em, f, fm, g, gm)	\
	{ .type	= (t),							\
	  .u = { .s = {	_LOC(a, am), _LOC(b, bm), _LOC(c, cm),		\
		        _LOC(d, dm), _LOC(e, em), _LOC(g, gm),		\
			_LOC(f, fm) } } }
#define _PIC(t, h, hm, n)						\
	{ .type	= (t),							\
 	  .u = { .p = { .name = (n), .a = (h), .m = (hm) } } }

static const struct lcd_segment_map {
	char	type;
	union {
		struct pictogram_map {
			u8	a,m;
			char	name[10];
		}	p;
		struct segment_map {
			u8	a,m;
		} s[7];
	} u;
} lcdMap[] = {
#include "yealink.h"
};

struct model_info {
	int (*keycode)(unsigned scancode);
	char *name;
};

struct yealink_dev {
	struct input_dev *idev;		/* input device */
	struct usb_device *udev;	/* usb device */
	struct model_info *model;

	/* irq input channel */
	struct yld_ctl_packet	*irq_data;
	dma_addr_t		irq_dma;
	struct urb		*urb_irq;

	/* control output channel */
	struct yld_ctl_packet	*ctl_data;
	dma_addr_t		ctl_dma;
	struct usb_ctrlrequest	*ctl_req;
	dma_addr_t		ctl_req_dma;
	struct urb		*urb_ctl;

	char phys[64];			/* physical device path */

	u8 lcdMap[ARRAY_SIZE(lcdMap)];	/* state of LCD, LED ... */
	int key_code;			/* last reported key	 */

	int	stat_ix;
	u8	last_cmd;
	u8	hookstate;		/* P4K hookstate	*/
	union {
		struct yld_status s;
		u8		  b[sizeof(struct yld_status)];
	} master, copy;
};

static char p1k_model[] = "P1K";
static char p4k_model[] = "P4K";
static char b2k_model[] = "B2K";
static int map_p1k_to_key(unsigned);
static int map_p4k_to_key(unsigned);
static int map_b2k_to_key(unsigned);

static struct model_info model[] = {
	{
		.keycode = map_p1k_to_key,
		.name	 = p1k_model,
	},{
		.keycode = map_p4k_to_key,
		.name	 = p4k_model,
	},{
		.keycode = map_b2k_to_key,
		.name	 = b2k_model,
	}
};

static char *default_model = p1k_model;

/*******************************************************************************
 * Yealink lcd interface
 ******************************************************************************/

/*
 * Register a default 7 segment character set
 */
static SEG7_DEFAULT_MAP(map_seg7);

 /* Display a char,
  * char '\9' and '\n' are placeholders and do not overwrite the original text.
  * A space will always hide an icon.
  */
static int setChar(struct yealink_dev *yld, int el, int chr)
{
	int i, a, m, val;

	if (el >= ARRAY_SIZE(lcdMap))
		return -EINVAL;

	if (chr == '\t' || chr == '\n')
	    return 0;

	yld->lcdMap[el] = chr;

	if (lcdMap[el].type == '.') {
		a = lcdMap[el].u.p.a;
		m = lcdMap[el].u.p.m;
		if (chr != ' ')
			yld->master.b[a] |= m;
		else
			yld->master.b[a] &= ~m;
		return 0;
	}

	val = map_to_seg7(&map_seg7, chr);
	for (i = 0; i < ARRAY_SIZE(lcdMap[0].u.s); i++) {
		m = lcdMap[el].u.s[i].m;

		if (m == 0)
			continue;

		a = lcdMap[el].u.s[i].a;
		if (val & 1)
			yld->master.b[a] |= m;
		else
			yld->master.b[a] &= ~m;
		val = val >> 1;
	}
	return 0;
};

/*******************************************************************************
 * Yealink key interface
 ******************************************************************************/

/* USB-P1K button layout:
 *
 *             up
 *       IN           OUT
 *            down
 *
 *     pickup   C    hangup
 *       1      2      3
 *       4      5      6
 *       7      8      9
 *       *      0      #
 *
 * The "up" and "down" keys, are symbolized by arrows on the button.
 * The "pickup" and "hangup" keys are symbolized by a green and red phone
 * on the button.
 */
static int map_p1k_to_key(unsigned scancode)
{
	static const int map[] = {		/* code	key	*/
		KEY_1,			 	/* 00 1		*/
		KEY_2,				/* 01 2		*/
		KEY_3,				/* 02 3		*/
		KEY_ENTER,			/* 03 pickup	*/
		KEY_RIGHT,			/* 04 OUT	*/
		-EINVAL,			/* 05		*/
		-EINVAL,			/* 06		*/
		-EINVAL,			/* 07		*/
		KEY_4,				/* 10 4		*/
		KEY_5,				/* 11 5		*/
		KEY_6,				/* 12 6		*/
		KEY_ESC,			/* 13 hangup	*/
		KEY_BACKSPACE,			/* 14 C		*/
		-EINVAL,			/* 15		*/
		-EINVAL,			/* 16		*/
		-EINVAL,			/* 17		*/
		KEY_7,				/* 20 7		*/
		KEY_8,				/* 21 8		*/
		KEY_9,				/* 22 9		*/
		KEY_LEFT,			/* 23 IN	*/
		KEY_DOWN,			/* 24 down	*/
		-EINVAL,			/* 25		*/
		-EINVAL,			/* 26		*/
		-EINVAL,			/* 27		*/
		KEY_KPASTERISK,			/* 30 *		*/
		KEY_0,				/* 31 0		*/
		KEY_LEFTSHIFT | KEY_3 << 8,	/* 32 #		*/
		KEY_UP				/* 33 up	*/
	};
	if(scancode & 0x8)
		return -EINVAL;
	scancode = (scancode & 7) | ((scancode & 0xf0) >> 1);

	if(scancode < ARRAY_SIZE(map))
		return map[scancode];

	return -EINVAL;
}

/* USB-P4K button layout:
 *
 *	     IN      up     OUT
 *	     VOL+	    DEL
 *	     VOL-   down    DIAL
 *
 *	       1      2      3
 *	       4      5      6
 *	       7      8      9
 *	       *      0      #
 *
 *       HELP                   SEND
 *      FLASH     handsfree     REDIAL
 */
static int map_p4k_to_key(unsigned scancode)
{
	static const int map[] = {		/* code	key	*/
		KEY_ENTER,		 	/* 00 DIAL	*/
		KEY_3,				/* 01 3		*/
		KEY_6,				/* 02 6		*/
		KEY_9,				/* 03 9		*/
		KEY_LEFTSHIFT | KEY_3 << 8,	/* 04 #		*/
		KEY_LEFTSHIFT | KEY_SLASH << 8,	/* 05 HELP	*/
		-EINVAL,			/* 06		*/
		-EINVAL,			/* 07		*/
		KEY_RIGHT,			/* 10 OUT	*/
		KEY_2,				/* 11 2		*/
		KEY_5,				/* 12 5		*/
		KEY_8,				/* 13 8		*/
		KEY_0,				/* 14 0		*/
		KEY_ESC,			/* 15 FLASH	*/
		-EINVAL,			/* 16		*/
		-EINVAL,			/* 17		*/
		KEY_H,				/* 20 handsfree	*/
		KEY_1,				/* 21 1		*/
		KEY_4,				/* 22 4		*/
		KEY_7,				/* 23 7		*/
		KEY_KPASTERISK,			/* 24 *		*/
		KEY_S,				/* 25 SEND	*/
		-EINVAL,			/* 26		*/
		-EINVAL,			/* 27		*/
		KEY_DOWN,			/* 30 DOWN	*/
		KEY_V,				/* 31 VOL+	*/
		KEY_UP,				/* 32 UP	*/
		KEY_BACKSPACE,			/* 33 DEL	*/
		KEY_LEFT,			/* 34 IN	*/
		-EINVAL,			/* 35		*/
		-EINVAL,			/* 36		*/
		-EINVAL,			/* 37		*/
		KEY_LEFTSHIFT | KEY_V << 8,	/* 40 VOL-	*/
		-EINVAL,			/* 41		*/
		-EINVAL,			/* 42		*/
		-EINVAL,			/* 43		*/
		KEY_R				/* 44 REDIAL	*/
	};
	if(scancode & 0x8)
		return -EINVAL;
	scancode = (scancode & 7) | ((scancode & 0xf0) >> 1);

	if(scancode < ARRAY_SIZE(map))
		return map[scancode];

	return -EINVAL;
}

/* USB-B2K buttons generated by the DTMF decoder in the device:
 *
 *	       1      2      3
 *	       4      5      6
 *	       7      8      9
 *	       *      0      #
 */
static int map_b2k_to_key(unsigned scancode)
{
	/* TODO */
	return -EINVAL;
}

/* Completes a request by converting the data into events for the
 * input subsystem.
 *
 * The key parameter can be cascaded: key2 << 8 | key1
 */
static void report_key(struct yealink_dev *yld, int key, struct pt_regs *regs)
{
	struct input_dev *idev = yld->idev;

	input_regs(idev, regs);
	if (yld->key_code >= 0) {
		/* old key up */
		input_report_key(idev, yld->key_code & 0xff, 0);
		if (yld->key_code >> 8)
			input_report_key(idev, yld->key_code >> 8, 0);
	}

	yld->key_code = key;
	if (key >= 0) {
		/* new valid key */
		input_report_key(idev, key & 0xff, 1);
		if (key >> 8)
			input_report_key(idev, key >> 8, 1);
	}
	input_sync(idev);
}

/*******************************************************************************
 * Yealink usb communication interface
 ******************************************************************************/

static int yealink_cmd(struct yealink_dev *yld, struct yld_ctl_packet *p)
{
	u8	*buf = (u8 *)p;
	int	i;
	u8	sum = 0;

	for(i=0; i<USB_PKT_LEN-1; i++)
		sum -= buf[i];
	p->sum = sum;
	return usb_control_msg(yld->udev,
			usb_sndctrlpipe(yld->udev, 0),
			USB_REQ_SET_CONFIGURATION,
			USB_TYPE_CLASS | USB_RECIP_INTERFACE | USB_DIR_OUT,
			0x200, 3,
			p, sizeof(*p),
			USB_CTRL_SET_TIMEOUT);
}

static u8 default_ringtone[] = {
	0xEF,			/* volume [0-255] */
	0xFB, 0x1E, 0x00, 0x0C,	/* 1250 [hz], 12/100 [s] */
	0xFC, 0x18, 0x00, 0x0C,	/* 1000 [hz], 12/100 [s] */
	0xFB, 0x1E, 0x00, 0x0C,
	0xFC, 0x18, 0x00, 0x0C,
	0xFB, 0x1E, 0x00, 0x0C,
	0xFC, 0x18, 0x00, 0x0C,
	0xFB, 0x1E, 0x00, 0x0C,
	0xFC, 0x18, 0x00, 0x0C,
	0xFF, 0xFF, 0x01, 0x90,	/* silent, 400/100 [s] */
	0x00, 0x00		/* end of sequence */
};

static int yealink_set_ringtone(struct yealink_dev *yld, u8 *buf, size_t size)
{
	struct yld_ctl_packet *p = yld->ctl_data;
	int	ix, len;

	if (size <= 0)
		return -EINVAL;

	/* Set the ringtone volume */
	memset(yld->ctl_data, 0, sizeof(*(yld->ctl_data)));
	yld->ctl_data->cmd	= CMD_RING_VOLUME;
	yld->ctl_data->size	= 1;
	yld->ctl_data->data[0]	= buf[0];
	yealink_cmd(yld, p);

	buf++;
	size--;

	p->cmd = CMD_RING_NOTE;
	ix = 0;
	while (size != ix) {
		len = size - ix;
		if (len > sizeof(p->data))
			len = sizeof(p->data);
		p->size	  = len;
		p->offset = cpu_to_be16(ix);
		memcpy(p->data, &buf[ix], len);
		yealink_cmd(yld, p);
		ix += len;
	}
	return 0;
}

/* keep stat_master & stat_copy in sync.
 */
static int yealink_do_idle_tasks(struct yealink_dev *yld)
{
	u8 val;
	int i, ix, len;

	ix = yld->stat_ix;

	memset(yld->ctl_data, 0, sizeof(*(yld->ctl_data)));

	if (yld->model->name == p4k_model && yld->last_cmd == CMD_KEYPRESS) {
	    yld->ctl_data->cmd  = CMD_HOOKPRESS;
	    yld->ctl_data->size = 1;
	    yld->ctl_data->sum  = 0xff - CMD_HOOKPRESS;
	    yld->last_cmd = CMD_HOOKPRESS;
	} else {
	    yld->ctl_data->cmd  = CMD_KEYPRESS;
	    yld->ctl_data->size = 1;
	    yld->ctl_data->sum  = 0xff - CMD_KEYPRESS;
	    yld->last_cmd = CMD_KEYPRESS;
	}

	/* If state update pointer wraps do a KEYPRESS first. */
	if (ix >= sizeof(yld->master)) {
		yld->stat_ix = 0;
		return 0;
	}

	/* find update candidates: copy != master */
	do {
		val = yld->master.b[ix];
		if (val != yld->copy.b[ix])
			goto send_update;
	} while (++ix < sizeof(yld->master));

	/* nothing todo, wait a bit and poll for a KEYPRESS */
	yld->stat_ix = 0;
	/* TODO how can we wait abit. ??
	 * msleep_interruptible(1000 / YEALINK_POLLING_FREQUENCY);
	 */
	return 0;

send_update:

	/* Setup an appropriate update request */
	yld->copy.b[ix] = val;
	yld->ctl_data->data[0] = val;

	switch(ix) {
	case offsetof(struct yld_status, led):
		yld->ctl_data->cmd	= CMD_LED;
		yld->ctl_data->sum	= -1 - CMD_LED - val;
		break;
	case offsetof(struct yld_status, dialtone):
		yld->ctl_data->cmd	= CMD_DIALTONE;
		yld->ctl_data->sum	= -1 - CMD_DIALTONE - val;
		break;
	case offsetof(struct yld_status, ringtone):
		yld->ctl_data->cmd	= CMD_RINGTONE;
		yld->ctl_data->sum	= -1 - CMD_RINGTONE - val;
		break;
	case offsetof(struct yld_status, backlight):
		yld->ctl_data->cmd	= CMD_LCD_BACKLIGHT;
		yld->ctl_data->sum	= -1 - CMD_LCD_BACKLIGHT - val;
		break;
	case offsetof(struct yld_status, speaker):
		yld->ctl_data->cmd	= CMD_SPEAKER;
		yld->ctl_data->sum	= -1 - CMD_SPEAKER - val;
		break;
	case offsetof(struct yld_status, pstn):
		yld->ctl_data->cmd	= CMD_PSTN_SWITCH;
		yld->ctl_data->sum	= -1 - CMD_PSTN_SWITCH - val;
		break;
	case offsetof(struct yld_status, keynum):
		val--;
		val &= 0x1f;
		yld->ctl_data->cmd	= CMD_SCANCODE;
		yld->ctl_data->offset	= cpu_to_be16(val);
		yld->ctl_data->data[0]	= 0;
		yld->ctl_data->sum	= -1 - CMD_SCANCODE - val;
		break;
	default:
		len = sizeof(yld->master.s.lcd) - ix;
		if (len > sizeof(yld->ctl_data->data))
			len = sizeof(yld->ctl_data->data);

		/* Combine up to <len> consecutive LCD bytes in a singe request
		 */
		yld->ctl_data->cmd	= CMD_LCD;
		yld->ctl_data->offset	= cpu_to_be16(ix);
		yld->ctl_data->size	= len;
		yld->ctl_data->sum	= -CMD_LCD - ix - val - len;
		for(i=1; i<len; i++) {
			ix++;
			val = yld->master.b[ix];
			yld->copy.b[ix]		= val;
			yld->ctl_data->data[i]	= val;
			yld->ctl_data->sum     -= val;
		}
	}
	yld->stat_ix = ix + 1;
	return 1;
}

/* Decide on how to handle responses
 *
 * The state transition diagram is something like:
 *
 *          syncState<--+
 *               |      |
 *               |    idle
 *              \|/     |
 * init --ok--> waitForKey --ok--> getKey
 *  ^               ^                |
 *  |               +-------ok-------+
 * error,start
 *
 */
static void urb_irq_callback(struct urb *urb, struct pt_regs *regs)
{
	struct yealink_dev *yld = urb->context;
	int ret;

	if (urb->status)
		err("%s - urb status %d", __FUNCTION__, urb->status);

	switch (yld->irq_data->cmd) {
	case CMD_KEYPRESS:

		yld->master.s.keynum = yld->irq_data->data[0];
		break;

	case CMD_HOOKPRESS:
		ret = yld->irq_data->data[0];
		if (yld->hookstate != ret) {
		    dbg("hookstate: %x", ret);
		    input_regs(yld->idev, regs);
		    input_report_key(yld->idev, KEY_PHONE, (ret & 0x10) >> 4);
		    input_sync(yld->idev);
		    yld->hookstate = ret;
		}
		break;

	case CMD_SCANCODE:
		dbg("get scancode %x", yld->irq_data->data[0]);

		ret = yld->model->keycode(yld->irq_data->data[0]);
		report_key(yld, ret, regs);
		if(ret < 0 && yld->irq_data->data[0] != 0xff)
			info("unknown scancode 0x%x", yld->irq_data->data[0]);
		break;

	default:
		err("unexpected response %x", yld->irq_data->cmd);
	}

	yealink_do_idle_tasks(yld);

	ret = usb_submit_urb(yld->urb_ctl, GFP_ATOMIC);
	if (ret)
		err("%s - usb_submit_urb failed %d", __FUNCTION__, ret);
}

static void urb_ctl_callback(struct urb *urb, struct pt_regs *regs)
{
	struct yealink_dev *yld = urb->context;
	int ret;

	if (urb->status)
		err("%s - urb status %d", __FUNCTION__, urb->status);

	switch (yld->ctl_data->cmd) {
	case CMD_HOOKPRESS:
	case CMD_KEYPRESS:
	case CMD_SCANCODE:
		/* ask for a response */
		ret = usb_submit_urb(yld->urb_irq, GFP_ATOMIC);
		break;
	default:
		/* send new command */
		yealink_do_idle_tasks(yld);
		ret = usb_submit_urb(yld->urb_ctl, GFP_ATOMIC);
	}

	if (ret)
		err("%s - usb_submit_urb failed %d", __FUNCTION__, ret);
}

/*******************************************************************************
 * input event interface
 ******************************************************************************/

/* TODO should we issue a ringtone on a SND_BELL event?
static int input_ev(struct input_dev *dev, unsigned int type,
		unsigned int code, int value)
{

	if (type != EV_SND)
		return -EINVAL;

	switch (code) {
	case SND_BELL:
	case SND_TONE:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}
*/

static int input_open(struct input_dev *dev)
{
	struct yealink_dev *yld = dev->private;
	int i, ret;

	dbg("%s", __FUNCTION__);

	/* force updates to device */
	for (i = 0; i<sizeof(yld->master); i++)
		yld->copy.b[i] = ~yld->master.b[i];
	yld->key_code = -1;	/* no keys pressed */

        yealink_set_ringtone(yld, default_ringtone, sizeof(default_ringtone));

	/* issue INIT */
	memset(yld->ctl_data, 0, sizeof(*(yld->ctl_data)));
	yld->ctl_data->cmd	= CMD_INIT;
	yld->ctl_data->size	= 10;
	yld->ctl_data->sum	= 0x100-CMD_INIT-10;
	if ((ret = usb_submit_urb(yld->urb_ctl, GFP_KERNEL)) != 0) {
		dbg("%s - usb_submit_urb failed with result %d",
		     __FUNCTION__, ret);
		return ret;
	}
	return 0;
}

static void input_close(struct input_dev *dev)
{
	struct yealink_dev *yld = dev->private;

	usb_kill_urb(yld->urb_ctl);
	usb_kill_urb(yld->urb_irq);
}

/*******************************************************************************
 * sysfs interface
 ******************************************************************************/

static DECLARE_RWSEM(sysfs_rwsema);

/* Interface to the 7-segments translation table aka. char set.
 */
static ssize_t show_map(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	memcpy(buf, &map_seg7, sizeof(map_seg7));
	return sizeof(map_seg7);
}

static ssize_t store_map(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t cnt)
{
	if (cnt != sizeof(map_seg7))
		return -EINVAL;
	memcpy(&map_seg7, buf, sizeof(map_seg7));
	return sizeof(map_seg7);
}

/* Interface to the LCD.
 */

/* Reading /sys/../lineX will return the format string with its settings:
 *
 * Example:
 * cat ./line3
 * 888888888888
 * Linux Rocks!
 */
static ssize_t show_line(struct device *dev, char *buf, int a, int b)
{
	struct yealink_dev *yld;
	int i;

	down_read(&sysfs_rwsema);
	yld = dev_get_drvdata(dev);
	if (yld == NULL) {
		up_read(&sysfs_rwsema);
		return -ENODEV;
	}

	for (i = a; i < b; i++)
		*buf++ = lcdMap[i].type;
	*buf++ = '\n';
	for (i = a; i < b; i++)
		*buf++ = yld->lcdMap[i];
	*buf++ = '\n';
	*buf = 0;

	up_read(&sysfs_rwsema);
	return 3 + ((b - a) << 1);
}

static ssize_t show_line1(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return show_line(dev, buf, LCD_LINE1_OFFSET, LCD_LINE2_OFFSET);
}

static ssize_t show_line2(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return show_line(dev, buf, LCD_LINE2_OFFSET, LCD_LINE3_OFFSET);
}

static ssize_t show_line3(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return show_line(dev, buf, LCD_LINE3_OFFSET, LCD_LINE4_OFFSET);
}

/* Writing to /sys/../lineX will set the coresponding LCD line.
 * - Excess characters are ignored.
 * - If less characters are written than allowed, the remaining digits are
 *   unchanged.
 * - The '\n' or '\t' char is a placeholder, it does not overwrite the
 *   original content.
 */
static ssize_t store_line(struct device *dev, const char *buf, size_t count,
		int el, size_t len)
{
	struct yealink_dev *yld;
	int i;

	down_write(&sysfs_rwsema);
	yld = dev_get_drvdata(dev);
	if (yld == NULL) {
		up_write(&sysfs_rwsema);
		return -ENODEV;
	}

	if (len > count)
		len = count;
	for (i = 0; i < len; i++)
		setChar(yld, el++, buf[i]);

	up_write(&sysfs_rwsema);
	return count;
}

static ssize_t store_line1(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	return store_line(dev, buf, count, LCD_LINE1_OFFSET, LCD_LINE1_SIZE);
}

static ssize_t store_line2(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	return store_line(dev, buf, count, LCD_LINE2_OFFSET, LCD_LINE2_SIZE);
}

static ssize_t store_line3(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	return store_line(dev, buf, count, LCD_LINE3_OFFSET, LCD_LINE3_SIZE);
}

/* Interface to visible and audible "icons", these include:
 * pictures on the LCD, the LED, and the dialtone signal.
 */

/* Get a list of "switchable elements" with their current state. */
static ssize_t get_icons(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct yealink_dev *yld;
	int i, ret = 1;

	down_read(&sysfs_rwsema);
	yld = dev_get_drvdata(dev);
	if (yld == NULL) {
		up_read(&sysfs_rwsema);
		return -ENODEV;
	}

	for (i = 0; i < ARRAY_SIZE(lcdMap); i++) {
		if (lcdMap[i].type != '.')
			continue;
		ret += sprintf(&buf[ret], "%s %s\n",
				yld->lcdMap[i] == ' ' ? "  " : "on",
				lcdMap[i].u.p.name);
	}
	up_read(&sysfs_rwsema);
	return ret;
}

/* Change the visibility of a particular element. */
static ssize_t set_icon(struct device *dev, const char *buf, size_t count,
			int chr)
{
	struct yealink_dev *yld;
	int i;

	down_write(&sysfs_rwsema);
	yld = dev_get_drvdata(dev);
	if (yld == NULL) {
		up_write(&sysfs_rwsema);
		return -ENODEV;
	}

	for (i = 0; i < ARRAY_SIZE(lcdMap); i++) {
		if (lcdMap[i].type != '.')
			continue;
		if (strncmp(buf, lcdMap[i].u.p.name, count) == 0) {
			setChar(yld, i, chr);
			break;
		}
	}

	up_write(&sysfs_rwsema);
	return count;
}

static ssize_t show_icon(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	return set_icon(dev, buf, count, buf[0]);
}

static ssize_t hide_icon(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	return set_icon(dev, buf, count, ' ');
}

/* Upload a ringtone to the device.
 */

/* Stores raw ringtone data in the phone */
static ssize_t store_ringtone(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct yealink_dev *yld;

	down_write(&sysfs_rwsema);
	yld = dev_get_drvdata(dev);
	if (yld == NULL) {
		up_write(&sysfs_rwsema);
		return -ENODEV;
	}

	/* TODO locking with async usb control interface??? */
	yealink_set_ringtone(yld, (char *)buf, count);
	up_write(&sysfs_rwsema);
	return count;
}

/* Get a list phone models with the currently selected one. */
static ssize_t show_model(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct yealink_dev *yld;
	int i, ret = 1;

	down_read(&sysfs_rwsema);
	yld = dev_get_drvdata(dev);
	if (yld == NULL) {
		up_read(&sysfs_rwsema);
		return -ENODEV;
	}

	for (i = 0; i < ARRAY_SIZE(model); i++) {
		ret += sprintf(&buf[ret], "%c %s\n",
				&model[i] == yld->model ? '*' : ' ',
				model[i].name);
	}
	up_read(&sysfs_rwsema);
	return ret;
}

/* Change phone model */
static ssize_t store_model(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct yealink_dev *yld;
	int i;

	down_write(&sysfs_rwsema);
	yld = dev_get_drvdata(dev);
	if (yld == NULL) {
		up_write(&sysfs_rwsema);
		return -ENODEV;
	}

	for(i=0; i<ARRAY_SIZE(model); i++) {
		if(strcmp(model[i].name, buf) == 0) {
			yld->model = &model[i];
			up_write(&sysfs_rwsema);
			return count;
		}
	}
	up_write(&sysfs_rwsema);
	return -ENXIO;
}

/* In order to prevent information leaks, only allow user and group access */
#define _M220	S_IWUSR	| S_IWGRP
#define _M440	S_IRUSR	| S_IRGRP
#define _M660	_M220	| _M440

static DEVICE_ATTR(map_seg7	, _M660, show_map	, store_map	);
static DEVICE_ATTR(line1	, _M660, show_line1	, store_line1	);
static DEVICE_ATTR(line2	, _M660, show_line2	, store_line2	);
static DEVICE_ATTR(line3	, _M660, show_line3	, store_line3	);
static DEVICE_ATTR(get_icons	, _M440, get_icons	, NULL		);
static DEVICE_ATTR(show_icon	, _M220, NULL		, show_icon	);
static DEVICE_ATTR(hide_icon	, _M220, NULL		, hide_icon	);
static DEVICE_ATTR(ringtone	, _M220, NULL		, store_ringtone);
static DEVICE_ATTR(model	, _M660, show_model	, store_model	);

static struct attribute *yld_attributes[] = {
	&dev_attr_line1.attr,
	&dev_attr_line2.attr,
	&dev_attr_line3.attr,
	&dev_attr_get_icons.attr,
	&dev_attr_show_icon.attr,
	&dev_attr_hide_icon.attr,
	&dev_attr_map_seg7.attr,
	&dev_attr_ringtone.attr,
	&dev_attr_model.attr,
	NULL
};

static struct attribute_group yld_attr_group = {
	.attrs = yld_attributes
};

/*******************************************************************************
 * Linux interface and usb initialisation
 ******************************************************************************/

static const struct usb_device_id usb_table [] = {
	{
		.match_flags		= USB_DEVICE_ID_MATCH_DEVICE |
						USB_DEVICE_ID_MATCH_INT_INFO,
		.idVendor		= 0x6993,
		.idProduct		= 0xb001,
		.bInterfaceClass	= USB_CLASS_HID,
		.bInterfaceSubClass	= 0,
		.bInterfaceProtocol	= 0,
	},
	{ }
};

static int usb_cleanup(struct yealink_dev *yld, int err)
{
	if (yld == NULL)
		return err;

	usb_kill_urb(yld->urb_irq);	/* parameter validation in core/urb */
	usb_kill_urb(yld->urb_ctl);	/* parameter validation in core/urb */

        if (yld->idev) {
		if (err)
			input_free_device(yld->idev);
		else
			input_unregister_device(yld->idev);
	}
	if (yld->ctl_req)
		usb_buffer_free(yld->udev, sizeof(*(yld->ctl_req)),
				yld->ctl_req, yld->ctl_req_dma);
	if (yld->ctl_data)
		usb_buffer_free(yld->udev, USB_PKT_LEN,
				yld->ctl_data, yld->ctl_dma);
	if (yld->irq_data)
		usb_buffer_free(yld->udev, USB_PKT_LEN,
				yld->irq_data, yld->irq_dma);

	usb_free_urb(yld->urb_irq);	/* parameter validation in core/urb */
	usb_free_urb(yld->urb_ctl);	/* parameter validation in core/urb */
	kfree(yld);
	return err;
}

static void usb_disconnect(struct usb_interface *intf)
{
	struct yealink_dev *yld;

	down_write(&sysfs_rwsema);
	yld = usb_get_intfdata(intf);
	sysfs_remove_group(&intf->dev.kobj, &yld_attr_group);
	usb_set_intfdata(intf, NULL);
	up_write(&sysfs_rwsema);

	usb_cleanup(yld, 0);
}

static int usb_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev (intf);
	struct usb_host_interface *interface;
	struct usb_endpoint_descriptor *endpoint;
	struct yealink_dev *yld;
	struct input_dev *input_dev;
	int ret, pipe, i, j;

	interface = intf->cur_altsetting;
	endpoint = &interface->endpoint[0].desc;
	if (!(endpoint->bEndpointAddress & USB_DIR_IN))
		return -EIO;
	if ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) != USB_ENDPOINT_XFER_INT)
		return -EIO;

	yld = kzalloc(sizeof(struct yealink_dev), GFP_KERNEL);
	if (!yld)
		return -ENOMEM;

	yld->udev = udev;

	yld->idev = input_dev = input_allocate_device();
	if (!input_dev)
		return usb_cleanup(yld, -ENOMEM);

	/* allocate usb buffers */
	yld->irq_data = usb_buffer_alloc(udev, USB_PKT_LEN,
					SLAB_ATOMIC, &yld->irq_dma);
	if (yld->irq_data == NULL)
		return usb_cleanup(yld, -ENOMEM);

	yld->ctl_data = usb_buffer_alloc(udev, USB_PKT_LEN,
					SLAB_ATOMIC, &yld->ctl_dma);
	if (!yld->ctl_data)
		return usb_cleanup(yld, -ENOMEM);

	yld->ctl_req = usb_buffer_alloc(udev, sizeof(*(yld->ctl_req)),
					SLAB_ATOMIC, &yld->ctl_req_dma);
	if (yld->ctl_req == NULL)
		return usb_cleanup(yld, -ENOMEM);

	/* allocate urb structures */
	yld->urb_irq = usb_alloc_urb(0, GFP_KERNEL);
        if (yld->urb_irq == NULL)
		return usb_cleanup(yld, -ENOMEM);

	yld->urb_ctl = usb_alloc_urb(0, GFP_KERNEL);
        if (yld->urb_ctl == NULL)
		return usb_cleanup(yld, -ENOMEM);

	/* get a handle to the interrupt data pipe */
	pipe = usb_rcvintpipe(udev, endpoint->bEndpointAddress);
	ret = usb_maxpacket(udev, pipe, usb_pipeout(pipe));
	if (ret != USB_PKT_LEN)
		err("invalid payload size %d, expected %zd", ret, USB_PKT_LEN);

	/* initialise irq urb */
	usb_fill_int_urb(yld->urb_irq, udev, pipe, yld->irq_data,
			USB_PKT_LEN,
			urb_irq_callback,
			yld, endpoint->bInterval);
	yld->urb_irq->transfer_dma = yld->irq_dma;
	yld->urb_irq->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	yld->urb_irq->dev = udev;

	/* initialise ctl urb */
	yld->ctl_req->bRequestType = USB_TYPE_CLASS | USB_RECIP_INTERFACE |
				      USB_DIR_OUT;
	yld->ctl_req->bRequest	= USB_REQ_SET_CONFIGURATION;
	yld->ctl_req->wValue	= cpu_to_le16(0x200);
	yld->ctl_req->wIndex	= cpu_to_le16(interface->desc.bInterfaceNumber);
	yld->ctl_req->wLength	= cpu_to_le16(USB_PKT_LEN);

	usb_fill_control_urb(yld->urb_ctl, udev, usb_sndctrlpipe(udev, 0),
			(void *)yld->ctl_req, yld->ctl_data, USB_PKT_LEN,
			urb_ctl_callback, yld);
	yld->urb_ctl->setup_dma	= yld->ctl_req_dma;
	yld->urb_ctl->transfer_dma	= yld->ctl_dma;
	yld->urb_ctl->transfer_flags	|= URB_NO_SETUP_DMA_MAP |
					URB_NO_TRANSFER_DMA_MAP;
	yld->urb_ctl->dev = udev;

	/* find out the physical bus location */
	usb_make_path(udev, yld->phys, sizeof(yld->phys));
	strlcat(yld->phys,  "/input0", sizeof(yld->phys));

	/* register settings for the input device */
//	input_dev->name = nfo->name; TODO better driver messages / registration
//	input_dev->name = yld_device[i].name;
	input_dev->phys = yld->phys;
	usb_to_input_id(udev, &input_dev->id);
	input_dev->cdev.dev = &intf->dev;

	input_dev->private = yld;
	input_dev->open = input_open;
	input_dev->close = input_close;
	/* input_dev->event = input_ev;	TODO */

	/* register available key events */
	input_dev->evbit[0] = BIT(EV_KEY);
	set_bit(KEY_PHONE, input_dev->keybit);
	for (j = 0; j < ARRAY_SIZE(model); j++) {
		for (i = 0; i < 256; i++) {
			int k = model[j].keycode(i);
			if (k >= 0) {
				set_bit(k & 0xff, input_dev->keybit);
				if (k >> 8)
					set_bit(k >> 8, input_dev->keybit);
			}
		}
	}

	usb_set_intfdata(intf, yld);

	/* initialize default model */
	ret = store_model(&intf->dev, NULL, default_model, 0);
	if (ret < 0)
		return usb_cleanup(yld, ret);

	/* clear visible elements */
	for (i = 0; i < ARRAY_SIZE(lcdMap); i++)
		setChar(yld, i, ' ');

	/* display driver version on LCD line 3 */
	store_line3(&intf->dev, NULL,
			DRIVER_VERSION, sizeof(DRIVER_VERSION));

	input_register_device(yld->idev);

	/* Register sysfs hooks (don't care about failure) */
	sysfs_create_group(&intf->dev.kobj, &yld_attr_group);
	return 0;
}

static struct usb_driver yealink_driver = {
	.name		= "yealink",
	.probe		= usb_probe,
	.disconnect	= usb_disconnect,
	.id_table	= usb_table,
};

static int __init yealink_dev_init(void)
{
	int ret = usb_register(&yealink_driver);
	if (ret == 0)
		info(DRIVER_DESC ":" DRIVER_VERSION);
	return ret;
}

static void __exit yealink_dev_exit(void)
{
	usb_deregister(&yealink_driver);
}

module_init(yealink_dev_init);
module_exit(yealink_dev_exit);

module_param(default_model, charp, 0444);
MODULE_PARM_DESC(default_model,
	"default handset model, see /sys/.../model for available types");

MODULE_DEVICE_TABLE (usb, usb_table);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
