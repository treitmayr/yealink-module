/*
 * drivers/usb/input/yealink.c
 *
 * Copyright (c) 2005,2006 Henk Vergonet <Henk.Vergonet@gmail.com>
 *               2008      Thomas Reitmayr <treitmayr@devbase.at>
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
 *   - the xpad-driver	drivers/input/joystick/xpad.c
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
 *   20080514 Thomas	Added support for P1KH, auto-detect handset model,
 *			added B2K keymap
 */

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/rwsem.h>
#include <linux/timer.h>
#include <linux/usb/input.h>

#include "map_to_7segment.h"
#include "yealink.h"

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)
#error "Need kernel version 2.6.21 or higher"
#endif

#define DRIVER_VERSION "yld-20080514"
#define DRIVER_AUTHOR "Henk Vergonet, Thomas Reitmayr"
#define DRIVER_DESC "Yealink phone driver"

#define USB_YEALINK_VENDOR_ID	0x6993
#define USB_YEALINK_PRODUCT_ID1	0xb001
#define USB_YEALINK_PRODUCT_ID2	0xb700

#define YEALINK_POLLING_FREQUENCY	10	/* in [Hz] */

struct yld_status {
	u8	init;
	u8	version;
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
	enum yld_ctl_protocols protocol;
};

struct yld_ctl_chan {
	union yld_ctl_packet	*ctl_data;
	dma_addr_t		ctl_dma;
	struct usb_ctrlrequest	*ctl_req;
	dma_addr_t		ctl_req_dma;
	struct urb		*urb_ctl;
};

struct yealink_dev {
	struct input_dev	*idev;		/* input device */
	struct usb_device	*udev;		/* usb device */
	struct usb_interface	*interface;	/* interface for the device */

	struct timer_list	timer;		/* timer for key/hook scans */
	unsigned long		next_expires;
	struct semaphore	update_sem;	/* G2 transfer protection */

	struct model_info *model;

	/* irq input channel */
	union yld_ctl_packet	*irq_data;
	dma_addr_t		irq_dma;
	struct urb		*urb_irq;
	int			pkt_len;

	/* control output channel for repetitive key/hook scans */
	struct yld_ctl_chan	ctl_scan;

	/* control output channel for all other commands */
	struct yld_ctl_chan	ctl_cmd;

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

static char p1k_model[]  = "P1K";
static char p4k_model[]  = "P4K";
static char b2k_model[]  = "B2K";
static char p1kh_model[] = "P1KH";
static int map_p1k_to_key(unsigned);
static int map_p4k_to_key(unsigned);
static int map_b2k_to_key(unsigned);
static int map_p1kh_to_key(unsigned);

/* model_info_idx_* have to match index in static model structure (below) */
enum model_info_idx {
	model_info_idx_p1k,
	model_info_idx_p4k,
	model_info_idx_b2k,
	model_info_idx_p1kh
};

static struct model_info model[] = {
	{
		.keycode  = map_p1k_to_key,
		.name     = p1k_model,
		.protocol = yld_ctl_protocol_g1
	},{
		.keycode  = map_p4k_to_key,
		.name     = p4k_model,
		.protocol = yld_ctl_protocol_g1
	},{
		.keycode  = map_b2k_to_key,
		.name     = b2k_model,
		.protocol = yld_ctl_protocol_g1
	},{
		.keycode  = map_p1kh_to_key,
		.name     = p1kh_model,
		.protocol = yld_ctl_protocol_g2
	}
};

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
		a = lcdMap[el].u.p.a + offsetof(struct yld_status, lcd);
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

		a = lcdMap[el].u.s[i].a + offsetof(struct yld_status, lcd);
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
	static const int map[] = {		/* code	key	*/
		KEY_0,			 	/* 00 0		*/
		KEY_1,			 	/* 01 1		*/
		KEY_2,				/* 02 2		*/
		KEY_3,				/* 03 3		*/
		KEY_4,				/* 04 4		*/
		KEY_5,				/* 05 5		*/
		KEY_6,				/* 06 6		*/
		KEY_7,				/* 07 7		*/
		KEY_8,				/* 08 8		*/
		KEY_9,				/* 09 9		*/
		-EINVAL,                        /* 0a		*/
		KEY_KPASTERISK,			/* 0b *		*/
		KEY_LEFTSHIFT | KEY_3 << 8	/* 0c #		*/
	};
	if(scancode < ARRAY_SIZE(map))
		return map[scancode];

	return -EINVAL;
}

/* USB-P1KH button layout:
 *
 *   See P1K.
 */
static int map_p1kh_to_key(unsigned scancode)
{
	static const int map[] = {		/* code	key	*/
		KEY_1,			 	/* 00 1		*/
		KEY_2,				/* 01 2		*/
		KEY_3,				/* 02 3		*/
		KEY_ENTER,			/* 03 pickup	*/
		KEY_RIGHT,			/* 04 OUT	*/
		KEY_4,				/* 05 4		*/
		KEY_5,				/* 06 5		*/
		KEY_6,				/* 07 6		*/
		KEY_ESC,			/* 08 hangup	*/
		KEY_BACKSPACE,			/* 09 C		*/
		KEY_7,				/* 0a 7		*/
		KEY_8,				/* 0b 8		*/
		KEY_9,				/* 0c 9		*/
		KEY_LEFT,			/* 0d IN	*/
		KEY_DOWN,			/* 0e down	*/
		KEY_KPASTERISK,			/* 0f *		*/
		KEY_0,				/* 10 0		*/
		KEY_LEFTSHIFT | KEY_3 << 8,	/* 11 #		*/
		KEY_UP				/* 12 up	*/
	};
	if(scancode < ARRAY_SIZE(map))
		return map[scancode];

	return -EINVAL;
}


/* Completes a request by converting the data into events for the
 * input subsystem.
 *
 * The key parameter can be cascaded: key2 << 8 | key1
 */
static void report_key(struct yealink_dev *yld, int key)
{
	struct input_dev *idev = yld->idev;

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

static int yealink_cmd(struct yealink_dev *yld, union yld_ctl_packet *p)
{
	enum yld_ctl_protocols proto;
	u8	*buf = (u8 *)p;
	int	i, len, ret;
	u8	sum = 0;

	if (!yld->model)
		return -ENODEV;

	proto = yld->model->protocol;
	
	len = (proto == yld_ctl_protocol_g1) ? USB_PKT_LEN_G1 : USB_PKT_LEN_G2;
	for (i = 0; i < len - 1; i++)
		sum -= buf[i];

	if (proto == yld_ctl_protocol_g1)
		p->g1.sum = sum;
	else
		p->g2.sum = sum;

	ret = usb_control_msg(yld->udev,
			usb_sndctrlpipe(yld->udev, 0),
			USB_REQ_SET_CONFIGURATION,
			USB_TYPE_CLASS | USB_RECIP_INTERFACE | USB_DIR_OUT,
			0x200, 3,
			p, len,
			USB_CTRL_SET_TIMEOUT);
	if (ret < 0) {
		err("%s - usb_control_msg failed %d", __FUNCTION__, ret);
		return ret;
	}
	return 0;
}

static u8 default_ringtone_g1[] = {
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

static u8 default_ringtone_g2[] = {
	0xEF,		/* volume [0-255] */
	0x1E, 0x0C,	/* 1250 [hz], 12/100 [s] */
	0x18, 0x0C,	/* 1000 [hz], 12/100 [s] */
	0x1E, 0x0C,
	0x18, 0x0C,
	0x1E, 0x0C,
	0x18, 0x0C,
	0x1E, 0x0C,
	0x18, 0x0C,
	0xFF, 0xFE,	/* silent, ? [s] */
	0x00, 0x00	/* end of sequence */
};

static int yealink_set_ringtone(struct yealink_dev *yld, u8 *buf, size_t size)
{
	enum	yld_ctl_protocols proto;
	union	yld_ctl_packet *p = yld->ctl_cmd.ctl_data;
	int	ix, len;

	if (!yld->model)
		return -ENODEV;
	if (size <= 0)
		return -EINVAL;

	proto = yld->model->protocol;

	/* Set the ringtone volume */
	memset(p, 0, sizeof(*p));
	p->cmd = CMD_RING_VOLUME;
	if (proto == yld_ctl_protocol_g1) {
		p->g1.size	= 1;
		p->g1.data[0]	= buf[0];
	} else {
		p->g2.data[0]	= buf[0];
	}
	yealink_cmd(yld, p);

	buf++;
	size--;

	p->cmd = CMD_RING_NOTE;
	ix = 0;
	while (size != ix) {
		len = size - ix;
		if (proto == yld_ctl_protocol_g1) {
			if (len > sizeof(p->g1.data))
				len = sizeof(p->g1.data);
			p->g1.size   = len;
			p->g1.offset = cpu_to_be16(ix);
			memcpy(p->g1.data, &buf[ix], len);
		} else {
			if (len > sizeof(p->g2.data))
				len = sizeof(p->g2.data);
			memcpy(p->g2.data, &buf[ix], len);
		}
		yealink_cmd(yld, p);
		ix += len;
	}
	return 0;
}

/* Submit a delayed control message
 * 
 * This function should be called for key scan commands to achieve
 * a polling frequency of 10 Hz. No other commands may be submitted between
 * calling this function and finally receiving the URB callback to guarantee
 * yld->urb_ctl not being modified before submission.
 * 
 * Note: After invoking this function updates to LCD/LED/buzzer/.. are
 * delayed until the timer expires.
 */
static int delayed_submit(struct yealink_dev *yld, int delayed, int mem_flags) {
	int ret;

	if (delayed)
		delayed = (yld->next_expires > jiffies);

	if (delayed) {
		mod_timer(&yld->timer, yld->next_expires);
		yld->next_expires += HZ / YEALINK_POLLING_FREQUENCY;
		ret = 0;
	} else {
		if (timer_pending(&yld->timer))	{	/* should not happen */
			del_timer(&yld->timer);
			err("Timer pending already!");
		}
		yld->next_expires = jiffies + HZ / YEALINK_POLLING_FREQUENCY;
		ret = usb_submit_urb(yld->ctl_scan.urb_ctl, mem_flags);
	}
	return ret;
}

/* Timer callback function
 * 
 * This function submits a pending key scan command.
 */
static void timer_callback(unsigned long ylda)
{
	struct yealink_dev *yld;
	int ret;
	
	yld = (struct yealink_dev *)ylda;
	
	ret = usb_submit_urb(yld->ctl_scan.urb_ctl, GFP_ATOMIC);
	
	if (ret)
		err("%s - usb_submit_urb failed %d", __FUNCTION__, ret);
}

/* keep stat_master & stat_copy in sync.
 * returns <0 upon error
 *          0 no error
 */
static int submit_next_request(struct yealink_dev *yld, int mem_flags)
{
	union yld_ctl_packet *ctl_data;
	enum yld_ctl_protocols proto;
	u8 val;
	u8 sum;
	u8 *data;
	u8 offset;
	int i, ix, len;
	int delayed = 0;

	if (!yld->model) {
		err("%s - unknown model", __FUNCTION__);
		return -ENODEV;
	}

	proto = yld->model->protocol;
	ctl_data = yld->ctl_scan.ctl_data;
	ix = yld->stat_ix;

	if (timer_pending(&yld->timer)) {
		/* this should really only happen for G2! */
		return 0;
	}

	data = (proto == yld_ctl_protocol_g1) ?
		ctl_data->g1.data : ctl_data->g2.data;

	memset(ctl_data, 0, sizeof(*ctl_data));

	/* find update candidates: copy != master */
	while (ix < sizeof(yld->master)) {
		val = yld->master.b[ix];
		if (val == yld->copy.b[ix]) {
			ix++;
			continue;
		}
		yld->copy.b[ix] = val;
		
		/* Setup an appropriate update request */
		sum = 0;

		switch(ix) {
		case offsetof(struct yld_status, init):
			ctl_data->cmd	= CMD_INIT;
			if (proto == yld_ctl_protocol_g1) {
				ctl_data->g1.size = 10;
				sum -= 10;
			}
			break;
		case offsetof(struct yld_status, version):
			ctl_data->cmd	= CMD_VERSION;
			if (proto == yld_ctl_protocol_g1) {
				ctl_data->g1.size = 2;
				sum -= 2;
			}
			delayed	= 1;
			break;
		case offsetof(struct yld_status, led):
			ctl_data->cmd	= CMD_LED;
			if (yld->model->name == b2k_model) {
				data[0] = (val) ? 0xff : 0x00;
				data[1] = (val) ? 0x00 : 0xff;
				ctl_data->g1.size = 2;
				sum -= 1;
			} else {
				data[0] = val;
				sum -= val;
				if (proto == yld_ctl_protocol_g1) {
					ctl_data->g1.size = 1;
					sum--;
				}
			}
			break;
		case offsetof(struct yld_status, dialtone):
			if (yld->model->name == b2k_model ||
			    yld->model->name == p4k_model) {
				ctl_data->cmd	= CMD_DIALTONE;
				ctl_data->g1.size = 1;
				data[0] = val;
				sum -= val+1;
			}
			break;
		case offsetof(struct yld_status, ringtone):
			if (yld->model->name == p1k_model ||
			    yld->model->name == p1kh_model) {
				ctl_data->cmd	= CMD_RINGTONE;
				data[0] = val;
				sum -= val;
				if (proto == yld_ctl_protocol_g1) {
					ctl_data->g1.size = 1;
					sum--;
				}
			}
			break;
		case offsetof(struct yld_status, backlight):
			if (yld->model->name == p4k_model) {
				ctl_data->cmd	= CMD_LCD_BACKLIGHT;
				ctl_data->g1.size = 1;
				data[0] = val;
				sum -= val+1;
			}
			break;
		case offsetof(struct yld_status, speaker):
			if (yld->model->name == p4k_model) {
				ctl_data->cmd	= CMD_SPEAKER;
				ctl_data->g1.size = 1;
				data[0] = val;
				sum -= val+1;
			}
			break;
		case offsetof(struct yld_status, pstn):
			if (yld->model->name == b2k_model) {
				ctl_data->cmd	= CMD_PSTN_SWITCH;
				ctl_data->g1.size = 1;
				data[0] = val;
				sum -= val+1;
			}
			break;
		case offsetof(struct yld_status, keynum):
			if (proto == yld_ctl_protocol_g2)
				break;
			val--;
			val &= 0x1f;
			sum -= val+1;
			ctl_data->cmd		= CMD_SCANCODE;
			ctl_data->g1.size	= 1;
			ctl_data->g1.offset	= cpu_to_be16(val);
			break;
		default:
			if (yld->model->name != b2k_model) {
			    	offset = ix - offsetof(struct yld_status, lcd);
				len = sizeof(yld->master.s.lcd) - offset;
				
				if (proto == yld_ctl_protocol_g1) {
					if (len > sizeof(ctl_data->g1.data))
						len = sizeof(ctl_data->g1.data);
					ctl_data->g1.offset = cpu_to_be16(offset);
					ctl_data->g1.size   = len;
				} else {
					if (len > sizeof(ctl_data->g2.data) - 2)
						len = sizeof(ctl_data->g2.data) - 2;
					data[0]	= len;		/* size */
					data[1]	= offset;	/* offset */
					data += 2;	/* urgh */
				}

				/* Combine up to <len> consecutive LCD bytes
				 * in a singe request */
				ctl_data->cmd	= CMD_LCD;
				sum		-= offset + len;
				for(i=0; i<len; i++) {
					val = yld->master.b[ix];
					yld->copy.b[ix]	= val;
					data[i]		= val;
					sum		-= val;
					ix++;
				}
				ix--;
			}
			break;
		}
		ix++;
		if (ctl_data->cmd != 0) {
			yld->stat_ix = ix;
			if (proto == yld_ctl_protocol_g1)
				ctl_data->g1.sum = sum - ctl_data->cmd;
			else
				ctl_data->g2.sum = sum - ctl_data->cmd;
			return delayed_submit(yld, delayed, mem_flags);
		}
	}

	yld->stat_ix = 0;
	if (proto == yld_ctl_protocol_g1) {
		ctl_data->g1.size = 1;
		if (yld->model->name == p4k_model && 
		    yld->last_cmd == CMD_KEYPRESS) {
			ctl_data->cmd  = CMD_HOOKPRESS;
			ctl_data->g1.sum  = 0xff - CMD_HOOKPRESS;
			yld->last_cmd = CMD_HOOKPRESS;
		} else {
		 	ctl_data->cmd  = CMD_KEYPRESS;
			ctl_data->g1.sum  = 0xff - CMD_KEYPRESS;
			yld->last_cmd = CMD_KEYPRESS;
		}
		return delayed_submit(yld, 1, mem_flags);
	} else {
		up(&yld->update_sem);
	}
	return 0;
}

/*
 */
static void urb_irq_callback(struct urb *urb)
{
	struct yealink_dev *yld = urb->context;
	enum yld_ctl_protocols proto;
	u8 data0, data1;
	u16 version;
	int ret;
	
	if (!yld->model) {
		err("%s - unknown model", __FUNCTION__);
		return;
	}

	proto = yld->model->protocol;
	data0 = (proto == yld_ctl_protocol_g1) ?
	          yld->irq_data->g1.data[0] : yld->irq_data->g2.data[0];

	if (urb->status)
		err("%s - urb status %d", __FUNCTION__, urb->status);

	switch (yld->irq_data->cmd) {
	case CMD_VERSION:
		data1 = (proto == yld_ctl_protocol_g1) ?
			yld->irq_data->g1.data[1] : yld->irq_data->g2.data[1];
		version = (data0 << 8) | data1;
		if (proto == yld_ctl_protocol_g1) {
			/* can only auto-detect G1 devices for now */
			if (YLD_IS_P1K(version))
				yld->model = &model[model_info_idx_p1k];
			else if (YLD_IS_P4K(version))
				yld->model = &model[model_info_idx_p4k];
			else if (YLD_IS_B2K(version))
				yld->model = &model[model_info_idx_b2k];
			else
				yld->model = NULL;
		}
		if (yld->model)
			info("Detected model %s", yld->model->name);
		else
			warn("Unknown model version %04x, driver disabled!",
				version);
		break;

	case CMD_KEYPRESS:
		yld->master.s.keynum = data0;
		break;

	case CMD_HOOKPRESS:
		ret = data0 & 0x10;
		if (yld->hookstate != ret) {
		    dbg("hookstate: %x", ret);
		    input_report_key(yld->idev, KEY_PHONE, ret >> 4);
		    input_sync(yld->idev);
		    yld->hookstate = ret;
		}
		break;

	case CMD_SCANCODE:
		dbg("get scancode %x", data0);

		ret = yld->model->keycode(data0);
		report_key(yld, ret);
		if (ret < 0 && data0 != 0xff)
			info("unknown scancode 0x%02x", data0);
		break;

	case STATE_BAD_PKT:
		warn("phone received invalid packet");
		break;

	default:
		if (proto == yld_ctl_protocol_g1)
			err("unexpected response %x", yld->irq_data->cmd);
		break;
	}
	
	if (proto == yld_ctl_protocol_g1) {
		ret = submit_next_request(yld, GFP_ATOMIC);
		if (ret)
			err("%s - submit_next_request failed %d", __FUNCTION__, ret);
	} else {
		/* always wait for a key or some other interrupt */
		ret = usb_submit_urb(yld->urb_irq, GFP_ATOMIC);
		if (ret)
			err("%s - usb_submit_urb failed %d", __FUNCTION__, ret);
	}
}

/* Control URB callback function
 * 
 * This function is invoked when the control URB was submitted.
 */
static void urb_ctl_callback(struct urb *urb)
{
	struct yealink_dev *yld = urb->context;
	enum yld_ctl_protocols proto;
	int ret;

	if (urb->status)
		err("%s - urb status %d", __FUNCTION__, urb->status);

	if (!yld->model) {
		err("%s - unknown model", __FUNCTION__);
		return;
	}

	proto = yld->model->protocol;

	switch (yld->ctl_scan.ctl_data->cmd) {
	case CMD_HOOKPRESS:
	case CMD_KEYPRESS:
	case CMD_SCANCODE:
	case CMD_VERSION:
		if (proto == yld_ctl_protocol_g1) {
			/* explicitly ask for a response */
			ret = usb_submit_urb(yld->urb_irq, GFP_ATOMIC);
		} else {
			/* send new command */
			ret = submit_next_request(yld, GFP_ATOMIC);
		}
		break;
	default:
		/* send new command */
		ret = submit_next_request(yld, GFP_ATOMIC);
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
	struct yealink_dev *yld = input_get_drvdata(dev);
	enum yld_ctl_protocols proto;
	int i, ret;

	dbg("%s", __FUNCTION__);
	
	if (!yld->model) {	/* return silently */
		warn("%s - no model selected!", __FUNCTION__);
		return 0;
	}

	proto = yld->model->protocol;

	/* force updates to device */
	for (i = 0; i<sizeof(yld->master); i++)
		yld->copy.b[i] = ~yld->master.b[i];
	yld->key_code = -1;	/* no keys pressed */

	if (proto == yld_ctl_protocol_g1) {
	        yealink_set_ringtone(yld, default_ringtone_g1,
	                             sizeof(default_ringtone_g1));
	} else {
	        yealink_set_ringtone(yld, default_ringtone_g2,
	                             sizeof(default_ringtone_g2));
		/* immediately start waiting for a key */
		ret = usb_submit_urb(yld->urb_irq, GFP_KERNEL);
		if (ret)
			err("%s - usb_submit_urb failed %d", __FUNCTION__, ret);
	}

	return submit_next_request(yld, GFP_KERNEL);
	//return 0;
}

static void input_close(struct input_dev *dev)
{
	struct yealink_dev *yld = input_get_drvdata(dev);

	if (timer_pending(&yld->timer))
		del_timer(&yld->timer);

	usb_kill_urb(yld->ctl_cmd.urb_ctl);
	usb_kill_urb(yld->ctl_scan.urb_ctl);
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

/* Writing to /sys/../lineX will set the corresponding LCD line.
 * - Excess characters are ignored.
 * - If less characters are written than allowed, the remaining digits are
 *   unchanged.
 * - The '\n' or '\t' char is a placeholder, it does not overwrite the
 *   original content.
 */
static ssize_t store_line(struct device *dev, const char *buf, size_t count,
		int el, size_t len, int submit)
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

	if (submit && yld->model && yld->model->protocol == yld_ctl_protocol_g2) {
		if (down_interruptible(&yld->update_sem)) //??
			return -ERESTARTSYS;
		submit_next_request(yld, GFP_KERNEL);
	}
	return count;
}

static ssize_t store_line1(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	return store_line(dev, buf, count, LCD_LINE1_OFFSET, LCD_LINE1_SIZE, 1);
}

static ssize_t store_line2(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	return store_line(dev, buf, count, LCD_LINE2_OFFSET, LCD_LINE2_SIZE, 1);
}

static ssize_t store_line3(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	return store_line(dev, buf, count, LCD_LINE3_OFFSET, LCD_LINE3_SIZE, 1);
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

	if (yld->model && yld->model->protocol == yld_ctl_protocol_g2) {
		if (down_interruptible(&yld->update_sem)) //??
			return -ERESTARTSYS;
		submit_next_request(yld, GFP_KERNEL);
	}

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

	if (down_interruptible(&yld->update_sem)) //??
		return -ERESTARTSYS;
	yealink_set_ringtone(yld, (char *)buf, count);
	up(&yld->update_sem);

	up_write(&sysfs_rwsema);
	return count;
}

/* Get a list phone models with the currently selected one. */
static ssize_t show_model(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct yealink_dev *yld;

	down_read(&sysfs_rwsema);
	yld = dev_get_drvdata(dev);
	if (yld == NULL) {
		up_read(&sysfs_rwsema);
		return -ENODEV;
	}

	if (yld->model)
		strcpy(buf, yld->model->name);
	else
		strcpy(buf, "unknown");
	up_read(&sysfs_rwsema);
	return strlen(buf)+1;
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
static DEVICE_ATTR(model	, _M440, show_model	, NULL		);

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
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,22)
	{
		.match_flags		= USB_DEVICE_ID_MATCH_DEVICE |
						USB_DEVICE_ID_MATCH_INT_INFO,
		.idVendor		= USB_YEALINK_VENDOR_ID,
		.idProduct		= USB_YEALINK_PRODUCT_ID1,
		.bInterfaceClass	= USB_CLASS_HID,
		.bInterfaceSubClass	= 0,
		.bInterfaceProtocol	= 0,
	},
	{
		.match_flags		= USB_DEVICE_ID_MATCH_DEVICE |
						USB_DEVICE_ID_MATCH_INT_INFO,
		.idVendor		= USB_YEALINK_VENDOR_ID,
		.idProduct		= USB_YEALINK_PRODUCT_ID2,
		.bInterfaceClass	= USB_CLASS_HID,
		.bInterfaceSubClass	= 0,
		.bInterfaceProtocol	= 0,
	},
#else
	{ USB_DEVICE_AND_INTERFACE_INFO(USB_YEALINK_VENDOR_ID, USB_YEALINK_PRODUCT_ID1,
	             USB_CLASS_HID, 0, 0) },
	{ USB_DEVICE_AND_INTERFACE_INFO(USB_YEALINK_VENDOR_ID, USB_YEALINK_PRODUCT_ID2,
	             USB_CLASS_HID, 0, 0) },
#endif
	{ }
};

static int usb_cleanup(struct yealink_dev *yld, int err)
{
	if (yld == NULL)
		return err;

	usb_kill_urb(yld->urb_irq);	/* parameter validation in core/urb */
	usb_kill_urb(yld->ctl_cmd.urb_ctl); /* parameter validation in core/urb */
	usb_kill_urb(yld->ctl_scan.urb_ctl); /* parameter validation in core/urb */

        if (yld->idev) {
		if (err)
			input_free_device(yld->idev);
		else
			input_unregister_device(yld->idev);
	}

	usb_free_urb(yld->urb_irq);
	usb_free_urb(yld->ctl_cmd.urb_ctl);
	usb_free_urb(yld->ctl_scan.urb_ctl);

	if (yld->ctl_cmd.ctl_req)
		usb_buffer_free(yld->udev, sizeof(*(yld->ctl_cmd.ctl_req)),
		                yld->ctl_cmd.ctl_req, yld->ctl_cmd.ctl_req_dma);
	if (yld->ctl_cmd.ctl_data)
		usb_buffer_free(yld->udev, yld->pkt_len,
		                yld->ctl_cmd.ctl_data, yld->ctl_cmd.ctl_dma);
	if (yld->ctl_scan.ctl_req)
		usb_buffer_free(yld->udev, sizeof(*(yld->ctl_scan.ctl_req)),
		                yld->ctl_scan.ctl_req, yld->ctl_scan.ctl_req_dma);
	if (yld->ctl_scan.ctl_data)
		usb_buffer_free(yld->udev, yld->pkt_len,
		                yld->ctl_scan.ctl_data, yld->ctl_scan.ctl_dma);
	if (yld->irq_data)
		usb_buffer_free(yld->udev, yld->pkt_len,
		                yld->irq_data, yld->irq_dma);

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

static int usb_allocate_init_ctl_req(struct yealink_dev *yld,
				struct yld_ctl_chan *ctl) {
	/* allocate usb buffers */
	ctl->ctl_data = usb_buffer_alloc(yld->udev, yld->pkt_len, GFP_ATOMIC,
					&(ctl->ctl_dma));
	if (ctl->ctl_data == NULL)
		return usb_cleanup(yld, -ENOMEM);
	ctl->ctl_req = usb_buffer_alloc(yld->udev, sizeof(*(ctl->ctl_req)),
					GFP_ATOMIC,
					&(ctl->ctl_req_dma));
	if (ctl->ctl_req == NULL)
		return usb_cleanup(yld, -ENOMEM);
	ctl->urb_ctl = usb_alloc_urb(0, GFP_KERNEL);
	if (ctl->urb_ctl == NULL)
		return usb_cleanup(yld, -ENOMEM);

	/* initialise ctl urb */
	ctl->ctl_req->bRequestType = USB_TYPE_CLASS | USB_RECIP_INTERFACE |
				      USB_DIR_OUT;
	ctl->ctl_req->bRequest	= USB_REQ_SET_CONFIGURATION;
	ctl->ctl_req->wValue	= cpu_to_le16(0x200);
	ctl->ctl_req->wIndex	= cpu_to_le16(yld->interface->cur_altsetting->desc.bInterfaceNumber);
	ctl->ctl_req->wLength	= cpu_to_le16(yld->pkt_len);

	usb_fill_control_urb(ctl->urb_ctl, yld->udev,
			usb_sndctrlpipe(yld->udev, 0),
			(void *)ctl->ctl_req, ctl->ctl_data, yld->pkt_len,
			urb_ctl_callback, yld);
	ctl->urb_ctl->setup_dma		= ctl->ctl_req_dma;
	ctl->urb_ctl->transfer_dma	= ctl->ctl_dma;
	ctl->urb_ctl->transfer_flags	|= URB_NO_SETUP_DMA_MAP |
					URB_NO_TRANSFER_DMA_MAP;
	ctl->urb_ctl->dev = yld->udev;
	return 0;
}

static int usb_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev (intf);
	struct usb_host_interface *interface;
	struct usb_endpoint_descriptor *endpoint;
	struct yealink_dev *yld;
	struct input_dev *input_dev;
	enum yld_ctl_protocols proto;
	int ret, pipe, i, j;
	int pid;
	int pkt_len;

	dbg("%s - start", __FUNCTION__);
	
	interface = intf->cur_altsetting;
	endpoint = &interface->endpoint[0].desc;
	if (!usb_endpoint_is_int_in(endpoint))
		return -ENODEV;

	yld = kzalloc(sizeof(struct yealink_dev), GFP_KERNEL);
	if (!yld)
		return -ENOMEM;

	yld->udev = udev;
	yld->interface = intf;
	sema_init(&yld->update_sem, 1);

	/* get a handle to the interrupt data pipe */
	pipe = usb_rcvintpipe(udev, endpoint->bEndpointAddress);
	yld->pkt_len = pkt_len = usb_maxpacket(udev, pipe, usb_pipeout(pipe));
	
	pid = le16_to_cpu(udev->descriptor.idProduct);
	dbg("PID = %04x", pid);
	if (pkt_len == USB_PKT_LEN_G1) {
		proto = yld_ctl_protocol_g1;
		yld->model = &model[model_info_idx_p1k];	/* changed later */
	} else if (pkt_len == USB_PKT_LEN_G2) {
		proto = yld_ctl_protocol_g2;
		yld->model = &model[model_info_idx_p1kh];
	} else {
		warn(KERN_INFO "Yealink model not supported "
			"(payload size %d).", pkt_len);
		return usb_cleanup(yld, -ENODEV);
	}

	yld->idev = input_dev = input_allocate_device();
	if (!input_dev)
		return usb_cleanup(yld, -ENOMEM);

	/* allocate urb structure for interrupt endpoint */
	yld->urb_irq = usb_alloc_urb(0, GFP_KERNEL);
        if (yld->urb_irq == NULL)
		return usb_cleanup(yld, -ENOMEM);

	/* allocate usb buffers for interrupt endpoint */
	yld->irq_data = usb_buffer_alloc(udev, pkt_len,
					GFP_ATOMIC,
					&yld->irq_dma);
	if (yld->irq_data == NULL)
		return usb_cleanup(yld, -ENOMEM);

	/* initialise irq urb */
	usb_fill_int_urb(yld->urb_irq, udev, pipe, yld->irq_data,
			pkt_len,
			urb_irq_callback,
			yld, endpoint->bInterval);
	yld->urb_irq->transfer_dma = yld->irq_dma;
	yld->urb_irq->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	yld->urb_irq->dev = udev;

	/* allocate usb buffers for control endpoint */
	ret = usb_allocate_init_ctl_req(yld, &(yld->ctl_cmd));
	if (ret != 0)
		return ret;

	/*if (proto == yld_ctl_protocol_g1) {*/
		ret = usb_allocate_init_ctl_req(yld, &(yld->ctl_scan));
		if (ret != 0)
			return ret;
	/*}*/

	/* set up the periodic scan timer */
	/*if (proto == yld_ctl_protocol_g1) {*/
		setup_timer(&yld->timer, timer_callback, (unsigned long)yld);
	/*}*/

	/* find out the physical bus location */
	usb_make_path(udev, yld->phys, sizeof(yld->phys));
	strlcat(yld->phys,  "/input0", sizeof(yld->phys));

	/* register settings for the input device */
//	input_dev->name = nfo->name; TODO better driver messages / registration
//	input_dev->name = yld_device[i].name;
	input_dev->phys = yld->phys;
	usb_to_input_id(udev, &input_dev->id);
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,24)
	input_dev->cdev.dev = &intf->dev;
#else
	input_dev->dev.parent = &intf->dev;
#endif

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,21)
	input_dev->private = yld;
#else
	input_set_drvdata(input_dev, yld);
#endif
	input_dev->open = input_open;
	input_dev->close = input_close;
	/* input_dev->event = input_ev;	TODO */

	/* register available key events */
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,23)
	input_dev->evbit[0] = BIT(EV_KEY);
#else
	input_dev->evbit[0] = BIT_MASK(EV_KEY);
#endif
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

	/* clear visible elements */
	for (i = 0; i < ARRAY_SIZE(lcdMap); i++)
		setChar(yld, i, ' ');

	/* display driver version on LCD line 3 */
	store_line(&intf->dev, DRIVER_VERSION, sizeof(DRIVER_VERSION),
		LCD_LINE3_OFFSET, LCD_LINE3_SIZE, 0);

	/* Register sysfs hooks (don't care about failure) */
	ret = sysfs_create_group(&intf->dev.kobj, &yld_attr_group);

	dbg("%s - register input device", __FUNCTION__);
	ret = input_register_device(input_dev);
	if (ret)
		return usb_cleanup(yld, ret);

	dbg("%s - done", __FUNCTION__);

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

/*
module_param(default_model, charp, 0444);
MODULE_PARM_DESC(default_model,
	"select model (disables autodetection), see /sys/.../model for available types");
*/

MODULE_DEVICE_TABLE (usb, usb_table);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
