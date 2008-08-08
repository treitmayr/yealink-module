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
 *   Driver for the USB-P1K, P1KH, B2K, and P4K VoIP USB phones.
 *   This device is produced by Yealink Network Technology Co Ltd
 *   but may be branded under several names:
 *	- Yealink usb-p1k
 *	- Tiptel 115
 *	- ...
 *
 * This driver is based on:
 *   - the usbb2k-api	http://savannah.nongnu.org/projects/usbb2k-api/
 *   - information from	http://memeteau.free.fr/usbb2k and
 *                      http://www.devbase.at/svn/view.cgi/yealink-logs/?root=experimental
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
 *   20050701 henk	sysfs write serialization, fix potential unload races
 *   20050801 henk	Added ringtone, restructure USB
 *   20050816 henk	Merge 2.6.13-rc6
 *   20060220 henk	Experimental P4K support
 *   20060430 Toshiharu	fixing key event initialization bug.
 *   20060503 Toshiharu	Added hookflash support.
 *   20060503 Henk	Added privacy, only allow user and group access to sysfs
 *   20060830 Henk	Proper urb cleanup cycle, thanks Ivan Jensen for
 *   			pointing this out.
 *   20080730 Thomas	Added support for P1KH, auto-detect handset model,
 *			added B2K keymap,
 *			thanks mmikel for testing with the P1KH.
 *
 * TODO:
 *   - P1KH: Better understand how the ring notes have to be set up.
 *   - P1K: Do we need to catch the case where no IRQ is generated?
 */

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/rwsem.h>
#include <linux/timer.h>
#include <linux/usb/input.h>

#include "map_to_7segment.h"
#include "yealink.h"

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)
#error "Need kernel version 2.6.21 or higher"
#endif

#define DRIVER_VERSION	"20080730"
#define DRIVER_AUTHOR	"Henk Vergonet, Thomas Reitmayr"
#define DRIVER_DESC	"Yealink phone driver"

#define USB_YEALINK_VENDOR_ID	0x6993
#define USB_YEALINK_PRODUCT_ID1	0xb001
#define USB_YEALINK_PRODUCT_ID2	0xb700

/* Timeout used for synchronous reads from interrupt endpoint */
#define YEALINK_USB_INT_TIMEOUT		200	/* in [ms] */

/* The following is the delay for polling the key matrix (G1 phones only) */
#define YEALINK_POLLING_DELAY		100	/* in [ms] */

/* The following is the delay between individual commands for
   LCD, Buzzer, ... (G2 phones only) to provide enough time for the
   handset to process the command. Otherwise effects like a partially
   updated LCD were observed. */
#define YEALINK_COMMAND_DELAY_G2	25	/* in [ms] */


struct yld_status {
	u8	lcd[24];
	u8	led;
	u8	backlight;
	u8	speaker;
	u8	pstn;
	u8	keynum;
	u8	ringvol;
	u8	ringnote_mod;
	u8	ringtone;
	u8	dialtone;
} __attribute__ ((packed));

/*
 * Register the LCD segment and icon map
 */
#define _LOC(k,l)	{ .a = ((k)+offsetof(struct yld_status, lcd)), .m = (l) }
#define _SEG(t, a, am, b, bm, c, cm, d, dm, e, em, f, fm, g, gm)	\
	{ .type	= (t),							\
	  .u = { .s = {	_LOC(a, am), _LOC(b, bm), _LOC(c, cm),		\
		        _LOC(d, dm), _LOC(e, em), _LOC(g, gm),		\
			_LOC(f, fm) } } }
#define _PIC(t, h, hm, n)						\
	{ .type	= (t),							\
 	  .u = { .p = { .name = (n), .a = ((h)+offsetof(struct yld_status, lcd)), .m = (hm) } } }

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

/* Structure to be initialized according to detected Yealink model */
struct model_info {
	int (*keycode)(unsigned scancode);
	char *name;
	enum yld_ctl_protocols protocol;
};

struct yealink_dev {
	struct input_dev	*idev;		/* input device */
	struct usb_device	*udev;		/* usb device */
	struct usb_interface	*interface;	/* interface for the device */
	struct usb_endpoint_descriptor *int_endpoint;	/* interrupt EP */

	struct timer_list	timer;		/* timer for key/hook scans */
	struct semaphore	update_sem;	/* TODO: remove!!! */

	struct model_info	*model;

	/* irq input channel */
	union yld_ctl_packet	*irq_data;
	dma_addr_t		irq_dma;
	struct urb		*urb_irq;

	/* control output channel */
	union yld_ctl_packet	*ctl_data;
	dma_addr_t		ctl_dma;
	struct usb_ctrlrequest	*ctl_req;
	dma_addr_t		ctl_req_dma;
	struct urb		*urb_ctl;

	/* flags */
	unsigned	scan_active:1;
	unsigned	update_active:1;
	unsigned	timer_expired:1;
	unsigned	usb_pause:1;
	unsigned	dirty:1;
	spinlock_t	flags_lock;		/* protects above flags */

	unsigned	open:1;
	unsigned	shutdown:1;

	/* control output channel for repetitive key/hook scans;
	   TODO: possibly revert to individual fields, like with irq channel */

	char	phys[64];		/* physical device path */
	char	uniq[27];		/* unique(?) device number */

	u8 lcdMap[ARRAY_SIZE(lcdMap)];	/* state of LCD, LED ... */
	int	key_code;		/* last reported key	 */
	u8	last_cmd;		/* last scan command: key/hook */
	u8	hookstate;		/* P4K hookstate	*/
	int	stat_ix;		/* index in master/copy */
	union {
		struct yld_status s;
		u8		  b[sizeof(struct yld_status)];
	} master, copy;
	int	notes_ix;		/* index in ring_notes */
	int	notes_len;		/* number of bytes in ring_notes[] */
	u8	*ring_notes;		/* ptr. to array of ring notes */
};

static DECLARE_RWSEM(sysfs_rwsema);

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
	model_info_idx_p1kh,
	model_info_unknown
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

/* forward declaration */
static void stop_traffic(struct yealink_dev *yld);

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

	if (unlikely(el >= ARRAY_SIZE(lcdMap)))
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
 * Yealink ringtone interface
 ******************************************************************************/

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

#if 0
/* not working: */
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
#endif
static u8 default_ringtone_g2[] = {
	0xFF,		/* volume [0-255] */
	0x1E, 0x0C,	/* 1250 [hz], 12/100 [s] */
	0x18, 0x0C,	/* 1000 [hz], 12/100 [s] */
	0x00, 0x00	/* end of sequence */
};

static int set_ringnotes(struct yealink_dev *yld, u8 *buf, size_t size)
{
	int	eos;		/* end of sequence */
	int	i;

	if ((buf == NULL) || (size == 0))
		return 0;

	/* adjust the volume */
	yld->master.s.ringvol = buf[0];
	if (size == 1)		/* do not touch the ring notes */
		return 0;

	buf++;
	size--;
	if (yld->ring_notes != NULL) {
		kfree(yld->ring_notes);
		yld->ring_notes = NULL;
	}
	yld->ring_notes = kmalloc(size + 2, GFP_KERNEL);
	if (!yld->ring_notes)
		return -ENOMEM;

	i = 0;
	eos = 0;
	while (i < (size - 1)) {
		yld->ring_notes[i] = buf[i];
		yld->ring_notes[i+1] = buf[i+1];
		eos = (buf[i] == 0) && (buf[i+1] == 0);
		i += 2;
		if (eos)
			break;
	}
	if (!eos) {
		/* create the end-of-sequence marker */
		yld->ring_notes[i++] = 0;
		yld->ring_notes[i++] = 0;
	}
	yld->notes_len = i;
	yld->notes_ix = 0;
	return 0;
}

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

/* Description of USB message and timer sequence:

   P1K/B2K/P4K:
   ------------
   1. submit control message
   2. callback control message
   3.   > if control message was INIT/VERSION/KEY/HOOK submit irq message
   4.     callback irq message
   5. start timer to wait until YEALINK_POLLING_DELAY has expired since 1
   6. timer expires, goto step 1
   
   This loop is executed continuously scanning the keypad/hook in regular
   intervals. No control message may be submitted from outside this loop.
   
   P1KH:
   -----
   IRQ Endpoint:
     1. submit irq message
     2. callback irq message
     3. goto step 1
   Control Endpoint:
     1. submit control message
     2. callback control message
     3. start timer to wait until YEALINK_COMMAND_DELAY_G2 has expired since 2
        Note: For INIT/VERSION the delay is multiplied by 4!
     4. timer expires, goto step 1
   
   The loop of the control endpoint is initiated when the driver is loaded or
   by the sysfs interface functions. It is important to decrement a protecting
   semaphore before submitting the first control message.
   Once all changes are updated, the semaphore is released (in interrupt
   context) and the loop terminates.
 */

/* ... @@ */

static inline void pkt_update_checksum(union yld_ctl_packet *p, int len)
{
	u8 *bp = (u8 *) p;
	u8 i, sum = 0;
	for (i = 0; i < len-1; i++)
		sum -= bp[i];
	bp[len-1] = sum;
}

static inline int pkt_verify_checksum(union yld_ctl_packet *p, int len)
{
	u8 *bp = (u8 *) p;
	u8 i, sum = 0;
	for (i = 0; i < len; i++)
		sum -= bp[i];
	return (int) sum;
}

static int submit_cmd_sync(struct yealink_dev *yld,
			   union yld_ctl_packet *p, int len)
{
	int ret;
	ret = usb_control_msg(yld->udev,
		usb_sndctrlpipe(yld->udev, 0),
		USB_REQ_SET_CONFIGURATION,
		USB_TYPE_CLASS | USB_RECIP_INTERFACE | USB_DIR_OUT,
		0x200, 3,
		p, len,
		USB_CTRL_SET_TIMEOUT);
	if (ret == len)
		ret = 0;
	else
	if (ret >= 0)
		ret = -ENODATA;
	return ret;
}

static int submit_int_sync(struct yealink_dev *yld,
			   union yld_ctl_packet *p, int len)
{
	int act_len, ret;

	ret = usb_interrupt_msg(yld->udev,
		usb_rcvintpipe(yld->udev, yld->int_endpoint->bEndpointAddress),
		p, len, &act_len,
		YEALINK_USB_INT_TIMEOUT);
	if (ret == 0) {
		if (len != act_len) {
			warn("Received short packet (%d/%d)", act_len, len);
			ret = -ENODATA;
		}
		if (pkt_verify_checksum(p, len) != 0) {
			warn("Received packet with invalid checksum");
			ret = -EBADMSG;
		}
	}
	return ret;
}

static int submit_cmd_int_sync(struct yealink_dev *yld,
			union yld_ctl_packet *cp, int clen,
			union yld_ctl_packet *ip, int ilen)
{
	int repeat = 3;
	int ret = -1;
	while ((ret != 0) && (repeat > 0)) {   //@@@@
		repeat--;
		ret = submit_cmd_sync(yld, cp, clen);
		//dbg("cmd %02x -> ret %d", cp->cmd, ret);
		if (ret != 0) {
			err("Could not submit command 0x%02x: %d", cp->cmd, ret);
			continue;
		}
		ret = submit_int_sync(yld, ip, ilen);
		//dbg("int %02x -> ret %d", ip->cmd, ret);
		if ((ret == 0) && (ip->cmd != cp->cmd))
			ret = -ENODATA;
	}
	return ret;
}

/* g1 only */
static int submit_scan_request(struct yealink_dev *yld, int mem_flags)
{
	union yld_ctl_packet *ctl_data = yld->ctl_data;
	int ret;

	dbg("%s", __FUNCTION__);
	if (yld->model->protocol != yld_ctl_protocol_g1)
		BUG();

	memset(ctl_data, 0, sizeof(*ctl_data));
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
	ret = usb_submit_urb(yld->urb_ctl, mem_flags);
	if (ret != 0)
		err("%s: usb_submit_urb failed %d", __func__, ret);
	return ret;
}

/* keep stat_master & stat_copy in sync.
 * returns  = 0 if no packet was prepared (= no releavant differences found)
 *         /= 0 if a command was assembled in yld->ctl_data
 */
static int prepare_update_cmd(struct yealink_dev *yld)
{
	union yld_ctl_packet *ctl_data;
	enum yld_ctl_protocols proto;
	u8 val;
	u8 *data;
	u8 offset;
	int i, ix, len;

	proto = yld->model->protocol;
	ix = yld->stat_ix;
	ctl_data = yld->ctl_data;
	data = (proto == yld_ctl_protocol_g1) ?
		ctl_data->g1.data : ctl_data->g2.data;

	ctl_data->cmd = 0;		/* no packet prepared so far */

	/* find update candidates: copy != master */
	/* the big loop for processing any mismatches */
	do {
		/* find update candidates: copy != master */
		do {
			val = yld->master.b[ix];
			if (val != yld->copy.b[ix])
				goto handle_difference;
			if (unlikely(++ix >= sizeof(yld->master)))
				ix = 0;
		} while (ix != yld->stat_ix);

		break;

handle_difference:

		yld->copy.b[ix] = val;

		/* Setup an appropriate update request */
		switch(ix) {
		case offsetof(struct yld_status, led):
			ctl_data->cmd	= CMD_LED;
			if (yld->model->name == b2k_model) {
				data[0] = (val) ? 0xff : 0x00;
				data[1] = (val) ? 0x00 : 0xff;
				ctl_data->g1.size = 2;
			} else {
				data[0] = (val) ? 0 : 1;	/* invert */
				if (proto == yld_ctl_protocol_g1)
					ctl_data->g1.size = 1;
			}
			break;
		case offsetof(struct yld_status, ringvol):
			if (yld->model->name == p1k_model ||
			    yld->model->name == p1kh_model) {
				ctl_data->cmd	= CMD_RING_VOLUME;
				if (proto == yld_ctl_protocol_g1)
					ctl_data->g1.size = 1;
				data[0] = val;
			}
			break;
		case offsetof(struct yld_status, ringnote_mod):
			if (yld->model->name != p1k_model &&
			    yld->model->name != p1kh_model)
				break;
			if (!yld->ring_notes ||
			    yld->notes_ix >= yld->notes_len)
				break;
			// TODO: check for pause and possibly only write 0 0
			len = yld->notes_len - yld->notes_ix;
			if (proto == yld_ctl_protocol_g1) {
				if (len > sizeof(ctl_data->g1.data))
					len = sizeof(ctl_data->g1.data);
				ctl_data->g1.offset = cpu_to_be16(yld->notes_ix);
				ctl_data->g1.size   = len;
			} else {
				if (len > sizeof(ctl_data->g2.data))
					len = sizeof(ctl_data->g2.data);
			}
			for (i = 0; i < len; i++)
				data[i] = yld->ring_notes[yld->notes_ix + i];
			ctl_data->cmd	= CMD_RING_NOTE;
			yld->notes_ix += len;
			if (yld->notes_ix < yld->notes_len) {
				yld->copy.b[ix] = ~val;	/* not done yet */
				ix--;			/* TODO: verify if correct */
			}
			else
				yld->notes_ix = 0;	/* reset for next time */
			break;
		case offsetof(struct yld_status, dialtone):
			if (yld->model->name == b2k_model ||
			    yld->model->name == p4k_model) {
				ctl_data->cmd	= CMD_DIALTONE;
				ctl_data->g1.size = 1;
				data[0] = val;
			}
			break;
		case offsetof(struct yld_status, ringtone):
			if (yld->model->name == p1k_model ||
			    yld->model->name == p1kh_model) {
				ctl_data->cmd	= CMD_RINGTONE;
				if (yld->model->name == p1k_model)
					data[0] = (val) ? 0x24 : 0x00;
				else
					data[0] = (val) ? 0xff : 0x00;
				if (proto == yld_ctl_protocol_g1)
					ctl_data->g1.size = 1;
			}
			break;
		case offsetof(struct yld_status, backlight):
			/* backlight supported by P4K only */
			if (yld->model->name != p4k_model)
				break;
			ctl_data->cmd	= CMD_LCD_BACKLIGHT;
			ctl_data->g1.size = 1;
			data[0] = val;
			break;
		case offsetof(struct yld_status, speaker):
			/* speakerphone supported by P4K only */
			if (yld->model->name != p4k_model)
				break;
			ctl_data->cmd	= CMD_SPEAKER;
			ctl_data->g1.size = 1;
			data[0] = val;
			break;
		case offsetof(struct yld_status, pstn):
			/* USB/PSTN switch supported by B2K only */
			if (yld->model->name != b2k_model)
				break;
			ctl_data->cmd	= CMD_PSTN_SWITCH;
			ctl_data->g1.size = 1;
			data[0] = val;
			break;
		case offsetof(struct yld_status, keynum):
			/* explicit query for key code only required for G1 phones */
			if (proto != yld_ctl_protocol_g1)
				break;
			val--;
			val &= 0x1f;
			ctl_data->cmd		= CMD_SCANCODE;
			ctl_data->g1.size	= 1;
			ctl_data->g1.offset	= cpu_to_be16(val);
			break;
		default:
			/* no LCD available for B2K */
			if (yld->model->name == b2k_model)
				break;

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
				data += 2;		/* data starts here */
			}

			/* Combine up to <len> consecutive LCD bytes
			 * in a singe request */
			ctl_data->cmd	= CMD_LCD;
			for(i=0; i<len; i++) {
				val = yld->master.b[ix];
				yld->copy.b[ix]	= val;
				data[i]		= val;
				ix++;
			}
			ix--;
			break;
		}
		ix++;
		if (ix >= sizeof(yld->master))
			ix = 0;
	} while ((ctl_data->cmd == 0) && (ix != yld->stat_ix));

	yld->stat_ix = ix;

	if (ctl_data->cmd != 0) {
		/* calculate the checksum */
		pkt_update_checksum(ctl_data, USB_PKT_LEN(proto));
		return 1;
	}
	return 0;
}
 
static int poke_update_from_userspace(struct yealink_dev *yld)
{
	enum yld_ctl_protocols proto;
	int timer_expired, idle;
	int do_update, do_scan;
	int send_update;
	int ret = 0;
	unsigned long spin_flags;

	dbg("%s", __FUNCTION__);

	if (yld->usb_pause)
		BUG();

	proto = yld->model->protocol;

	/* TODO for next 15 lines: Handle G2 differently!! */
	if (proto == yld_ctl_protocol_g1) {
		spin_lock_irqsave(&yld->flags_lock, spin_flags);
		timer_expired = yld->timer_expired;
		idle = !yld->update_active && !yld->scan_active;
		do_update = idle && !timer_expired;
		do_scan = idle && timer_expired;
		yld->timer_expired = timer_expired && !idle;
		yld->dirty = yld->update_active && !do_update;
		yld->update_active = yld->update_active || do_update;
		yld->scan_active = yld->scan_active || do_scan;
		spin_unlock_irqrestore(&yld->flags_lock, spin_flags);
		dbg("t=%d,u=%d,s=%d",yld->timer_expired,yld->update_active,yld->scan_active);
		if (yld->dirty) dbg("set dirty");

		if (do_scan) {
			return submit_scan_request(yld, GFP_KERNEL);
		}
	} else {	/* yld_ctl_protocol_g2 */
		do_update = 0; //@@@@
	}

	if (!do_update) {
		dbg("cycle already active");
		return 0;
	}

	/* find update candidates: copy != master */
	send_update = prepare_update_cmd(yld);

	if (send_update) {
		/* submit the assembled command */
		if (likely(!yld->shutdown)) {
			ret = usb_submit_urb(yld->urb_ctl, GFP_KERNEL);
		}
	} else {
		/* As no updates have to be performed, the update_active
		 * flag has to be reset. In addition if the timer already
		 * expired, a scan has to be performed. */
		if (proto == yld_ctl_protocol_g1) {
			spin_lock_irqsave(&yld->flags_lock, spin_flags);
			do_scan = yld->timer_expired;
			yld->scan_active = do_scan;
			yld->timer_expired = 0;
			yld->update_active = 0;
			spin_unlock_irqrestore(&yld->flags_lock, spin_flags);
			dbg("t=0,u=0,s=%d",yld->scan_active);
			if (do_scan)
				ret = submit_scan_request(yld, GFP_KERNEL);
		} else {
			/* yld_ctl_protocol_g2 */
			//... @@
		}
	}
	return ret;
}

static int perform_single_update(struct yealink_dev *yld)
{
	enum yld_ctl_protocols proto;
	int dont_break, timer_expired, pause, dirty;
	int do_update, do_scan;
	int send_update;
	int ix;
	int ret = 0;
	int max_dirty_scans = 2;	/* allow a total of two status scans */

	dbg("%s", __FUNCTION__);

	proto = yld->model->protocol;

rescan_dirty:
	ix = yld->stat_ix;

	/* TODO for next 15 lines: Handle G2 differently!! */
	if (proto == yld_ctl_protocol_g1) {
		/* writing ringtone notes must not be interrupted */
		/* same for key scan */
		dont_break = ((ix == offsetof(struct yld_status, ringnote_mod)) &&
			      (yld->notes_ix != 0)) ||
			     ((ix == offsetof(struct yld_status, keynum)) &&
			      (yld->master.b[ix] != yld->copy.b[ix]));

		spin_lock(&yld->flags_lock);
		timer_expired = yld->timer_expired;
		pause = yld->usb_pause;
		do_update = (!timer_expired && !pause) || dont_break;
		do_scan = !do_update && !pause;
		yld->timer_expired = timer_expired && !do_scan;
		yld->update_active = do_update;
		yld->scan_active = do_scan;
		yld->dirty = 0;
		spin_unlock(&yld->flags_lock);
		dbg("t=%d,u=%d,s=%d",yld->timer_expired,do_update,do_scan);

		if (do_scan) {
			return submit_scan_request(yld, GFP_ATOMIC);
		}
	} else {	/* yld_ctl_protocol_g2 */
		do_update = 0; //@@@@
	}

	if (!do_update) {
		dbg("pausing updates");
		return 0;
	}

	/* find update candidates: copy != master */
	send_update = prepare_update_cmd(yld);
	if (!send_update && yld->dirty) {
		/* does this catch all corner cases? @@ */
		yld->dirty = 0;
		send_update = prepare_update_cmd(yld);
	}

	if (send_update) {
		/* submit the assembled command */
		if (likely(!yld->shutdown)) {
			ret = usb_submit_urb(yld->urb_ctl, GFP_ATOMIC);
		}
	} else {
		/* As no updates have to be performed, the update_active
		 * flag has to be reset. In addition if the timer already
		 * expired, a scan has to be performed. */
		if (proto == yld_ctl_protocol_g1) {
			spin_lock(&yld->flags_lock);
			timer_expired = yld->timer_expired;
			pause = yld->usb_pause;
			do_scan = timer_expired && !pause;
			yld->timer_expired = timer_expired && pause;
			yld->scan_active = do_scan;
			yld->update_active = 0;
			dirty = yld->dirty = yld->dirty && !do_scan;
			spin_unlock(&yld->flags_lock);
			dbg("t=%d,u=0,s=%d",yld->timer_expired,yld->scan_active);
			if (do_scan)
				ret = submit_scan_request(yld, GFP_ATOMIC);
			else
			if (unlikely(dirty)) {
				/* This can only happen if another processor
				 * modified the master buffer we did not catch
				 * that modification anymore during the
				 * comparison -> compare a 2nd time, but not
				 * more to not get caught up inside IRQ context!
				 */
				dbg("got dirty during comparison!!");
				max_dirty_scans--;
				if (max_dirty_scans > 0)
					goto rescan_dirty;
				/* else give up until next timeout */
			}
		} else {
			/* yld_ctl_protocol_g2 */
			//... @@
		}
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
	enum yld_ctl_protocols proto;
	unsigned long delay;
	
	dbg("%s", __FUNCTION__);
	yld = (struct yealink_dev *)ylda;
	proto = yld->model->protocol;

	if (proto == yld_ctl_protocol_g1) {
		int timer_expired, scan_active;
		int do_submit;

		spin_lock_irq(&yld->flags_lock);
		scan_active = yld->scan_active;
		timer_expired = yld->timer_expired;
		do_submit = !yld->usb_pause && !yld->update_active && !scan_active;
		yld->scan_active = scan_active || do_submit;
		yld->timer_expired = !do_submit;
		spin_unlock_irq(&yld->flags_lock);
		dbg("t=%d,u=%d,s=%d",yld->timer_expired,yld->update_active,yld->scan_active);
		
		if (unlikely(timer_expired))
			warn("timeout was not serviced in time!");
		if (likely(!yld->shutdown)) {
			delay = (yld->model->name == p4k_model) ?
					YEALINK_POLLING_DELAY / 2 :
					YEALINK_POLLING_DELAY;
			delay = (HZ * delay + 999) / 1000;	/* calculate ticks */
			mod_timer(&yld->timer, jiffies + delay);
			if (do_submit) {
				(void) submit_scan_request(yld, GFP_ATOMIC);
			}
		}
	} else {	/* yld_ctl_protocol_g2 */
		
	}
}

/*
 */
static void urb_irq_callback(struct urb *urb)
{
	struct yealink_dev *yld = urb->context;
	enum yld_ctl_protocols proto;
	u8 data0;
	int ret;
	int status = urb->status;

	dbg("%s", __FUNCTION__);
	proto = yld->model->protocol;
	data0 = (proto == yld_ctl_protocol_g1) ?
	          yld->irq_data->g1.data[0] : yld->irq_data->g2.data[0];

	if (status) {
		err("%s - urb status %d", __FUNCTION__, status);
		goto send_next;		/* do not process the irq_data */
	}
	if (pkt_verify_checksum(yld->irq_data, USB_PKT_LEN(proto)) != 0) {
		warn("Received packet with invalid checksum, dropping it");
		goto send_next;		/* do not process the irq_data */
	}

	switch (yld->irq_data->cmd) {
	case CMD_KEYPRESS:
		yld->master.s.keynum = data0;
		break;

	case CMD_HOOKPRESS:
		ret = data0 & 0x10;
		if (yld->hookstate == ret)
			break;
		dbg("hookstate: %x", ret);
		if (yld->open) {
			input_report_key(yld->idev, KEY_PHONE, ret >> 4);
			input_sync(yld->idev);
		}
		yld->hookstate = ret;
		break;

	case CMD_SCANCODE:
		dbg("get scancode %x", data0);
		ret = yld->model->keycode(data0);
		if (yld->open)
			report_key(yld, ret);
		if (ret < 0 && data0 != 0xff)
			info("unknown scancode 0x%02x", data0);
		break;

	case STATE_BAD_PKT:
		warn("phone received invalid packet");
		break;

	default:
		err("unexpected response %x", yld->irq_data->cmd);
	}

send_next:
	if (proto == yld_ctl_protocol_g1) {
		ret = perform_single_update(yld);
		if (ret)
			err("%s - perform_single_update failed %d",
			    __FUNCTION__, ret);
	} else {
		/* always wait for a key or some other interrupt */
		if (likely(!yld->shutdown)) {
			ret = usb_submit_urb(yld->urb_irq, GFP_ATOMIC);
			if (ret)
				err("%s - usb_submit_urb failed %d",
				    __FUNCTION__, ret);
		}
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
	int ret = 0;
	int status = urb->status;
	
	dbg("%s", __FUNCTION__);
	if (status)
		err("%s - urb status %d", __FUNCTION__, status);

	proto = yld->model->protocol;

	switch (yld->ctl_data->cmd) {
	case CMD_HOOKPRESS:
	case CMD_KEYPRESS:
	case CMD_SCANCODE:
		if (proto == yld_ctl_protocol_g1) {
			/* Expect a response on the irq endpoint! */
			if (likely(!yld->shutdown))
				ret = usb_submit_urb(yld->urb_irq, GFP_ATOMIC);
		} else {
			/* queue up the next request after a longer delay */
			/* TODO: How can we detect that the irq was generated
			 *       as expected? */
			/* TODO: DELAY + need to check the pause flag!! */
			ret = perform_single_update(yld);
		}
		break;
	default:
		if (proto == yld_ctl_protocol_g1) {
			/* immediately send new command */
			ret = perform_single_update(yld);
		} else {
			/* queue up the next request after some small delay */
			/* TODO: DELAY + need to check the pause flag!! */
			ret = perform_single_update(yld);
			//		YEALINK_COMMAND_DELAY_G2);
		}
		break;
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
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,21)
	struct yealink_dev *yld = dev->private;
#else
	struct yealink_dev *yld = input_get_drvdata(dev);
#endif

	down_write(&sysfs_rwsema);
	yld->open = 1;
	up_write(&sysfs_rwsema);

	return 0;
}

static void input_close(struct input_dev *dev)
{
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,21)
	struct yealink_dev *yld = dev->private;
#else
	struct yealink_dev *yld = input_get_drvdata(dev);
#endif

	down_write(&sysfs_rwsema);
	yld->open = 0;
	up_write(&sysfs_rwsema);
	stop_traffic(yld);
}

/*******************************************************************************
 * sysfs interface
 ******************************************************************************/

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
	int ret = count;

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

	if (submit && (poke_update_from_userspace(yld) != 0))
		ret = -ERESTARTSYS;

	up_write(&sysfs_rwsema);

	return ret;
#if 0
	if (submit && yld->model && yld->model->protocol == yld_ctl_protocol_g2) {
		if (down_interruptible(&yld->update_sem)) //??
			return -ERESTARTSYS;
		//TODO submit_next_request(yld, GFP_KERNEL, 0);
	}
	return count;
#endif
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
	enum yld_ctl_protocols proto;
	int early_wait = 0;
	int i;
	int ret = count;

	down_write(&sysfs_rwsema);
	yld = dev_get_drvdata(dev);
	if (yld == NULL || yld->model == NULL) {
		up_write(&sysfs_rwsema);
		return -ENODEV;
	}
	proto = yld->model->protocol;
	if (proto == yld_ctl_protocol_g2) {
		early_wait = (chr != ' ') &&
		             (strncmp(buf, "RINGTONE", count) == 0);
		if (early_wait) {
			/* must update the ringnotes before activating the buzzer */
			if (down_interruptible(&yld->update_sem)) {
				up_write(&sysfs_rwsema);
				return -ERESTARTSYS;
			}
			yld->master.s.ringnote_mod++;
		}
	}

	for (i = 0; i < ARRAY_SIZE(lcdMap); i++) {
		if (lcdMap[i].type != '.')
			continue;
		if (strncmp(buf, lcdMap[i].u.p.name, count) == 0) {
			setChar(yld, i, chr);
			break;
		}
	}

	if (poke_update_from_userspace(yld) != 0)
		ret = -ERESTARTSYS;

	up_write(&sysfs_rwsema);

	return ret;
#if 0
	if (proto == yld_ctl_protocol_g2) {
		if (!early_wait && down_interruptible(&yld->update_sem))
			return -ERESTARTSYS;
		//TODO submit_next_request(yld, GFP_KERNEL, 0);
	}
	return count;
#endif
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
	int stopped;
	int i;
	int ret = count;

	down_write(&sysfs_rwsema);
	yld = dev_get_drvdata(dev);
	if (yld == NULL || yld->model == NULL) {
		up_write(&sysfs_rwsema);
		return -ENODEV;
	}
	
	/* first stop the whole USB cycle */
	yld->usb_pause = 1;
	i = 10;
	while ((yld->scan_active || yld->update_active) && (i-- > 0)) {
		dbg("sleep");
		msleep_interruptible(50);
	}
	spin_lock_irq(&yld->flags_lock);
	stopped = !yld->scan_active && !yld->update_active;
	if (!stopped)
		yld->usb_pause = 0;
	spin_unlock_irq(&yld->flags_lock);

	if (stopped) {
		dbg("t=%d,u=%d,s=%d",yld->timer_expired,yld->update_active,yld->scan_active);
		set_ringnotes(yld, (char *)buf, count);
		yld->master.s.ringnote_mod++;
		yld->usb_pause = 0;
		if (poke_update_from_userspace(yld) != 0)
			ret = -ERESTARTSYS;
	} else {
		err("Could not stop update cycle to write ringnotes!");
	}

	
/*	if (!stopped) {
		dbg("need to wait");
		wait_event(&my_event,
			(!yld->scan_active && !yld->update_active) );
	}

	set_ringnotes(yld, (char *)buf, count);
	if (yld->model->protocol == yld_ctl_protocol_g1 && count > 1) {
		// force immediate download update of ringnotes
		yld->master.s.ringnote_mod++;
	}*/

	up_write(&sysfs_rwsema);
	return ret;
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
 * Initialization / shutdown of the device
 ******************************************************************************/

static int update_version_init(struct yealink_dev *yld)
{
	union yld_ctl_packet *ctl_data, *int_data;
	enum yld_ctl_protocols proto;
	int len, i;
	u8 *data;
	u16 version;
	int ret = 0;

	might_sleep();
	
	if (!yld->model) {	/* return silently */
		warn("%s - no model preselected!", __FUNCTION__);
		return -ENODEV;
	}
	/*dbg("%s", __FUNCTION__);*/

	proto = yld->model->protocol;
	len = USB_PKT_LEN(proto);

	ctl_data = kzalloc(len, GFP_KERNEL);
	if (!ctl_data)
		return -ENOMEM;
	int_data = kzalloc(len, GFP_KERNEL);
	if (!int_data) {
		kfree(ctl_data);
		return -ENOMEM;
	}

	/* prepare the VERSION command */
	ctl_data->cmd = CMD_VERSION;
	if (proto == yld_ctl_protocol_g1)
		ctl_data->g1.size = 2;
	pkt_update_checksum(ctl_data, len);
	
	ret = submit_cmd_int_sync(yld, ctl_data, len, int_data, len);
	if (ret != 0)
		goto leave_clean;

	/* update model information */
	data = (proto == yld_ctl_protocol_g1) ? int_data->g1.data :
						int_data->g2.data;
	version = (data[0] << 8) | data[1];
	if (proto == yld_ctl_protocol_g1) {
		/* can only auto-detect G1 devices for now */
		enum model_info_idx info_idx;
		info_idx = (YLD_IS_P1K(version)) ? model_info_idx_p1k :
			   (YLD_IS_P4K(version)) ? model_info_idx_p4k :
			   (YLD_IS_B2K(version)) ? model_info_idx_b2k :
			   model_info_unknown;
		yld->model = (info_idx != model_info_unknown) ?
				&model[info_idx] : NULL;
	}
	if (!yld->model) {
		int pid = le16_to_cpu(yld->udev->descriptor.idProduct);
		warn(KERN_INFO "Yealink model not supported: "
			"PID %04x, version 0x%04x.", pid, version);
		ret = -ENODEV;
		goto leave_clean;
	}

	info("Detected Model %s (Version 0x%04x)",
		yld->model->name, version);
	sprintf(yld->uniq, "%04x", version);

	/* prepare and submit next command */
	ctl_data->cmd = CMD_INIT;
	if (proto == yld_ctl_protocol_g1)
		ctl_data->g1.size = sizeof(ctl_data->g1.data);
	pkt_update_checksum(ctl_data, len);
	
	ret = submit_cmd_int_sync(yld, ctl_data, len, int_data, len);
	if (ret != 0)
		return ret;

	len = USB_PKT_DATA_LEN(proto);
	if ((len * 2 + 5) > sizeof(yld->uniq)) {
		BUG();
		ret = -ENODEV;
		goto leave_clean;
	}

	for (i = 0; i < len; i++) {
	  sprintf(yld->uniq+4+(i*2), "%02x", data[i]);
	}
	info("Serial Number %s", yld->uniq+4);

leave_clean:
        kfree(int_data);
        kfree(ctl_data);
	return ret;
}

static void restore_state(struct yealink_dev *yld)
{
	int i;

	/* force updates to device */
	for (i = 0; i<sizeof(yld->master); i++)
		yld->copy.b[i] = ~yld->master.b[i];
	yld->key_code = -1;
	yld->last_cmd = CMD_KEYPRESS;
	yld->hookstate = 0;
	yld->stat_ix = 0;
	yld->notes_ix = 0;
	/* flags */
	yld->scan_active = 0;
	yld->update_active = 0;
	yld->timer_expired = 0;
	yld->usb_pause = 0;
	yld->dirty = 0;
}

static int init_state(struct yealink_dev *yld)
{
	enum yld_ctl_protocols proto;
	int i;

	if (!yld->model) {	/* return silently */
		warn("%s - no model selected!", __FUNCTION__);
		return -ENODEV;
	}
	/*dbg("%s", __FUNCTION__);*/

	proto = yld->model->protocol;

	/* clear visible elements */
	for (i = 0; i < ARRAY_SIZE(lcdMap); i++)
		setChar(yld, i, ' ');

	/* display driver version on LCD line 3 */
	store_line(&yld->interface->dev, "yld-" DRIVER_VERSION,
		sizeof("yld-" DRIVER_VERSION),
		LCD_LINE3_OFFSET, LCD_LINE3_SIZE, 0);

	if (proto == yld_ctl_protocol_g1)
	        set_ringnotes(yld, default_ringtone_g1,
	                      sizeof(default_ringtone_g1));
	else
	        set_ringnotes(yld, default_ringtone_g2,
	                      sizeof(default_ringtone_g2));

	restore_state(yld);
	return 0;
}

static int start_traffic(struct yealink_dev *yld)
{
	enum yld_ctl_protocols proto;
	int ret = 0;

	proto = yld->model->protocol;
	if (likely(!yld->shutdown)) {
		if (proto == yld_ctl_protocol_g1) {
			mod_timer(&yld->timer, jiffies + 2);
		} else {
			/* immediately start waiting for a key */
			ret = usb_submit_urb(yld->urb_irq, GFP_KERNEL);
		}
		if (ret == 0)
			ret = poke_update_from_userspace(yld);
	}
	return ret;
}

static void stop_traffic(struct yealink_dev *yld)
{
	yld->shutdown = 1;
	smp_wmb();			/* make sure other CPUs see this */

	if (timer_pending(&yld->timer))
		del_timer(&yld->timer);

	usb_kill_urb(yld->urb_irq);
	usb_kill_urb(yld->urb_ctl);

	yld->shutdown = 0;
	smp_wmb();
}

/*******************************************************************************
 * Linux interface and usb initialization
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

	stop_traffic(yld);

        if (yld->idev) {
		if (err)
			input_free_device(yld->idev);
		else
			input_unregister_device(yld->idev);
	}

	usb_free_urb(yld->urb_irq);
	usb_free_urb(yld->urb_ctl);

	if (yld->ctl_req)
		usb_buffer_free(yld->udev, sizeof(*(yld->ctl_req)),
		                yld->ctl_req, yld->ctl_req_dma);
	if (yld->ctl_data)
		usb_buffer_free(yld->udev, USB_PKT_LEN(yld->model->protocol),
		                yld->ctl_data, yld->ctl_dma);
	if (yld->irq_data)
		usb_buffer_free(yld->udev, USB_PKT_LEN(yld->model->protocol),
		                yld->irq_data, yld->irq_dma);

	kfree(yld);
	return err;
}

static int usb_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct yealink_dev *yld = usb_get_intfdata(intf);

	stop_traffic(yld);
	return 0;
}

static int usb_resume(struct usb_interface *intf)
{
	struct yealink_dev *yld = usb_get_intfdata(intf);
	int ret;

	restore_state(yld);
	ret = start_traffic(yld);
	return ret;
}

static int usb_reset_resume(struct usb_interface *intf)
{
	struct yealink_dev *yld = usb_get_intfdata(intf);
	int ret;

	ret = update_version_init(yld);
	if (ret == 0) {
		restore_state(yld);
		ret = start_traffic(yld);
	}
	return ret;
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
	enum yld_ctl_protocols proto;
	int ret, pipe, i, j;
	int pkt_len;

	//dbg("%s - start", __FUNCTION__);
	
	interface = intf->cur_altsetting;
	endpoint = &interface->endpoint[0].desc;
	if (!usb_endpoint_is_int_in(endpoint))
		return -ENODEV;

	yld = kzalloc(sizeof(struct yealink_dev), GFP_KERNEL);
	if (!yld)
		return -ENOMEM;

	yld->udev = udev;
	yld->interface = intf;
	yld->int_endpoint = endpoint;

	/* get a handle to the interrupt data pipe */
	pipe = usb_rcvintpipe(udev, endpoint->bEndpointAddress);
	pkt_len = usb_maxpacket(udev, pipe, usb_pipeout(pipe));

	if (pkt_len == USB_PKT_LEN_G1) {
		proto = yld_ctl_protocol_g1;
		yld->model = &model[model_info_idx_p1k];	/* changed later */
	} else if (pkt_len == USB_PKT_LEN_G2) {
		proto = yld_ctl_protocol_g2;
		yld->model = &model[model_info_idx_p1kh];
	} else {
		int pid = le16_to_cpu(udev->descriptor.idProduct);
		warn(KERN_INFO "Yealink model not supported: "
			"PID %04x, payload size %d.", pid, pkt_len);
		return usb_cleanup(yld, -ENODEV);
	}

	ret = update_version_init(yld);
	if (ret != 0)
		return usb_cleanup(yld, -ENODEV);
	/* right now bail out for G2 phones @@@ */
	if (proto == yld_ctl_protocol_g2) {
		warn("This is a developer version of the yealink driver, "
		     "which does not support the USB-P1KH!!");
		return usb_cleanup(yld, -ENODEV);
	}

	sema_init(&yld->update_sem, 1);
	spin_lock_init(&yld->flags_lock);

	yld->idev = input_dev = input_allocate_device();
	if (!input_dev)
		return usb_cleanup(yld, -ENOMEM);

	/* allocate usb buffers */
	yld->irq_data = usb_buffer_alloc(udev, pkt_len,
					GFP_ATOMIC, &yld->irq_dma);
	if (yld->irq_data == NULL)
		return usb_cleanup(yld, -ENOMEM);

	yld->ctl_data = usb_buffer_alloc(udev, pkt_len,
					GFP_ATOMIC, &yld->ctl_dma);
	if (!yld->ctl_data)
		return usb_cleanup(yld, -ENOMEM);

	yld->ctl_req = usb_buffer_alloc(udev, sizeof(*(yld->ctl_req)),
					GFP_ATOMIC, &yld->ctl_req_dma);
	if (yld->ctl_req == NULL)
		return usb_cleanup(yld, -ENOMEM);

	/* allocate urb structures */
	yld->urb_irq = usb_alloc_urb(0, GFP_KERNEL);
        if (yld->urb_irq == NULL)
		return usb_cleanup(yld, -ENOMEM);

	yld->urb_ctl = usb_alloc_urb(0, GFP_KERNEL);
        if (yld->urb_ctl == NULL)
		return usb_cleanup(yld, -ENOMEM);

	/* initialize irq urb */
	usb_fill_int_urb(yld->urb_irq, udev, pipe, yld->irq_data,
			pkt_len,
			urb_irq_callback,
			yld, endpoint->bInterval);
	yld->urb_irq->transfer_dma = yld->irq_dma;
	yld->urb_irq->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	yld->urb_irq->dev = udev;

	/* initialize ctl urb */
	yld->ctl_req->bRequestType = USB_TYPE_CLASS | USB_RECIP_INTERFACE |
				      USB_DIR_OUT;
	yld->ctl_req->bRequest	= USB_REQ_SET_CONFIGURATION;
	yld->ctl_req->wValue	= cpu_to_le16(0x200);
	yld->ctl_req->wIndex	= cpu_to_le16(interface->desc.bInterfaceNumber);
	//ctl->ctl_req->wIndex	= cpu_to_le16(interface->cur_altsetting->desc.bInterfaceNumber); ???
	yld->ctl_req->wLength	= cpu_to_le16(pkt_len);

	usb_fill_control_urb(yld->urb_ctl, udev, usb_sndctrlpipe(udev, 0),
			(void *)yld->ctl_req, yld->ctl_data, pkt_len,
			urb_ctl_callback, yld);
	yld->urb_ctl->setup_dma	= yld->ctl_req_dma;
	yld->urb_ctl->transfer_dma	= yld->ctl_dma;
	yld->urb_ctl->transfer_flags	|= URB_NO_SETUP_DMA_MAP |
					URB_NO_TRANSFER_DMA_MAP;
	yld->urb_ctl->dev = udev;

	/* set up the periodic scan/update timer */
	setup_timer(&yld->timer, timer_callback, (unsigned long)yld);

	/* find out the physical bus location */
	usb_make_path(udev, yld->phys, sizeof(yld->phys));
	strlcat(yld->phys,  "/input0", sizeof(yld->phys));

	/* register settings for the input device */
//	input_dev->name = nfo->name; TODO better driver messages / registration
//	input_dev->name = yld_device[i].name;
	input_dev->name = "yealink";
	input_dev->uniq = yld->uniq;
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
	set_bit(KEY_PHONE, input_dev->keybit);   // ???@@@ always?
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

	/* initialize the device and start the key/hook scanner */
	init_state(yld);
	ret = start_traffic(yld);
	if (ret != 0)
		return usb_cleanup(yld, ret);

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
	.suspend	= usb_suspend,
	.resume		= usb_resume,
	.reset_resume	= usb_reset_resume,
	.id_table	= usb_table,
};

static int __init yealink_dev_init(void)
{
	int ret = usb_register(&yealink_driver);
	if (ret == 0)
		info(DRIVER_DESC ": " DRIVER_VERSION " (C) " DRIVER_AUTHOR);
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
