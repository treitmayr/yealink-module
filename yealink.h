/*
 * drivers/usb/input/yealink.h
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
#ifndef INPUT_YEALINK_H
#define INPUT_YEALINK_H

/* Using the control channel on interface 3 various aspects of the phone
 * can be controlled like LCD, LED, dialtone and the ringtone.
 */

/* "Generation 1" models use 16 byte packets */

#define USB_PKT_DATA_LEN_G1	11
#define USB_PKT_LEN_G1	sizeof(struct yld_ctl_packet_g1)

struct yld_ctl_packet_g1 {
	u8	cmd;		/* command code, see below */
	u8	size;		/* 1-11, size of used data bytes. */
	u16	offset;		/* internal packet offset */
	u8	data[USB_PKT_DATA_LEN_G1];
	s8	sum;		/* negative sum of 15 preceding bytes */
} __attribute__ ((packed));

/* "Generation 2" models use 8 byte packets */

#define USB_PKT_DATA_LEN_G2	6
#define USB_PKT_LEN_G2	sizeof(struct yld_ctl_packet_g2)

struct yld_ctl_packet_g2 {
	u8	cmd;		/* command code, see below */
	u8	data[USB_PKT_DATA_LEN_G2];
	s8	sum;		/* negative sum of 7 preceding bytes */
} __attribute__ ((packed));

union yld_ctl_packet {
	u8	cmd;		/* command code is always the first byte */
	struct	yld_ctl_packet_g1 g1;
	struct	yld_ctl_packet_g2 g2;
};

enum yld_ctl_protocols {
  yld_ctl_protocol_g1,
  yld_ctl_protocol_g2
};

#define USB_PKT_LEN(p) (((p) == yld_ctl_protocol_g1) ? \
			USB_PKT_LEN_G1 : USB_PKT_LEN_G2)

#define USB_PKT_DATA_LEN(p) (((p) == yld_ctl_protocol_g1) ? \
			USB_PKT_DATA_LEN_G1 : USB_PKT_DATA_LEN_G2)

/* version ranges determined according to
 * http://www.devbase.at/svn/view.cgi/yealink-logs/version-numbers.txt?root=experimental
 */
#define YLD_IS_P1K(v)	((v) >= 0x0100 && (v) <= 0x01ff)
#define YLD_IS_P4K(v)	((v) >= 0x0230 && (v) <= 0x02ff)
#define YLD_IS_B2K(v)	(((v) >= 0x0520 && (v) <= 0x053f) || \
			 ((v) >= 0x0570 && (v) <= 0x058f))
#define YLD_IS_B3G(v)	((v) >= 0x0540 && (v) <= 0x056f)

/* The following yld_ctl_packet's are available: */

/* Init registers
 *
 * cmd		0x8e
 * size		10
 * offset	0
 * data		0,0,0,0....
 */
#define CMD_INIT		0x8e

/* Read version
 *
 * cmd		0x87
 * size		2
 * offset	0
 * data		0,0
 */
#define CMD_VERSION		0x87

/* Request scan of attached PSTN
 *
 * models       B2K (not B3G!)
 * cmd          0x8d
 * size         1
 * offset       0
 * data[0]      on return: 
 *                bit 0: 0 .. PSTN ring off or ring pause
                         1 .. PSTN ring on
 *                bit 1: 0 .. on hook
 *                       1 .. off hook
 * Note: Bit 0 turns on and off along with the PSTN ring signal. Then end
 *       of the ring sequence (other party hung up) is detected when bit 0
 *       stays 0 for a few seconds.
 *       If not already in PSTN mode, SW should switch to PSTN immediately
 *       when detecting the first ring signal (via CMD_PSTN_SWITCH).
 */
#define CMD_HANDSET	0x8d

/* Request key scan
 *
 * cmd		0x80
 * size		1
 * offset	0
 * data[0]	on return returns the key event sequence, if it changes
 *		there's a new key pressed.
 */
#define CMD_KEYPRESS		0x80

/* Request scancode
 *
 * cmd		0x81
 * size		1
 * offset	key number [0-1f]
 * data[0]	on return returns the scancode
 */
#define CMD_SCANCODE		0x81

/* Request Hook scan
 *
 * models	P3K, P4K, V1K
 * cmd		0x8b
 * size		1
 * offset	0
 * data[0]	on return: bit 4: 1 .. on hook / 0 .. off hook
 */
#define	CMD_HOOKPRESS		0x8b

/* Set LCD
 *
 * cmd		0x04
 * size		1-11
 * offset	0-23
 * data		segment bits
 */
#define CMD_LCD			0x04

/* Set led
 *
 * models       P1K, P1KH, B2K, B3G
 * cmd		0x05
 * size		1
 * offset	0
 * data[0]	1 OFF / 0 ON (P1K, P1KH)
 * data[0]	USB LED (00 OFF / ff ON)	(B2K, B3G)
 * data[1]	PSTN LED (00 OFF / ff ON)	(B2K, B3G)
 */
#define CMD_LED			0x05

/* Set ringtone volume
 *
 * models       P1K, P1KH
 * cmd		0x11
 * size		1
 * offset	0
 * data[0]	0-0xff  volume
 */
#define CMD_RING_VOLUME		0x11

/* Turn on the speaker for ringing and hands-free operation
 *
 * models       P3K, P4K, V1K
 * cmd		0x0c
 * size		1
 * offset	0
 * data[0]	0 .. off, 1 .. on
 * Note: The P4K has no buzzer, so the speaker and an audio ring tone
 *       have to be used for ringing.
 */
#define CMD_SPEAKER             0x0c

/* Set ringtone notes
 *
 * models	P1K, P1KH
 * cmd		0x02
 * size		1-11
 * offset	0->
 * data		P1K:  binary representation LE16(-freq), LE16(duration) ....
 *		P1KH: only 1 byte for each -freq, duration
 * Note: Current investigations with the P1KH suggest that the P1KH requires
 *       the ring notes to be downloaded each time before turning on the
 *       ringtone.
 */
#define CMD_RING_NOTE		0x02

/* Sound ringtone via the speaker on the back
 *
 * models	P1K, P1KH
 * cmd		0x03
 * size		1
 * offset	0
 * data[0]	0 OFF / 0x24 ON (P1K)
 *              0 OFF / 0xff ON (P1KH)
 * Note: For the P1KH it was observed that ringing stops after ~20s by itself.
 */
#define CMD_RINGTONE		0x03

/* Sound dial tone via the ear speaker
 *
 * models       B2K, B3G, P3K, P4K, V1K
 * cmd		0x09
 * size		1
 * offset	0
 * data[0]	0 OFF / 1 ON
 */
#define CMD_DIALTONE		0x09

/* LCD backlight control
 *
 * models	P3K, P4K, V1K
 * cmd		0x12
 * size		1
 * offset	0
 * data[0]	0 OFF / 1 ON
 */
#define CMD_LCD_BACKLIGHT	0x12

/* B2K Ring control
 *
 * models	B2K, B3G
 * cmd		0x01
 * size		1
 * offset	0
 * data[0]	0 OFF / 1 ON
 * Note: For a periodic ring tone this bit has to be turned on and off.
 */
#define CMD_B2K_RING		0x01

/* B2K PSTN/USB switch
 *
 * models	B2K, B3G
 * cmd		0x0e
 * size		1
 * offset	0
 * data[0]	0 USB / 1 PSTN
 * Note: It is recommended to turn this on (PSTN) when the program shuts down.
 *       For the B3G if call forwarding is enabled, this has to be 0.
 */
#define CMD_PSTN_SWITCH		0x0e

/* Overload state
 *
 * models	any
 * state	0xfd
 * Note: This state seems to be replied whenever the received packet had an
 *       invalid checksum or some other internal error occurred in the phone.
 */
#define STATE_BAD_PKT		0xfd


#endif /* INPUT_YEALINK_H */


#if defined(_SEG) && defined(_PIC)
/* This table maps the LCD segments onto individual bit positions in the
 * yld_status struct.
 */

/* LCD, each segment must be driven seperately.
 *
 * Layout:
 *
 *   |[]   [][]   [][]   [][]   in   |[][]
 *   |[] M [][] D [][] : [][]   out  |[][]
 *                             store
 *
 *    NEW REP         SU MO TU WE TH FR SA
 *
 *    [] [] [] [] [] [] [] [] [] [] [] []
 *    [] [] [] [] [] [] [] [] [] [] [] []
 */

/* Line 1
 *	Format		: 18.e8.M8.88...188
 *	Icon names	: M D : IN OUT STORE
 */
#define LCD_LINE1_OFFSET	0
#define LCD_LINE1_SIZE		17

/* Note: first g then f =>			       !      !      */
/* _SEG(    type    a      b      c      d      e      g      f   )  */
	_SEG('1',  0,0 , 22,2 , 22,2 ,  0,0 ,  0,0 ,  0,0 ,  0,0	),
	_SEG('8', 20,1 , 20,2 , 20,4 , 20,8 , 21,4 , 21,2 , 21,1	),
	_PIC('.', 22,1 , "M"						),
	_SEG('e', 18,1 , 18,2 , 18,4 , 18,1 , 19,2 , 18,1 , 19,1	),
	_SEG('8', 16,1 , 16,2 , 16,4 , 16,8 , 17,4 , 17,2 , 17,1	),
	_PIC('.', 15,8 , "D"						),
	_SEG('M', 14,1 , 14,2 , 14,4 , 14,1 , 15,4 , 15,2 , 15,1	),
	_SEG('8', 12,1 , 12,2 , 12,4 , 12,8 , 13,4 , 13,2 , 13,1	),
	_PIC('.', 11,8 , ":"						),
	_SEG('8', 10,1 , 10,2 , 10,4 , 10,8 , 11,4 , 11,2 , 11,1	),
	_SEG('8',  8,1 ,  8,2 ,  8,4 ,  8,8 ,  9,4 ,  9,2 ,  9,1	),
	_PIC('.',  7,1 , "IN"						),
	_PIC('.',  7,2 , "OUT"						),
	_PIC('.',  7,4 , "STORE"					),
	_SEG('1',  0,0 ,  5,1 ,  5,1 ,  0,0 ,  0,0 ,  0,0 ,  0,0	),
	_SEG('8',  4,1 ,  4,2 ,  4,4 ,  4,8 ,  5,8 ,  5,4 ,  5,2	),
	_SEG('8',  2,1 ,  2,2 ,  2,4 ,  2,8 ,  3,4 ,  3,2 ,  3,1	),

/* Line 2
 *	Format		: .........
 *	Pict. name	: NEW REP SU MO TU WE TH FR SA
 */
#define LCD_LINE2_OFFSET	LCD_LINE1_OFFSET + LCD_LINE1_SIZE
#define LCD_LINE2_SIZE		9

	_PIC('.', 23,2 , "NEW"	),
	_PIC('.', 23,4 , "REP"	),
	_PIC('.',  1,8 , "SU"	),
	_PIC('.',  1,4 , "MO"	),
	_PIC('.',  1,2 , "TU"	),
	_PIC('.',  1,1 , "WE"	),
	_PIC('.',  0,1 , "TH"	),
	_PIC('.',  0,2 , "FR"	),
	_PIC('.',  0,4 , "SA"	),

/* Line 3
 *	Format		: 888888888888
 */
#define LCD_LINE3_OFFSET	LCD_LINE2_OFFSET + LCD_LINE2_SIZE
#define LCD_LINE3_SIZE		12

	_SEG('8', 22,16, 22,32, 22,64, 22,128, 23,128, 23,64, 23,32  ),
	_SEG('8', 20,16, 20,32, 20,64, 20,128, 21,128, 21,64, 21,32  ),
	_SEG('8', 18,16, 18,32, 18,64, 18,128, 19,128, 19,64, 19,32  ),
	_SEG('8', 16,16, 16,32, 16,64, 16,128, 17,128, 17,64, 17,32  ),
	_SEG('8', 14,16, 14,32, 14,64, 14,128, 15,128, 15,64, 15,32  ),
	_SEG('8', 12,16, 12,32, 12,64, 12,128, 13,128, 13,64, 13,32  ),
	_SEG('8', 10,16, 10,32, 10,64, 10,128, 11,128, 11,64, 11,32  ),
	_SEG('8',  8,16,  8,32,  8,64,  8,128,  9,128,  9,64,  9,32  ),
	_SEG('8',  6,16,  6,32,  6,64,  6,128,  7,128,  7,64,  7,32  ),
	_SEG('8',  4,16,  4,32,  4,64,  4,128,  5,128,  5,64,  5,32  ),
	_SEG('8',  2,16,  2,32,  2,64,  2,128,  3,128,  3,64,  3,32  ),
	_SEG('8',  0,16,  0,32,  0,64,  0,128,  1,128,  1,64,  1,32  ),

/* Line 4
 *
 * The LED, DIALTONE and RINGTONE are implemented as icons and use the same
 * sysfs interface.
 */
#define LCD_LINE4_OFFSET	LCD_LINE3_OFFSET + LCD_LINE3_SIZE

	_PIC('.', offsetof(struct yld_status, led) -
		  offsetof(struct yld_status, lcd)	, 0x01, "LED"	   ),
	_PIC('.', offsetof(struct yld_status, dialtone) -
		  offsetof(struct yld_status, lcd)	, 0x01, "DIALTONE" ),
	_PIC('.', offsetof(struct yld_status, ringtone) -
		  offsetof(struct yld_status, lcd)	, 0x01, "RINGTONE" ),
	/* P4K specific: */
	_PIC('.', offsetof(struct yld_status, backlight) -
		  offsetof(struct yld_status, lcd)	, 0x01, "BACKLIGHT"),
	_PIC('.', offsetof(struct yld_status, speaker) -
		  offsetof(struct yld_status, lcd)	, 0x01, "SPEAKER"  ),
	/* B2K specific: */
	_PIC('.', offsetof(struct yld_status, pstn) -
		  offsetof(struct yld_status, lcd)	, 0x01, "PSTN"	   ),

#undef _SEG
#undef _PIC
#endif /* _SEG && _PIC */
