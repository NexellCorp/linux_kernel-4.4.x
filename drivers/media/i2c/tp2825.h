#ifndef __TP2825_H__
#define __TP2825_H__

#define TP2825_VERSION_CODE KERNEL_VERSION(0, 0, 1)

enum {
	TP2825 = 0x2825,
};

enum {
	TP2825_NONE = 0x00,
	TP2825_1080P25 = 0x03,
	TP2825_1080P30 = 0x02,
	TP2825_720P25 = 0x05,
	TP2825_720P30 = 0x04,
	TP2825_720P50 = 0x01,
	TP2825_720P60 = 0x00,
	TP2825_SD = 0x06,
	INVALID_FORMAT = 0x07,
	TP2825_720P25V2 = 0x0D,
	TP2825_720P30V2 = 0x0C,
	TP2825_PAL = 0x08,
	TP2825_NTSC = 0x09,
	TP2825_PAL_HALF = 0x0A,
	TP2825_NTSC_HALF = 0x0B,
	TP2825_HALF1080P25 = 0x43,
	TP2825_HALF1080P30 = 0x42,
	TP2825_HALF720P25 = 0x45,
	TP2825_HALF720P30 = 0x44,
	TP2825_HALF720P50 = 0x41,
	TP2825_HALF720P60 = 0x40
};

enum {
	VIDEO_UNPLUG,
	VIDEO_IN,
	VIDEO_LOCKED,
	VIDEO_UNLOCK
};

enum {
	MUX656_8BIT, /* Y/C-mux 4:2:2 8-bit with embedded sync */
	SEP656_8BIT, /* Y/C-mux 4:2:2 8-bit with seperate sync */
	EMB422_16BIT, /* YCbCr 4:2:2 16-bit with embedded sync */
	SEP422_16BIT /* YCbCr 4:2:2 10-bit with embedded sync */
};

enum {
	CH_1 = 0,
	CH_2 = 1,
	CH_3 = 2,
	CH_4 = 3,
	CH_ALL = 4,
	DATA_PAGE = 5,
	AUDIO_PAGE = 9
};

enum {
	SCAN_DISABLE = 0,
	SCAN_AUTO,
	SCAN_TVI,
	SCAN_HDA,
	SCAN_HDC,
	SCAN_MANUAL,
	SCAN_TEST
};

enum {
	STD_TVI,
	STD_HDA,
	STD_HDC,
	STD_HDA_DEFAULT,
	STD_HDC_DEFAULT
};

enum {
	PTZ_TVI=0,
	PTZ_HDA=1,
	PTZ_HDC=4
};

#define FLAG_LOSS 0x80
#define FLAG_H_LOCKED 0x20
#define FLAG_HV_LOCKED 0x60

#define FLAG_HDC_MODE 0x80
#define FLAG_HALF_MODE 0x40
#define FLAG_MEGA_MODE 0x20
#define FLAG_HDA_MODE 0x10

#define CHANNELS_PER_CHIP 1
#define MAX_CHIPS 2
#define SUCCESS 0
#define FAILURE -1

#define BRIGHTNESS 0x10
#define CONTRAST 0x11
#define SATURATION 0x12
#define HUE 0X13
#define SHARPNESS 0X14

#define MAX_COUNT 0xffff

typedef struct _tp2825_register {
	unsigned char chip;
	unsigned char ch;
	unsigned int reg_addr;
	unsigned int value;
} tp2825_register;

typedef struct _tp2825_work_mode {
	unsigned char chip;
	unsigned char ch;
	unsigned char mode;
} tp2825_work_mode;

typedef struct _tp2825_video_mode {
	unsigned char chip;
	unsigned char ch;
	unsigned char mode;
	unsigned char std;
} tp2825_video_mode;

typedef struct _tp2825_video_loss {
	unsigned char chip;
	unsigned char ch;
	unsigned char is_lost;
} tp2825_video_loss;

typedef struct _tp2825_image_adjust {
	unsigned char chip;
	unsigned char ch;
	unsigned int hue;
	unsigned int contrast;
	unsigned int brightness;
	unsigned int saturation;
	unsigned int sharpness;
} tp2825_image_adjust;

typedef struct _tp2825_PTZ_data {
	unsigned char chip;
	unsigned char ch;
	unsigned char mode;
	unsigned char data[16];
} tp2825_PTZ_data;

#endif
