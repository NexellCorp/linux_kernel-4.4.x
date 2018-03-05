#ifndef _MAX9286_PRESET_H
#define _MAX9286_PRESET_H

struct reg_val {
    u8 addr;
    u8 reg;
    u8 val;
};

#undef READ
#define DELAY		0xDE
#define END_VAL		0xFF
#define READ		0xEA

static struct reg_val max96705_partron[] =
{
{	144	,	0	,	130	},
{	144	,	1	,	162	},
{	144	,	2	,	0	},
{	144	,	3	,	1	},
{	144	,	4	,	0	},
{	144	,	5	,	0	},
{	144	,	6	,	0	},
{	144	,	7	,	0	},
{	144	,	8	,	0	},
{	144	,	10	,	255	},
{	144	,	11	,	228	},
{	144	,	12	,	153	},
{	144	,	13	,	243	},
{	144	,	14	,	80	},
{	144	,	15	,	11	},
{	144	,	16	,	0	},
{	144	,	17	,	0	},
{	144	,	18	,	243	},
{	144	,	19	,	15	},
{	144	,	20	,	228	},
{	144	,	21	,	11	},
{	144	,	22	,	0	},
{	144	,	23	,	1	},
{	144	,	24	,	0	},
{	144	,	25	,	0	},
{	144	,	26	,	96	},
{	144	,	27	,	0	},
{	144	,	28	,	4	},
{	144	,	29	,	255	},
{	144	,	30	,	64	},
{	144	,	50	,	153	},
{	144	,	51	,	153	},
{	144	,	52	,	53	},
{	144	,	73	,	2	},
{	144	,	91	,	2	},
{	144	,	92	,	0	},
{	144	,	93	,	0	},
{	144	,	94	,	255	},
{	144	,	95	,	0	},
{	144	,	96	,	69	},
{	144	,	97	,	0	},
{	144	,	98	,	15	},
{	144	,	99	,	128	},
{	144	,	100	,	22	},
{	144	,	101	,	18	},
{	144	,	102	,	150	},
{	144	,	103	,	0	},
{	144	,	104	,	80	},
{	144	,	105	,	0	},
{	144	,	252	,	0	},


	{END_VAL, 0x00, 0x00},
};

static struct reg_val partron_init[] = 
{
{	100	,	3	,	0	},
{	100	,	45	,	1	},
{	100	,	41	,	152	},
{	100	,	3	,	0	},
{	100	,	4	,	2	},
{	100	,	5	,	3	},
{	100	,	65	,	2	},
{	100	,	66	,	11	},
{	100	,	66	,	11	},
{	100	,	64	,	60	},
{	100	,	64	,	60	},
{	100	,	64	,	44	},
{	100	,	64	,	44	},
{	100	,	6	,	6	},
{	100	,	7	,	113	},
{	100	,	8	,	2	},
{	100	,	9	,	237	},
{	100	,	10	,	2	},
{	100	,	11	,	237	},
{	100	,	12	,	0	},
{	100	,	13	,	5	},
{	100	,	14	,	0	},
{	100	,	15	,	5	},
{	100	,	16	,	5	},
{	100	,	17	,	4	},
{	100	,	18	,	2	},
{	100	,	19	,	212	},
{	100	,	20	,	0	},
{	100	,	21	,	23	},
{	100	,	22	,	2	},
{	100	,	23	,	232	},
{	100	,	3	,	0	},
{	100	,	51	,	2	},
{	100	,	52	,	1	},
{	100	,	3	,	1	},
{	100	,	177	,	48	},
{	100	,	3	,	0	},
{	100	,	56	,	95	},
{	100	,	3	,	1	},
{	100	,	30	,	14	},
{	100	,	38	,	3	},
{	100	,	3	,	4	},
{	100	,	6	,	152	},
{	100	,	3	,	1	},
{	100	,	164	,	136	},
{	100	,	165	,	136	},
{	100	,	166	,	136	},
{	100	,	167	,	0	},
{	100	,	168	,	0	},
{	100	,	169	,	8	},
{	100	,	3	,	4	},
{	100	,	6	,	184	},
{	100	,	3	,	4	},
{	100	,	117	,	40	},
{	100	,	118	,	40	},
{	100	,	119	,	120	},
{	100	,	120	,	120	},
{	100	,	121	,	72	},
{	100	,	122	,	72	},
{	100	,	123	,	184	},
{	100	,	124	,	184	},
{	100	,	125	,	1	},
{	100	,	126	,	0	},
{	100	,	127	,	2	},
{	100	,	128	,	7	},
{	100	,	3	,	4	},
{	100	,	115	,	8	},
{	100	,	116	,	4	},
{	100	,	3	,	4	},
{	100	,	81	,	16	},
{	100	,	82	,	224	},
{	100	,	83	,	2	},
{	100	,	84	,	2	},
{	100	,	85	,	64	},
{	100	,	86	,	192	},
{	100	,	87	,	4	},
{	100	,	88	,	110	},
{	100	,	89	,	69	},
{	100	,	3	,	4	},
{	100	,	90	,	35	},
{	100	,	91	,	75	},
{	100	,	92	,	100	},
{	100	,	93	,	170	},
{	100	,	94	,	35	},
{	100	,	95	,	40	},
{	100	,	96	,	75	},
{	100	,	97	,	115	},
{	100	,	98	,	60	},
{	100	,	99	,	135	},
{	100	,	100	,	45	},
{	100	,	101	,	45	},
{	100	,	3	,	4	},
{	100	,	110	,	58	},
{	100	,	111	,	80	},
{	100	,	112	,	96	},
{	100	,	3	,	3	},
{	100	,	22	,	58	},
{	100	,	23	,	80	},
{	100	,	24	,	96	},
{	100	,	3	,	4	},
{	100	,	5	,	100	},
{	100	,	59	,	160	},
{	100	,	60	,	112	},
{	100	,	61	,	112	},
{	100	,	62	,	80	},
{	100	,	63	,	36	},
{	100	,	64	,	75	},
{	100	,	3	,	4	},
{	100	,	18	,	2	},
{	100	,	19	,	232	},
{	100	,	20	,	2	},
{	100	,	21	,	232	},
{	100	,	22	,	2	},
{	100	,	23	,	232	},
{	100	,	26	,	12	},
{	100	,	27	,	0	},
{	100	,	28	,	93	},
{	100	,	29	,	0	},
{	100	,	30	,	0	},
{	100	,	31	,	93	},
{	100	,	32	,	0	},
{	100	,	3	,	4	},
{	100	,	72	,	8	},
{	100	,	73	,	8	},
{	100	,	74	,	8	},
{	100	,	3	,	4	},
{	100	,	44	,	102	},
{	100	,	3	,	4	},
{	100	,	65	,	4	},
{	100	,	66	,	8	},
{	100	,	67	,	16	},
{	100	,	68	,	32	},
{	100	,	46	,	5	},
{	100	,	3	,	0	},
{	100	,	79	,	8	},
{	100	,	3	,	0	},
{	100	,	89	,	0	},
{	100	,	90	,	186	},
{	100	,	91	,	0	},
{	100	,	3	,	2	},
{	100	,	51	,	55	},
{	100	,	52	,	144	},
{	100	,	53	,	135	},
{	100	,	54	,	142	},
{	100	,	55	,	59	},
{	100	,	56	,	140	},
{	100	,	57	,	130	},
{	100	,	58	,	152	},
{	100	,	59	,	58	},
{	100	,	3	,	3	},
{	100	,	12	,	37	},
{	100	,	13	,	136	},
{	100	,	14	,	0	},
{	100	,	15	,	37	},
{	100	,	3	,	2	},
{	100	,	61	,	0	},
{	100	,	62	,	15	},
{	100	,	63	,	38	},
{	100	,	64	,	55	},
{	100	,	65	,	67	},
{	100	,	66	,	84	},
{	100	,	67	,	98	},
{	100	,	68	,	119	},
{	100	,	69	,	136	},
{	100	,	70	,	164	},
{	100	,	71	,	187	},
{	100	,	72	,	207	},
{	100	,	73	,	224	},
{	100	,	74	,	241	},
{	100	,	75	,	255	},
{	100	,	3	,	2	},
{	100	,	91	,	0	},
{	100	,	92	,	5	},
{	100	,	93	,	20	},
{	100	,	94	,	37	},
{	100	,	95	,	52	},
{	100	,	96	,	75	},
{	100	,	97	,	91	},
{	100	,	98	,	115	},
{	100	,	99	,	134	},
{	100	,	100	,	163	},
{	100	,	101	,	186	},
{	100	,	102	,	206	},
{	100	,	103	,	224	},
{	100	,	104	,	240	},
{	100	,	105	,	255	},
{	100	,	3	,	2	},
{	100	,	106	,	0	},
{	100	,	107	,	11	},
{	100	,	108	,	19	},
{	100	,	109	,	26	},
{	100	,	110	,	32	},
{	100	,	111	,	43	},
{	100	,	112	,	54	},
{	100	,	113	,	73	},
{	100	,	114	,	90	},
{	100	,	115	,	123	},
{	100	,	116	,	152	},
{	100	,	117	,	180	},
{	100	,	118	,	206	},
{	100	,	119	,	231	},
{	100	,	120	,	255	},
{	100	,	3	,	2	},
{	100	,	141	,	48	},
{	100	,	3	,	2	},
{	100	,	47	,	32	},
{	100	,	48	,	64	},
{	100	,	49	,	64	},
{	100	,	3	,	4	},
{	100	,	9	,	0	},
{	100	,	3	,	3	},
{	100	,	38	,	0	},
{	100	,	39	,	16	},
{	100	,	40	,	64	},
{	100	,	3	,	3	},
{	100	,	119	,	0	},
{	100	,	120	,	8	},
{	100	,	121	,	16	},
{	100	,	3	,	3	},
{	100	,	161	,	32	},
{	100	,	162	,	18	},
{	100	,	163	,	4	},
{	100	,	3	,	2	},
{	100	,	149	,	0	},
{	100	,	150	,	0	},
{	100	,	151	,	16	},
{	100	,	3	,	4	},
{	100	,	5	,	100	},
{	100	,	59	,	144	},
{	100	,	60	,	120	},
{	100	,	61	,	112	},
{	100	,	62	,	120	},
{	100	,	63	,	36	},
{	100	,	64	,	75	},
{	100	,	3	,	2	},
{	100	,	179	,	0	},
{	100	,	180	,	220	},
{	100	,	181	,	4	},
{	100	,	182	,	56	},
{	100	,	183	,	0	},
{	100	,	184	,	5	},
{	100	,	185	,	2	},
{	100	,	186	,	212	},
{	100	,	3	,	2	},
{	100	,	187	,	1	},
{	100	,	188	,	226	},
{	100	,	189	,	3	},
{	100	,	190	,	14	},
{	100	,	191	,	0	},
{	100	,	192	,	245	},
{	100	,	193	,	1	},
{	100	,	194	,	228	},
{	100	,	3	,	2	},
{	100	,	61	,	0	},
{	100	,	62	,	39	},
{	100	,	63	,	54	},
{	100	,	64	,	64	},
{	100	,	65	,	73	},
{	100	,	66	,	88	},
{	100	,	67	,	100	},
{	100	,	68	,	120	},
{	100	,	69	,	137	},
{	100	,	70	,	164	},
{	100	,	71	,	187	},
{	100	,	72	,	207	},
{	100	,	73	,	224	},
{	100	,	74	,	241	},
{	100	,	75	,	255	},
{	100	,	3	,	2	},
{	100	,	91	,	0	},
{	100	,	92	,	11	},
{	100	,	93	,	19	},
{	100	,	94	,	26	},
{	100	,	95	,	32	},
{	100	,	96	,	43	},
{	100	,	97	,	54	},
{	100	,	98	,	73	},
{	100	,	99	,	90	},
{	100	,	100	,	123	},
{	100	,	101	,	152	},
{	100	,	102	,	180	},
{	100	,	103	,	206	},
{	100	,	104	,	231	},
{	100	,	105	,	255	},
{	100	,	3	,	3	},
{	100	,	119	,	0	},
{	100	,	120	,	0	},
{	100	,	121	,	0	},
{	100	,	3	,	3	},
{	100	,	161	,	32	},
{	100	,	162	,	32	},
{	100	,	163	,	32	},
{	100	,	3	,	4	},
{	100	,	18	,	2	},
{	100	,	19	,	232	},
{	100	,	20	,	2	},
{	100	,	21	,	232	},
{	100	,	22	,	2	},
{	100	,	23	,	232	},
{	100	,	27	,	0	},
{	100	,	28	,	116	},
{	100	,	29	,	64	},
{	100	,	30	,	0	},
{	100	,	31	,	116	},
{	100	,	32	,	64	},
//{	130	,	4	,	135	},

	{END_VAL, 0x00, 0x00},
};

static struct reg_val Reverse_Channel_Setup_1[] =
{
	{0x90, 0x3F, 0x4F},
	{DELAY, 0x02, 0x00},

	{0x90, 0x3B, 0x1E},
	{DELAY, 0x02, 0x00},

	{0x80, 0x04, 0x43},
	{DELAY, 0x05, 0x00},

	{0x80, 0x08, 0x01},
	{0x80, 0x07, 0xAF},
	{DELAY, 0x02, 0x00},

	{0x90, 0x3B, 0x19},
	{DELAY, 0x02, 0x00},

	{END_VAL, 0x00, 0x00},
};

static struct reg_val MAX9286_Initial_Setup_2[] =
{
	{0x90, 0x15, 0x03},
	{0x90, 0x12, 0xF3},	//{0x90, 0x12, 0xF5},
	{0x90, 0x01, 0xA2},	//{0x90, 0x01, 0x01},
	{0x90, 0x00, 0xEF},
	{END_VAL, 0x00, 0x00},
};

static struct reg_val GMSL_Link_Setup_3[] =
{
	{0x90, 0x0A, 0xF1},	// Enable Link 0 Reverse Channel
	{0x80, 0x00, 0x82},	// Change serializer slave address
	{0x82, 0x07, 0x84},	// Enable DBL,  Set Edge Select 1=Rise / 0=Fall,  Enable HS/VS encoding
	{DELAY, 0x02, 0x00},
	{0x82, 0x09, 0x62},	// Unique Link 0 image sensor slave address
	{0x82, 0x0A, 0x60},	// Link 0 image sensor slave address
	{0x82, 0x0B, 0x8A},	// Serializer broadcast address
	{0x82, 0x0C, 0x82},	// Link 0 serializer address
	{0x90, 0x0A, 0xF2},	// Enable Link 1 Reverse Channel
	{0x80, 0x00, 0x84},	// Change serializer slave address
	{0x84, 0x07, 0x84},	// Enable DBL,  Set Edge Select 1=Rise / 0=Fall,  Enable HS/VS encoding
	{DELAY, 0x02, 0x00},
	{0x84, 0x09, 0x64},	// Unique Link 1 image sensor slave address
	{0x84, 0x0A, 0x60},	// Link 1 image sensor slave address
	{0x84, 0x0B, 0x8A},	// Serializer broadcast address
	{0x84, 0x0C, 0x84},	// Link 1 serializer address
	{0x90, 0x0A, 0xF4},	// Enable Link 2 Reverse Channel
	{0x80, 0x00, 0x86},	// Change serializer slave address
	{0x86, 0x07, 0x84},	// Enable DBL,  Set Edge Select 1=Rise / 0=Fall,  Enable HS/VS encoding
	{DELAY, 0x02, 0x00},
	{0x86, 0x09, 0x66},	// Unique Link 2 image sensor slave address
	{0x86, 0x0A, 0x60},	// Link 2 image sensor slave address
	{0x86, 0x0B, 0x8A},	// Serializer broadcast address
	{0x86, 0x0C, 0x86},	// Link 2 serializer address
	{0x90, 0x0A, 0xF8},	// Enable Link 3 Reverse Channel
	{0x80, 0x00, 0x88},	// Change serializer slave address
	{0x88, 0x07, 0x84},	// Enable DBL,  Set Edge Select 1=Rise / 0=Fall,  Enable HS/VS encoding
	{DELAY, 0x02, 0x00},
	{0x88, 0x09, 0x68},	// Unique Link 3 image sensor slave address
	{0x88, 0x0A, 0x60},	// Link 3 image sensor slave address
	{0x88, 0x0B, 0x8A},	// Serializer broadcast address
	{0x88, 0x0C, 0x88},	// Link 3 serializer address
	{0x90, 0x0A, 0xFF},	// Enable all I2C reverse channels

	{END_VAL, 0x00, 0x00},
};

static struct reg_val Image_Sensor_Initialization_4_1[] =
{
	{0x90, 0x34, 0x36},	// Disable auto acknowledge
				//Initialize image sensor
	{END_VAL, 0x00, 0x00},
};

static struct reg_val Image_Sensor_Initialization_4_2[] =
{

				//Initialize image sensor
	{0x90, 0x34, 0xB6},	// Enable auto ack -optional-
	//Read
	{READ, 0x82, 0x15},
	{READ, 0x84, 0x15},
	{READ, 0x86, 0x15},
	{READ, 0x88, 0x15},

	{END_VAL, 0x00, 0x00},
};

static struct reg_val Enable_GMSL_CSI2_5[] =
{
	//{0x8A, 0x06, 0xAX},	// Set all devices Preemphasis settings (if needed)
	//{DELAY, 0x05, 0x00},
	//{0x90, 0x32, 0xXX},	// Set Deserializer Equalizer settings (if needed)
	//{0x90, 0x33, 0xXX},	// Set Deserializer Equalizer settings (if needed)
	//{0x90, 0x1B, 0x0F},	// Enable EQs (if needed)
	//{DELAY, 0x05, 0x00},
	//{0x8A, 0x07, 0xXX},	// Set EDC, for all devices (if needed)
	//{DELAY, 0x02, 0x00},
	//{0x90, 0x0C, 0xXX},	// Set EDC, for all devices (if needed)
	//{DELAY, 0x05, 0x00},
	{0x8A, 0x04, 0x83},	// Enable all serial links
	{DELAY, 0x05, 0x00},
	//Read
	{READ, 0x90, 0x31},	// Poll frame synchronization bit

	{0x90, 0x15, 0x0B},	// Enable CSI-2 output

	{END_VAL, 0x00, 0x00},
};

static struct reg_val Verification_6[] =
{
	{READ, 0x90, 0x4D},	// GMSL link 0 pixel count
	{READ, 0x90, 0x4E},

	{READ, 0x90, 0x51},	// GMSL link 1 pixel count
	{READ, 0x90, 0x52},

	{READ, 0x90, 0x55},	//GMSL link 2 pixel count
	{READ, 0x90, 0x56},

	{READ, 0x90, 0x59},	// GMSL link 3 pixel count
	{READ, 0x90, 0x5A},

	{READ, 0x90, 0x5B},	// Calculated VSYNC period base on master link in terms of PCLK
	{READ, 0x90, 0x5C},
	{READ, 0x90, 0x5D},

	{READ, 0x90, 0x5E},	//Frame sync error counter

	{END_VAL, 0x00, 0x00},
};


#define MAX9268_ADDR	0x6a
#define MAX9271_ADDR	0x40

#define MAX9271_ADDR0	0x42
#define MAX9271_ADDR1	0x44
#define MAX9271_ADDR2	0x46
#define MAX9271_ADDR3	0x48

static struct reg_val maxim_init[] =
{
	// #reverse controll channel setup
	{MAX9268_ADDR, 0x3F, 0x4F}, //  #DESER Enable Custom Reverse Channel & First Pulse Length
	{DELAY, 2, 0},

	{MAX9268_ADDR, 0x3B, 0x1E}, //  #DESER Reverse Channel Amplitude to mid level and transition time
	{DELAY, 2, 0},

	{MAX9271_ADDR, 0x04, 0x43}, //  #SER Enable Configuration Link
	{DELAY, 5, 0},

	{MAX9271_ADDR, 0x97, 0x5F}, //  #SER dont know what this is
	{DELAY, 2, 0},

	{MAX9271_ADDR, 0x08, 0x01}, //  #SER reverse channel input gain and thresholds
	{DELAY, 2, 0},

	{MAX9268_ADDR, 0x3B, 0x19}, //  #DESER Reverse Channel Amplitude level
	{DELAY, 2, 0},

	{MAX9268_ADDR, 0x15, 0x03}, //  #DESER Disable CSI output 
	{MAX9268_ADDR, 0x12, 0xF7}, //  #DESER DBL=1, CSI_DBL=1 , double input mode, sets 4 CSI lanes + RAW12
	{MAX9268_ADDR, 0x01, 0xC0}, //  #DESER frame sync off, manual, set to internal framesync. 
	{MAX9268_ADDR, 0x00, 0xEF}, //  #DESER enables all input links, Autodetect CSI clocksource
	{MAX9268_ADDR, 0x63, 0x00}, //  #DES disable overlap window
	{MAX9268_ADDR, 0x64, 0x00}, //  #DES disable overlap window
	{MAX9268_ADDR, 0x0c, 0x91}, //  #DESER Enable VS/HS encoding , use HS as line valid source, D14/D15 as H/Vsync
	{DELAY, 2, 0},

	//#/*********************************************************************/
	//#/*                           Setup Link 0                             */
	//#/*********************************************************************/
	{MAX9268_ADDR, 0x0A, 0xF1}, //  #Enable Link 0 Reverse Channel
	{MAX9271_ADDR, 0x00, MAX9271_ADDR0}, //  #Change serializer I2C Slave Address
	{MAX9271_ADDR0, 0x09, 0x12}, //  #Unique Link 0 Image Sensor I2C Slave Address
	{MAX9271_ADDR0, 0x0A, 0x10}, //  #Link 0 Image Sensor Address
	{MAX9271_ADDR0, 0x0B, 0x4A}, //  #Serializer Broadcast Address
	{MAX9271_ADDR0, 0x0C, MAX9271_ADDR0}, //  #Link 0 Serializer Address
	{MAX9271_ADDR0, 0x0E, 0x22}, //  #GPIO1_EN, GPIO5_EN -> Sensor OUTPUT EN and RST
	{MAX9271_ADDR0, 0x0F, 0x3F}, //  #GPIO5 high, GPIO4 high, GPO=high (trigger) ,GPIO1,2,3=high
	{DELAY, 2, 0},
#if 0
//#setup sensor for link 0 
	{0x12, 0x30, 0x0055}, //  w
	{0x12, 0x30, 0x0854}, //  w
	{0x12, 0x30, 0x004D}, //  w
	{0x12, 0x30, 0x204C}, //  w
	{0x12, 0x30, 0x104B}, //  w
	{0x12, 0x30, 0x024A}, //  w
	{DELAY, 200, 0},
	{0x12, 0x30, 0xC81F}, //  w
	{0x12, 0x30, 0x001E}, //  w
	{0x12, 0x3E, 0x03DB}, //  w
	{0x12, 0x3E, 0x0FDA}, //  w
	{0x12, 0x3E, 0x05DF}, //  w
	{0x12, 0x3E, 0xC0DE}, //  w
	{0x12, 0x3E, 0xDFD9}, //  w
	{0x12, 0x3E, 0x09D8}, //  w
	{0x12, 0x3E, 0x6BE3}, //  w
	{0x12, 0x3E, 0xA4E2}, //  w
	{0x12, 0x3E, 0x7DE1}, //  w
	{0x12, 0x3E, 0x06E0}, //  w
	{0x12, 0x3E, 0x70DD}, //  w
	{0x12, 0x3E, 0x00DC}, //  w
	{0x12, 0x30, 0x0445}, //  w
	{0x12, 0x30, 0x0444}, //  w
	{0x12, 0x3E, 0x03E7}, //  w
	{0x12, 0x3E, 0x83E6}, //  w
	{0x12, 0x3E, 0x08E5}, //  w
	{0x12, 0x3E, 0xD2E4}, //  w
	{0x12, 0x3E, 0xBBD7}, //  w
	{0x12, 0x3E, 0x00D6}, //  w
	{0x12, 0x30, 0x72E5}, //  w
	{0x12, 0x30, 0x63E4}, //  w
	{0x12, 0x30, 0x53E3}, //  w
	{0x12, 0x30, 0x72E2}, //  w
	{0x12, 0x30, 0x70E1}, //  w
	{0x12, 0x30, 0x54E0}, //  w
	{0x12, 0x30, 0xCCE7}, //  w
	{0x12, 0x30, 0xC4E6}, //  w
	{0x12, 0x30, 0x50E9}, //  w
	{0x12, 0x30, 0x80E8}, //  w
	{DELAY, 10, 0},
	{0x12, 0x30, 0x2013}, //  w
	{0x12, 0x30, 0x0012}, //  w
	{0x12, 0x30, 0x0203}, //  w
	{0x12, 0x30, 0x0002}, //  w
	{DELAY, 10, 0},
	{0x12, 0x30, 0x0005}, //  w
	{0x12, 0x30, 0x0004}, //  w
	{DELAY, 10, 0},
	{0x12, 0x30, 0xC107}, //  w
	{0x12, 0x30, 0x0306}, //  w
//#VGA mode
//#	{0x12, 0x30, 0xe107}, //  w
//#	{0x12, 0x30, 0x0106}, //  w
	{DELAY, 100, 0},
	{0x12, 0x30, 0xFF09}, //  w
	{0x12, 0x30, 0x0408}, //  w
//#VGA mode
//#	{0x12, 0x30, 0x7F09}, //  w
//#	{0x12, 0x30, 0x0208}, //  w
//#VGA modeonly
//#	{0x12, 0x30, 0xFE0B}, //  w
//#	{0x12, 0x30, 0x010A}, //  w
	{DELAY, 10, 0},
	{0x12, 0x30, 0x30B1}, //  w
	{0x12, 0x30, 0x13B0}, //  w
//#45 fps
	{0x12, 0x30, 0x720D}, //  w
	{0x12, 0x30, 0x060C}, //  w
//# framerate to 30fps
//#	{0x12, 0x30, 0xAB0D}, //  w
//#	{0x12, 0x30, 0x090C}, //  w
//#vGA mode only
//#	{0x12, 0x30, 0xF40D}, //  w
//#	{0x12, 0x30, 0x120C}, //  w
	{0x12, 0x30, 0x022D}, //  w
	{0x12, 0x30, 0x002C}, //  w
	{0x12, 0x30, 0x042B}, //  w
	{0x12, 0x30, 0x002A}, //  w
	{0x12, 0x30, 0x8265}, //  w
	{0x12, 0x30, 0x1964}, //  w
	{0x12, 0x31, 0x0001}, //  w
	{0x12, 0x31, 0x0000}, //  w
	{0x12, 0x30, 0x106F}, //  w
	{0x12, 0x30, 0x966E}, //  w
	{DELAY, 10, 0},
	{0x12, 0x30, 0xDC1B}, //  w
	{0x12, 0x30, 0x101A}, //  w
//#----------------------------
#endif

	//#/*********************************************************************/
	//#/*                          Setup Link 1                             */
	//#/*********************************************************************/
	{MAX9268_ADDR, 0x0A, 0xF2}, //  #Enable Link 0 Reverse Channel
	{MAX9271_ADDR, 0x00, MAX9271_ADDR1}, //  #Change serializer I2C Slave Address
	{MAX9271_ADDR1, 0x09, 0x14}, //  #Unique Link 0 Image Sensor I2C Slave Address
	{MAX9271_ADDR1, 0x0A, 0x10}, //  #Link 0 Image Sensor Address
	{MAX9271_ADDR1, 0x0B, 0x4A}, //  #Serializer Broadcast Address
	{MAX9271_ADDR1, 0x0C, MAX9271_ADDR1}, //  #Link 0 Serializer Address
	{MAX9271_ADDR1, 0x0E, 0x22}, //  #GPIO1_EN, GPIO5_EN -> Sensor OUTPUT EN and RST
	{MAX9271_ADDR1, 0x0F, 0x3F}, //  #GPIO5 high, GPIO4 high, GPO=high (trigger) ,GPIO1,2,3=high
	{DELAY, 2, 0},
#if 0
//#setup sensor for link 0 
	{0x14, 0x30, 0x0055}, //  w
	{0x14, 0x30, 0x0854}, //  w
	{0x14, 0x30, 0x004D}, //  w
	{0x14, 0x30, 0x204C}, //  w
	{0x14, 0x30, 0x104B}, //  w
	{0x14, 0x30, 0x024A}, //  w
	{DELAY, 200, 0},
	{0x14, 0x30, 0xC81F}, //  w
	{0x14, 0x30, 0x001E}, //  w
	{0x14, 0x3E, 0x03DB}, //  w
	{0x14, 0x3E, 0x0FDA}, //  w
	{0x14, 0x3E, 0x05DF}, //  w
	{0x14, 0x3E, 0xC0DE}, //  w
	{0x14, 0x3E, 0xDFD9}, //  w
	{0x14, 0x3E, 0x09D8}, //  w
	{0x14, 0x3E, 0x6BE3}, //  w
	{0x14, 0x3E, 0xA4E2}, //  w
	{0x14, 0x3E, 0x7DE1}, //  w
	{0x14, 0x3E, 0x06E0}, //  w
	{0x14, 0x3E, 0x70DD}, //  w
	{0x14, 0x3E, 0x00DC}, //  w
	{0x14, 0x30, 0x0445}, //  w
	{0x14, 0x30, 0x0444}, //  w
	{0x14, 0x3E, 0x03E7}, //  w
	{0x14, 0x3E, 0x83E6}, //  w
	{0x14, 0x3E, 0x08E5}, //  w
	{0x14, 0x3E, 0xD2E4}, //  w
	{0x14, 0x3E, 0xBBD7}, //  w
	{0x14, 0x3E, 0x00D6}, //  w
	{0x14, 0x30, 0x72E5}, //  w
	{0x14, 0x30, 0x63E4}, //  w
	{0x14, 0x30, 0x53E3}, //  w
	{0x14, 0x30, 0x72E2}, //  w
	{0x14, 0x30, 0x70E1}, //  w
	{0x14, 0x30, 0x54E0}, //  w
	{0x14, 0x30, 0xCCE7}, //  w
	{0x14, 0x30, 0xC4E6}, //  w
	{0x14, 0x30, 0x50E9}, //  w
	{0x14, 0x30, 0x80E8}, //  w
	{DELAY, 10, 0},
	{0x14, 0x30, 0x2013}, //  w
	{0x14, 0x30, 0x0012}, //  w
	{0x14, 0x30, 0x0203}, //  w
	{0x14, 0x30, 0x0002}, //  w
	{DELAY, 10, 0},
	{0x14, 0x30, 0x0005}, //  w
	{0x14, 0x30, 0x0004}, //  w
	{DELAY, 10, 0},
	{0x14, 0x30, 0xC107}, //  w
	{0x14, 0x30, 0x0306}, //  w
//#VGA mode
//#	{0x14, 0x30, 0xe107}, //  w
//#	{0x14, 0x30, 0x0106}, //  w
	{DELAY, 100, 0},
	{0x14, 0x30, 0xFF09}, //  w
	{0x14, 0x30, 0x0408}, //  w
//#VGA mode
//#	{0x14, 0x30, 0x7F09}, //  w
//#	{0x14, 0x30, 0x0208}, //  w
//#VGA modeonly
//#	{0x14, 0x30, 0xFE0B}, //  w
//#	{0x14, 0x30, 0x010A}, //  w
	{DELAY, 10, 0},
	{0x14, 0x30, 0x30B1}, //  w
	{0x14, 0x30, 0x13B0}, //  w
//#45 fps
	{0x14, 0x30, 0x720D}, //  w
	{0x14, 0x30, 0x060C}, //  w
//# framerate to 30fps
//#	{0x14, 0x30, 0xAB0D}, //  w
//#	{0x14, 0x30, 0x090C}, //  w
//#vGA mode only
//#	{0x14, 0x30, 0xF40D}, //  w
//#	{0x14, 0x30, 0x120C}, //  w
	{0x14, 0x30, 0x022D}, //  w
	{0x14, 0x30, 0x002C}, //  w
	{0x14, 0x30, 0x042B}, //  w
	{0x14, 0x30, 0x002A}, //  w
	{0x14, 0x30, 0x8265}, //  w
	{0x14, 0x30, 0x1964}, //  w
	{0x14, 0x31, 0x0001}, //  w
	{0x14, 0x31, 0x0000}, //  w
	{0x14, 0x30, 0x106F}, //  w
	{0x14, 0x30, 0x966E}, //  w
	{DELAY, 100, 0},
	{0x14, 0x30, 0xDC1B}, //  w
	{0x14, 0x30, 0x101A}, //  w
//#------------------------
#endif
	//#/*********************************************************************/
	//#/*                          Setup Link 2                             */
	//#/*********************************************************************/
	{MAX9268_ADDR, 0x0A, 0xF4}, //  #Enable Link 0 Reverse Channel
	{MAX9271_ADDR, 0x00, MAX9271_ADDR2}, //  #Change serializer I2C Slave Address
	{MAX9271_ADDR2, 0x09, 0x16}, //  #Unique Link 0 Image Sensor I2C Slave Address
	{MAX9271_ADDR2, 0x0A, 0x10}, //  #Link 0 Image Sensor Address
	{MAX9271_ADDR2, 0x0B, 0x4A}, //  #Serializer Broadcast Address
	{MAX9271_ADDR2, 0x0C, MAX9271_ADDR2}, //  #Link 0 Serializer Address
	{MAX9271_ADDR2, 0x0E, 0x22}, //  #GPIO1_EN, GPIO5_EN -> Sensor OUTPUT EN and RST
	{MAX9271_ADDR2, 0x0F, 0x3F}, //  #GPIO5 high, GPIO4 high, GPO=high (trigger) ,GPIO1,2,3=high
	{DELAY, 2, 0},
#if 0
//#setup sensor for link 2
	{0x16, 0x30, 0x0055}, //  w
	{0x16, 0x30, 0x0854}, //  w
	{0x16, 0x30, 0x004D}, //  w
	{0x16, 0x30, 0x204C}, //  w
	{0x16, 0x30, 0x104B}, //  w
	{0x16, 0x30, 0x024A}, //  w
	{DELAY, 200, 0},
	{0x16, 0x30, 0xC81F}, //  w
	{0x16, 0x30, 0x001E}, //  w
	{0x16, 0x3E, 0x03DB}, //  w
	{0x16, 0x3E, 0x0FDA}, //  w
	{0x16, 0x3E, 0x05DF}, //  w
	{0x16, 0x3E, 0xC0DE}, //  w
	{0x16, 0x3E, 0xDFD9}, //  w
	{0x16, 0x3E, 0x09D8}, //  w
	{0x16, 0x3E, 0x6BE3}, //  w
	{0x16, 0x3E, 0xA4E2}, //  w
	{0x16, 0x3E, 0x7DE1}, //  w
	{0x16, 0x3E, 0x06E0}, //  w
	{0x16, 0x3E, 0x70DD}, //  w
	{0x16, 0x3E, 0x00DC}, //  w
	{0x16, 0x30, 0x0445}, //  w
	{0x16, 0x30, 0x0444}, //  w
	{0x16, 0x3E, 0x03E7}, //  w
	{0x16, 0x3E, 0x83E6}, //  w
	{0x16, 0x3E, 0x08E5}, //  w
	{0x16, 0x3E, 0xD2E4}, //  w
	{0x16, 0x3E, 0xBBD7}, //  w
	{0x16, 0x3E, 0x00D6}, //  w
	{0x16, 0x30, 0x72E5}, //  w
	{0x16, 0x30, 0x63E4}, //  w
	{0x16, 0x30, 0x53E3}, //  w
	{0x16, 0x30, 0x72E2}, //  w
	{0x16, 0x30, 0x70E1}, //  w
	{0x16, 0x30, 0x54E0}, //  w
	{0x16, 0x30, 0xCCE7}, //  w
	{0x16, 0x30, 0xC4E6}, //  w
	{0x16, 0x30, 0x50E9}, //  w
	{0x16, 0x30, 0x80E8}, //  w
	{DELAY, 10, 0},
	{0x16, 0x30, 0x2013}, //  w
	{0x16, 0x30, 0x0012}, //  w
	{0x16, 0x30, 0x0203}, //  w
	{0x16, 0x30, 0x0002}, //  w
	{DELAY, 10, 0},
	{0x16, 0x30, 0x0005}, //  w
	{0x16, 0x30, 0x0004}, //  w
	{DELAY, 10, 0},
	{0x16, 0x30, 0xC107}, //  w
	{0x16, 0x30, 0x0306}, //  w
//#VGA mode
//#	{0x16, 0x30, 0xe107}, //  w
//#	{0x16, 0x30, 0x0106}, //  w
	{DELAY, 100, 0},
	{0x16, 0x30, 0xFF09}, //  w
	{0x16, 0x30, 0x0408}, //  w
//#VGA mode
//#	{0x16, 0x30, 0x7F09}, //  w
//#	{0x16, 0x30, 0x0208}, //  w
//#VGA modeonly
//#	{0x16, 0x30, 0xFE0B}, //  w
//#	{0x16, 0x30, 0x010A}, //  w
	{DELAY, 10, 0},
	{0x16, 0x30, 0x30B1}, //  w
	{0x16, 0x30, 0x13B0}, //  w
//#45 fps
	{0x16, 0x30, 0x720D}, //  w
	{0x16, 0x30, 0x060C}, //  w
//# framerate to 30fps
//#	{0x16, 0x30, 0xAB0D}, //  w
//#	{0x16, 0x30, 0x090C}, //  w
//#vGA mode only
//#	{0x16, 0x30, 0xF40D}, //  w
//#	{0x16, 0x30, 0x120C}, //  w
	{0x16, 0x30, 0x022D}, //  w
	{0x16, 0x30, 0x002C}, //  w
	{0x16, 0x30, 0x042B}, //  w
	{0x16, 0x30, 0x002A}, //  w
	{0x16, 0x30, 0x8265}, //  w
	{0x16, 0x30, 0x1964}, //  w
	{0x16, 0x31, 0x0001}, //  w
	{0x16, 0x31, 0x0000}, //  w
	{0x16, 0x30, 0x106F}, //  w
	{0x16, 0x30, 0x966E}, //  w
	{DELAY, 100, 0},
	{0x16, 0x30, 0xDC1B}, //  w
	{0x16, 0x30, 0x101A}, //  w
//#-------------------------
#endif

	//#/*********************************************************************/
	//#/*                          Setup Link 3                             */
	//#/*********************************************************************/
	{MAX9268_ADDR, 0x0A, 0xF8}, //  #Enable Link 0 Reverse Channel
	{MAX9271_ADDR, 0x00, MAX9271_ADDR3}, //  #Change serializer I2C Slave Address
	{MAX9271_ADDR3, 0x09, 0x18}, //  #Unique Link 0 Image Sensor I2C Slave Address
	{MAX9271_ADDR3, 0x0A, 0x10}, //  #Link 0 Image Sensor Address
	{MAX9271_ADDR3, 0x0B, 0x4A}, //  #Serializer Broadcast Address
	{MAX9271_ADDR3, 0x0C, MAX9271_ADDR3}, //  #Link 0 Serializer Address
	{MAX9271_ADDR3, 0x0E, 0x22}, //  #GPIO1_EN, GPIO5_EN -> Sensor OUTPUT EN and RST
	{MAX9271_ADDR3, 0x0F, 0x3F}, //  #GPIO5 high, GPIO4 high, GPO=high (trigger) ,GPIO1,2,3=high
#if 0
//#setup sensor f,or link 3
	{0x18, 0x30, 0x0055}, //  w
	{0x18, 0x30, 0x0854}, //  w
	{0x18, 0x30, 0x004D}, //  w
	{0x18, 0x30, 0x204C}, //  w
	{0x18, 0x30, 0x104B}, //  w
	{0x18, 0x30, 0x024A}, //  w
	{DELAY, 200, 0},
	{0x18, 0x30, 0xC81F}, //  w
	{0x18, 0x30, 0x001E}, //  w
	{0x18, 0x3E, 0x03DB}, //  w
	{0x18, 0x3E, 0x0FDA}, //  w
	{0x18, 0x3E, 0x05DF}, //  w
	{0x18, 0x3E, 0xC0DE}, //  w
	{0x18, 0x3E, 0xDFD9}, //  w
	{0x18, 0x3E, 0x09D8}, //  w
	{0x18, 0x3E, 0x6BE3}, //  w
	{0x18, 0x3E, 0xA4E2}, //  w
	{0x18, 0x3E, 0x7DE1}, //  w
	{0x18, 0x3E, 0x06E0}, //  w
	{0x18, 0x3E, 0x70DD}, //  w
	{0x18, 0x3E, 0x00DC}, //  w
	{0x18, 0x30, 0x0445}, //  w
	{0x18, 0x30, 0x0444}, //  w
	{0x18, 0x3E, 0x03E7}, //  w
	{0x18, 0x3E, 0x83E6}, //  w
	{0x18, 0x3E, 0x08E5}, //  w
	{0x18, 0x3E, 0xD2E4}, //  w
	{0x18, 0x3E, 0xBBD7}, //  w
	{0x18, 0x3E, 0x00D6}, //  w
	{0x18, 0x30, 0x72E5}, //  w
	{0x18, 0x30, 0x63E4}, //  w
	{0x18, 0x30, 0x53E3}, //  w
	{0x18, 0x30, 0x72E2}, //  w
	{0x18, 0x30, 0x70E1}, //  w
	{0x18, 0x30, 0x54E0}, //  w
	{0x18, 0x30, 0xCCE7}, //  w
	{0x18, 0x30, 0xC4E6}, //  w
	{0x18, 0x30, 0x50E9}, //  w
	{0x18, 0x30, 0x80E8}, //  w
	{DELAY, 10, 0},
	{0x18, 0x30, 0x2013}, //  w
	{0x18, 0x30, 0x0012}, //  w
	{0x18, 0x30, 0x0203}, //  w
	{0x18, 0x30, 0x0002}, //  w
	{DELAY, 10, 0},
	{0x18, 0x30, 0x0005}, //  w
	{0x18, 0x30, 0x0004}, //  w
	{DELAY, 10, 0},
	{0x18, 0x30, 0xC107}, //  w
	{0x18, 0x30, 0x0306}, //  w
//#VGA mode
//#	{0x18, 0x30, 0xe107}, //  w
//#	{0x18, 0x30, 0x0106}, //  w
	{DELAY, 100, 0},
	{0x18, 0x30, 0xFF09}, //  w
	{0x18, 0x30, 0x0408}, //  w
//#VGA mode
//#	{0x18, 0x30, 0x7F09}, //  w
//#	{0x18, 0x30, 0x0208}, //  w
//#VGA modeonly
//#	{0x18, 0x30, 0xFE0B}, //  w
//#	{0x18, 0x30, 0x010A}, //  w
	{DELAY, 10, 0},
	{0x18, 0x30, 0x30B1}, //  w
	{0x18, 0x30, 0x13B0}, //  w
//#45 fps
	{0x18, 0x30, 0x720D}, //  w
	{0x18, 0x30, 0x060C}, //  w
//# framerate to 30fps
//#	{0x18, 0x30, 0xAB0D}, //  w
//#	{0x18, 0x30, 0x090C}, //  w
//#vGA mode only
//#	{0x18, 0x30, 0xF40D}, //  w
//#	{0x18, 0x30, 0x120C}, //  w
	{0x18, 0x30, 0x022D}, //  w
	{0x18, 0x30, 0x002C}, //  w
	{0x18, 0x30, 0x042B}, //  w
	{0x18, 0x30, 0x002A}, //  w
	{0x18, 0x30, 0x8265}, //  w
	{0x18, 0x30, 0x1964}, //  w
	{0x18, 0x31, 0x0001}, //  w
	{0x18, 0x31, 0x0000}, //  w
	{0x18, 0x30, 0x106F}, //  w
	{0x18, 0x30, 0x966E}, //  w
	{DELAY, 100, 0},
	{0x18, 0x30, 0xDC1B}, //  w
	{0x18, 0x30, 0x101A}, //  w
//#-----------------------
#endif
	{MAX9268_ADDR, 0x0A, 0xFF}, //  #Enable all I2C reverse and forward channels
	{0x4A, 0x04, 0x83}, //  #Enable all serializers and disable configuration link
	{DELAY, 2, 0},
	{0x6d, 0x15, 0x0F}, //  #DESER Enable CSI output and flip the bit ordered to little endian.

	{END_VAL, 0x00, 0x00},
};



#endif

