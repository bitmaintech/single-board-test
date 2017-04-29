/*
 * Copyright 2016-2017 Fazio Bai <yang.bai@bitmain.com>
 * Copyright 2016-2017 Clement Duan <kai.duan@bitmain.com>
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <semaphore.h>
#include <sys/time.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/inotify.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/mman.h>
#include <time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <errno.h>

#include "zlib.h"

#include "driver-bitmain.h"

#define MAX_CHAR_NUM 1024

#define BITMAIN_TEST_PRINT_WORK 0
#define BITMAIN_READ_SIZE 12

#define CONFIG_FILE "/etc/config/Config.ini"
#define ID_FILE "/etc/config/Id.ini"
#define PIC_PROGRAM "/etc/config/hash_s8_app.txt"
#define DSPIC33EP16GS202_PIC_PROGRAM "/etc/config/dsPIC33EP16GS202_app.txt"
#define FORCE_FREQ_FILE	"/etc/config/forcefreq.txt"
#define LAST_FORCE_FREQ_FILE	"/etc/config/last_forcefreq.txt"

bool opt_bitmain_hwerror = false;

pthread_mutex_t reg_mutex = PTHREAD_MUTEX_INITIALIZER;

extern struct cgpu_info cgpu;
unsigned int *axi_fpga_addr = NULL;
extern struct reg_buf *reg_value_buf;

int fd;                                         // axi fpga
int fd_fpga_mem;                                // fpga memory
unsigned int *fpga_mem_addr = NULL;             // fpga memory address
unsigned int *nonce2_jobid_address = NULL;      // the value should be filled in NONCE2_AND_JOBID_STORE_ADDRESS

int fan_num=0;
unsigned char	fan_exist[BITMAIN_MAX_FAN_NUM]={0};
unsigned int	fan_speed_value[BITMAIN_MAX_FAN_NUM]={0};

#ifdef T9_18
unsigned char T9_18_chain_voltage[BITMAIN_MAX_CHAIN_NUM]={0};	// used to record current voltage for T9+, if same as we want to set, then just ignore set voltage cmd
																// because T9+ work badlly when set voltage very ofen.
#endif

extern int searchStatus;
extern char search_failed_info[64];

extern int asic_nonce_num[BITMAIN_MAX_CHAIN_NUM][256];
extern int asic_core_nonce_num[BITMAIN_MAX_CHAIN_NUM][256][256];   // 1st: which asic, 2nd: which core
extern int asic_core_enabled_flag[BITMAIN_MAX_CHAIN_NUM][256][256];	// [chainindex][chipindex][coreindex]=0  disabled, 1:enabled
extern int chain_badcore_num[BITMAIN_MAX_CHAIN_NUM][256];


extern int last_opencore_result[BITMAIN_MAX_CHAIN_NUM][256];
extern int last_result[BITMAIN_MAX_CHAIN_NUM][256];
extern int last_freq[BITMAIN_MAX_CHAIN_NUM][256];
extern unsigned char pic_freq[BITMAIN_MAX_CHAIN_NUM][128];
extern bool testDone[BITMAIN_MAX_CHAIN_NUM];
extern int repeated_nonce_num[BITMAIN_MAX_CHAIN_NUM];
extern uint32_t repeated_nonce_id[BITMAIN_MAX_CHAIN_NUM][256];

extern int last_nonce_num[BITMAIN_MAX_CHAIN_NUM];
extern int valid_nonce_num[BITMAIN_MAX_CHAIN_NUM];
extern int err_nonce_num[BITMAIN_MAX_CHAIN_NUM];

extern int chain_DataCount[BITMAIN_MAX_CHAIN_NUM];
extern int temp_chip_index[BITMAIN_MAX_CHAIN_NUM];

extern struct configuration Conf;  //store information that read from Config.ini
extern struct _CONFIG conf;        //store the information that handled from Config.ini
extern unsigned int send_work_num[BITMAIN_MAX_CHAIN_NUM];
unsigned int gName_len = 0;
unsigned char hash_board_id[12]= {0,1,2,3,4,5,6,7,8,9,10,11};
unsigned int hash_board_id_recorder = 0;
extern unsigned char time_data[6];
extern bool gIsReadTemp;
extern bool gReadingTemp;
extern int total_valid_nonce_num;
extern signed char chip_temp_offset[BITMAIN_MAX_CHAIN_NUM];

extern pthread_mutex_t iic_mutex;
extern pthread_mutex_t fpga_mutex;

//ASIC register address
unsigned int CHIP_ADDRESS           = 0x0;
unsigned int GOLDEN_NONCE_COUNTER   = 0x0;
unsigned int PLL_PARAMETER          = 0x0;
unsigned int START_NONCE_OFFSET     = 0x0;
unsigned int HASH_COUNTING_NUMBER   = 0x0;
unsigned int TICKET_MASK            = 0x0;
unsigned int MISC_CONTROL           = 0x0;
unsigned int HASH_RATE              = 0x0;
unsigned int GENERAL_I2C_COMMAND    = 0x0;
unsigned int SECURITY_I2C_COMMAND   = 0x0;
unsigned int SIGNATURE_INPUT        = 0x0;
unsigned int SIGNATURE_NONCE        = 0x0;
unsigned int SIGNATURE_ID           = 0x0;
unsigned int SECURITY_CONTROL_AND_STATUS    = 0x0;
unsigned int JOB_INFORMATION        = 0x0;

const unsigned int read_loop = 2;   // read ECT218 TIMEs
const unsigned int crc_error_loop = 50; // loop times when register value's crc is error
bool gIsSetTime = 1;
extern bool gStartTest;
extern void writeLogFile(char *logstr);

int GetForceFreq(int *pForceFlag)
{
	char cmd[256];
	int freqForce,lastfreqForce;
	char freqstr[32];
	FILE *fd;
	fd=fopen(FORCE_FREQ_FILE,"rb");
	if(fd)
	{
		memset(freqstr,'\0',32);
		fread(freqstr,1,32,fd);
		freqForce=atoi(freqstr);

		fclose(fd);

		if(strcmp(freqstr+16,"wsdk")!=0)	// special file content, as password to search freq
		{
			*pForceFlag=0;
			lastfreqForce=-1;
			return lastfreqForce;
		}

		fd=fopen(LAST_FORCE_FREQ_FILE,"rb");
		if(fd)
		{
			memset(freqstr,'\0',32);
			fread(freqstr,1,32,fd);
			lastfreqForce=atoi(freqstr);
			fclose(fd);
		}
		else
		{
			lastfreqForce=-1;
		}

		if(lastfreqForce == freqForce)
		{
			sprintf(cmd,"rm %s",FORCE_FREQ_FILE);
			system(cmd);
			
			*pForceFlag=0;
			return lastfreqForce;
		}
		else
		{
			printf("force freq=%d\n",freqForce);
			*pForceFlag=1;
			return freqForce;
		}
	}
	else
	{
		*pForceFlag=0;

		fd=fopen(LAST_FORCE_FREQ_FILE,"rb");
		if(fd)
		{
			memset(freqstr,'\0',32);
			fread(freqstr,1,32,fd);
			lastfreqForce=atoi(freqstr);
			fclose(fd);
		}
		else
		{
			lastfreqForce=-1;
		}

		return lastfreqForce;
	}
}

void ClearForceFreq()
{
	char cmd[256];
	//sprintf(cmd,"mv %s %s",FORCE_FREQ_FILE,LAST_FORCE_FREQ_FILE);
	sprintf(cmd,"rm %s -f",FORCE_FREQ_FILE);
	system(cmd);
}

bool last_all_pass(int chainIndex)
{
    int i = 0;
    for(i=0; i<ASIC_NUM; i++)
        if (!last_result[chainIndex][i])
            return false;
    return true;
}

bool last_opencore_pass(int chainIndex, int check_core_num)
{
    int i = 0;
    for(i=0; i<ASIC_NUM; i++)
        if (last_opencore_result[chainIndex][i]!=check_core_num)
            return false;
    return true;
}

void set_result_all_pass(int chainIndex)
{
    int i = 0;
    for(i=0; i<ASIC_NUM; i++)
        last_result[chainIndex][i]=1;
}

const uint8_t chCRCHTalbe[] =                                 // CRC high byte table
{
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40
};
const uint8_t chCRCLTalbe[] =                                 // CRC low byte table
{
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
    0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
    0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
    0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
    0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
    0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
    0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
    0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB,
    0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
    0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
    0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
    0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
    0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
    0x41, 0x81, 0x80, 0x40
};

unsigned char CRC5(unsigned char *ptr, unsigned char len)
{
    unsigned char i, j, k;
    unsigned char crc = 0x1f;

    unsigned char crcin[5] = {1, 1, 1, 1, 1};
    unsigned char crcout[5] = {1, 1, 1, 1, 1};
    unsigned char din = 0;

    j = 0x80;
    k = 0;
    for (i = 0; i < len; i++)
    {
        if (*ptr & j)
        {
            din = 1;
        }
        else
        {
            din = 0;
        }
        crcout[0] = crcin[4] ^ din;
        crcout[1] = crcin[0];
        crcout[2] = crcin[1] ^ crcin[4] ^ din;
        crcout[3] = crcin[2];
        crcout[4] = crcin[3];

        j = j >> 1;
        k++;
        if (k == 8)
        {
            j = 0x80;
            k = 0;
            ptr++;
        }
        memcpy(crcin, crcout, 5);
    }
    crc = 0;
    if(crcin[4])
    {
        crc |= 0x10;
    }
    if(crcin[3])
    {
        crc |= 0x08;
    }
    if(crcin[2])
    {
        crc |= 0x04;
    }
    if(crcin[1])
    {
        crc |= 0x02;
    }
    if(crcin[0])
    {
        crc |= 0x01;
    }
    return crc;
}

/*
static uint16_t CRC16(const uint8_t* p_data, uint16_t w_len)
{
    uint8_t chCRCHi = 0xFF; // CRC high byte initialize
    uint8_t chCRCLo = 0xFF; // CRC low byte initialize
    uint16_t wIndex = 0;    // CRC cycling index
    while (w_len--)
    {
        wIndex = chCRCLo ^ *p_data++;
        chCRCLo = chCRCHi ^ chCRCHTalbe[wIndex];
        chCRCHi = chCRCLTalbe[wIndex];
    }
    return ((chCRCHi << 8) | chCRCLo);
}

static uint32_t num2bit(int num)
{
    switch(num)
    {
        case 0:
            return 0x80000000;
        case 1:
            return 0x40000000;
        case 2:
            return 0x20000000;
        case 3:
            return 0x10000000;
        case 4:
            return 0x08000000;
        case 5:
            return 0x04000000;
        case 6:
            return 0x02000000;
        case 7:
            return 0x01000000;
        case 8:
            return 0x00800000;
        case 9:
            return 0x00400000;
        case 10:
            return 0x00200000;
        case 11:
            return 0x00100000;
        case 12:
            return 0x00080000;
        case 13:
            return 0x00040000;
        case 14:
            return 0x00020000;
        case 15:
            return 0x00010000;
        case 16:
            return 0x00008000;
        case 17:
            return 0x00004000;
        case 18:
            return 0x00002000;
        case 19:
            return 0x00001000;
        case 20:
            return 0x00000800;
        case 21:
            return 0x00000400;
        case 22:
            return 0x00000200;
        case 23:
            return 0x00000100;
        case 24:
            return 0x00000080;
        case 25:
            return 0x00000040;
        case 26:
            return 0x00000020;
        case 27:
            return 0x00000010;
        case 28:
            return 0x00000008;
        case 29:
            return 0x00000004;
        case 30:
            return 0x00000002;
        case 31:
            return 0x00000001;
        default:
            return 0x00000000;
    }
}
*/

void rev(unsigned char *s, unsigned char  l)
{
    unsigned char  i, j;
    unsigned char t;

    for (i = 0, j = l - 1; i < j; i++, j--)
    {
        t = s[i];
        s[i] = s[j];
        s[j] = t;
    }
}

void reset_nonce_arg()
{
    memset(asic_nonce_num, 0, sizeof(asic_nonce_num));
    memset(asic_core_nonce_num, 0, sizeof(asic_core_nonce_num));
    memset(repeated_nonce_id, 0xff, sizeof(repeated_nonce_id));
	memset(err_nonce_num, 0, sizeof(err_nonce_num));
	memset(last_nonce_num, 0, sizeof(last_nonce_num));
	memset(repeated_nonce_num, 0, sizeof(repeated_nonce_num));
	memset(valid_nonce_num, 0, sizeof(valid_nonce_num));
	memset(send_work_num, 0, sizeof(send_work_num));

	total_valid_nonce_num=0;
}


void reset_global_arg()
{
    int i=0;
    memset(asic_nonce_num, 0, sizeof(asic_nonce_num));
    memset(asic_core_nonce_num, 0, sizeof(asic_core_nonce_num));
    memset(repeated_nonce_id, 0xff, sizeof(repeated_nonce_id));
	memset(err_nonce_num, 0, sizeof(err_nonce_num));
	memset(last_nonce_num, 0, sizeof(last_nonce_num));
	memset(repeated_nonce_num, 0, sizeof(repeated_nonce_num));
	memset(valid_nonce_num, 0, sizeof(valid_nonce_num));
	memset(send_work_num, 0, sizeof(send_work_num));

	total_valid_nonce_num=0;
	
    gName_len = 0;
	
    for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
    {
        cgpu.chain_asic_num[i] = 0;
    }
	
    cgpu.baud = 128;
    cgpu.T1_offset_value = conf.DefaultTempOffset;
    cgpu.T2_offset_value = conf.DefaultTempOffset;
    cgpu.T3_offset_value = conf.DefaultTempOffset;
    cgpu.T4_offset_value = conf.DefaultTempOffset;
}

unsigned char c2hex(unsigned char value)
{
    unsigned char ret = 0xFF;
    if (value >= 0x30 && value <= 0x39)
        ret = value & 0x0F;
    else if (value == 'a' || value == 'A')
        ret = 0x0A;
    else if (value == 'b' || value == 'B')
        ret = 0x0B;
    else if (value == 'c' || value == 'C')
        ret = 0x0C;
    else if (value == 'd' || value == 'D')
        ret = 0x0D;
    else if (value == 'e' || value == 'E')
        ret = 0x0E;
    else if (value == 'f' || value == 'F')
        ret = 0x0F;
    else
        applog(LOG_ERR, "input value error: %c\n", value);
    return ret;
}

unsigned char twoc2hex(unsigned char high, unsigned char low)
{
    unsigned char ret = 0x00;
    high = c2hex(high);
    low = c2hex(low);
    ret = high << 4 & 0xF0;
    ret = ret ^ low;
    return ret;
}

int s2hex(unsigned char * dst, const unsigned char * src, int inlen)
{
    int i = 0, len = 0, p = 0;
    unsigned char high, low;

    if(src == NULL || inlen <= 0 || dst == NULL)
    {
        applog(LOG_DEBUG, "s2hex para error dst(%p), src(%p), inlen(%d)\n", dst, src, inlen);
        return -1;
    }
    len = inlen/2;
    p = inlen%2;
    for(i = 0; i < len; i++)
    {
        high = src[i*2];
        low = src[i*2+1];
        dst[i] = twoc2hex(high, low);
    }
    if(p)
    {
        high = src[i*2];
        dst[i] = twoc2hex(high, 0);
    }
    return len+p;
}

void freeWorks()
{
	int i;
	for(i=0;i<MAX_ASIC_NUM;i++)
	{
		if(cgpu.works[i]!=NULL)
			free(cgpu.works[i]);
		cgpu.works[i]=NULL;
	}
}

static int get_work(int id, int count)
{
    struct work * new_work;
    int subid = 0;
	unsigned long DataLen=MAX_WORK*48;	// midstate + data + nonce = 48 bytes
	unsigned char *workData;
	unsigned char *zipData;
	unsigned long zipLen;
	
	workData=(unsigned char *)malloc(DataLen);

	fseek(cgpu.fps[id],0,SEEK_END);
	zipLen = ftell(cgpu.fps[id]);
	fseek(cgpu.fps[id],0,SEEK_SET);


	zipData=(unsigned char *)malloc(zipLen);
	zipLen=fread(zipData,1,zipLen,cgpu.fps[id]);

	uncompress(workData,&DataLen,zipData,zipLen);
	free(zipData);

    cgpu.works[id] = (struct work *)malloc(count * sizeof(struct work));
    if(NULL == cgpu.works[id])
    {
        applog(LOG_ERR, "malloc struct work err\n");
        return 0;
    }

    while(subid*48<DataLen)
    {
        if(subid >= count)
            break;
		
        new_work = cgpu.works[id] + subid;

        memcpy((uint8_t *)(&new_work->nonce) ,workData+subid*48+44, 4);
        new_work->nonce = htonl(new_work->nonce);

        memcpy(new_work->midstate ,workData+subid*48, 32);
        memcpy(new_work->data ,workData+subid*48+32, 12);
		
        new_work->id = subid;
        subid++;
    }
	free(workData);
    return subid;
}

/*
static void freq_protocol(void)
{
    switch (Conf.Freq)
    {
        case 100:
            conf.freq = 0x0783;
            break;
        case 125:
            conf.freq = 0x0983;
            break;
        case 150:
            conf.freq = 0x0b83;
            break;
        case 175:
            conf.freq = 0x0d83;
            break;
        case 193:
            conf.freq = 0x0f03;
            break;
        case 196:
            conf.freq = 0x1f07;
            break;
        case 200:
            conf.freq = 0x0782;
            break;
        case 206:
            conf.freq = 0x1006;
            break;
        case 212:
            conf.freq = 0x0802;
            break;
        case 218:
            conf.freq = 0x1106;
            break;
        case 225:
            conf.freq = 0x0882;
            break;
        case 237:
            conf.freq = 0x0902;
            break;
        case 243:
            conf.freq = 0x1306;
            break;
        case 250:
            conf.freq = 0x0982;
            break;
        case 275:
            conf.freq = 0x0a82;
            break;
        case 300:
            conf.freq = 0x0b82;
            break;
        case 306:
            conf.freq = 0x1806;
            break;
        case 312:
            conf.freq = 0x0c02;
            break;
        case 318:
            conf.freq = 0x1906;
            break;
        case 325:
            conf.freq = 0x0c82;
            break;
        case 331://331.25
            conf.freq = 0x1a06;
            break;
        case 337://337.5
            conf.freq = 0x0d02;
            break;
        case 343://343.75
            conf.freq = 0x1b06;
            break;
        case 350:
            conf.freq = 0x0d82;
            break;
        case 356://356.25
            conf.freq = 0x1c06;
            break;
        case 362://362.5
            conf.freq = 0x0e02;
            break;
        case 368://368.75
            conf.freq = 0x1d06;
            break;
        case 375:
            conf.freq = 0x0e82;
            break;
        case 381:
            conf.freq = 0x1e06;
            break;
        case 387:
            conf.freq = 0x0f02;
            break;
        case 393:
            conf.freq = 0x1f06;
            break;
        case 400:
            conf.freq = 0x0781;
            break;
        case 425:
            conf.freq = 0x0801;
            break;
        case 450:
            conf.freq = 0x0881;
            break;
        case 475:
            conf.freq = 0x0901;
            break;
        case 500:
            conf.freq = 0x0981;
            break;
        default:
            Conf.Freq = 200;
            conf.freq = 0x0782;
            break;
    }
}
*/

unsigned int get_crc_count()
{
	unsigned int ret;
	ret= *((unsigned int *)(axi_fpga_addr + CRC_ERROR_CNT_ADDR));
	return (ret&0xffff);
}

void reset_crc_count()
{
	*((unsigned int *)(axi_fpga_addr + CRC_ERROR_CNT_ADDR)) = 0;
}

void read_id()
{
    unsigned int i=0, rand_num=0;
    unsigned int fpga_chip_id_l = 0, fpga_chip_id_h = 0;
    struct timeval ts;

    printf("\n--- %s\n", __FUNCTION__);

    fpga_chip_id_l = *((unsigned int *)(axi_fpga_addr + FPGA_CHIP_ID_ADDR));
    fpga_chip_id_h = *((unsigned int *)(axi_fpga_addr + FPGA_CHIP_ID_ADDR + 1));
    printf("\n\n--- FPGA chip id:    low: 0x%08x, high: 0x%08x\n", fpga_chip_id_l, fpga_chip_id_h);

    hash_board_id[0] = (unsigned char)((fpga_chip_id_l >> 24) & 0x000000ff);
    hash_board_id[1] = (unsigned char)((fpga_chip_id_l >> 16) & 0x000000ff);
    hash_board_id[2] = (unsigned char)((fpga_chip_id_l >> 8) & 0x000000ff);
    hash_board_id[3] = (unsigned char)((fpga_chip_id_l >> 0) & 0x000000ff);

    gettimeofday(&ts, NULL);
    srand((unsigned int)ts.tv_usec);
    rand_num = rand()%256;
    printf("rand_num = 0x%08x\n", rand_num);

    hash_board_id[4] = (unsigned char)(((unsigned int)ts.tv_sec >> 24) & 0x000000ff);
    hash_board_id[5] = (unsigned char)(((unsigned int)ts.tv_sec >> 16) & 0x000000ff);
    hash_board_id[6] = (unsigned char)(((unsigned int)ts.tv_sec >> 8) & 0x000000ff);
    hash_board_id[7] = (unsigned char)(((unsigned int)ts.tv_sec >> 0) & 0x000000ff);

    hash_board_id[8] = (unsigned char)(rand_num & 0x000000ff);
    hash_board_id[9] = (unsigned char)(((unsigned int)ts.tv_usec>> 16) & 0x000000ff);
    hash_board_id[10] = (unsigned char)(((unsigned int)ts.tv_usec >> 8) & 0x000000ff);
    hash_board_id[11] = (unsigned char)((unsigned int)ts.tv_usec & 0x000000ff);

    for(i=0; i<12; i++)
    {
        printf("--- %s: hash_board_id[%d] = 0x%02x\n", __FUNCTION__, i, hash_board_id[i]);
    }
}

void write_id(unsigned int id)
{
    FILE * id_file;
    char data[16] = {'I', 'd', '=', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '};
    unsigned char i=0;

    printf("\n--- %s\n", __FUNCTION__);

    id_file = fopen(ID_FILE, "w");
    if(!id_file)
    {
        printf("\n%s: open Id.ini failed\n", __FUNCTION__);
        return;
    }

    sprintf(data+3, "%d" , id);

    for(i=0; i<13; i++)
    {
        printf("--- %s: hash_board_id[%d] = %c\n", __FUNCTION__, i, data[i]);
    }

    fwrite(data, strlen(data), 1, id_file);
    fclose(id_file);
}

static int read_config()
{
    FILE * file;
	int forceFreq,forceFlag;
    struct configuration *m_conf = &Conf;
    char str[MAX_CHAR_NUM] = {0};
    char * temp;
    int offset = 0, starttemp = 0;
    int i;
    file = fopen(CONFIG_FILE, "r");
	char logstr[256];
	
    while(fgets(str, MAX_CHAR_NUM - 1 , file))
    {
        if(str[0] == '#' || str[1] == '#')
            continue;

        if((temp = strstr(str, "Name="))!=NULL)
        {
            temp += 5;
            for(i = 0; i < 64; i++)
            {
                m_conf->name[i] = *temp++;
                if(*temp == '\n' || *temp == '\r')
                    break;
            }
            i++;
            gName_len = i;
            m_conf->name[i] = '\0';
        }
        else if((temp = strstr(str, "TestDir="))!=NULL)
        {
            temp += 8;
            for(i = 0; i < 64; i++)
            {
                cgpu.workdataPathPrefix[i] = *temp++;
                //printf("%c", *temp);
                if(*temp == '\n' || *temp == '\r')
                    break;
            }
            i++;
            cgpu.workdataPathPrefix[i] = '\0';
            printf("workdataPathPrefix:%s\n", cgpu.workdataPathPrefix);
        }
        else if((temp = strstr(str, "DataCount="))!=NULL)
        {
            temp += 10;
            sscanf(temp, "%d", &m_conf->DataCount);
        }
        else if((temp = strstr(str, "PassCount1="))!=NULL)
        {
            temp += 11;
            sscanf(temp, "%d", &m_conf->PassCount1);
        }
        else if((temp = strstr(str, "PassCount2="))!=NULL)
        {
            temp += 11;
            sscanf(temp, "%d", &m_conf->PassCount2);
        }
        else if((temp = strstr(str, "PassCount3="))!=NULL)
        {
            temp += 11;
            sscanf(temp, "%d", &m_conf->PassCount3);
        }
        else if((temp = strstr(str, "Freq="))!=NULL)
        {
            temp += 5;
            sscanf(temp, "%d", &m_conf->Freq);
			
			m_conf->force_freq=0;
			forceFreq=GetForceFreq(&forceFlag);
			if(forceFlag>0)
			{
				if(forceFreq>=200)
				{
					m_conf->Freq=forceFreq;
					m_conf->force_freq=1;

					sprintf(logstr,"Find Force Freq=%d will search Freq again!\n",forceFreq);
					writeLogFile(logstr);
				}
			}
        }
        else if((temp = strstr(str, "freq_e="))!=NULL)
        {
            temp += 7;
            sscanf(temp, "%d", &m_conf->freq_e);
        }
		else if((temp = strstr(str, "UseConfigVol="))!=NULL)
        {
            temp += 13;
            sscanf(temp, "%d", &m_conf->UseConfigVol);
        }
        else if((temp = strstr(str, "freq_m="))!=NULL)
        {
            temp += 7;
            sscanf(temp, "%d", &m_conf->freq_m);
        }
        else if((temp = strstr(str, "freq_a="))!=NULL)
        {
            temp += 7;
            sscanf(temp, "%d", &m_conf->freq_a);
        }
        else if((temp = strstr(str, "freq_t="))!=NULL)
        {
            temp += 7;
            sscanf(temp, "%d", &m_conf->freq_t);
        }
        else if((temp = strstr(str, "force_freq="))!=NULL)
        {
            temp += 11;
        //    sscanf(temp, "%d", &m_conf->force_freq);
        }
        else if((temp = strstr(str, "Timeout="))!=NULL)
        {
            temp +=8;
            sscanf(temp, "%d", &m_conf->Timeout);
        }
        else if((temp = strstr(str, "AsicNum="))!=NULL)
        {
            temp += 8;
            sscanf(temp , "%d", &m_conf->AsicNum);
        }
		else if((temp = strstr(str, "UseFreqPIC="))!=NULL)
        {
            temp += 11;
            sscanf(temp , "%d", &m_conf->UseFreqPIC);
        }
        else if((temp = strstr(str, "TestMode="))!=NULL)
        {
            temp += 9;
            sscanf(temp, "%d", &m_conf->TestMode);
        }
        else if((temp = strstr(str, "CheckChain="))!=NULL)
        {
            temp += 11;
            sscanf(temp, "%d", &m_conf->CheckChain);
        }
        else if((temp = strstr(str, "CommandMode="))!=NULL)
        {
            temp += 12;
            sscanf(temp, "%d", &m_conf->CommandMode);
        }
        else if((temp = strstr(str, "ValidNonce1="))!=NULL)
        {
            temp += 12;
            sscanf(temp, "%d", &m_conf->ValidNonce1);
        }
        else if((temp = strstr(str, "ValidNonce2="))!=NULL)
        {
            temp += 12;
            sscanf(temp, "%d", &m_conf->ValidNonce2);
        }
        else if((temp = strstr(str, "ValidNonce3="))!=NULL)
        {
            temp += 12;
            sscanf(temp, "%d", &m_conf->ValidNonce3);
        }
        else if((temp = strstr(str, "Pic_VOLTAGE="))!=NULL)
        {
            temp += 12;
            sscanf(temp, "%d", &m_conf->Pic);
        }
        else if((temp = strstr(str, "Voltage1="))!=NULL)
        {
            temp += 9;
            sscanf(temp, "%d", &m_conf->Voltage1);
        }
        else if((temp = strstr(str, "Voltage2="))!=NULL)
        {
            temp += 9;
            sscanf(temp, "%d", &m_conf->Voltage2);
        }
        else if((temp = strstr(str, "Voltage3="))!=NULL)
        {
            temp += 9;
            sscanf(temp, "%d", &m_conf->Voltage3);
        }
		else if((temp = strstr(str, "final_voltage1="))!=NULL)
        {
            temp += 15;
            sscanf(temp, "%ud", &m_conf->final_voltage1);
        }
		else if((temp = strstr(str, "final_voltage2="))!=NULL)
        {
            temp += 15;
            sscanf(temp, "%ud", &m_conf->final_voltage2);
        }
		else if((temp = strstr(str, "final_voltage3="))!=NULL)
        {
            temp += 15;
            sscanf(temp, "%ud", &m_conf->final_voltage3);
        }
		else if((temp = strstr(str, "freq_gap="))!=NULL)
        {
            temp += 9;
            sscanf(temp, "%ud", &m_conf->freq_gap);
        }
        else if((temp = strstr(str, "OpenCoreGap="))!=NULL)
        {
            temp += 12;
            sscanf(temp, "%d", &m_conf->OpenCoreGap);
        }
        else if((temp = strstr(str, "CheckTemp="))!=NULL)
        {
            temp += 10;
            sscanf(temp, "%d", &m_conf->checktemp);
        }
        else if((temp = strstr(str, "IICPic="))!=NULL)
        {
            temp += 7;
            sscanf(temp, "%d", &m_conf->IICPic);
        }
        else if((temp = strstr(str, "Open_Core_Num1="))!=NULL)
        {
            temp += 15;
            sscanf(temp, "%ud", &m_conf->OpenCoreNum1);
        }
        else if((temp = strstr(str, "Open_Core_Num2="))!=NULL)
        {
            temp += 15;
            sscanf(temp, "%ud", &m_conf->OpenCoreNum2);
        }
        else if((temp = strstr(str, "Open_Core_Num3="))!=NULL)
        {
            temp += 15;
            sscanf(temp, "%ud", &m_conf->OpenCoreNum3);
        }
        else if((temp = strstr(str, "Open_Core_Num4="))!=NULL)
        {
            temp += 15;
            sscanf(temp, "%ud", &m_conf->OpenCoreNum4);
        }
        else if((temp = strstr(str, "DAC="))!=NULL)
        {
            temp += 4;
            sscanf(temp, "%ud", &m_conf->dac);
        }
        else if((temp = strstr(str, "GetTempFrom="))!=NULL)
        {
            temp += 12;
            sscanf(temp, "%ud", &m_conf->GetTempFrom);
        }
        else if((temp = strstr(str, "TempSel="))!=NULL)
        {
            temp += 8;
            sscanf(temp, "%ud", &m_conf->TempSel);
        }
        else if((temp = strstr(str, "TempSensor1="))!=NULL)
        {
            temp += 12;
            sscanf(temp, "%ud", &m_conf->TempSensor1);
        }
        else if((temp = strstr(str, "TempSensor2="))!=NULL)
        {
            temp += 12;
            sscanf(temp, "%ud", &m_conf->TempSensor2);
        }
        else if((temp = strstr(str, "TempSensor3="))!=NULL)
        {
            temp += 12;
            sscanf(temp, "%ud", &m_conf->TempSensor3);
        }
        else if((temp = strstr(str, "TempSensor4="))!=NULL)
        {
            temp += 12;
            sscanf(temp, "%ud", &m_conf->TempSensor4);
        }
        else if((temp = strstr(str, "DefaultTempOffset="))!=NULL)
        {
            temp += 18;
            sscanf(temp, "%d", &offset);
            if(offset < 0)
            {
                offset -= 2*offset;
                m_conf->DefaultTempOffset = (signed char)offset;
                m_conf->DefaultTempOffset -= 2*m_conf->DefaultTempOffset;
                //printf("~~~~~~~~~ m_conf->DefaultTempOffset = %d\n", m_conf->DefaultTempOffset);
            }
            else
            {
                m_conf->DefaultTempOffset = offset;
                //printf("~~~~~~~~~ m_conf->DefaultTempOffset = %d\n", m_conf->DefaultTempOffset);
            }
        }
        else if((temp = strstr(str, "year="))!=NULL)
        {
            temp += 5;
            sscanf(temp, "%d", &m_conf->year);
            //printf("year = %d\n", m_conf->year);
        }
        else if((temp = strstr(str, "month="))!=NULL)
        {
            temp += 6;
            sscanf(temp, "%d", &m_conf->month);
        }
        else if((temp = strstr(str, "date="))!=NULL)
        {
            temp += 5;
            sscanf(temp, "%d", &m_conf->date);
        }
        else if((temp = strstr(str, "hour="))!=NULL)
        {
            temp += 5;
            sscanf(temp, "%d", &m_conf->hour);
        }
        else if((temp = strstr(str, "minute="))!=NULL)
        {
            temp += 7;
            sscanf(temp, "%d", &m_conf->minute);
        }
        else if((temp = strstr(str, "second="))!=NULL)
        {
            temp += 7;
            sscanf(temp, "%d", &m_conf->second);
        }
        else if((temp = strstr(str, "StartTemp="))!=NULL)
        {
            temp += 10;
            sscanf(temp, "%d", &m_conf->StartTemp);
            sscanf(temp, "%d", &starttemp);
            if(starttemp < 0)
            {
                starttemp -= 2*starttemp;
                m_conf->StartTemp = (signed char)starttemp;
                m_conf->StartTemp -= 2*m_conf->StartTemp;
                //printf("~~~~~~~~~ m_conf->DefaultTempOffset = %d\n", m_conf->DefaultTempOffset);
            }
            else
            {
                m_conf->StartTemp = starttemp;
                //printf("~~~~~~~~~ m_conf->DefaultTempOffset = %d\n", m_conf->DefaultTempOffset);
            }
        }
    }

	// fix ASIC number for S9+
	m_conf->AsicNum=ASIC_NUM;
	
	return 0;
}

static int process_config()
{
    uint32_t rBaudrate;
    int temp_corenum = 0;

    conf.CommandMode = Conf.CommandMode;

    conf.TempSel = Conf.TempSel;

    conf.GetTempFrom = Conf.GetTempFrom;

    if(Conf.CommandMode == FIL)
    {
        if(conf.GetTempFrom == 1)   // read temp from asic
        {
            applog(LOG_ERR, "Can't get temperature from ASIC in FIL mode!\n");
            return -1;
        }
    }

    if(Conf.CommandMode == VIL)
    {
        if(conf.GetTempFrom == 1)   // read temp from asic
        {
            cgpu.temp_sel = Conf.TempSel;
            cgpu.rfs = 1;
            cgpu.tfs = 3;
            //printf("cgpu.temp_sel = %d, cgpu.rfs = %d, cgpu.tfs = %d\n", cgpu.temp_sel, cgpu.rfs, cgpu.tfs);

            if(Conf.TempSensor1 + Conf.TempSensor2 + Conf.TempSensor3 + Conf.TempSensor4)
            {
                conf.TempSensor1 = Conf.TempSensor1;
                conf.TempSensor2 = Conf.TempSensor2;
                conf.TempSensor3 = Conf.TempSensor3;
                conf.TempSensor4 = Conf.TempSensor4;
                conf.DefaultTempOffset = Conf.DefaultTempOffset;
                cgpu.T1_offset_value = Conf.DefaultTempOffset;
                cgpu.T2_offset_value = Conf.DefaultTempOffset;
                cgpu.T3_offset_value = Conf.DefaultTempOffset;
                cgpu.T4_offset_value = Conf.DefaultTempOffset;
                conf.StartTemp = Conf.StartTemp;
            }
            else
            {
                applog(LOG_ERR, "Must set temperature sensor address!\n");
                return -1;
            }
        }
    }

    conf.freq_e = Conf.freq_e;

    conf.freq_m = Conf.freq_m;

    conf.freq_a = Conf.freq_a;

    conf.freq_t = Conf.freq_t;

    conf.force_freq = Conf.force_freq;

	conf.UseConfigVol = Conf.UseConfigVol;

    conf.OpenCoreNum1 = Conf.OpenCoreNum1;

    conf.OpenCoreNum2 = Conf.OpenCoreNum2;

    conf.OpenCoreNum3 = Conf.OpenCoreNum3;

    conf.OpenCoreNum4 = Conf.OpenCoreNum4;

    temp_corenum = calculate_core_number(ASIC_CORE_NUM);

    conf.testMode = Conf.TestMode;

    conf.ValidNonce1 = Conf.ValidNonce1;

    conf.ValidNonce2 = Conf.ValidNonce2;

    conf.ValidNonce3 = Conf.ValidNonce3;

    conf.Pic = Conf.Pic;

    conf.IICPic = Conf.IICPic;

    conf.dac= Conf.dac;

    conf.Voltage1 = Conf.Voltage1;

    conf.Voltage2 = Conf.Voltage2;

    conf.Voltage3 = Conf.Voltage3;

    conf.OpenCoreGap = Conf.OpenCoreGap;

    conf.checktemp = Conf.checktemp;

    if(ASIC_TYPE==1385 || ASIC_TYPE == 1387)
    {
        conf.freq = Conf.Freq;
    }
    else
    {
        printf("%s: ASIC_TYPE = %d, but it is not correct!\n", __FUNCTION__, ASIC_TYPE);
    }

    if(ASIC_TYPE==1385)
    {
        CHIP_ADDRESS            = 0x0;
        GOLDEN_NONCE_COUNTER    = 0x8;
        PLL_PARAMETER           = 0xc;
        START_NONCE_OFFSET      = 0x10;
        HASH_COUNTING_NUMBER    = 0x14;
        TICKET_MASK             = 0x18;
        MISC_CONTROL            = 0x1c;
    }

    if(ASIC_TYPE==1387)
    {
        CHIP_ADDRESS            = 0x0;
        HASH_RATE               = 0x8;
        PLL_PARAMETER           = 0xc;
        START_NONCE_OFFSET      = 0x10;
        HASH_COUNTING_NUMBER    = 0x14;
        TICKET_MASK             = 0x18;
        MISC_CONTROL            = 0x1c;
        GENERAL_I2C_COMMAND     = 0x20;
        SECURITY_I2C_COMMAND    = 0x24;
        SIGNATURE_INPUT         = 0x28;
        SIGNATURE_NONCE         = 0x2C;
        SIGNATURE_ID            = 0x30;
        SECURITY_CONTROL_AND_STATUS = 0x34;
        JOB_INFORMATION         = 0x38;
    }

    conf.year = Conf.year;
    conf.month = Conf.month;
    conf.date = Conf.date;
    conf.hour = Conf.hour;
    conf.minute = Conf.minute;
    conf.second = Conf.second;

    if(Conf.Timeout <= 0)
        conf.timeout = 0x1000000/temp_corenum*CHIP_ADDR_INTERVAL/Conf.Freq*95/100;
    else
        conf.timeout = Conf.Timeout;

    rBaudrate = 1000000 * 5/3 / conf.timeout * (64*8);//64*8 need send bit, ratio=2/3
    conf.baud = 25000000/rBaudrate/8 - 1;
    if(conf.baud > 26)
    {
        conf.baud = 26;
    }
    else if(conf.baud <= 0)
    {
        applog(LOG_ERR, "$$$$Config argument Baudrate:%d err\n", conf.baud);
        return -1;
    }

    if(Conf.DataCount > MAX_WORK || Conf.DataCount <= 0)
    {
        applog(LOG_ERR, "$$$$Config argument DataCount:%d err\n", Conf.DataCount);
    }
    else
        conf.dataCount = Conf.DataCount;

    if(Conf.PassCount1 > conf.dataCount || Conf.PassCount1 < 0)
    {
        applog(LOG_ERR, "$$$$Config argument DataCount:%d err\n", Conf.DataCount);
    }
    else
        conf.passCount1 = Conf.PassCount1;

    if(Conf.PassCount2 > conf.dataCount || Conf.PassCount2 < 0)
    {
        applog(LOG_ERR, "$$$$Config argument DataCount:%d err\n", Conf.DataCount);
    }
    else
        conf.passCount2 = Conf.PassCount2;

    if(Conf.PassCount3 > conf.dataCount || Conf.PassCount3 < 0)
    {
        applog(LOG_ERR, "$$$$Config argument DataCount:%d err\n", Conf.DataCount);
    }
    else
        conf.passCount3 = Conf.PassCount3;

	return 0;
}

void display_arguments(void)
{
}


static void print_config()
{
    const struct configuration *m_conf = &Conf;
    printf("\n\nRead Config.ini\n");
    printf("Name:%s\n", m_conf->name);
    printf("DataCount:%d\n", m_conf->DataCount);
    printf("PassCount1:%d\n", m_conf->PassCount1);
    printf("PassCount2:%d\n", m_conf->PassCount2);
    printf("PassCount3:%d\n", m_conf->PassCount3);
    printf("Freq:%d\n", m_conf->Freq);
    printf("Timeout:%d\n", m_conf->Timeout);
    printf("OpenCoreGap:%d\n", m_conf->OpenCoreGap);
    printf("CheckTemp:%d\n", m_conf->checktemp);
    printf("freq_e:%d\n", m_conf->freq_e);
    printf("AsicNum:%d\n", m_conf->AsicNum);
    printf("TestMode:%d\n", m_conf->TestMode);
    printf("CheckChain:%d\n", m_conf->CheckChain);
    printf("CommandMode:%d\n", m_conf->CommandMode);
    printf("ValidNonce1:%d\n", m_conf->ValidNonce1);
    printf("ValidNonce2:%d\n", m_conf->ValidNonce2);
    printf("ValidNonce3:%d\n", m_conf->ValidNonce3);
    printf("Pic:%ud\n", m_conf->Pic);
    printf("IICPic:%ud\n", m_conf->IICPic);
    printf("dac = %ud\n", m_conf->dac);
    printf("Voltage1:%ud\n", m_conf->Voltage1);
    printf("Voltage2:%ud\n", m_conf->Voltage2);
    printf("Voltage3:%ud\n", m_conf->Voltage3);
    printf("OpenCoreNum1 = %ud = 0x%x\n", m_conf->OpenCoreNum1, m_conf->OpenCoreNum1);
    printf("OpenCoreNum2 = %ud = 0x%x\n", m_conf->OpenCoreNum2, m_conf->OpenCoreNum2);
    printf("OpenCoreNum3 = %ud = 0x%x\n", m_conf->OpenCoreNum3, m_conf->OpenCoreNum3);
    printf("OpenCoreNum4 = %ud = 0x%x\n", m_conf->OpenCoreNum4, m_conf->OpenCoreNum4);
    printf("GetTempFrom:%d\n", m_conf->GetTempFrom);
    printf("TempSel:%d\n", m_conf->TempSel);
    printf("TempSensor1:%d\n", m_conf->TempSensor1);
    printf("TempSensor2:%d\n", m_conf->TempSensor2);
    printf("TempSensor3:%d\n", m_conf->TempSensor3);
    printf("TempSensor4:%d\n", m_conf->TempSensor4);
    printf("DefaultTempOffset:%d\n", m_conf->DefaultTempOffset);
    printf("StartTemp:%d\n", m_conf->StartTemp);
    printf("year:%04d\n", m_conf->year);
    printf("month:%02d\n", m_conf->month);
    printf("date:%02d\n", m_conf->date);
    printf("hour:%02d\n", m_conf->hour);
    printf("minute:%02d\n", m_conf->minute);
    printf("second:%02d\n", m_conf->second);
    //printf("\n");
    //printf("FPGA ID: 0x%02x%02x%02x%02x%02x%02x%02x%02x\n", hash_board_id[0],hash_board_id[1],hash_board_id[2],hash_board_id[3],hash_board_id[4],hash_board_id[5],hash_board_id[6],hash_board_id[7]);
    //printf("ID: 0x%02x%02x%02x%02x\n", hash_board_id[8],hash_board_id[9],hash_board_id[10],hash_board_id[11]);
    printf("\n\n");
}

static void print_CONFIG(void)
{
    const struct _CONFIG *m_conf = &conf;
    printf("\n\nparameter processed after Reading Config.ini\n");
    printf("DataCount:%d\n", m_conf->dataCount);
    printf("PassCount1:%d\n", m_conf->passCount1);
    printf("PassCount2:%d\n", m_conf->passCount2);
    printf("PassCount3:%d\n", m_conf->passCount3);
    printf("Freq:%d\n", m_conf->freq);
    printf("Timeout:%d\n", m_conf->timeout);
    printf("OpenCoreGap:%d\n", m_conf->OpenCoreGap);
    printf("CheckTemp:%d\n", m_conf->checktemp);
    printf("TestMode:%d\n", m_conf->testMode);
    printf("CommandMode:%d\n", m_conf->CommandMode);
    printf("ValidNonce1:%d\n", m_conf->ValidNonce1);
    printf("ValidNonce2:%d\n", m_conf->ValidNonce2);
    printf("ValidNonce3:%d\n", m_conf->ValidNonce3);
    printf("Pic:%ud\n", m_conf->Pic);
    printf("IICPic:%ud\n", m_conf->IICPic);
    printf("dac:%ud\n", m_conf->dac);
    printf("Voltage1:%ud\n", m_conf->Voltage1);
    printf("Voltage2:%ud\n", m_conf->Voltage2);
    printf("Voltage3:%ud\n", m_conf->Voltage3);
    printf("OpenCoreNum1 = %ud = 0x%x\n", m_conf->OpenCoreNum1, m_conf->OpenCoreNum1);
    printf("OpenCoreNum2 = %ud = 0x%x\n", m_conf->OpenCoreNum2, m_conf->OpenCoreNum2);
    printf("OpenCoreNum3 = %ud = 0x%x\n", m_conf->OpenCoreNum3, m_conf->OpenCoreNum3);
    printf("OpenCoreNum4 = %ud = 0x%x\n", m_conf->OpenCoreNum4, m_conf->OpenCoreNum4);
    printf("GetTempFrom:%d\n", m_conf->GetTempFrom);
    printf("TempSel:%d\n", m_conf->TempSel);
    printf("TempSensor1:%d\n", m_conf->TempSensor1);
    printf("TempSensor2:%d\n", m_conf->TempSensor2);
    printf("TempSensor3:%d\n", m_conf->TempSensor3);
    printf("TempSensor4:%d\n", m_conf->TempSensor4);
    printf("DefaultTempOffset:%d\n", m_conf->DefaultTempOffset);
    printf("StartTemp:%d\n", m_conf->StartTemp);
    printf("year:%04d\n", m_conf->year);
    printf("month:%02d\n", m_conf->month);
    printf("date:%02d\n", m_conf->date);
    printf("hour:%02d\n", m_conf->hour);
    printf("minute:%02d\n", m_conf->minute);
    printf("second:%02d\n", m_conf->second);
    printf("\n\n");
}

static int get_works()
{
    char strFilePath[256] = {0};
    int i, j, record, loop=0;
    unsigned int OpenCoreNum1 = conf.OpenCoreNum1;
    unsigned int OpenCoreNum2 = conf.OpenCoreNum2;
    unsigned int OpenCoreNum3 = conf.OpenCoreNum3;
    unsigned int OpenCoreNum4 = conf.OpenCoreNum4;
    //getcwd(Path, 128);
    //applog(LOG_DEBUG, "Path:%s\n", Path);

    //printf("%s: loop = %d\n", __FUNCTION__, loop);

    if(ASIC_NUM == 1)
    {
        for(j=0; j < 32; j++)
        {
            if(OpenCoreNum1 & 0x00000001)
            {
                loop++;
            }
            OpenCoreNum1 = OpenCoreNum1 >> 1;

            if(OpenCoreNum2 & 0x00000001)
            {
                loop++;
            }
            OpenCoreNum2 = OpenCoreNum2 >> 1;

            if(OpenCoreNum3 & 0x00000001)
            {
                loop++;
            }
            OpenCoreNum3 = OpenCoreNum3 >> 1;

            if(OpenCoreNum4 & 0x00000001)
            {
                loop++;
            }
            OpenCoreNum4 = OpenCoreNum4 >> 1;
        }

        printf("%s: loop = %d\n", __FUNCTION__, loop);
    }
    else
    {
        loop = calculate_asic_number(ASIC_NUM);
    }

    j=0;
    OpenCoreNum1 = conf.OpenCoreNum1;
    OpenCoreNum2 = conf.OpenCoreNum2;
    OpenCoreNum3 = conf.OpenCoreNum3;
    OpenCoreNum4 = conf.OpenCoreNum4;

    for(i = 0; i < loop; i++)
    {
        if(ASIC_NUM == 1)
        {
            for(; j < 128; j++)
            {
                if(j < 32)
                {
                    if(OpenCoreNum1 & 0x00000001)
                    {
                        sprintf(strFilePath, "%s%02i.bin", cgpu.workdataPathPrefix, j+1);
                        printf("dir:%s\n", strFilePath);
                        OpenCoreNum1 = OpenCoreNum1 >> 1;
                        j++;
                        break;
                    }
                    else
                    {
                        OpenCoreNum1 = OpenCoreNum1 >> 1;
                    }
                }
                else if((j >= 32) && (j < 64))
                {
                    if(OpenCoreNum2 & 0x00000001)
                    {
                        sprintf(strFilePath, "%s%02i.bin", cgpu.workdataPathPrefix, j+1);
                        printf("dir:%s\n", strFilePath);
                        OpenCoreNum2 = OpenCoreNum2 >> 1;
                        j++;
                        break;
                    }
                    else
                    {
                        OpenCoreNum2 = OpenCoreNum2 >> 1;
                    }
                }
                else if((j >= 64) && (j < 96))
                {
                    if(OpenCoreNum3 & 0x00000001)
                    {
                        sprintf(strFilePath, "%s%02i.bin", cgpu.workdataPathPrefix, j+1);
                        printf("dir:%s\n", strFilePath);
                        OpenCoreNum3 = OpenCoreNum3 >> 1;
                        j++;
                        break;
                    }
                    else
                    {
                        OpenCoreNum3 = OpenCoreNum3 >> 1;
                    }
                }
                else
                {
                    if(OpenCoreNum4 & 0x00000001)
                    {
                        sprintf(strFilePath, "%s%02i.bin", cgpu.workdataPathPrefix, j+1);
                        printf("dir:%s\n", strFilePath);
                        OpenCoreNum4 = OpenCoreNum4 >> 1;
                        j++;
                        break;
                    }
                    else
                    {
                        OpenCoreNum4 = OpenCoreNum4 >> 1;
                    }
                }
            }
        }
        else
        {
            sprintf(strFilePath, "%s%02i.bin", cgpu.workdataPathPrefix, i+1);
        }

        cgpu.fps[i] = fopen(strFilePath, "rb");
        if(NULL == cgpu.fps[i])
        {
            applog(LOG_ERR, "Open test file %s error\n", strFilePath);
            return -1;
        }
        cgpu.subid[i] = get_work(i, MAX_WORK);
        //applog(LOG_DEBUG, "asic[%d] get work %d\n", i, cgpu.subid[i]);
        fclose(cgpu.fps[i]);
    }

    cgpu.min_work_subid = cgpu.subid[0];
    record = 0;
    for(i = 0; i < loop; i++)
    {
        if(cgpu.min_work_subid > cgpu.subid[i])
        {
            cgpu.min_work_subid = cgpu.subid[i];
            record = i;
        }
    }
    applog(LOG_DEBUG, "min work minertest[%d]:%d\n\n\n", record, cgpu.min_work_subid);
    if(conf.dataCount > cgpu.min_work_subid)
    {
        applog(LOG_ERR, "$$$$dataCount=%d, but min work subid=%d\n",
               conf.dataCount, cgpu.min_work_subid);
        return -1;
    }
    return 0;
}

int get_works_32X()
{
    char strFilePath[256] = {0};
    int i, loop=0;
    unsigned int OpenCoreNum1 = conf.OpenCoreNum1;
    unsigned int OpenCoreNum2 = conf.OpenCoreNum2;
    unsigned int OpenCoreNum3 = conf.OpenCoreNum3;
    unsigned int OpenCoreNum4 = conf.OpenCoreNum4;

    loop = calculate_asic_number(ASIC_NUM);

    OpenCoreNum1 = conf.OpenCoreNum1;
    OpenCoreNum2 = conf.OpenCoreNum2;
    OpenCoreNum3 = conf.OpenCoreNum3;
    OpenCoreNum4 = conf.OpenCoreNum4;

    for(i = 0; i < loop; i++)
    {
        sprintf(strFilePath, "%s/minertest64_%02d.bin", TESTPATTEN_DIR, i+1);

        cgpu.fps[i] = fopen(strFilePath, "rb");
        if(NULL == cgpu.fps[i])
        {
            applog(LOG_ERR, "Open test file %s error\n", strFilePath);
            return -1;
        }
        cgpu.subid[i] = get_work(i, MAX_WORK);
        fclose(cgpu.fps[i]);
    }

    cgpu.min_work_subid = cgpu.subid[0];
    for(i = 0; i < loop; i++)
    {
        if(cgpu.min_work_subid > cgpu.subid[i])
        {
            cgpu.min_work_subid = cgpu.subid[i];
        }
    }
    
    return 0;
}


int configMiner()
{
    int ret;
    //init_config(cgpu);
    //read_id();
    read_config();
    print_config();
    ret = process_config();
    if(ret < 0) return -EFAULT;

    print_CONFIG();
    ret = get_works();
    if(ret < 0) return -EFAULT;
    return 0;
}


//FPGA related
int get_nonce2_and_job_id_store_address(void)
{
    int ret = -1;
    ret = *((unsigned int *)(axi_fpga_addr + NONCE2_AND_JOBID_STORE_ADDRESS));
    applog(LOG_DEBUG,"%s: NONCE2_AND_JOBID_STORE_ADDRESS is 0x%x\n", __FUNCTION__, ret);
    return ret;
}

void set_nonce2_and_job_id_store_address(unsigned int value)
{
    *((unsigned int *)(axi_fpga_addr + NONCE2_AND_JOBID_STORE_ADDRESS)) = value;
    applog(LOG_DEBUG,"%s: set NONCE2_AND_JOBID_STORE_ADDRESS is 0x%x\n", __FUNCTION__, value);
    get_nonce2_and_job_id_store_address();
}

int get_job_start_address(void)
{
    int ret = -1;
    ret = *((unsigned int *)(axi_fpga_addr + JOB_START_ADDRESS));
    applog(LOG_DEBUG,"%s: JOB_START_ADDRESS is 0x%x\n", __FUNCTION__, ret);
    return ret;
}

void set_job_start_address(unsigned int value)
{
    *((unsigned int *)(axi_fpga_addr + JOB_START_ADDRESS)) = value;
    applog(LOG_DEBUG,"%s: set JOB_START_ADDRESS is 0x%x\n", __FUNCTION__, value);
    get_job_start_address();
}

int get_QN_write_data_command(void)
{
    int ret = -1;
    ret = *((axi_fpga_addr + QN_WRITE_DATA_COMMAND));
    //applog(LOG_DEBUG,"%s: QN_WRITE_DATA_COMMAND is 0x%x\n", __FUNCTION__, ret);
    return ret;
}

void set_QN_write_data_command(unsigned int value)
{
    *(axi_fpga_addr + QN_WRITE_DATA_COMMAND) = value;
    //applog(LOG_DEBUG,"%s: set QN_WRITE_DATA_COMMAND is 0x%x\n", __FUNCTION__, value);
    //get_QN_write_data_command();
}

void set_reset_hashboard(int chainIndex, int resetBit)
{
	unsigned int ret;
	unsigned int resetFlag;
	char logstr[256];
	ret = *((axi_fpga_addr + RESET_HASHBOARD_COMMAND));
	resetFlag=(1<<chainIndex);
	
	if(resetBit>0)
		ret = ret | resetFlag;
	else
		ret = ret & (~resetFlag);

	sprintf(logstr,"set_reset_hashboard = 0x%08x\n",ret);
	writeLogFile(logstr);
	*(axi_fpga_addr + RESET_HASHBOARD_COMMAND) = ret;
}

void set_reset_allhashboard(int resetBit)
{
	unsigned int ret;
	char logstr[256];
	ret = *((axi_fpga_addr + RESET_HASHBOARD_COMMAND));

	if(resetBit>0)
		ret = ret | 0x0000ffff;
	else
		ret = ret & 0xffff0000;

	sprintf(logstr,"set_reset_allhashboard = 0x%08x\n",ret);
	writeLogFile(logstr);
	*(axi_fpga_addr + RESET_HASHBOARD_COMMAND) = ret;
}


int bitmain_axi_init()
{
    unsigned int data;
    int ret=0;

    cgpu.device_fd = open("/dev/axi_fpga_dev", O_RDWR);
    if(cgpu.device_fd < 0)
    {
        applog(LOG_DEBUG,"/dev/axi_fpga_dev open failed. fd = %d\n", cgpu.device_fd);
        perror("open");
        return -1;
    }

    axi_fpga_addr = mmap(NULL, TOTAL_LEN, PROT_READ|PROT_WRITE, MAP_SHARED, cgpu.device_fd, 0);
    if(!axi_fpga_addr)
    {
        applog(LOG_DEBUG,"mmap axi_fpga_addr failed. axi_fpga_addr = 0x%x\n", (unsigned int)axi_fpga_addr);
        return -1;
    }
    applog(LOG_DEBUG,"mmap axi_fpga_addr = 0x%x\n", (unsigned int)axi_fpga_addr);

    //check the value in address 0xff200000
    data = (*axi_fpga_addr & 0x0000ffff);

    applog(LOG_DEBUG,"axi_fpga_addr data = 0x%x\n", data);

    fd_fpga_mem = open("/dev/fpga_mem", O_RDWR);
    if(fd_fpga_mem < 0)
    {
        applog(LOG_DEBUG,"/dev/fpga_mem open failed. fd_fpga_mem = %d\n", fd_fpga_mem);
        perror("open");
        return -1;
    }

    fpga_mem_addr = mmap(NULL, FPGA_MEM_TOTAL_LEN, PROT_READ|PROT_WRITE, MAP_SHARED, fd_fpga_mem, 0);
    if(!fpga_mem_addr)
    {
        applog(LOG_DEBUG,"mmap fpga_mem_addr failed. fpga_mem_addr = 0x%x\n", (unsigned int)fpga_mem_addr);
        return -1;
    }
    applog(LOG_DEBUG,"mmap fpga_mem_addr = 0x%x\n", (unsigned int)fpga_mem_addr);

    nonce2_jobid_address = fpga_mem_addr;

    set_nonce2_and_job_id_store_address(PHY_MEM_NONCE2_JOBID_ADDRESS);

    //malloc register value buffer
    reg_value_buf = malloc(sizeof(struct reg_buf));
    if(!reg_value_buf)
    {
        applog(LOG_DEBUG,"%s: malloc reg_value_buf failed\n", __FUNCTION__);
        return -4;
    }
    else
    {
        memset(reg_value_buf, 0, sizeof(struct reg_buf));
    }

    return ret;
}

int bitmain_axi_close()
{
    int ret = 0;

    ret = munmap((void *)axi_fpga_addr, TOTAL_LEN);
    if(ret<0)
    {
        applog(LOG_DEBUG,"munmap failed!\n");
    }

    ret = munmap((void *)fpga_mem_addr, FPGA_MEM_TOTAL_LEN);
    if(ret<0)
    {
        applog(LOG_DEBUG,"munmap failed!\n");
    }

    //free_pages((unsigned long)nonce2_jobid_address, NONCE2_AND_JOBID_STORE_SPACE_ORDER);
    //free(temp_job_start_address_1);
    //free(temp_job_start_address_2);

    close(fd);
    close(fd_fpga_mem);
	return 0;
}

int get_fan_control(void)
{
    int ret = -1;
    ret = *((unsigned int *)(axi_fpga_addr + FAN_CONTROL));
//    applog(LOG_DEBUG,"%s: FAN_CONTROL is 0x%x\n", __FUNCTION__, ret);
    return ret;
}

void set_fan_control(unsigned int value)
{
    *((unsigned int *)(axi_fpga_addr + FAN_CONTROL)) = value;
//    applog(LOG_DEBUG,"%s: set FAN_CONTROL is 0x%x\n", __FUNCTION__, value);
    get_fan_control();
}

int get_hash_on_plug(void)
{
    int ret = -1;
    ret = *(axi_fpga_addr + HASH_ON_PLUG);
    applog(LOG_DEBUG,"%s: HASH_ON_PLUG is 0x%x\n", __FUNCTION__, ret);
    return ret;
}

int get_fan_speed(unsigned char *fan_id, unsigned int *fan_speed)
{
    int ret = -1;
    ret = *((unsigned int *)(axi_fpga_addr + FAN_SPEED));
    *fan_speed = 0x000000ff & ret;
    *fan_id = (unsigned char)(0x00000007 & (ret >> 8));
    if(*fan_speed > 0)
    {
        applog(LOG_DEBUG,"%s: fan_id is 0x%x, fan_speed is 0x%x\n", __FUNCTION__, *fan_id, *fan_speed);
    }
    return ret;
}

int get_temperature_0_3(void)
{
    int ret = -1;
    ret = *((int *)(axi_fpga_addr + TEMPERATURE_0_3));
    //applog(LOG_DEBUG,"%s: TEMPERATURE_0_3 is 0x%x\n", __FUNCTION__, ret);
    return ret;
}

int get_temperature_4_7(void)
{
    int ret = -1;
    ret = *((int *)(axi_fpga_addr + TEMPERATURE_4_7));
    //applog(LOG_DEBUG,"%s: TEMPERATURE_4_7 is 0x%x\n", __FUNCTION__, ret);
    return ret;
}

int get_temperature_8_11(void)
{
    int ret = -1;
    ret = *((int *)(axi_fpga_addr + TEMPERATURE_8_11));
    //applog(LOG_DEBUG,"%s: TEMPERATURE_8_11 is 0x%x\n", __FUNCTION__, ret);
    return ret;
}

int get_temperature_12_15(void)
{
    int ret = -1;
    ret = *((int *)(axi_fpga_addr + TEMPERATURE_12_15));
    //applog(LOG_DEBUG,"%s: TEMPERATURE_12_15 is 0x%x\n", __FUNCTION__, ret);
    return ret;
}

int get_time_out_control(void)
{
    int ret = -1;
    ret = *((unsigned int *)(axi_fpga_addr + TIME_OUT_CONTROL)) & 0x7fffffff;
    applog(LOG_DEBUG,"%s: TIME_OUT_CONTROL is 0x%x\n", __FUNCTION__, ret);
    return ret;
}

void set_time_out_control(unsigned int value)
{
    *((unsigned int *)(axi_fpga_addr + TIME_OUT_CONTROL)) = value | 0x80000000;
    applog(LOG_DEBUG,"%s: set TIME_OUT_CONTROL is 0x%x\n", __FUNCTION__, value);
    get_time_out_control();
}

int get_BC_command_buffer(unsigned int *buf)
{
    int ret = -1;
    ret = *((unsigned int *)(axi_fpga_addr + BC_COMMAND_BUFFER));
    *(buf + 0) = ret;   //this is for FIL
    ret = *((unsigned int *)(axi_fpga_addr + BC_COMMAND_BUFFER + 1));
    *(buf + 1) = ret;
    ret = *((unsigned int *)(axi_fpga_addr + BC_COMMAND_BUFFER + 2));
    *(buf + 2) = ret;
    //applog(LOG_DEBUG,"%s: BC_COMMAND_BUFFER buf[0]: 0x%x, buf[1]: 0x%x, buf[2]: 0x%x\n", __FUNCTION__, *(buf + 0), *(buf + 1), *(buf + 2));
    return ret;
}

void set_BC_command_buffer(unsigned int *value)
{
    unsigned int buf[4] = {0};

    *((unsigned int *)(axi_fpga_addr + BC_COMMAND_BUFFER)) = *(value + 0);      //this is for FIL
    *((unsigned int *)(axi_fpga_addr + BC_COMMAND_BUFFER + 1)) = *(value + 1);
    *((unsigned int *)(axi_fpga_addr + BC_COMMAND_BUFFER + 2)) = *(value + 2);

    //applog(LOG_DEBUG,"%s: set BC_COMMAND_BUFFER value[0]: 0x%x, value[1]: 0x%x, value[2]: 0x%x\n", __FUNCTION__, *(value + 0), *(value + 1), *(value + 2));
    get_BC_command_buffer(buf);
}

int get_nonce_number_in_fifo(void)
{
    int ret = -1;
    ret = *((unsigned int *)(axi_fpga_addr + NONCE_NUMBER_IN_FIFO));
    //applog(LOG_DEBUG,"%s: NONCE_NUMBER_IN_FIFO is 0x%x\n", __FUNCTION__, ret);
    return ret;
}

int get_return_nonce(unsigned int *buf)
{
    int ret = -1;

    ret = *((unsigned int *)(axi_fpga_addr + RETURN_NONCE));
    *(buf + 0) = ret;
    ret = *((unsigned int *)(axi_fpga_addr + RETURN_NONCE + 1));
    *(buf + 1) = ret;   //there is nonce3

    //applog(LOG_DEBUG,"%s: RETURN_NONCE buf[0] is 0x%x, buf[1] is 0x%x\n", __FUNCTION__, *(buf + 0), *(buf + 1));
    return ret;
}

int get_BC_write_command(void)
{
    int ret = -1;
    ret = *((unsigned int *)(axi_fpga_addr + BC_WRITE_COMMAND));
    //applog(LOG_DEBUG,"%s: BC_WRITE_COMMAND is 0x%x\n", __FUNCTION__, ret);
    return ret;
}

void set_BC_write_command(unsigned int value)
{
	int wait_count=0;
	char logstr[256];
	
    *((unsigned int *)(axi_fpga_addr + BC_WRITE_COMMAND)) = value;
    //applog(LOG_DEBUG,"%s: set BC_WRITE_COMMAND is 0x%x\n", __FUNCTION__, value);

    if(value & BC_COMMAND_BUFFER_READY)
    {
        while(get_BC_write_command() & BC_COMMAND_BUFFER_READY)
        {
            usleep(1000);
            //applog(LOG_DEBUG,"%s waiting ...\n", __FUNCTION__);

			wait_count++;
			if(wait_count>3000)
			{
				sprintf(logstr,"Error: set_BC_write_command wait buffer ready timeout!\n");
				writeLogFile(logstr);
				break;
			}
        }
    }
    else
    {
        get_BC_write_command();
    }
}

int get_ticket_mask(void)
{
    int ret = -1;
    ret = *((unsigned int *)(axi_fpga_addr + TICKET_MASK_FPGA));
    applog(LOG_DEBUG,"%s: TICKET_MASK_FPGA is 0x%x\n", __FUNCTION__, ret);
    return ret;
}

void set_ticket_mask(unsigned int value)
{
    *((unsigned int *)(axi_fpga_addr + TICKET_MASK_FPGA)) = value;
    applog(LOG_DEBUG,"%s: set TICKET_MASK_FPGA is 0x%x\n", __FUNCTION__, value);
    get_ticket_mask();
}

int get_job_id(void)
{
    int ret = -1;
    ret = *((unsigned int *)(axi_fpga_addr + JOB_ID));
    applog(LOG_DEBUG,"%s: JOB_ID is 0x%x\n", __FUNCTION__, ret);
    return ret;
}

void set_job_id(unsigned int value)
{
    *((unsigned int *)(axi_fpga_addr + JOB_ID)) = value;
    applog(LOG_DEBUG,"%s: set JOB_ID is 0x%x\n", __FUNCTION__, value);
    get_job_id();
}

int get_job_length(void)
{
    int ret = -1;
    ret = *((unsigned int *)(axi_fpga_addr + JOB_LENGTH));
    applog(LOG_DEBUG,"%s: JOB_LENGTH is 0x%x\n", __FUNCTION__, ret);
    return ret;
}

void set_job_length(unsigned int value)
{
    *((unsigned int *)(axi_fpga_addr + JOB_LENGTH)) = value;
    applog(LOG_DEBUG,"%s: set JOB_LENGTH is 0x%x\n", __FUNCTION__, value);
    get_job_id();
}


int get_block_header_version(void)
{
    int ret = -1;
    ret = *((unsigned int *)(axi_fpga_addr + BLOCK_HEADER_VERSION));
    applog(LOG_DEBUG,"%s: BLOCK_HEADER_VERSION is 0x%x\n", __FUNCTION__, ret);
    return ret;
}

void set_block_header_version(unsigned int value)
{
    *((unsigned int *)(axi_fpga_addr + BLOCK_HEADER_VERSION)) = value;
    applog(LOG_DEBUG,"%s: set BLOCK_HEADER_VERSION is 0x%x\n", __FUNCTION__, value);
    get_block_header_version();
}

int get_time_stamp()
{
    int ret = -1;
    ret = *((unsigned int *)(axi_fpga_addr + TIME_STAMP));
    applog(LOG_DEBUG,"%s: TIME_STAMP is 0x%x\n", __FUNCTION__, ret);
    return ret;
}

void set_time_stamp(unsigned int value)
{
    *((unsigned int *)(axi_fpga_addr + TIME_STAMP)) = value;
    applog(LOG_DEBUG,"%s: set TIME_STAMP is 0x%x\n", __FUNCTION__, value);
    get_time_stamp();
}

int get_target_bits(void)
{
    int ret = -1;
    ret = *((unsigned int *)(axi_fpga_addr + TARGET_BITS));
    applog(LOG_DEBUG,"%s: TARGET_BITS is 0x%x\n", __FUNCTION__, ret);
    return ret;
}

void set_target_bits(unsigned int value)
{
    *((unsigned int *)(axi_fpga_addr + TARGET_BITS)) = value;
    applog(LOG_DEBUG,"%s: set TARGET_BITS is 0x%x\n", __FUNCTION__, value);
    get_target_bits();
}

int get_pre_header_hash(unsigned int *buf)
{
    int ret = -1;

    *(buf + 0) = *((unsigned int *)(axi_fpga_addr + PRE_HEADER_HASH));
    *(buf + 1) = *((unsigned int *)(axi_fpga_addr + PRE_HEADER_HASH + 1));
    *(buf + 2) = *((unsigned int *)(axi_fpga_addr + PRE_HEADER_HASH + 2));
    *(buf + 3) = *((unsigned int *)(axi_fpga_addr + PRE_HEADER_HASH + 3));
    *(buf + 4) = *((unsigned int *)(axi_fpga_addr + PRE_HEADER_HASH + 4));
    *(buf + 5) = *((unsigned int *)(axi_fpga_addr + PRE_HEADER_HASH + 5));
    *(buf + 6) = *((unsigned int *)(axi_fpga_addr + PRE_HEADER_HASH + 6));
    *(buf + 7) = *((unsigned int *)(axi_fpga_addr + PRE_HEADER_HASH + 7));

    applog(LOG_DEBUG,"%s: PRE_HEADER_HASH buf[0]: 0x%x, buf[1]: 0x%x, buf[2]: 0x%x, buf[3]: 0x%x, buf[4]: 0x%x, buf[5]: 0x%x, buf[6]: 0x%x, buf[7]: 0x%x\n", __FUNCTION__, *(buf + 0), *(buf + 1), *(buf + 2), *(buf + 3), *(buf + 4), *(buf + 5), *(buf + 6), *(buf + 7));
    ret = *(buf + 7);
    return ret;
}

void set_pre_header_hash(unsigned int *value)
{
    *(axi_fpga_addr + PRE_HEADER_HASH) = *(value + 0);
    *(axi_fpga_addr + PRE_HEADER_HASH + 1) = *(value + 1);
    *(axi_fpga_addr + PRE_HEADER_HASH + 2) = *(value + 2);
    *(axi_fpga_addr + PRE_HEADER_HASH + 3) = *(value + 3);
    *(axi_fpga_addr + PRE_HEADER_HASH + 4) = *(value + 4);
    *(axi_fpga_addr + PRE_HEADER_HASH + 5) = *(value + 5);
    *(axi_fpga_addr + PRE_HEADER_HASH + 6) = *(value + 6);
    *(axi_fpga_addr + PRE_HEADER_HASH + 7) = *(value + 7);

    applog(LOG_DEBUG,"%s: set PRE_HEADER_HASH value[0]: 0x%x, value[1]: 0x%x, value[2]: 0x%x, value[3]: 0x%x, value[4]: 0x%x, value[5]: 0x%x, value[6]: 0x%x, value[7]: 0x%x\n", __FUNCTION__, *(value + 0), *(value + 1), *(value + 2), *(value + 3), *(value + 4), *(value + 5), *(value + 6), *(value + 7));
    //get_pre_header_hash(buf);
}

int get_coinbase_length_and_nonce2_length(void)
{
    int ret = -1;
    ret = *((unsigned int *)(axi_fpga_addr + COINBASE_AND_NONCE2_LENGTH));
    applog(LOG_DEBUG,"%s: COINBASE_AND_NONCE2_LENGTH is 0x%x\n", __FUNCTION__, ret);
    return ret;
}

void set_coinbase_length_and_nonce2_length(unsigned int value)
{
    *((unsigned int *)(axi_fpga_addr + COINBASE_AND_NONCE2_LENGTH)) = value;
    applog(LOG_DEBUG,"%s: set COINBASE_AND_NONCE2_LENGTH is 0x%x\n", __FUNCTION__, value);
    get_coinbase_length_and_nonce2_length();
}

int get_work_nonce2(unsigned int *buf)
{
    int ret = -1;
    *(buf + 0) = *((unsigned int *)(axi_fpga_addr + WORK_NONCE_2));
    *(buf + 1) = *((unsigned int *)(axi_fpga_addr + WORK_NONCE_2 + 1));
    applog(LOG_DEBUG,"%s: WORK_NONCE_2 buf[0]: 0x%x, buf[1]: 0x%x\n", __FUNCTION__, *(buf + 0), *(buf + 1));
    return ret;
}

void set_work_nonce2(unsigned int *value)
{
    unsigned int buf[2] = {0};
    *((unsigned int *)(axi_fpga_addr + WORK_NONCE_2)) = *(value + 0);
    *((unsigned int *)(axi_fpga_addr + WORK_NONCE_2 + 1)) = *(value + 1);
    applog(LOG_DEBUG,"%s: set WORK_NONCE_2 value[0]: 0x%x, value[1]: 0x%x\n", __FUNCTION__, *(value + 0), *(value + 1));
    get_work_nonce2(buf);
}

int get_merkle_bin_number(void)
{
    int ret = -1;
    ret = *((unsigned int *)(axi_fpga_addr + MERKLE_BIN_NUMBER));
    ret = ret & 0x0000ffff;
    applog(LOG_DEBUG,"%s: MERKLE_BIN_NUMBER is 0x%x\n", __FUNCTION__, ret);
    return ret;
}

void set_merkle_bin_number(unsigned int value)
{
    *((unsigned int *)(axi_fpga_addr + MERKLE_BIN_NUMBER)) = value & 0x0000ffff;
    applog(LOG_DEBUG,"%s: set MERKLE_BIN_NUMBER is 0x%x\n", __FUNCTION__, value & 0x0000ffff);
    get_merkle_bin_number();
}

int get_nonce_fifo_interrupt(void)
{
    int ret = -1;
    ret = *((unsigned int *)(axi_fpga_addr + NONCE_FIFO_INTERRUPT));
    //applog(LOG_DEBUG,"%s: NONCE_FIFO_INTERRUPT is 0x%x\n", __FUNCTION__, ret);
    return ret;
}

void set_nonce_fifo_interrupt(unsigned int value)
{
    *((unsigned int *)(axi_fpga_addr + NONCE_FIFO_INTERRUPT)) = value;
    //applog(LOG_DEBUG,"%s: set NONCE_FIFO_INTERRUPT is 0x%x\n", __FUNCTION__, value);
    get_nonce_fifo_interrupt();
}

int get_dhash_acc_control(void)
{
    int ret = -1;
    ret = *((unsigned int *)(axi_fpga_addr + DHASH_ACC_CONTROL));
    applog(LOG_DEBUG,"%s: DHASH_ACC_CONTROL is 0x%x\n", __FUNCTION__, ret);
    return ret;
}

void set_dhash_acc_control(unsigned int value)
{
    *((unsigned int *)(axi_fpga_addr + DHASH_ACC_CONTROL)) = value;
    applog(LOG_DEBUG,"%s: set DHASH_ACC_CONTROL is 0x%x\n", __FUNCTION__, value);
    get_dhash_acc_control();
}

void set_TW_write_command(unsigned int *value)
{
    unsigned int i;
    for(i=0; i<TW_WRITE_COMMAND_LEN/sizeof(unsigned int); i++)
    {
        *((unsigned int *)(axi_fpga_addr + TW_WRITE_COMMAND + i)) = *(value + i);       //this is for FIL
        //applog(LOG_DEBUG,"%s: set TW_WRITE_COMMAND value[%d]: 0x%x\n", __FUNCTION__, i, *(value + i));
    }
    //applog(LOG_DEBUG,"%s: set TW_WRITE_COMMAND value[0]: 0x%x, value[1]: 0x%x, value[2]: 0x%x, value[3]: 0x%x\n", __FUNCTION__, *(value + 0), *(value + 1), *(value + 2), *(value + 3));
}

void set_TW_write_command_vil(unsigned int *value)
{
    unsigned int i;
	pthread_mutex_lock(&fpga_mutex);
    for(i=0; i<TW_WRITE_COMMAND_LEN_VIL/sizeof(unsigned int); i++)
    {
    	if(i==0)
			*((unsigned int *)(axi_fpga_addr + TW_WRITE_COMMAND + i)) = *(value + i);       //this is for VIL
        else *((unsigned int *)(axi_fpga_addr + TW_WRITE_COMMAND + 1)) = *(value + i);       //this is for VIL
        //applog(LOG_DEBUG,"%s: set TW_WRITE_COMMAND value[%d]: 0x%x\n", __FUNCTION__, i, *(value + i));
    }
	pthread_mutex_unlock(&fpga_mutex);
}

int get_buffer_space(void)
{
    int ret = -1;

    ret = *((unsigned int *)(axi_fpga_addr + BUFFER_SPACE));
    //applog(LOG_DEBUG,"%s: work fifo ready is 0x%x\n", __FUNCTION__, ret);
    return ret;
}

int get_hash_counting_number(void)
{
    int ret = -1;
    ret = *((unsigned int *)(axi_fpga_addr + HASH_COUNTING_NUMBER_FPGA));
    applog(LOG_DEBUG,"%s: DHASH_ACC_CONTROL is 0x%x\n", __FUNCTION__, ret);
    return ret;
}

void set_hash_counting_number(unsigned int value)
{
    *((unsigned int *)(axi_fpga_addr + HASH_COUNTING_NUMBER_FPGA)) = value;
    applog(LOG_DEBUG,"%s: set DHASH_ACC_CONTROL is 0x%x\n", __FUNCTION__, value);
    get_hash_counting_number();
}

int get_chain_number(int chainIndex)
{
    int j = 0, i;

    for(i=0; i < BITMAIN_MAX_CHAIN_NUM; i++)	// MUST use i from 0 to loop here !!!!
    {
        if(cgpu.chain_exist[i])
        {
			if(i==chainIndex)
				return j;
			j++;
        }
    }

	return -1;
}


//application related
void check_chain()
{
	char logstr[256];
    int ret = 0, i;
	
    cgpu.chain_num = 0;

    ret = get_hash_on_plug();

#ifdef DEBUG_TEST_ONE_BOARD_ONLY
	ret=0x01000007;	// for T9+ , only test J4
	//ret=0x040001C0;	// for T9+ , only test J2
	//ret=0x02000038;	// for T9+ , only test J3
#endif
	sprintf(logstr,"get PLUG ON=0x%08x\n",(unsigned int)ret);
	writeLogFile(logstr);

    if(ret < 0)
    {
        applog(LOG_DEBUG,"%s: get_hash_on_plug functions error\n",__FUNCTION__);
    }
    else
    {
#ifdef T9_18
//		if(fpga_version>=0xE)
//		{
//			ret>>24;	// bit 24 25 ... 31 is T9+ plug on signal
//		}
//		else
		{
			for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
	        {
	            if((ret >> i) & 0x1)
	            {
#ifdef DEBUG_MODE
					if(cgpu.chain_num>=DEBUG_BOARD_NUM)
					{
						sprintf(logstr,"Find hashboard on Chain[%d], Debug Mode ignore!!!\n",i);
						writeLogFile(logstr);
					
						cgpu.chain_exist[i] = 0;
						continue;
					}
#endif
	                cgpu.chain_exist[i] = 1;
	                cgpu.chain_num++;

	                sprintf(logstr,"Find hashboard on Chain[%d]\n",i);
					writeLogFile(logstr);
	            }
	            else
	            {
	                cgpu.chain_exist[i] = 0;
	            }
	        }
		}
#else
        for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
        {
            if((ret >> i) & 0x1)
            {
#ifdef DEBUG_MODE
				if(cgpu.chain_num>=DEBUG_BOARD_NUM)
				{
					sprintf(logstr,"Find hashboard on Chain[%d], Debug Mode ignore!!!\n",i);
					writeLogFile(logstr);
				
					cgpu.chain_exist[i] = 0;
					continue;
				}
#endif

                cgpu.chain_exist[i] = 1;
                cgpu.chain_num++;

                sprintf(logstr,"Find hashboard on Chain[%d]\n",i);
				writeLogFile(logstr);
            }
            else
            {
                cgpu.chain_exist[i] = 0;
            }
        }
#endif
    }
}

void clear_register_value_buf()
{
    pthread_mutex_lock(&reg_mutex);
    reg_value_buf->p_wr = 0;
    reg_value_buf->p_rd = 0;
    reg_value_buf->reg_value_num = 0;
    reg_value_buf->loop_back = 0;
    memset(reg_value_buf->reg_buffer, 0, sizeof(struct reg_content)*MAX_NONCE_NUMBER_IN_FIFO);
	pthread_mutex_unlock(&reg_mutex);
}

void read_asic_register(unsigned char chain, unsigned char mode, unsigned char chip_addr, unsigned char reg_addr)
{
    unsigned char buf[9] = {0};
    unsigned int cmd_buf[3] = {0,0,0};
    unsigned int ret, value;

    if(cgpu.CommandMode)    // fil mode
    {
        buf[0] = GET_STATUS;
        buf[1] = chip_addr;
        buf[2] = reg_addr;
        if (mode)   //all
            buf[0] |= COMMAND_FOR_ALL;
        buf[3] = CRC5(buf, 4*8 - 5);
        //applog(LOG_DEBUG,"%s: buf[0]=0x%x, buf[1]=0x%x, buf[2]=0x%x, buf[3]=0x%x\n", __FUNCTION__, buf[0], buf[1], buf[2], buf[3]);

        cmd_buf[0] = buf[0]<<24 | buf[1]<<16 | buf[2]<<8 | buf[3];
        set_BC_command_buffer(cmd_buf);

        ret = get_BC_write_command();
        value = BC_COMMAND_BUFFER_READY | BC_COMMAND_EN_CHAIN_ID | (chain << 16) | (ret & 0xfff0ffff);
        set_BC_write_command(value);
    }
    else    // vil mode
    {
        buf[0] = VIL_COMMAND_TYPE | GET_STATUS;
        if(mode)
            buf[0] |= VIL_ALL;
        buf[1] = 0x05;
        buf[2] = chip_addr;
        buf[3] = reg_addr;
        buf[4] = CRC5(buf, 4*8);
        //applog(LOG_DEBUG,"%s: buf[0]=0x%x, buf[1]=0x%x, buf[2]=0x%x, buf[3]=0x%x, buf[4]=0x%x\n", __FUNCTION__, buf[0], buf[1], buf[2], buf[3], buf[4]);

        cmd_buf[0] = buf[0]<<24 | buf[1]<<16 | buf[2]<<8 | buf[3];
        cmd_buf[1] = buf[4]<<24;
        set_BC_command_buffer(cmd_buf);

        ret = get_BC_write_command();
        value = BC_COMMAND_BUFFER_READY | BC_COMMAND_EN_CHAIN_ID | (chain << 16) | (ret & 0xfff0ffff);
        set_BC_write_command(value);
    }
}


void check_asic_reg(unsigned char reg)
{
	int i;
    int j, not_reg_data_time=0;
    unsigned short reg_value_num=0;
    unsigned char reg_buf[7] = {0,0,0,0,0};
	int asic_index=0;
	char logstr[256];
	int reg_processed_counter=0;
	
    clear_register_value_buf();

    for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
    {
    	reg_processed_counter=0;
        if(cgpu.chain_exist[i] == 1 
#ifndef ALL_CHAIN_INTEST
			&& (!testDone[i])
#endif
			)
        {
		//	sprintf(logstr,"do check_asic_reg on Chain[%d]\n", i);
		//	writeLogFile(logstr);
	
            read_asic_register(i, 1, 0, reg);

		//	sprintf(logstr,"Done read_asic_register on Chain[%d]\n", i);
		//	writeLogFile(logstr);

			asic_index=0;

            while(not_reg_data_time < 18)    //if there is no register value for 3 times, we can think all asic return their address
            {
                pthread_mutex_lock(&reg_mutex);
                reg_value_num = reg_value_buf->reg_value_num;
                //applog(LOG_DEBUG,"%s: reg_value_num = %d\n", __FUNCTION__, reg_value_num);
                pthread_mutex_unlock(&reg_mutex);

                //applog(LOG_DEBUG,"%s: reg_value_buf->reg_value_num = 0x%x\n", __FUNCTION__, reg_value_num);

                if(reg_value_num > 0)
                {
                	reg_processed_counter+=reg_value_num;
                    not_reg_data_time = 0;

					if(reg_processed_counter>600)
					{
                    	sprintf(logstr,"Fatal Error: reg_processed_counter=%d on Chain[%d]\n", reg_processed_counter, i);
						writeLogFile(logstr);
						return;
					}
					
                    for(j = 0; j < reg_value_num; j++)
                    {
                        pthread_mutex_lock(&reg_mutex);
                        //applog(LOG_DEBUG,"%\n");
                        if(reg_value_buf->reg_buffer[reg_value_buf->p_rd].chain_number != i)
                        {
                        	reg_value_buf->p_rd++;
	                        reg_value_buf->reg_value_num--;
	                        if(reg_value_buf->p_rd >= MAX_NONCE_NUMBER_IN_FIFO)
	                        {
	                            reg_value_buf->p_rd = 0;
	                        }
							
                        	pthread_mutex_unlock(&reg_mutex);
                        //    applog(LOG_DEBUG,"%s: the return data is from chain%d, but it should be from chain%d\n", __FUNCTION__, reg_value_buf->reg_buffer[reg_value_buf->p_rd].chain_number, i);
                            continue;
                        }
                        //applog(LOG_DEBUG,"@\n");

                        memset(reg_buf,0,sizeof(reg_buf));
                        reg_buf[3] = (unsigned char)(reg_value_buf->reg_buffer[reg_value_buf->p_rd].reg_value & 0xff);
                        reg_buf[2] = (unsigned char)((reg_value_buf->reg_buffer[reg_value_buf->p_rd].reg_value >> 8) & 0xff);
                        reg_buf[1] = (unsigned char)((reg_value_buf->reg_buffer[reg_value_buf->p_rd].reg_value >> 16)& 0xff);
                        reg_buf[0] = (unsigned char)((reg_value_buf->reg_buffer[reg_value_buf->p_rd].reg_value >> 24)& 0xff);

#ifdef ENABLE_REGISTER_CRC_CHECK
					    if(CRC5(reg_buf, (REGISTER_DATA_LENGTH+3)*8-5) != reg_value_buf->reg_buffer[reg_value_buf->p_rd].crc)
						{
						//	applog(LOG_DEBUG,"%s: crc is 0x%x, but it should be 0x%x\n", __FUNCTION__, CRC5(reg_buf, (REGISTER_DATA_LENGTH+1)*8-5), reg_value_buf->reg_buffer[reg_value_buf->p_rd].crc);
							reg_value_buf->p_rd++;
	                        reg_value_buf->reg_value_num--;
	                        if(reg_value_buf->p_rd >= MAX_NONCE_NUMBER_IN_FIFO)
	                        {
	                            reg_value_buf->p_rd = 0;
	                        }
							
							pthread_mutex_unlock(&reg_mutex);
							continue;
						}
#endif
						if(reg == CHIP_ADDRESS)
                        {
                            cgpu.chain_asic_num[i]++;
                            //printf("%s: cgpu.chain_asic_num[%d] = %d\n", __FUNCTION__, i, cgpu.chain_asic_num[i]);
                        }

                        if(reg == PLL_PARAMETER)
                        {
                            sprintf(logstr,"chain[%d]: the asic[%d] freq is 0x%x\n", i, asic_index, reg_value_buf->reg_buffer[reg_value_buf->p_rd].reg_value);
							writeLogFile(logstr);
							
							asic_index++;
                        }

						if(reg == TICKET_MASK)
                        {
                        	if(reg_value_buf->reg_buffer[reg_value_buf->p_rd].reg_value != 0)	// must be 0 when searching freq
                        	{
                            	sprintf(logstr,"chain[%d]: the asic[%d] TICKET_MASK is 0x%x\n", i, asic_index, reg_value_buf->reg_buffer[reg_value_buf->p_rd].reg_value);
								writeLogFile(logstr);
                        	}
							
							asic_index++;
                        }

						if(reg == HASH_COUNTING_NUMBER)
						{
                            if(reg_value_buf->reg_buffer[reg_value_buf->p_rd].reg_value != 0)	// must be 0 when searching freq
                        	{
                            	sprintf(logstr,"chain[%d]: the asic[%d] HCN is 0x%x\n", i, asic_index, reg_value_buf->reg_buffer[reg_value_buf->p_rd].reg_value);
								writeLogFile(logstr);
                        	}
							
							asic_index++;
                        }
						
                        reg_value_buf->p_rd++;
                        reg_value_buf->reg_value_num--;
                        if(reg_value_buf->p_rd >= MAX_NONCE_NUMBER_IN_FIFO)
                        {
                            reg_value_buf->p_rd = 0;
                        }
                        //applog(LOG_DEBUG,"%s: reg_value_buf->reg_value_num = %d\n", __FUNCTION__, reg_value_buf->reg_value_num);
                        pthread_mutex_unlock(&reg_mutex);
                    }

				//	sprintf(logstr,"Done reg_value_num=%d on Chain[%d]\n", reg_value_num, i);
				//	writeLogFile(logstr);
                }
                else
                {
                    usleep(100000);
                    not_reg_data_time++;
                    //applog(LOG_DEBUG,"%s: no asic address register come back for %d time.\n", __FUNCTION__, not_reg_data_time);
                }
            }

            not_reg_data_time = 0;

            if(reg == CHIP_ADDRESS)
            {
                if(cgpu.chain_asic_num[i] > cgpu.max_asic_num_in_one_chain)
                {
                    cgpu.max_asic_num_in_one_chain = cgpu.chain_asic_num[i];
                }
                applog(LOG_DEBUG,"%s: chain J%d has %d ASIC\n", __FUNCTION__, i+1, cgpu.chain_asic_num[i]);
            }

            //set_nonce_fifo_interrupt(get_nonce_fifo_interrupt() & ~(FLUSH_NONCE3_FIFO));
            clear_register_value_buf();
        }
    }
}

void check_asic_reg_oneChain(int chainIndex, unsigned char reg)
{
	int i;
    int j, not_reg_data_time=0;
    unsigned short reg_value_num=0;
    unsigned char reg_buf[7] = {0};
	int asic_index=0;
	char logstr[256];
	int reg_processed_counter=0;
	
    clear_register_value_buf();
	i=chainIndex;
    {
    	reg_processed_counter=0;
        if(cgpu.chain_exist[i] == 1
#ifndef ALL_CHAIN_INTEST
			&& (!testDone[i])
#endif
			)
        {
		//	sprintf(logstr,"do check_asic_reg on Chain[%d]\n", i);
		//	writeLogFile(logstr);
	
            read_asic_register(i, 1, 0, reg);

		//	sprintf(logstr,"Done read_asic_register on Chain[%d]\n", i);
		//	writeLogFile(logstr);

			asic_index=0;

            while(not_reg_data_time < 18)    //if there is no register value for 3 times, we can think all asic return their address
            {
                pthread_mutex_lock(&reg_mutex);
                reg_value_num = reg_value_buf->reg_value_num;
                //applog(LOG_DEBUG,"%s: reg_value_num = %d\n", __FUNCTION__, reg_value_num);
                pthread_mutex_unlock(&reg_mutex);

                //applog(LOG_DEBUG,"%s: reg_value_buf->reg_value_num = 0x%x\n", __FUNCTION__, reg_value_num);

                if(reg_value_num > 0)
                {
                	reg_processed_counter+=reg_value_num;
                    not_reg_data_time = 0;

					if(reg_processed_counter>600)
					{
                    	sprintf(logstr,"Fatal Error: reg_processed_counter=%d on Chain[%d]\n", reg_processed_counter, i);
						writeLogFile(logstr);
						return;
					}
					
                    for(j = 0; j < reg_value_num; j++)
                    {
                        pthread_mutex_lock(&reg_mutex);
                        //applog(LOG_DEBUG,"%\n");
                        if(reg_value_buf->reg_buffer[reg_value_buf->p_rd].chain_number != i)
                        {
                        	reg_value_buf->p_rd++;
	                        reg_value_buf->reg_value_num--;
	                        if(reg_value_buf->p_rd >= MAX_NONCE_NUMBER_IN_FIFO)
	                        {
	                            reg_value_buf->p_rd = 0;
	                        }
							
                        	pthread_mutex_unlock(&reg_mutex);
                        //    applog(LOG_DEBUG,"%s: the return data is from chain%d, but it should be from chain%d\n", __FUNCTION__, reg_value_buf->reg_buffer[reg_value_buf->p_rd].chain_number, i);
                            continue;
                        }
                        //applog(LOG_DEBUG,"@\n");

                        memset(reg_buf,0,sizeof(reg_buf));
                        reg_buf[3] = (unsigned char)(reg_value_buf->reg_buffer[reg_value_buf->p_rd].reg_value & 0xff);
                        reg_buf[2] = (unsigned char)((reg_value_buf->reg_buffer[reg_value_buf->p_rd].reg_value >> 8) & 0xff);
                        reg_buf[1] = (unsigned char)((reg_value_buf->reg_buffer[reg_value_buf->p_rd].reg_value >> 16)& 0xff);
                        reg_buf[0] = (unsigned char)((reg_value_buf->reg_buffer[reg_value_buf->p_rd].reg_value >> 24)& 0xff);
#ifdef ENABLE_REGISTER_CRC_CHECK
					    if(CRC5(reg_buf, (REGISTER_DATA_LENGTH+3)*8-5) != reg_value_buf->reg_buffer[reg_value_buf->p_rd].crc)
						{
						//	applog(LOG_DEBUG,"%s: crc is 0x%x, but it should be 0x%x\n", __FUNCTION__, CRC5(reg_buf, (REGISTER_DATA_LENGTH+1)*8-5), reg_value_buf->reg_buffer[reg_value_buf->p_rd].crc);
							reg_value_buf->p_rd++;
	                        reg_value_buf->reg_value_num--;
	                        if(reg_value_buf->p_rd >= MAX_NONCE_NUMBER_IN_FIFO)
	                        {
	                            reg_value_buf->p_rd = 0;
	                        }
							
							pthread_mutex_unlock(&reg_mutex);
							continue;
						}
#endif
						if(reg == CHIP_ADDRESS)
                        {
                            cgpu.chain_asic_num[i]++;
                            //printf("%s: cgpu.chain_asic_num[%d] = %d\n", __FUNCTION__, i, cgpu.chain_asic_num[i]);
                        }

                        if(reg == PLL_PARAMETER)
                        {
                            applog(LOG_DEBUG,"%s: the asic freq is 0x%x\n", __FUNCTION__, reg_value_buf->reg_buffer[reg_value_buf->p_rd].reg_value);
							asic_index++;
                        }
						
                        reg_value_buf->p_rd++;
                        reg_value_buf->reg_value_num--;
                        if(reg_value_buf->p_rd >= MAX_NONCE_NUMBER_IN_FIFO)
                        {
                            reg_value_buf->p_rd = 0;
                        }
                        //applog(LOG_DEBUG,"%s: reg_value_buf->reg_value_num = %d\n", __FUNCTION__, reg_value_buf->reg_value_num);
                        pthread_mutex_unlock(&reg_mutex);
                    }

				//	sprintf(logstr,"Done reg_value_num=%d on Chain[%d]\n", reg_value_num, i);
				//	writeLogFile(logstr);
                }
                else
                {
                    usleep(100000);
                    not_reg_data_time++;
                    //applog(LOG_DEBUG,"%s: no asic address register come back for %d time.\n", __FUNCTION__, not_reg_data_time);
                }
            }

            not_reg_data_time = 0;

            if(reg == CHIP_ADDRESS)
            {
                if(cgpu.chain_asic_num[i] > cgpu.max_asic_num_in_one_chain)
                {
                    cgpu.max_asic_num_in_one_chain = cgpu.chain_asic_num[i];
                }
                applog(LOG_DEBUG,"%s: chain J%d has %d ASIC\n", __FUNCTION__, i+1, cgpu.chain_asic_num[i]);
            }

            //set_nonce_fifo_interrupt(get_nonce_fifo_interrupt() & ~(FLUSH_NONCE3_FIFO));
            clear_register_value_buf();
        }
    }
}


unsigned int check_one_asic_reg(int chainIndex, unsigned char reg, unsigned char addr)
{
    int i, j, not_reg_data_time=0;
    unsigned int reg_value_num=0;
    unsigned int ret = 0x80000000;
    unsigned char reg_buf[7] = {0,0,0,0,0};
	int reg_processed_counter=0;
	char logstr[256];
	
    clear_register_value_buf();

	i=chainIndex;
//    for(i=0; i < BITMAIN_MAX_CHAIN_NUM; i++)
    {
    	reg_processed_counter=0;
		
        if(cgpu.chain_exist[i] == 1
#ifndef ALL_CHAIN_INTEST
			&& (!testDone[i])
#endif
			)
        {
            //applog(LOG_DEBUG,"%s: check chain J%d, ASIC address is 0x%x\n", __FUNCTION__, i+1, addr);
            read_asic_register(i, 0, addr, reg);

            while(not_reg_data_time < 4)    //if there is no register value for 2 times, we can think all asic return their register value
            {
                pthread_mutex_lock(&reg_mutex);
                reg_value_num = reg_value_buf->reg_value_num;
                //applog(LOG_DEBUG,"%s: reg_value_num = %d\n", __FUNCTION__, reg_value_num);
                pthread_mutex_unlock(&reg_mutex);

                //applog(LOG_DEBUG,"%s: reg_value_buf->reg_value_num = 0x%x\n", __FUNCTION__, reg_value_num);

                if(reg_value_num > 0)
                {
                    reg_processed_counter+=reg_value_num;
                    not_reg_data_time = 0;

					if(reg_processed_counter>300)
					{
                    	sprintf(logstr,"Fatal Error: reg_processed_counter=%d on Chain[%d]\n", reg_processed_counter, i);
						writeLogFile(logstr);
						return -1;
					}

                    //applog(LOG_DEBUG,"%s: reg_value_buf->reg_value_num = %d\n", __FUNCTION__, reg_value_num);

                    for(j = 0; j < reg_value_num; j++)
                    {
                        pthread_mutex_lock(&reg_mutex);
                        //applog(LOG_DEBUG,"%\n");
                        if(reg_value_buf->reg_buffer[reg_value_buf->p_rd].chain_number != i)
                        {
                        	pthread_mutex_unlock(&reg_mutex);
                        //    applog(LOG_DEBUG,"%s: the return data is from chain%d, but it should be from chain%d\n", __FUNCTION__, reg_value_buf->reg_buffer[reg_value_buf->p_rd].chain_number, i);
                            continue;
                        }
                        //applog(LOG_DEBUG,"@\n");

                        memset(reg_buf,0,sizeof(reg_buf));
                        reg_buf[3] = (unsigned char)(reg_value_buf->reg_buffer[reg_value_buf->p_rd].reg_value & 0xff);
                        reg_buf[2] = (unsigned char)((reg_value_buf->reg_buffer[reg_value_buf->p_rd].reg_value >> 8) & 0xff);
                        reg_buf[1] = (unsigned char)((reg_value_buf->reg_buffer[reg_value_buf->p_rd].reg_value >> 16)& 0xff);
                        reg_buf[0] = (unsigned char)((reg_value_buf->reg_buffer[reg_value_buf->p_rd].reg_value >> 24)& 0xff);
                        
                        //applog(LOG_DEBUG,"$\n");
                        //applog(LOG_DEBUG,"%s: reg_value = 0x%x\n", __FUNCTION__, reg_value_buf->reg_buffer[reg_value_buf->p_rd].reg_value);
                        ret = reg_value_buf->reg_buffer[reg_value_buf->p_rd].reg_value;

                        reg_value_buf->p_rd++;
                        reg_value_buf->reg_value_num--;
                        if(reg_value_buf->p_rd >= MAX_NONCE_NUMBER_IN_FIFO)
                        {
                            reg_value_buf->p_rd = 0;
                        }
                        //applog(LOG_DEBUG,"%s: reg_value_buf->reg_value_num = %d\n", __FUNCTION__, reg_value_buf->reg_value_num);
                        pthread_mutex_unlock(&reg_mutex);

                        if(reg == CHIP_ADDRESS)
                        {
                         //   applog(LOG_DEBUG,"%s: the asic addr is 0x%x\n", __FUNCTION__, ret);
                        }

                        if(reg == PLL_PARAMETER)
                        {
                         //   applog(LOG_DEBUG,"%s: the asic freq is 0x%x\n", __FUNCTION__, ret);
                        }

                        if(reg == GENERAL_I2C_COMMAND)
                        {
                            //applog(LOG_DEBUG,"%s: the asic general i2c command is 0x%x\n", __FUNCTION__, ret);
                        }
                    }
                }
                else
                {
                    usleep(10000);
                    not_reg_data_time++;
                    //applog(LOG_DEBUG,"%s: no asic address register come back for %d time.\n", __FUNCTION__, not_reg_data_time);
                }
            }

            not_reg_data_time = 0;

            //set_nonce_fifo_interrupt(get_nonce_fifo_interrupt() & ~(FLUSH_NONCE3_FIFO));
            clear_register_value_buf();

            return ret;
        }
    }
	return 0;
}

void reset_fpga()
{
	char logstr[256];
	set_QN_write_data_command(RESET_FPGA | RESET_HASH_BOARD | RESET_ALL | RESET_TIME(15));
#ifdef USE_NEW_RESET_FPGA
	sleep(2);
#else
	while(get_QN_write_data_command() & RESET_HASH_BOARD)
    {
        usleep(10000);
    }
	usleep(500000);
#endif

#ifndef ENABLE_TEMP_PROCESS
	set_PWM(FAN_SPEED_PWM);	// after reset , must set fan speed!!!!
#endif

#ifdef T9_18
	// config fpga into T9+ mode
	set_Hardware_version(0x80000000);
#endif
}

void reset_hashboard()
{
	set_QN_write_data_command(RESET_HASH_BOARD | RESET_ALL | RESET_TIME(15));
	//set_QN_write_data_command(RESET_FPGA | RESET_HASH_BOARD | RESET_ALL | RESET_TIME(15));
    while(get_QN_write_data_command() & RESET_HASH_BOARD)
    {
        usleep(10000);
    }
	sleep(1);
}

void reset_one_hashboard(int chainIndex)
{
	set_QN_write_data_command(RESET_HASH_BOARD | CHAIN_ID(chainIndex) | RESET_TIME(15));
	//set_QN_write_data_command(RESET_FPGA | RESET_HASH_BOARD | RESET_ALL | RESET_TIME(15));
    while(get_QN_write_data_command() & RESET_HASH_BOARD)
    {
        usleep(10000);
    }
	sleep(1);
}

void bitmain_set_pic_voltage(unsigned int voltage)
{
    unsigned char buf[9] = {0};
    unsigned int cmd_buf[3] = {0,0,0};
    unsigned int ret, value;
    unsigned char n=0,j;
	int i;

    n = (char)voltage;
    printf("%s: n = %d\n", __FUNCTION__, n);

    buf[0] = 0xab;
    buf[1] = 0xb0;
    buf[2] = n;
    buf[3] = CRC5(buf, 4*8 - 5);
    buf[3] |=0xc0;
    printf("set_pic_voltage buf[0]: 0x%x, buf[1]: 0x%x, buf[2]: 0x%x, buf[3]: 0x%x\n", buf[0], buf[1], buf[2], buf[3]);

    cmd_buf[0] = buf[0]<<24 | buf[1]<<16 | buf[2]<<8 | buf[3];

    for(j=0; j<3; j++)  // send 3 times
    {
        for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
        {
            if(cgpu.chain_exist[i] == 1)
            {
                set_BC_command_buffer(cmd_buf);
                ret = get_BC_write_command();
                value = BC_COMMAND_BUFFER_READY | BC_COMMAND_EN_CHAIN_ID | (i << 16) | (ret & 0xfff0ffff);
                set_BC_write_command(value);
            }
        }
    }
}

void change_time_format(struct tm *time, unsigned char *buf)
{
    *(buf + 0) = (unsigned char)(1900+time->tm_year-2000)/10*16 + (unsigned char)(1900+time->tm_year-2000)%10;
    *(buf + 1) = (unsigned char)(1+time->tm_mon)/10*16 + (unsigned char)(1+time->tm_mon)%10;
    *(buf + 2) = (unsigned char)(time->tm_mday)/10*16 + (unsigned char)(time->tm_mday)%10;
    *(buf + 3) = (unsigned char)(time->tm_hour)/10*16 + (unsigned char)(time->tm_hour)%10;
    *(buf + 4) = (unsigned char)(time->tm_min)/10*16 + (unsigned char)(time->tm_min)%10;
    *(buf + 5) = (unsigned char)(time->tm_sec)/10*16 + (unsigned char)(time->tm_sec)%10;
    printf("%s: year = 0x%02x\n", __FUNCTION__, *(buf + 0));
    printf("%s: month = 0x%02x\n", __FUNCTION__, *(buf + 1));
    printf("%s: day = 0x%02x\n", __FUNCTION__, *(buf + 2));
    printf("%s: hour = 0x%02x\n", __FUNCTION__, *(buf + 3));
    printf("%s: minute = 0x%02x\n", __FUNCTION__, *(buf + 4));
    printf("%s: second = 0x%02x\n", __FUNCTION__, *(buf + 5));
}

static void get_plldata(int type,int freq,uint32_t * reg_data,uint16_t * reg_data2, uint32_t *vil_data)
{
    uint32_t i;
    char freq_str[10];
    sprintf(freq_str,"%d", freq);
    char plldivider1[32] = {0};
    char plldivider2[32] = {0};
    char vildivider[32] = {0};

    if(type == 1385 || type == 1387)
    {
        for(i=0; i < sizeof(freq_pll_1385)/sizeof(freq_pll_1385[0]); i++)
        {
            if( memcmp(freq_pll_1385[i].freq, freq_str, sizeof(freq_pll_1385[i].freq)) == 0)
                break;
        }
    }

    //printf("%s: i = %d, sizeof(freq_pll_1385)/sizeof(freq_pll_1385[0]) = %d\n", __FUNCTION__, i, sizeof(freq_pll_1385)/sizeof(freq_pll_1385[0]));

    if(i == sizeof(freq_pll_1385)/sizeof(freq_pll_1385[0]))
    {
        printf("Freq set Err!!!!\n");
        printf("Using 200M\n");
        i = 4;
    }

    sprintf(plldivider1, "%08x", freq_pll_1385[i].fildiv1);
    sprintf(plldivider2, "%04x", freq_pll_1385[i].fildiv2);
    sprintf(vildivider, "%04x", freq_pll_1385[i].vilpll);

    printf("Freq %s, PLL1 %s, PLL2 %s, vilpll %s\n", freq_str, plldivider1, plldivider2, vildivider);

    *reg_data = freq_pll_1385[i].fildiv1;
    *reg_data2 = freq_pll_1385[i].fildiv2;
    *vil_data = freq_pll_1385[i].vilpll;

    printf("PLL1 %#x, PLL2 %#x, vilpll %#x\n",*reg_data, *reg_data2, *vil_data);
}

void set_frequency_with_addr_plldatai(int pllindex,unsigned char mode,unsigned char addr, unsigned char chain)
{
    unsigned char buf[9] = {0,0,0,0,0,0,0,0,0};
    unsigned int cmd_buf[3] = {0,0,0};
    int i;
    unsigned int ret, value;
    uint32_t reg_data_pll = 0;
    uint16_t reg_data_pll2 = 0;
    uint32_t reg_data_vil = 0;

    i = chain;

    reg_data_vil = freq_pll_1385[pllindex].vilpll;;

    //applog(LOG_DEBUG,"%s: i = %d\n", __FUNCTION__, i);
    if(cgpu.CommandMode)  // fil mode
    {
        memset(buf,0,sizeof(buf));
        memset(cmd_buf,0,sizeof(cmd_buf));
        buf[0] = 0;
        buf[0] |= SET_PLL_DIVIDER1;
        buf[1] = (reg_data_pll >> 16) & 0xff;
        buf[2] = (reg_data_pll >> 8) & 0xff;
        buf[3] = (reg_data_pll >> 0) & 0xff;
        buf[3] |= CRC5(buf, 4*8 - 5);
        cmd_buf[0] = buf[0]<<24 | buf[1]<<16 | buf[2]<<8 | buf[3];

        set_BC_command_buffer(cmd_buf);
        ret = get_BC_write_command();
        value = BC_COMMAND_BUFFER_READY | BC_COMMAND_EN_CHAIN_ID| (i << 16) | (ret & 0xfff0ffff);
        set_BC_write_command(value);

        usleep(3000);

        memset(buf,0,sizeof(buf));
        memset(cmd_buf,0,sizeof(cmd_buf));
        buf[0] = SET_PLL_DIVIDER2;
        buf[0] |= COMMAND_FOR_ALL;
        buf[1] = 0;     //addr
        buf[2] = reg_data_pll2 >> 8;
        buf[3] = reg_data_pll2& 0x0ff;
        buf[3] |= CRC5(buf, 4*8 - 5);
        cmd_buf[0] = buf[0]<<24 | buf[1]<<16 | buf[2]<<8 | buf[3];

        set_BC_command_buffer(cmd_buf);
        ret = get_BC_write_command();
        value = BC_COMMAND_BUFFER_READY | BC_COMMAND_EN_CHAIN_ID| (i << 16) | (ret & 0xfff0ffff);
        set_BC_write_command(value);

        usleep(5000);
    }
    else    // vil
    {
        memset(buf,0,9);
        memset(cmd_buf,0,3*sizeof(int));
        if(mode)
        {
            buf[0] = VIL_COMMAND_TYPE | VIL_ALL | SET_CONFIG;
        }
        else
        {
            buf[0] = VIL_COMMAND_TYPE | SET_CONFIG;
        }
		
        buf[1] = 0x09;
        buf[2] = addr;
        buf[3] = PLL_PARAMETER;
        buf[4] = (reg_data_vil >> 24) & 0xff;
        buf[5] = (reg_data_vil >> 16) & 0xff;
        buf[6] = (reg_data_vil >> 8) & 0xff;
        buf[7] = (reg_data_vil >> 0) & 0xff;
        buf[8] = CRC5(buf, 8*8);

        cmd_buf[0] = buf[0]<<24 | buf[1]<<16 | buf[2]<<8 | buf[3];
        cmd_buf[1] = buf[4]<<24 | buf[5]<<16 | buf[6]<<8 | buf[7];;
        cmd_buf[2] = buf[8]<<24;

        while (1)
        {
            ret = get_BC_write_command();
            if ((ret & 0x80000000) == 0)
                break;
            usleep(500);
        }
        set_BC_command_buffer(cmd_buf);
        value = BC_COMMAND_BUFFER_READY | BC_COMMAND_EN_CHAIN_ID| (i << 16) | (ret & 0xfff0ffff);
        set_BC_write_command(value);
    }
}


void set_frequency_with_addr(unsigned short int frequency,unsigned char mode,unsigned char addr, unsigned char chain)
{
    unsigned char buf[9] = {0,0,0,0,0,0,0,0,0};
    unsigned int cmd_buf[3] = {0,0,0};
    int i;
    unsigned int ret, value;
    uint32_t reg_data_pll = 0;
    uint16_t reg_data_pll2 = 0;
    uint32_t reg_data_vil = 0;
    i = chain;

    get_plldata(1385, frequency, &reg_data_pll, &reg_data_pll2, &reg_data_vil);


    //applog(LOG_DEBUG,"%s: i = %d\n", __FUNCTION__, i);
    if(cgpu.CommandMode)  // fil mode
    {
        memset(buf,0,sizeof(buf));
        memset(cmd_buf,0,sizeof(cmd_buf));
        buf[0] = 0;
        buf[0] |= SET_PLL_DIVIDER1;
        buf[1] = (reg_data_pll >> 16) & 0xff;
        buf[2] = (reg_data_pll >> 8) & 0xff;
        buf[3] = (reg_data_pll >> 0) & 0xff;
        buf[3] |= CRC5(buf, 4*8 - 5);
        cmd_buf[0] = buf[0]<<24 | buf[1]<<16 | buf[2]<<8 | buf[3];

        set_BC_command_buffer(cmd_buf);
        ret = get_BC_write_command();
        value = BC_COMMAND_BUFFER_READY | BC_COMMAND_EN_CHAIN_ID| (i << 16) | (ret & 0xfff0ffff);
        set_BC_write_command(value);

        usleep(3000);

        memset(buf,0,sizeof(buf));
        memset(cmd_buf,0,sizeof(cmd_buf));
        buf[0] = SET_PLL_DIVIDER2;
        buf[0] |= COMMAND_FOR_ALL;
        buf[1] = 0;     //addr
        buf[2] = reg_data_pll2 >> 8;
        buf[3] = reg_data_pll2& 0x0ff;
        buf[3] |= CRC5(buf, 4*8 - 5);
        cmd_buf[0] = buf[0]<<24 | buf[1]<<16 | buf[2]<<8 | buf[3];

        set_BC_command_buffer(cmd_buf);
        ret = get_BC_write_command();
        value = BC_COMMAND_BUFFER_READY | BC_COMMAND_EN_CHAIN_ID| (i << 16) | (ret & 0xfff0ffff);
        set_BC_write_command(value);

        usleep(5000);
    }
    else    // vil
    {
        memset(buf,0,9);
        memset(cmd_buf,0,3*sizeof(int));
        if(mode)
            buf[0] = VIL_COMMAND_TYPE | VIL_ALL | SET_CONFIG;
        else
            buf[0] = VIL_COMMAND_TYPE | SET_CONFIG;
        buf[1] = 0x09;
        buf[2] = addr;
        buf[3] = PLL_PARAMETER;
        buf[4] = (reg_data_vil >> 24) & 0xff;
        buf[5] = (reg_data_vil >> 16) & 0xff;
        buf[6] = (reg_data_vil >> 8) & 0xff;
        buf[7] = (reg_data_vil >> 0) & 0xff;
        buf[8] = CRC5(buf, 8*8);

        cmd_buf[0] = buf[0]<<24 | buf[1]<<16 | buf[2]<<8 | buf[3];
        cmd_buf[1] = buf[4]<<24 | buf[5]<<16 | buf[6]<<8 | buf[7];;
        cmd_buf[2] = buf[8]<<24;

        while (1)
        {
            ret = get_BC_write_command();
            if ((ret & 0x80000000) == 0)
                break;
            usleep(500);
        }
        set_BC_command_buffer(cmd_buf);
        value = BC_COMMAND_BUFFER_READY | BC_COMMAND_EN_CHAIN_ID| (i << 16) | (ret & 0xfff0ffff);
        set_BC_write_command(value);
    }
}


void set_frequency(int chainIndex, unsigned short int frequency)
{
    unsigned char buf[9] = {0,0,0,0,0,0,0,0,0};
    unsigned int cmd_buf[3] = {0,0,0};
    int i;
    unsigned int ret, value;
    uint32_t reg_data_pll = 0;
    uint16_t reg_data_pll2 = 0;
    uint32_t reg_data_vil = 0;

    applog(LOG_DEBUG,"\n--- %s\n", __FUNCTION__);

    get_plldata(ASIC_TYPE, frequency, &reg_data_pll, &reg_data_pll2, &reg_data_vil);
    applog(LOG_DEBUG,"%s: frequency = %d\n", __FUNCTION__, frequency);

    //for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
    i=chainIndex;
    {
        if(cgpu.chain_exist[i] == 1)
        {
            //applog(LOG_DEBUG,"%s: i = %d\n", __FUNCTION__, i);
            if(cgpu.CommandMode)    // fil mode
            {
                memset(buf,0,sizeof(buf));
                memset(cmd_buf,0,sizeof(cmd_buf));
                buf[0] = 0;
                buf[0] |= SET_PLL_DIVIDER1;
                buf[1] = (reg_data_pll >> 16) & 0xff;
                buf[2] = (reg_data_pll >> 8) & 0xff;
                buf[3] = (reg_data_pll >> 0) & 0xff;
                buf[3] |= CRC5(buf, 4*8 - 5);
                cmd_buf[0] = buf[0]<<24 | buf[1]<<16 | buf[2]<<8 | buf[3];

                set_BC_command_buffer(cmd_buf);
                ret = get_BC_write_command();
                value = BC_COMMAND_BUFFER_READY | BC_COMMAND_EN_CHAIN_ID| (i << 16) | (ret & 0xfff0ffff);
                set_BC_write_command(value);

                usleep(3000);

                memset(buf,0,sizeof(buf));
                memset(cmd_buf,0,sizeof(cmd_buf));
                buf[0] = SET_PLL_DIVIDER2;
                buf[0] |= COMMAND_FOR_ALL;
                buf[1] = 0;     //addr
                buf[2] = reg_data_pll2 >> 8;
                buf[3] = reg_data_pll2& 0x0ff;
                buf[3] |= CRC5(buf, 4*8 - 5);
                cmd_buf[0] = buf[0]<<24 | buf[1]<<16 | buf[2]<<8 | buf[3];

                set_BC_command_buffer(cmd_buf);
                ret = get_BC_write_command();
                value = BC_COMMAND_BUFFER_READY | BC_COMMAND_EN_CHAIN_ID| (i << 16) | (ret & 0xfff0ffff);
                set_BC_write_command(value);

                cgpu.freq[i] = frequency;

                usleep(5000);
            }
            else    // vil
            {
                memset(buf,0,9);
                memset(cmd_buf,0,3*sizeof(int));
                buf[0] = VIL_COMMAND_TYPE | VIL_ALL | SET_CONFIG;
                buf[1] = 0x09;
                buf[2] = 0;
                buf[3] = PLL_PARAMETER;
                buf[4] = (reg_data_vil >> 24) & 0xff;
                buf[5] = (reg_data_vil >> 16) & 0xff;
                buf[6] = (reg_data_vil >> 8) & 0xff;
                buf[7] = (reg_data_vil >> 0) & 0xff;
                buf[8] = CRC5(buf, 8*8);

                cmd_buf[0] = buf[0]<<24 | buf[1]<<16 | buf[2]<<8 | buf[3];
                cmd_buf[1] = buf[4]<<24 | buf[5]<<16 | buf[6]<<8 | buf[7];;
                cmd_buf[2] = buf[8]<<24;
                printf("%s: cmd_buf[0] = 0x%x, cmd_buf[1] = 0x%x, cmd_buf[2] = 0x%x\n", __FUNCTION__, cmd_buf[0], cmd_buf[1], cmd_buf[2]);

                set_BC_command_buffer(cmd_buf);
                ret = get_BC_write_command();
                value = BC_COMMAND_BUFFER_READY | BC_COMMAND_EN_CHAIN_ID| (i << 16) | (ret & 0xfff0ffff);
                set_BC_write_command(value);

                cgpu.freq[i] = frequency;
                usleep(10000);
            }
        }
    }
}

int calculate_asic_number(unsigned int actual_asic_number)
{
    int i = 0;
    if(actual_asic_number == 1)
    {
        i = 1;
    }
    else if(actual_asic_number == 2)
    {
        i = 2;
    }
    else if((actual_asic_number > 2) && (actual_asic_number <= 4))
    {
        i = 4;
    }
    else if((actual_asic_number > 4) && (actual_asic_number <= 8))
    {
        i = 8;
    }
    else if((actual_asic_number > 8) && (actual_asic_number <= 16))
    {
        i = 16;
    }
    else if((actual_asic_number > 16) && (actual_asic_number <= 32))
    {
        i = 32;
    }
    else if((actual_asic_number > 32) && (actual_asic_number <= 64))
    {
	        i = 64;	
    }
    else if((actual_asic_number > 64) && (actual_asic_number <= 128))
    {
        i = 128;
    }
    else
    {
        applog(LOG_DEBUG,"actual_asic_number = %d, but it is error\n", actual_asic_number);
        return -1;
    }
    return i;
}

int calculate_core_number(unsigned int actual_core_number)
{
    int i = 0;
    if(actual_core_number == 1)
    {
        i = 1;
    }
    else if(actual_core_number == 2)
    {
        i = 2;
    }
    else if((actual_core_number > 2) && (actual_core_number <= 4))
    {
        i = 4;
    }
    else if((actual_core_number > 4) && (actual_core_number <= 8))
    {
        i = 8;
    }
    else if((actual_core_number > 8) && (actual_core_number <= 16))
    {
        i = 16;
    }
    else if((actual_core_number > 16) && (actual_core_number <= 32))
    {
        i = 32;
    }
    else if((actual_core_number > 32) && (actual_core_number <= 64))
    {
        i = 64;
    }
    else if((actual_core_number > 64) && (actual_core_number <= 128))
    {
        i = 128;
    }
    else
    {
        applog(LOG_DEBUG,"actual_core_number = %d, but it is error\n", actual_core_number);
        return -1;
    }
    return i;
}


void chain_inactive(unsigned char chain)
{
    unsigned char buf[5] = {0,0,0,0,5};
    unsigned int cmd_buf[3] = {0,0,0};
    unsigned int ret, value;

    if(cgpu.CommandMode)    // fil mode
    {
        buf[0] = CHAIN_INACTIVE | COMMAND_FOR_ALL;
        buf[1] = 0;
        buf[2] = 0;
        buf[3] = CRC5(buf, 4*8 - 5);
        applog(LOG_DEBUG,"%s: buf[0]=0x%x, buf[1]=0x%x, buf[2]=0x%x, buf[3]=0x%x\n", __FUNCTION__, buf[0], buf[1], buf[2], buf[3]);

        cmd_buf[0] = buf[0]<<24 | buf[1]<<16 | buf[2]<<8 | buf[3];
        set_BC_command_buffer(cmd_buf);

        ret = get_BC_write_command();
        value = BC_COMMAND_BUFFER_READY | BC_COMMAND_EN_CHAIN_ID | (chain << 16) | (ret & 0xfff0ffff);
        set_BC_write_command(value);
    }
    else    // vil mode
    {
        buf[0] = VIL_COMMAND_TYPE | VIL_ALL | CHAIN_INACTIVE;
        buf[1] = 0x05;
        buf[2] = 0;
        buf[3] = 0;
        buf[4] = CRC5(buf, 4*8);
        applog(LOG_DEBUG,"%s: buf[0]=0x%x, buf[1]=0x%x, buf[2]=0x%x, buf[3]=0x%x, buf[4]=0x%x\n", __FUNCTION__, buf[0], buf[1], buf[2], buf[3], buf[4]);

        cmd_buf[0] = buf[0]<<24 | buf[1]<<16 | buf[2]<<8 | buf[3];
        cmd_buf[1] = buf[4]<<24;
        set_BC_command_buffer(cmd_buf);

        ret = get_BC_write_command();
        value = BC_COMMAND_BUFFER_READY | BC_COMMAND_EN_CHAIN_ID | (chain << 16) | (ret & 0xfff0ffff);
        set_BC_write_command(value);
    }
}


void set_address(unsigned char chain, unsigned char mode, unsigned char address)
{
    unsigned char buf[9] = {0};
    unsigned int cmd_buf[3] = {0,0,0};
    unsigned int ret, value;

    if(cgpu.CommandMode)    // fil mode
    {
        buf[0] = SET_ADDRESS;
        buf[1] = address;
        buf[2] = 0;
        if (mode)   //all
            buf[0] |= COMMAND_FOR_ALL;
        buf[3] = CRC5(buf, 4*8 - 5);
        //applog(LOG_DEBUG,"%s: buf[0]=0x%x, buf[1]=0x%x, buf[2]=0x%x, buf[3]=0x%x\n", __FUNCTION__, buf[0], buf[1], buf[2], buf[3]);

        cmd_buf[0] = buf[0]<<24 | buf[1]<<16 | buf[2]<<8 | buf[3];
        set_BC_command_buffer(cmd_buf);

        ret = get_BC_write_command();
        value = BC_COMMAND_BUFFER_READY | BC_COMMAND_EN_CHAIN_ID | (chain << 16) | (ret & 0xfff0ffff);
        set_BC_write_command(value);
    }
    else    // vil mode
    {
        buf[0] = VIL_COMMAND_TYPE | SET_ADDRESS;
        buf[1] = 0x05;
        buf[2] = address;
        buf[3] = 0;
        buf[4] = CRC5(buf, 4*8);
        //applog(LOG_DEBUG,"%s: buf[0]=0x%x, buf[1]=0x%x, buf[2]=0x%x, buf[3]=0x%x, buf[4]=0x%x\n", __FUNCTION__, buf[0], buf[1], buf[2], buf[3], buf[4]);

        cmd_buf[0] = buf[0]<<24 | buf[1]<<16 | buf[2]<<8 | buf[3];
        cmd_buf[1] = buf[4]<<24;
        set_BC_command_buffer(cmd_buf);

        ret = get_BC_write_command();
        value = BC_COMMAND_BUFFER_READY | BC_COMMAND_EN_CHAIN_ID | (chain << 16) | (ret & 0xfff0ffff);
        set_BC_write_command(value);
    }
}

void software_set_address_onChain(int chainIndex)
{
    int i, j;
    unsigned int chip_addr = 0;

    chip_addr = 0;
    chain_inactive(chainIndex);
    usleep(5000);

    for(j = 0; j < 0x100/CHIP_ADDR_INTERVAL; j++)
    {
        set_address(chainIndex, 0, chip_addr);
        chip_addr += CHIP_ADDR_INTERVAL;
        usleep(5000);
    }
}

void software_set_address()
{
    int i, j;
    unsigned int chip_addr = 0;

    for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
    {
        if(cgpu.chain_exist[i] == 1
#ifndef ALL_CHAIN_INTEST
			&& (!testDone[i])
#endif
			)
        {
            chip_addr = 0;
            chain_inactive(i);
            usleep(5000);

            for(j = 0; j < 0x100/CHIP_ADDR_INTERVAL; j++)
            {
                set_address(i, 0, chip_addr);
                chip_addr += CHIP_ADDR_INTERVAL;
                usleep(5000);
            }
        }
    }
}

void set_baud_onChain(int chainIndex, unsigned char bauddiv)
{
    unsigned char buf[9] = {0};
    unsigned int cmd_buf[3] = {0};
    unsigned int ret, value;
	int i;

    printf("\n--- %s\n", __FUNCTION__);

    if(cgpu.baud == bauddiv)
    {
        applog(LOG_DEBUG,"%s: the setting bauddiv(%d) is the same as before\n", __FUNCTION__, bauddiv);
        return;
    }

    i=chainIndex;
    {
        if(cgpu.chain_exist[i] == 1 
#ifndef ALL_CHAIN_INTEST
			&& (!testDone[i])
#endif
			)
        {
            //first step: send new bauddiv to ASIC, but FPGA doesn't change its bauddiv, it uses old bauddiv to send BC command to ASIC
            if(cgpu.CommandMode)    // fil mode
            {
                buf[0] = SET_BAUD_OPS;
                buf[1] = 0x10;
                buf[2] = bauddiv & 0x1f;
                buf[0] |= COMMAND_FOR_ALL;
                buf[3] = CRC5(buf, 4*8 - 5);
                applog(LOG_DEBUG,"%s: buf[0]=0x%x, buf[1]=0x%x, buf[2]=0x%x, buf[3]=0x%x\n", __FUNCTION__, buf[0], buf[1], buf[2], buf[3]);

                cmd_buf[0] = buf[0]<<24 | buf[1]<<16 | buf[2]<<8 | buf[3];
                set_BC_command_buffer(cmd_buf);

                ret = get_BC_write_command();
                value = BC_COMMAND_BUFFER_READY | BC_COMMAND_EN_CHAIN_ID | (i << 16) | (ret & 0xfff0ffff);
                set_BC_write_command(value);
            }
            else    // vil mode
            {
                /* 20160510
                buf[0] = VIL_COMMAND_TYPE | VIL_ALL | SET_CONFIG;
                buf[1] = 0x09;
                buf[2] = 0;
                buf[3] = MISC_CONTROL;
                buf[4] = 0x40;
                buf[5] = INV_CLKO | cgpu.temp_sel;
                buf[6] = (bauddiv & 0x1f) | (cgpu.rfs << 6);
                buf[7] = cgpu.tfs << 5;
                buf[8] = CRC5(buf, 8*8);
                */

                buf[0] = VIL_COMMAND_TYPE | VIL_ALL | SET_CONFIG;
                buf[1] = 0x09;
                buf[2] = 0;
                buf[3] = MISC_CONTROL;
                buf[4] = 0x40;
                buf[5] = INV_CLKO;
                buf[6] = (bauddiv & 0x1f);
                buf[7] = 0;
                buf[8] = CRC5(buf, 8*8);

                cmd_buf[0] = buf[0]<<24 | buf[1]<<16 | buf[2]<<8 | buf[3];
                cmd_buf[1] = buf[4]<<24 | buf[5]<<16 | buf[6]<<8 | buf[7];
                cmd_buf[2] = buf[8]<<24;
                printf("%s: cmd_buf[0]=0x%x, cmd_buf[1]=0x%x, cmd_buf[2]=0x%x\n", __FUNCTION__, cmd_buf[0], cmd_buf[1], cmd_buf[2]);

                set_BC_command_buffer(cmd_buf);
                ret = get_BC_write_command();
                value = BC_COMMAND_BUFFER_READY | BC_COMMAND_EN_CHAIN_ID| (i << 16) | (ret & 0xfff0ffff);
                set_BC_write_command(value);
            }
        }
    }

    // second step: change FPGA's bauddiv
    usleep(50000);
    ret = get_BC_write_command();
    value = (ret & 0xffffffe0) | (bauddiv & 0x1f);
    set_BC_write_command(value);
    cgpu.baud = bauddiv;
    printf("%s: system baudrate is: 0x%x\n", __FUNCTION__, cgpu.baud);
}


void set_baud(unsigned char bauddiv)
{
    unsigned char buf[9] = {0};
    unsigned int cmd_buf[3] = {0};
    unsigned int ret, value;
	int i;

    printf("\n--- %s\n", __FUNCTION__);

    if(cgpu.baud == bauddiv)
    {
        applog(LOG_DEBUG,"%s: the setting bauddiv(%d) is the same as before\n", __FUNCTION__, bauddiv);
        return;
    }

    for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
    {
        if(cgpu.chain_exist[i] == 1 
#ifndef ALL_CHAIN_INTEST
			&& (!testDone[i])
#endif
			)
        {
            //first step: send new bauddiv to ASIC, but FPGA doesn't change its bauddiv, it uses old bauddiv to send BC command to ASIC
            if(cgpu.CommandMode)    // fil mode
            {
                buf[0] = SET_BAUD_OPS;
                buf[1] = 0x10;
                buf[2] = bauddiv & 0x1f;
                buf[0] |= COMMAND_FOR_ALL;
                buf[3] = CRC5(buf, 4*8 - 5);
                applog(LOG_DEBUG,"%s: buf[0]=0x%x, buf[1]=0x%x, buf[2]=0x%x, buf[3]=0x%x\n", __FUNCTION__, buf[0], buf[1], buf[2], buf[3]);

                cmd_buf[0] = buf[0]<<24 | buf[1]<<16 | buf[2]<<8 | buf[3];
                set_BC_command_buffer(cmd_buf);

                ret = get_BC_write_command();
                value = BC_COMMAND_BUFFER_READY | BC_COMMAND_EN_CHAIN_ID | (i << 16) | (ret & 0xfff0ffff);
                set_BC_write_command(value);
            }
            else    // vil mode
            {
                /* 20160510
                buf[0] = VIL_COMMAND_TYPE | VIL_ALL | SET_CONFIG;
                buf[1] = 0x09;
                buf[2] = 0;
                buf[3] = MISC_CONTROL;
                buf[4] = 0x40;
                buf[5] = INV_CLKO | cgpu.temp_sel;
                buf[6] = (bauddiv & 0x1f) | (cgpu.rfs << 6);
                buf[7] = cgpu.tfs << 5;
                buf[8] = CRC5(buf, 8*8);
                */

                buf[0] = VIL_COMMAND_TYPE | VIL_ALL | SET_CONFIG;
                buf[1] = 0x09;
                buf[2] = 0;
                buf[3] = MISC_CONTROL;
                buf[4] = 0x40;
                buf[5] = INV_CLKO;
                buf[6] = (bauddiv & 0x1f);
                buf[7] = 0;
                buf[8] = CRC5(buf, 8*8);

                cmd_buf[0] = buf[0]<<24 | buf[1]<<16 | buf[2]<<8 | buf[3];
                cmd_buf[1] = buf[4]<<24 | buf[5]<<16 | buf[6]<<8 | buf[7];
                cmd_buf[2] = buf[8]<<24;
                printf("%s: cmd_buf[0]=0x%x, cmd_buf[1]=0x%x, cmd_buf[2]=0x%x\n", __FUNCTION__, cmd_buf[0], cmd_buf[1], cmd_buf[2]);

                set_BC_command_buffer(cmd_buf);
                ret = get_BC_write_command();
                value = BC_COMMAND_BUFFER_READY | BC_COMMAND_EN_CHAIN_ID| (i << 16) | (ret & 0xfff0ffff);
                set_BC_write_command(value);
            }
        }
    }

    // second step: change FPGA's bauddiv
    usleep(50000);
    ret = get_BC_write_command();
    value = (ret & 0xffffffe0) | (bauddiv & 0x1f);
    set_BC_write_command(value);
    cgpu.baud = bauddiv;
    printf("%s: system baudrate is: 0x%x\n", __FUNCTION__, cgpu.baud);
}

void open_core_onChain(int chainIndex, int coreNum, int opencore_num, bool nullwork_enable)
{
	int i;
    unsigned int j = 0, m, work_id = 0, ret = 0, value = 0, work_fifo_ready = 0, loop=0;
    unsigned char gateblk[4] = {0,0,0,0};
    unsigned int cmd_buf[3] = {0,0,0}, buf[TW_WRITE_COMMAND_LEN/sizeof(unsigned int)]= {0};
    unsigned int buf_vil_tw[TW_WRITE_COMMAND_LEN_VIL/sizeof(unsigned int)]= {0};
    unsigned char data[TW_WRITE_COMMAND_LEN] = {0xff};
    unsigned char buf_vil[9] = {0};
    struct vil_work work_vil;
    struct vil_work_1387 work_vil_1387;
    unsigned int OpenCoreNum1 = conf.OpenCoreNum1;
    unsigned int OpenCoreNum2 = conf.OpenCoreNum2;
    unsigned int OpenCoreNum3 = conf.OpenCoreNum3;
    unsigned int OpenCoreNum4 = conf.OpenCoreNum4;

    printf("\n--- %s\n", __FUNCTION__);

    set_dhash_acc_control(get_dhash_acc_control() & (~OPERATION_MODE));
    set_hash_counting_number(0);

    loop = coreNum;

    if(cgpu.CommandMode)    // fil mode
    {
        gateblk[0] = SET_BAUD_OPS;
        gateblk[1] = 0;//0x10; //16-23
        gateblk[2] = cgpu.baud | 0x80; //8-15 gateblk=1
        gateblk[0] |= 0x80;
        //gateblk[3] = CRC5(gateblk, 4*8 - 5);
        gateblk[3] = 0x80;  // MMEN=1
        gateblk[3] = 0x80 | (0x1f & CRC5(gateblk, 4*8 - 5));
        applog(LOG_DEBUG,"%s: gateblk[0]=0x%x, gateblk[1]=0x%x, gateblk[2]=0x%x, gateblk[3]=0x%x\n", __FUNCTION__, gateblk[0], gateblk[1], gateblk[2], gateblk[3]);
        cmd_buf[0] = gateblk[0]<<24 | gateblk[1]<<16 | gateblk[2]<<8 | gateblk[3];

        memset(data, 0x00, TW_WRITE_COMMAND_LEN);
        data[TW_WRITE_COMMAND_LEN - 1] = 0xff;
        data[TW_WRITE_COMMAND_LEN - 12] = 0xff;

//        for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
		i=chainIndex;
        {
            if(cgpu.chain_exist[i] == 1
#ifndef ALL_CHAIN_INTEST
				&& (!testDone[i])
#endif
				)
            {
                set_BC_command_buffer(cmd_buf);
                ret = get_BC_write_command();
                value = BC_COMMAND_BUFFER_READY | BC_COMMAND_EN_CHAIN_ID | (i << 16) | (ret & 0xfff0ffff);
                set_BC_write_command(value);
                usleep(10000);

                for(m=0; m<loop; m++)
                {
                    //applog(LOG_DEBUG,"%s: m = %d\n", __FUNCTION__, m);
                    if(ASIC_NUM == 1)
                    {
                        if(m < 32)
                        {
                            //printf("%s: m = %d, OpenCoreNum1 = %d\n", __FUNCTION__, m, OpenCoreNum1);
                            if( !(OpenCoreNum1 & 0x00000001) )
                            {
                                data[TW_WRITE_COMMAND_LEN - 1] = 0x0;
                                data[TW_WRITE_COMMAND_LEN - 12] = 0x0;
                                //printf("%s: Do not open core: %d\n", __FUNCTION__, m);
                            }
                            else
                            {
                                data[TW_WRITE_COMMAND_LEN - 1] = 0xff;
                                data[TW_WRITE_COMMAND_LEN - 12] = 0xff;
                                //printf("%s: open core: %d\n", __FUNCTION__, m);
                            }
                            OpenCoreNum1 = OpenCoreNum1 >> 1;
                        }
                        else if((m < 64) && (m >= 32))
                        {
                            //printf("%s: m = %d, OpenCoreNum2 = %d\n", __FUNCTION__, m, OpenCoreNum2);
                            if( !(OpenCoreNum2 & 0x00000001) )
                            {
                                data[TW_WRITE_COMMAND_LEN - 1] = 0x0;
                                data[TW_WRITE_COMMAND_LEN - 12] = 0x0;
                                //printf("%s: Do not open core: %d\n", __FUNCTION__, m);
                            }
                            else
                            {
                                data[TW_WRITE_COMMAND_LEN - 1] = 0xff;
                                data[TW_WRITE_COMMAND_LEN - 12] = 0xff;
                                //printf("%s: open core: %d\n", __FUNCTION__, m);
                            }
                            OpenCoreNum2 = OpenCoreNum2 >> 1;
                        }
                        else if((m < 96) && (m >= 64))
                        {
                            //printf("%s: m = %d, OpenCoreNum3 = %d\n", __FUNCTION__, m, OpenCoreNum3);
                            if( !(OpenCoreNum3 & 0x00000001) )
                            {
                                data[TW_WRITE_COMMAND_LEN - 1] = 0x0;
                                data[TW_WRITE_COMMAND_LEN - 12] = 0x0;
                                //printf("%s: Do not open core: %d\n", __FUNCTION__, m);
                            }
                            else
                            {
                                data[TW_WRITE_COMMAND_LEN - 1] = 0xff;
                                data[TW_WRITE_COMMAND_LEN - 12] = 0xff;
                                //printf("%s: open core: %d\n", __FUNCTION__, m);
                            }
                            OpenCoreNum3 = OpenCoreNum3 >> 1;
                        }
                        else if((m < 128) && (m >= 96))
                        {
                            //printf("%s: m = %d, OpenCoreNum4 = %d\n", __FUNCTION__, m, OpenCoreNum4);
                            if( !(OpenCoreNum4 & 0x00000001) )
                            {
                                data[TW_WRITE_COMMAND_LEN - 1] = 0x0;
                                data[TW_WRITE_COMMAND_LEN - 12] = 0x0;
                                //printf("%s: Do not open core: %d\n", __FUNCTION__, m);
                            }
                            else
                            {
                                data[TW_WRITE_COMMAND_LEN - 1] = 0xff;
                                data[TW_WRITE_COMMAND_LEN - 12] = 0xff;
                                //printf("%s: open core: %d\n", __FUNCTION__, m);
                            }
                            OpenCoreNum4 = OpenCoreNum4 >> 1;
                        }
                        else
                        {
                            ;
                        }
                    }

                    do
                    {
                        work_fifo_ready = get_buffer_space();
                        if(work_fifo_ready & (0x1 << i))
                        {
                            break;
                        }
                        else    //work fifo is full, wait for 50ms
                        {
                            //applog(LOG_DEBUG,"%s: chain%d work fifo not ready: 0x%x\n", __FUNCTION__, i, work_fifo_ready);
                            usleep(1000);
                        }
                    }
                    while(1);

                    if(m==0)    //new block
                    {
                        data[0] = NEW_BLOCK_MARKER;
                    }
                    else
                    {
                        data[0] = NORMAL_BLOCK_MARKER;
                    }

                    data[1] = i | 0x80; //set chain id and enable it

                    if(m==0)
                    {
                        ret = get_BC_write_command();   //disable null work
                        ret &= ~BC_COMMAND_EN_NULL_WORK;
                        set_BC_write_command(ret);
                    }

                    if(m==loop - 1 && nullwork_enable)
                    {
                        ret = get_BC_write_command();   //enable null work
                        ret &= (BC_COMMAND_EN_CHAIN_ID | BC_COMMAND_EN_NULL_WORK | ((i & 0xf) << 16));
                        set_BC_write_command(ret);
                    }

                    memset(buf, 0, TW_WRITE_COMMAND_LEN/sizeof(unsigned int));

                    /*
                    for(j=0; j<TW_WRITE_COMMAND_LEN; j++)
                    {
                        applog(LOG_DEBUG,"data[%d] = 0x%x\n", j, data[i]);
                    }
                    */

                    for(j=0; j<TW_WRITE_COMMAND_LEN/sizeof(unsigned int); j++)
                    {
                        buf[j] = (data[4*j + 0] << 24) | (data[4*j + 1] << 16) | (data[4*j + 2] << 8) | data[4*j + 3];
                        if(j==9)
                        {
                            buf[j] = work_id++;
                        }
                        //applog(LOG_DEBUG,"buf[%d] = 0x%x\n", j, buf[i]);
                    }

                    set_TW_write_command(buf);
                }
            }
        }
    }
    else    // vil mode
    {
        if(ASIC_TYPE == 1387)
        {

            // prepare gateblk
            buf_vil[0] = VIL_COMMAND_TYPE | VIL_ALL | SET_CONFIG;
            buf_vil[1] = 0x09;
            buf_vil[2] = 0;
            buf_vil[3] = MISC_CONTROL;
            if(ASIC_TYPE == 1387)
            {
                /* 20160510
                buf_vil[4] = 0x40;
                buf_vil[5] = INV_CLKO | cgpu.temp_sel;  // enable INV_CLKO
                buf_vil[6] = (cgpu.baud & 0x1f) | GATEBCLK | (cgpu.rfs << 6);   // enable gateblk
                buf_vil[7] = MMEN | cgpu.tfs << 5;  // MMEN=1
                */

                buf_vil[4] = 0x40;
                buf_vil[5] = INV_CLKO;  // enable INV_CLKO
                buf_vil[6] = (cgpu.baud & 0x1f) | GATEBCLK; // enable gateblk
                buf_vil[7] = MMEN;  // MMEN=1
            }
            else
            {
                buf_vil[4] = 0;
                buf_vil[5] = INV_CLKO;
                buf_vil[6] = (cgpu.baud & 0x1f) | GATEBCLK;
                buf_vil[7] = 0x80;  // MMEN=1
            }
            buf_vil[8] = CRC5(buf_vil, 8*8);

            cmd_buf[0] = buf_vil[0]<<24 | buf_vil[1]<<16 | buf_vil[2]<<8 | buf_vil[3];
            cmd_buf[1] = buf_vil[4]<<24 | buf_vil[5]<<16 | buf_vil[6]<<8 | buf_vil[7];
            cmd_buf[2] = buf_vil[8]<<24;
            printf("%s: gateblk: cmd_buf[0] = 0x%x, cmd_buf[1] = 0x%x, cmd_buf[2] = 0x%x\n", __FUNCTION__, cmd_buf[0], cmd_buf[1], cmd_buf[2]);

            // prepare special work for openning core
            memset(buf_vil_tw, 0x00, TW_WRITE_COMMAND_LEN_VIL/sizeof(unsigned int));
            memset(&work_vil_1387, 0xff, sizeof(struct vil_work_1387));
            work_vil_1387.work_type = NORMAL_BLOCK_MARKER;
            work_vil_1387.chain_id = 0x80 | 0;	// gChain is changed to 0, this chain_id will filled at below
            work_vil_1387.reserved1[0]= 0;
            work_vil_1387.reserved1[1]= 0;
            work_vil_1387.work_count = 0;
			
            //for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
            i=chainIndex;
            {
                if(cgpu.chain_exist[i] == 1
#ifndef ALL_CHAIN_INTEST
					&& (!testDone[i])
#endif
					)
                {
                	ret = get_BC_write_command();   //disable null work
		            ret = BC_COMMAND_EN_CHAIN_ID | (i << 16) | (ret & 0xfff0ffff);
					ret &= ~BC_COMMAND_EN_NULL_WORK;
		            set_BC_write_command(ret);
					usleep(10000);
					
                	set_BC_command_buffer(cmd_buf);
					
                    ret = get_BC_write_command();
                    value = BC_COMMAND_BUFFER_READY | BC_COMMAND_EN_CHAIN_ID | (i << 16) | (ret & 0xfff0ffff);
					ret &= ~BC_COMMAND_EN_NULL_WORK;
                    set_BC_write_command(value);
					usleep(10000);

                    for(m=0; m<loop; m++)
                    {
                    	if(m>=opencore_num)
                    	{
                    		work_vil_1387.data[0] = 0x0;
                            work_vil_1387.data[11] = 0x0;
                    	}
						else
						{
							work_vil_1387.data[0] = 0xff;
                            work_vil_1387.data[11] = 0xff;
						}
						
                        do
                        {
                            work_fifo_ready = get_buffer_space();
                            if(work_fifo_ready & (0x1 << i))
                            {
                                break;
                            }
                            else    //work fifo is full, wait for 50ms
                            {
                                //applog(LOG_DEBUG,"%s: chain%d work fifo not ready: 0x%x\n", __FUNCTION__, i, work_fifo_ready);
                                usleep(1000);
                            }
                        }
                        while(1);

                        if(m==0)    //new block
                        {
                            work_vil_1387.work_type = NEW_BLOCK_MARKER;
                        }
                        else
                        {
                            work_vil_1387.work_type = NORMAL_BLOCK_MARKER;
                        }

                        work_vil_1387.chain_id = i | 0x80; //set chain id and enable it

                        buf_vil_tw[0] = (work_vil_1387.work_type<< 24) | (work_vil_1387.chain_id << 16) | (work_vil_1387.reserved1[0] << 8) | work_vil_1387.reserved1[1];
                        buf_vil_tw[1] = work_vil_1387.work_count;
                        for(j=2; j<DATA2_LEN/sizeof(unsigned int)+2; j++)
                        {
                            buf_vil_tw[j] = (work_vil_1387.data[4*(j-2) + 0] << 24) | (work_vil_1387.data[4*(j-2) + 1] << 16) | (work_vil_1387.data[4*(j-2) + 2] << 8) | work_vil_1387.data[4*(j-2) + 3];
                        }
                        for(j=5; j<MIDSTATE_LEN/sizeof(unsigned int)+5; j++)
                        {
                            buf_vil_tw[j] = 0;
                        }

                        set_TW_write_command_vil(buf_vil_tw);

						if(m==loop - 1 && nullwork_enable)
			            {
			                ret = get_BC_write_command();   //enable null work
			                ret |= BC_COMMAND_EN_NULL_WORK;
			                set_BC_write_command(ret);
			            }
                    }
                }
            }
        }
        else
        {
            // prepare gateblk
            buf_vil[0] = VIL_COMMAND_TYPE | VIL_ALL | SET_CONFIG;
            buf_vil[1] = 0x09;
            buf_vil[2] = 0;
            buf_vil[3] = MISC_CONTROL;
            if(ASIC_TYPE == 1387)
            {
                buf_vil[4] = 0x40;
                buf_vil[5] = INV_CLKO;  // enable INV_CLKO
                buf_vil[6] = (cgpu.baud & 0x1f) | GATEBCLK; // enable gateblk
                buf_vil[7] = MMEN;  // MMEN=1
            }
            else
            {
                buf_vil[4] = 0;
                buf_vil[5] = INV_CLKO;
                buf_vil[6] = (cgpu.baud & 0x1f) | GATEBCLK;
                buf_vil[7] = 0x80;  // MMEN=1
            }
            buf_vil[8] = CRC5(buf_vil, 8*8);

            cmd_buf[0] = buf_vil[0]<<24 | buf_vil[1]<<16 | buf_vil[2]<<8 | buf_vil[3];
            cmd_buf[1] = buf_vil[4]<<24 | buf_vil[5]<<16 | buf_vil[6]<<8 | buf_vil[7];
            cmd_buf[2] = buf_vil[8]<<24;
  //          printf("%s: gateblk: cmd_buf[0] = 0x%x, cmd_buf[1] = 0x%x, cmd_buf[2] = 0x%x\n", __FUNCTION__, cmd_buf[0], cmd_buf[1], cmd_buf[2]);

            // prepare special work for openning core
            memset(&work_vil, 0, sizeof(struct vil_work));
            work_vil.type = 0x01 << 5;
            work_vil.length = sizeof(struct vil_work);
            work_vil.wc_base = 0;
            work_vil.mid_num = 1;
            //work_vil.sno = 0;
            work_vil.data2[0] = 0xff;
            work_vil.data2[11] = 0xff;

            memset(data, 0x00, TW_WRITE_COMMAND_LEN);
            memset(buf_vil_tw, 0x00, TW_WRITE_COMMAND_LEN_VIL/sizeof(unsigned int));

			ret = get_BC_write_command();   //disable null work
            ret &= ~BC_COMMAND_EN_NULL_WORK;
            set_BC_write_command(ret);
							
            //for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
            i=chainIndex;
            {
                if(cgpu.chain_exist[i] == 1 
#ifndef ALL_CHAIN_INTEST
					&& (!testDone[i])
#endif
					)
                {
                    set_BC_command_buffer(cmd_buf);
                    ret = get_BC_write_command();
                    value = BC_COMMAND_BUFFER_READY | BC_COMMAND_EN_CHAIN_ID | (i << 16) | (ret & 0xfff0ffff);
                    set_BC_write_command(value);
                    usleep(10000);

                    for(m=0; m<loop; m++)
                    {
                        //applog(LOG_DEBUG,"%s: m = %d\n", __FUNCTION__, m);
                        if(ASIC_NUM == 1)
                        {
                            if(m < 32)
                            {
                                //printk("%s: m = %d, OpenCoreNum1 = %d\n", __FUNCTION__, m, OpenCoreNum1);
                                if( !(OpenCoreNum1 & 0x00000001) )
                                {
                                    data[TW_WRITE_COMMAND_LEN - 1] = 0x0;
                                    data[TW_WRITE_COMMAND_LEN - 12] = 0x0;
                                    //printk("%s: Do not open core: %d\n", __FUNCTION__, m);
                                }
                                else
                                {
                                    data[TW_WRITE_COMMAND_LEN - 1] = 0xff;
                                    data[TW_WRITE_COMMAND_LEN - 12] = 0xff;
                                    //printk("%s: open core: %d\n", __FUNCTION__, m);
                                }
                                OpenCoreNum1 = OpenCoreNum1 >> 1;
                            }
                            else if((m < 64) && (m >= 32))
                            {
                                //printk("%s: m = %d, OpenCoreNum2 = %d\n", __FUNCTION__, m, OpenCoreNum2);
                                if( !(OpenCoreNum2 & 0x00000001) )
                                {
                                    data[TW_WRITE_COMMAND_LEN - 1] = 0x0;
                                    data[TW_WRITE_COMMAND_LEN - 12] = 0x0;
                                    //printk("%s: Do not open core: %d\n", __FUNCTION__, m);
                                }
                                else
                                {
                                    data[TW_WRITE_COMMAND_LEN - 1] = 0xff;
                                    data[TW_WRITE_COMMAND_LEN - 12] = 0xff;
                                    //printk("%s: open core: %d\n", __FUNCTION__, m);
                                }
                                OpenCoreNum2 = OpenCoreNum2 >> 1;
                            }
                            else if((m < 96) && (m >= 64))
                            {
                                //printk("%s: m = %d, OpenCoreNum3 = %d\n", __FUNCTION__, m, OpenCoreNum3);
                                if( !(OpenCoreNum3 & 0x00000001) )
                                {
                                    data[TW_WRITE_COMMAND_LEN - 1] = 0x0;
                                    data[TW_WRITE_COMMAND_LEN - 12] = 0x0;
                                    //printk("%s: Do not open core: %d\n", __FUNCTION__, m);
                                }
                                else
                                {
                                    data[TW_WRITE_COMMAND_LEN - 1] = 0xff;
                                    data[TW_WRITE_COMMAND_LEN - 12] = 0xff;
                                    //printk("%s: open core: %d\n", __FUNCTION__, m);
                                }
                                OpenCoreNum3 = OpenCoreNum3 >> 1;
                            }
                            else if((m < 128) && (m >= 96))
                            {
                                //printk("%s: m = %d, OpenCoreNum4 = %d\n", __FUNCTION__, m, OpenCoreNum4);
                                if( !(OpenCoreNum4 & 0x00000001) )
                                {
                                    data[TW_WRITE_COMMAND_LEN - 1] = 0x0;
                                    data[TW_WRITE_COMMAND_LEN - 12] = 0x0;
                                    //printk("%s: Do not open core: %d\n", __FUNCTION__, m);
                                }
                                else
                                {
                                    data[TW_WRITE_COMMAND_LEN - 1] = 0xff;
                                    data[TW_WRITE_COMMAND_LEN - 12] = 0xff;
                                    //printk("%s: open core: %d\n", __FUNCTION__, m);
                                }
                                OpenCoreNum4 = OpenCoreNum4 >> 1;
                            }
                            else
                            {
                                ;
                            }
                        }

                        do
                        {
                            work_fifo_ready = get_buffer_space();
                            if(work_fifo_ready & (0x1 << i))
                            {
                                break;
                            }
                            else    //work fifo is full, wait for 50ms
                            {
                                //applog(LOG_DEBUG,"%s: chain%d work fifo not ready: 0x%x\n", __FUNCTION__, i, work_fifo_ready);
                                usleep(1000);
                            }
                        }
                        while(1);

                        if(m==0)    //new block
                        {
                            data[0] = NEW_BLOCK_MARKER;
                        }
                        else
                        {
                            data[0] = NORMAL_BLOCK_MARKER;
                        }

                        data[1] = i | 0x80; //set chain id and enable it

                        buf_vil_tw[0] = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
                        buf_vil_tw[1] = (work_vil.type << 24) | (work_vil.length << 16) | (work_vil.wc_base++ << 8) | work_vil.mid_num;
                        //buf_vil_tw[2] = work_vil.sno;
                        for(j=2; j<MIDSTATE_LEN/sizeof(unsigned int)+2; j++)
                        {
                            buf_vil_tw[j] = 0;
                        }
                        for(j=10; j<DATA2_LEN/sizeof(unsigned int)+10; j++)
                        {
                            buf_vil_tw[j] = (work_vil.data2[4*(j-10) + 0] << 24) | (work_vil.data2[4*(j-10) + 1] << 16) | (work_vil.data2[4*(j-10) + 2] << 8) | work_vil.data2[4*(j-10) + 3];
                        }

                        set_TW_write_command_vil(buf_vil_tw);
                    }
                }
            }
        }
    }
    printf("--- %s end\n", __FUNCTION__);
}

void open_core(bool nullwork_enable)
{
	int i;
    unsigned int j = 0, m, work_id = 0, ret = 0, value = 0, work_fifo_ready = 0, loop=0;
    unsigned char gateblk[4] = {0,0,0,0};
    unsigned int cmd_buf[3] = {0,0,0}, buf[TW_WRITE_COMMAND_LEN/sizeof(unsigned int)]= {0};
    unsigned int buf_vil_tw[TW_WRITE_COMMAND_LEN_VIL/sizeof(unsigned int)]= {0};
    unsigned char data[TW_WRITE_COMMAND_LEN] = {0xff};
    unsigned char buf_vil[9] = {0};
    struct vil_work work_vil;
    struct vil_work_1387 work_vil_1387;
    unsigned int OpenCoreNum1 = conf.OpenCoreNum1;
    unsigned int OpenCoreNum2 = conf.OpenCoreNum2;
    unsigned int OpenCoreNum3 = conf.OpenCoreNum3;
    unsigned int OpenCoreNum4 = conf.OpenCoreNum4;

    printf("\n--- %s\n", __FUNCTION__);

    set_dhash_acc_control(get_dhash_acc_control() & (~OPERATION_MODE));
    set_hash_counting_number(0);

    loop = ASIC_CORE_NUM;

    if(cgpu.CommandMode)    // fil mode
    {
        gateblk[0] = SET_BAUD_OPS;
        gateblk[1] = 0;//0x10; //16-23
        gateblk[2] = cgpu.baud | 0x80; //8-15 gateblk=1
        gateblk[0] |= 0x80;
        //gateblk[3] = CRC5(gateblk, 4*8 - 5);
        gateblk[3] = 0x80;  // MMEN=1
        gateblk[3] = 0x80 | (0x1f & CRC5(gateblk, 4*8 - 5));
        applog(LOG_DEBUG,"%s: gateblk[0]=0x%x, gateblk[1]=0x%x, gateblk[2]=0x%x, gateblk[3]=0x%x\n", __FUNCTION__, gateblk[0], gateblk[1], gateblk[2], gateblk[3]);
        cmd_buf[0] = gateblk[0]<<24 | gateblk[1]<<16 | gateblk[2]<<8 | gateblk[3];

        memset(data, 0x00, TW_WRITE_COMMAND_LEN);
        data[TW_WRITE_COMMAND_LEN - 1] = 0xff;
        data[TW_WRITE_COMMAND_LEN - 12] = 0xff;

        for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
        {
            if(cgpu.chain_exist[i] == 1 
#ifndef ALL_CHAIN_INTEST
				&& (!testDone[i])
#endif
				)
            {
                set_BC_command_buffer(cmd_buf);
                ret = get_BC_write_command();
                value = BC_COMMAND_BUFFER_READY | BC_COMMAND_EN_CHAIN_ID | (i << 16) | (ret & 0xfff0ffff);
                set_BC_write_command(value);
                usleep(10000);

                for(m=0; m<loop; m++)
                {
                    //applog(LOG_DEBUG,"%s: m = %d\n", __FUNCTION__, m);
                    if(ASIC_NUM == 1)
                    {
                        if(m < 32)
                        {
                            //printf("%s: m = %d, OpenCoreNum1 = %d\n", __FUNCTION__, m, OpenCoreNum1);
                            if( !(OpenCoreNum1 & 0x00000001) )
                            {
                                data[TW_WRITE_COMMAND_LEN - 1] = 0x0;
                                data[TW_WRITE_COMMAND_LEN - 12] = 0x0;
                                //printf("%s: Do not open core: %d\n", __FUNCTION__, m);
                            }
                            else
                            {
                                data[TW_WRITE_COMMAND_LEN - 1] = 0xff;
                                data[TW_WRITE_COMMAND_LEN - 12] = 0xff;
                                //printf("%s: open core: %d\n", __FUNCTION__, m);
                            }
                            OpenCoreNum1 = OpenCoreNum1 >> 1;
                        }
                        else if((m < 64) && (m >= 32))
                        {
                            //printf("%s: m = %d, OpenCoreNum2 = %d\n", __FUNCTION__, m, OpenCoreNum2);
                            if( !(OpenCoreNum2 & 0x00000001) )
                            {
                                data[TW_WRITE_COMMAND_LEN - 1] = 0x0;
                                data[TW_WRITE_COMMAND_LEN - 12] = 0x0;
                                //printf("%s: Do not open core: %d\n", __FUNCTION__, m);
                            }
                            else
                            {
                                data[TW_WRITE_COMMAND_LEN - 1] = 0xff;
                                data[TW_WRITE_COMMAND_LEN - 12] = 0xff;
                                //printf("%s: open core: %d\n", __FUNCTION__, m);
                            }
                            OpenCoreNum2 = OpenCoreNum2 >> 1;
                        }
                        else if((m < 96) && (m >= 64))
                        {
                            //printf("%s: m = %d, OpenCoreNum3 = %d\n", __FUNCTION__, m, OpenCoreNum3);
                            if( !(OpenCoreNum3 & 0x00000001) )
                            {
                                data[TW_WRITE_COMMAND_LEN - 1] = 0x0;
                                data[TW_WRITE_COMMAND_LEN - 12] = 0x0;
                                //printf("%s: Do not open core: %d\n", __FUNCTION__, m);
                            }
                            else
                            {
                                data[TW_WRITE_COMMAND_LEN - 1] = 0xff;
                                data[TW_WRITE_COMMAND_LEN - 12] = 0xff;
                                //printf("%s: open core: %d\n", __FUNCTION__, m);
                            }
                            OpenCoreNum3 = OpenCoreNum3 >> 1;
                        }
                        else if((m < 128) && (m >= 96))
                        {
                            //printf("%s: m = %d, OpenCoreNum4 = %d\n", __FUNCTION__, m, OpenCoreNum4);
                            if( !(OpenCoreNum4 & 0x00000001) )
                            {
                                data[TW_WRITE_COMMAND_LEN - 1] = 0x0;
                                data[TW_WRITE_COMMAND_LEN - 12] = 0x0;
                                //printf("%s: Do not open core: %d\n", __FUNCTION__, m);
                            }
                            else
                            {
                                data[TW_WRITE_COMMAND_LEN - 1] = 0xff;
                                data[TW_WRITE_COMMAND_LEN - 12] = 0xff;
                                //printf("%s: open core: %d\n", __FUNCTION__, m);
                            }
                            OpenCoreNum4 = OpenCoreNum4 >> 1;
                        }
                        else
                        {
                            ;
                        }
                    }

                    do
                    {
                        work_fifo_ready = get_buffer_space();
                        if(work_fifo_ready & (0x1 << i))
                        {
                            break;
                        }
                        else    //work fifo is full, wait for 50ms
                        {
                            //applog(LOG_DEBUG,"%s: chain%d work fifo not ready: 0x%x\n", __FUNCTION__, i, work_fifo_ready);
                            usleep(1000);
                        }
                    }
                    while(1);

                    if(m==0)    //new block
                    {
                        data[0] = NEW_BLOCK_MARKER;
                    }
                    else
                    {
                        data[0] = NORMAL_BLOCK_MARKER;
                    }

                    data[1] = i | 0x80; //set chain id and enable it

                    if(m==0)
                    {
                        ret = get_BC_write_command();   //disable null work
                        ret &= ~BC_COMMAND_EN_NULL_WORK;
                        set_BC_write_command(ret);
                    }

                    if(m==loop - 1 && nullwork_enable)
                    {
                        ret = get_BC_write_command();   //enable null work
                        ret &= (BC_COMMAND_EN_CHAIN_ID | BC_COMMAND_EN_NULL_WORK | ((i & 0xf) << 16));
                        set_BC_write_command(ret);
                    }

                    memset(buf, 0, TW_WRITE_COMMAND_LEN/sizeof(unsigned int));

                    /*
                    for(j=0; j<TW_WRITE_COMMAND_LEN; j++)
                    {
                        applog(LOG_DEBUG,"data[%d] = 0x%x\n", j, data[i]);
                    }
                    */

                    for(j=0; j<TW_WRITE_COMMAND_LEN/sizeof(unsigned int); j++)
                    {
                        buf[j] = (data[4*j + 0] << 24) | (data[4*j + 1] << 16) | (data[4*j + 2] << 8) | data[4*j + 3];
                        if(j==9)
                        {
                            buf[j] = work_id++;
                        }
                        //applog(LOG_DEBUG,"buf[%d] = 0x%x\n", j, buf[i]);
                    }

                    set_TW_write_command(buf);
                }
            }
        }
    }
    else    // vil mode
    {
        if(ASIC_TYPE == 1387)
        {

            // prepare gateblk
            buf_vil[0] = VIL_COMMAND_TYPE | VIL_ALL | SET_CONFIG;
            buf_vil[1] = 0x09;
            buf_vil[2] = 0;
            buf_vil[3] = MISC_CONTROL;
            if(ASIC_TYPE == 1387)
            {
                /* 20160510
                buf_vil[4] = 0x40;
                buf_vil[5] = INV_CLKO | cgpu.temp_sel;  // enable INV_CLKO
                buf_vil[6] = (cgpu.baud & 0x1f) | GATEBCLK | (cgpu.rfs << 6);   // enable gateblk
                buf_vil[7] = MMEN | cgpu.tfs << 5;  // MMEN=1
                */

                buf_vil[4] = 0x40;
                buf_vil[5] = INV_CLKO;  // enable INV_CLKO
                buf_vil[6] = (cgpu.baud & 0x1f) | GATEBCLK; // enable gateblk
                buf_vil[7] = MMEN;  // MMEN=1
            }
            else
            {
                buf_vil[4] = 0;
                buf_vil[5] = INV_CLKO;
                buf_vil[6] = (cgpu.baud & 0x1f) | GATEBCLK;
                buf_vil[7] = 0x80;  // MMEN=1
            }
            buf_vil[8] = CRC5(buf_vil, 8*8);

            cmd_buf[0] = buf_vil[0]<<24 | buf_vil[1]<<16 | buf_vil[2]<<8 | buf_vil[3];
            cmd_buf[1] = buf_vil[4]<<24 | buf_vil[5]<<16 | buf_vil[6]<<8 | buf_vil[7];
            cmd_buf[2] = buf_vil[8]<<24;
            printf("%s: gateblk: cmd_buf[0] = 0x%x, cmd_buf[1] = 0x%x, cmd_buf[2] = 0x%x\n", __FUNCTION__, cmd_buf[0], cmd_buf[1], cmd_buf[2]);

            // prepare special work for openning core
            memset(buf_vil_tw, 0x00, TW_WRITE_COMMAND_LEN_VIL/sizeof(unsigned int));
            memset(&work_vil_1387, 0xaa, sizeof(struct vil_work_1387));
            work_vil_1387.work_type = NORMAL_BLOCK_MARKER;
            work_vil_1387.chain_id = 0x80 | 0;	// gChain is changed to 0, this chain_id will filled at below
            work_vil_1387.reserved1[0]= 0;
            work_vil_1387.reserved1[1]= 0;
            work_vil_1387.work_count = 0;
			
            for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
            {
                if(cgpu.chain_exist[i] == 1
#ifndef ALL_CHAIN_INTEST
					&& (!testDone[i])
#endif
					)
                {
                	ret = get_BC_write_command();   //disable null work
		            ret = BC_COMMAND_EN_CHAIN_ID | (i << 16) | (ret & 0xfff0ffff);
					ret &= ~BC_COMMAND_EN_NULL_WORK;
		            set_BC_write_command(ret);
					usleep(10000);
					
                	set_BC_command_buffer(cmd_buf);
					
                    ret = get_BC_write_command();
                    value = BC_COMMAND_BUFFER_READY | BC_COMMAND_EN_CHAIN_ID | (i << 16) | (ret & 0xfff0ffff);
					ret &= ~BC_COMMAND_EN_NULL_WORK;
                    set_BC_write_command(value);
					usleep(10000);

                    for(m=0; m<loop; m++)
                    {
                        //applog(LOG_DEBUG,"%s: m = %d\n", __FUNCTION__, m);
                        if(ASIC_NUM == 1)
                        {
                            if(m < 32)
                            {
                                //printk("%s: m = %d, OpenCoreNum1 = %d\n", __FUNCTION__, m, OpenCoreNum1);
                                if( !(OpenCoreNum1 & 0x00000001) )
                                {
                                    work_vil_1387.data[0] = 0x0;
                                    work_vil_1387.data[11] = 0x0;
                                    //printk("%s: Do not open core: %d\n", __FUNCTION__, m);
                                }
                                else
                                {
                                    work_vil_1387.data[0] = 0xff;
                                    work_vil_1387.data[11] = 0xff;
                                    //printk("%s: open core: %d\n", __FUNCTION__, m);
                                }
                                OpenCoreNum1 = OpenCoreNum1 >> 1;
                            }
                            else if((m < 64) && (m >= 32))
                            {
                                //printk("%s: m = %d, OpenCoreNum2 = %d\n", __FUNCTION__, m, OpenCoreNum2);
                                if( !(OpenCoreNum2 & 0x00000001) )
                                {
                                    work_vil_1387.data[0] = 0x0;
                                    work_vil_1387.data[11] = 0x0;
                                    //printk("%s: Do not open core: %d\n", __FUNCTION__, m);
                                }
                                else
                                {
                                    work_vil_1387.data[0] = 0xff;
                                    work_vil_1387.data[11] = 0xff;
                                    //printk("%s: open core: %d\n", __FUNCTION__, m);
                                }
                                OpenCoreNum2 = OpenCoreNum2 >> 1;
                            }
                            else if((m < 96) && (m >= 64))
                            {
                                //printk("%s: m = %d, OpenCoreNum3 = %d\n", __FUNCTION__, m, OpenCoreNum3);
                                if( !(OpenCoreNum3 & 0x00000001) )
                                {
                                    work_vil_1387.data[0] = 0x0;
                                    work_vil_1387.data[11] = 0x0;
                                    //printk("%s: Do not open core: %d\n", __FUNCTION__, m);
                                }
                                else
                                {
                                    work_vil_1387.data[0] = 0xff;
                                    work_vil_1387.data[11] = 0xff;
                                    //printk("%s: open core: %d\n", __FUNCTION__, m);
                                }
                                OpenCoreNum3 = OpenCoreNum3 >> 1;
                            }
                            else if((m < 128) && (m >= 96))
                            {
                                //printk("%s: m = %d, OpenCoreNum4 = %d\n", __FUNCTION__, m, OpenCoreNum4);
                                if( !(OpenCoreNum4 & 0x00000001) )
                                {
                                    work_vil_1387.data[0] = 0x0;
                                    work_vil_1387.data[11] = 0x0;
                                    //printk("%s: Do not open core: %d\n", __FUNCTION__, m);
                                }
                                else
                                {
                                    work_vil_1387.data[0] = 0xff;
                                    work_vil_1387.data[11] = 0xff;
                                    //printk("%s: open core: %d\n", __FUNCTION__, m);
                                }
                                OpenCoreNum4 = OpenCoreNum4 >> 1;
                            }
                            else
                            {
                                ;
                            }
                        }

                        do
                        {
                            work_fifo_ready = get_buffer_space();
                            if(work_fifo_ready & (0x1 << i))
                            {
                                break;
                            }
                            else    //work fifo is full, wait for 50ms
                            {
                                //applog(LOG_DEBUG,"%s: chain%d work fifo not ready: 0x%x\n", __FUNCTION__, i, work_fifo_ready);
                                usleep(1000);
                            }
                        }
                        while(1);

                        if(m==0)    //new block
                        {
                            work_vil_1387.work_type = NEW_BLOCK_MARKER;
                        }
                        else
                        {
                            work_vil_1387.work_type = NORMAL_BLOCK_MARKER;
                        }

                        work_vil_1387.chain_id = i | 0x80; //set chain id and enable it

                        buf_vil_tw[0] = (work_vil_1387.work_type<< 24) | (work_vil_1387.chain_id << 16) | (work_vil_1387.reserved1[0] << 8) | work_vil_1387.reserved1[1];
                        buf_vil_tw[1] = work_vil_1387.work_count;
                        for(j=2; j<DATA2_LEN/sizeof(unsigned int)+2; j++)
                        {
                            buf_vil_tw[j] = (work_vil_1387.data[4*(j-2) + 0] << 24) | (work_vil_1387.data[4*(j-2) + 1] << 16) | (work_vil_1387.data[4*(j-2) + 2] << 8) | work_vil_1387.data[4*(j-2) + 3];
                        }
                        for(j=5; j<MIDSTATE_LEN/sizeof(unsigned int)+5; j++)
                        {
                            buf_vil_tw[j] = 0;
                        }

                        set_TW_write_command_vil(buf_vil_tw);

						if(m==loop - 1 && nullwork_enable)
			            {
			                ret = get_BC_write_command();   //enable null work
			                ret |= BC_COMMAND_EN_NULL_WORK;
			                set_BC_write_command(ret);
			            }
                    }
                }
            }
        }
        else
        {
            // prepare gateblk
            buf_vil[0] = VIL_COMMAND_TYPE | VIL_ALL | SET_CONFIG;
            buf_vil[1] = 0x09;
            buf_vil[2] = 0;
            buf_vil[3] = MISC_CONTROL;
            if(ASIC_TYPE == 1387)
            {
                buf_vil[4] = 0x40;
                buf_vil[5] = INV_CLKO;  // enable INV_CLKO
                buf_vil[6] = (cgpu.baud & 0x1f) | GATEBCLK; // enable gateblk
                buf_vil[7] = MMEN;  // MMEN=1
            }
            else
            {
                buf_vil[4] = 0;
                buf_vil[5] = INV_CLKO;
                buf_vil[6] = (cgpu.baud & 0x1f) | GATEBCLK;
                buf_vil[7] = 0x80;  // MMEN=1
            }
            buf_vil[8] = CRC5(buf_vil, 8*8);

            cmd_buf[0] = buf_vil[0]<<24 | buf_vil[1]<<16 | buf_vil[2]<<8 | buf_vil[3];
            cmd_buf[1] = buf_vil[4]<<24 | buf_vil[5]<<16 | buf_vil[6]<<8 | buf_vil[7];
            cmd_buf[2] = buf_vil[8]<<24;
  //          printf("%s: gateblk: cmd_buf[0] = 0x%x, cmd_buf[1] = 0x%x, cmd_buf[2] = 0x%x\n", __FUNCTION__, cmd_buf[0], cmd_buf[1], cmd_buf[2]);

            // prepare special work for openning core
            memset(&work_vil, 0, sizeof(struct vil_work));
            work_vil.type = 0x01 << 5;
            work_vil.length = sizeof(struct vil_work);
            work_vil.wc_base = 0;
            work_vil.mid_num = 1;
            //work_vil.sno = 0;
            work_vil.data2[0] = 0xff;
            work_vil.data2[11] = 0xff;

            memset(data, 0x00, TW_WRITE_COMMAND_LEN);
            memset(buf_vil_tw, 0x00, TW_WRITE_COMMAND_LEN_VIL/sizeof(unsigned int));

			ret = get_BC_write_command();   //disable null work
            ret &= ~BC_COMMAND_EN_NULL_WORK;
            set_BC_write_command(ret);
							
            for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
            {
                if(cgpu.chain_exist[i] == 1 
#ifndef ALL_CHAIN_INTEST
					&& (!testDone[i])
#endif
					)
                {
                    set_BC_command_buffer(cmd_buf);
                    ret = get_BC_write_command();
                    value = BC_COMMAND_BUFFER_READY | BC_COMMAND_EN_CHAIN_ID | (i << 16) | (ret & 0xfff0ffff);
                    set_BC_write_command(value);
                    usleep(10000);

                    for(m=0; m<loop; m++)
                    {
                        //applog(LOG_DEBUG,"%s: m = %d\n", __FUNCTION__, m);
                        if(ASIC_NUM == 1)
                        {
                            if(m < 32)
                            {
                                //printk("%s: m = %d, OpenCoreNum1 = %d\n", __FUNCTION__, m, OpenCoreNum1);
                                if( !(OpenCoreNum1 & 0x00000001) )
                                {
                                    data[TW_WRITE_COMMAND_LEN - 1] = 0x0;
                                    data[TW_WRITE_COMMAND_LEN - 12] = 0x0;
                                    //printk("%s: Do not open core: %d\n", __FUNCTION__, m);
                                }
                                else
                                {
                                    data[TW_WRITE_COMMAND_LEN - 1] = 0xff;
                                    data[TW_WRITE_COMMAND_LEN - 12] = 0xff;
                                    //printk("%s: open core: %d\n", __FUNCTION__, m);
                                }
                                OpenCoreNum1 = OpenCoreNum1 >> 1;
                            }
                            else if((m < 64) && (m >= 32))
                            {
                                //printk("%s: m = %d, OpenCoreNum2 = %d\n", __FUNCTION__, m, OpenCoreNum2);
                                if( !(OpenCoreNum2 & 0x00000001) )
                                {
                                    data[TW_WRITE_COMMAND_LEN - 1] = 0x0;
                                    data[TW_WRITE_COMMAND_LEN - 12] = 0x0;
                                    //printk("%s: Do not open core: %d\n", __FUNCTION__, m);
                                }
                                else
                                {
                                    data[TW_WRITE_COMMAND_LEN - 1] = 0xff;
                                    data[TW_WRITE_COMMAND_LEN - 12] = 0xff;
                                    //printk("%s: open core: %d\n", __FUNCTION__, m);
                                }
                                OpenCoreNum2 = OpenCoreNum2 >> 1;
                            }
                            else if((m < 96) && (m >= 64))
                            {
                                //printk("%s: m = %d, OpenCoreNum3 = %d\n", __FUNCTION__, m, OpenCoreNum3);
                                if( !(OpenCoreNum3 & 0x00000001) )
                                {
                                    data[TW_WRITE_COMMAND_LEN - 1] = 0x0;
                                    data[TW_WRITE_COMMAND_LEN - 12] = 0x0;
                                    //printk("%s: Do not open core: %d\n", __FUNCTION__, m);
                                }
                                else
                                {
                                    data[TW_WRITE_COMMAND_LEN - 1] = 0xff;
                                    data[TW_WRITE_COMMAND_LEN - 12] = 0xff;
                                    //printk("%s: open core: %d\n", __FUNCTION__, m);
                                }
                                OpenCoreNum3 = OpenCoreNum3 >> 1;
                            }
                            else if((m < 128) && (m >= 96))
                            {
                                //printk("%s: m = %d, OpenCoreNum4 = %d\n", __FUNCTION__, m, OpenCoreNum4);
                                if( !(OpenCoreNum4 & 0x00000001) )
                                {
                                    data[TW_WRITE_COMMAND_LEN - 1] = 0x0;
                                    data[TW_WRITE_COMMAND_LEN - 12] = 0x0;
                                    //printk("%s: Do not open core: %d\n", __FUNCTION__, m);
                                }
                                else
                                {
                                    data[TW_WRITE_COMMAND_LEN - 1] = 0xff;
                                    data[TW_WRITE_COMMAND_LEN - 12] = 0xff;
                                    //printk("%s: open core: %d\n", __FUNCTION__, m);
                                }
                                OpenCoreNum4 = OpenCoreNum4 >> 1;
                            }
                            else
                            {
                                ;
                            }
                        }

                        do
                        {
                            work_fifo_ready = get_buffer_space();
                            if(work_fifo_ready & (0x1 << i))
                            {
                                break;
                            }
                            else    //work fifo is full, wait for 50ms
                            {
                                //applog(LOG_DEBUG,"%s: chain%d work fifo not ready: 0x%x\n", __FUNCTION__, i, work_fifo_ready);
                                usleep(1000);
                            }
                        }
                        while(1);

                        if(m==0)    //new block
                        {
                            data[0] = NEW_BLOCK_MARKER;
                        }
                        else
                        {
                            data[0] = NORMAL_BLOCK_MARKER;
                        }

                        data[1] = i | 0x80; //set chain id and enable it

                        buf_vil_tw[0] = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
                        buf_vil_tw[1] = (work_vil.type << 24) | (work_vil.length << 16) | (work_vil.wc_base++ << 8) | work_vil.mid_num;
                        //buf_vil_tw[2] = work_vil.sno;
                        for(j=2; j<MIDSTATE_LEN/sizeof(unsigned int)+2; j++)
                        {
                            buf_vil_tw[j] = 0;
                        }
                        for(j=10; j<DATA2_LEN/sizeof(unsigned int)+10; j++)
                        {
                            buf_vil_tw[j] = (work_vil.data2[4*(j-10) + 0] << 24) | (work_vil.data2[4*(j-10) + 1] << 16) | (work_vil.data2[4*(j-10) + 2] << 8) | work_vil.data2[4*(j-10) + 3];
                        }

                        set_TW_write_command_vil(buf_vil_tw);
                    }
                }
            }
        }
    }
    printf("--- %s end\n", __FUNCTION__);
}

int get_TestRet_ByCore(int chainIndex, int asicIndex, int passCount)
{
	int m;
	int n = passCount/ASIC_CORE_NUM;
	for(m=0; m<ASIC_CORE_NUM; m++)
	{
		if(asic_core_nonce_num[chainIndex][asicIndex][m] != n && asic_core_enabled_flag[chainIndex][asicIndex][m]>0)
			return 0;	// failed
	}
	return 1; // success
}

int get_OpenCoreResult(int chainIndex, int asicIndex, int passCount)
{
	int m;
	int n = passCount/ASIC_CORE_NUM;
	int core_num=0;
	for(m=0; m<ASIC_CORE_NUM; m++)
	{
		if(asic_core_nonce_num[chainIndex][asicIndex][m]>0)
			core_num++;
	}

	return core_num;
}

void print_OpenCoreDetails(int chainIndex, int passCount, int check_opencore_num)
{
	char logstr[256];
	int asicIndex;
	int m;
	int n = passCount/ASIC_CORE_NUM;

	for(asicIndex=0;asicIndex<ASIC_NUM;asicIndex++)
		for(m=0; m<check_opencore_num; m++)
		{
			if(asic_core_nonce_num[chainIndex][asicIndex][m]!=n)
			{
				sprintf(logstr,"chain[%d] asic[%d] core[%d] get %d nonce < 8 !\n",chainIndex,asicIndex,m,asic_core_nonce_num[chainIndex][asicIndex][m]);
				writeLogFile(logstr);
			}
		}
}


int get_TestRet_ByBadCoreNum(int chainIndex, int asicIndex, int passCount)
{
	int m;
	int n = passCount/ASIC_CORE_NUM;
	
	if(asic_nonce_num[chainIndex][asicIndex] < passCount-chain_badcore_num[chainIndex][asicIndex]*n)
		return 0;	// failed
	return 1; // success
}

void UpdateTestResultFlag(int chainIndex, int passCount)
{
	int i;
	for(i = 0; i < ASIC_NUM; i++)
	{
		last_result[chainIndex][i] = get_TestRet_ByCore(chainIndex,i,passCount);
	}
}

int get_result(int chainIndex, int passCount, int validnonce)
{
	char logstr[256];
    int ret = 3;
    int i, j=0, loop=0, m, n;
    unsigned int OpenCoreNum1 = conf.OpenCoreNum1;
    unsigned int OpenCoreNum2 = conf.OpenCoreNum2;
    unsigned int OpenCoreNum3 = conf.OpenCoreNum3;
    unsigned int OpenCoreNum4 = conf.OpenCoreNum4;

    printf("\n------------------------------------------------------------------------------------------------------\n");
    if(conf.CommandMode)
    {
        printf("Command mode is FIL\n");
    }
    else
    {
        printf("Command mode is VIL\n");
    }

    loop = ASIC_NUM;

    sprintf(logstr,"require nonce number:%d\n", passCount);
	writeLogFile(logstr);

	sprintf(logstr,"require validnonce number:%d\n", validnonce);
	writeLogFile(logstr);

    for(i = 0; i < loop; i++)
    {
#ifdef LOG_CHIPS_CORE_DETAIL
        if(ASIC_NUM == 1)
        {
            sprintf(logstr,"core[%02d]=%02d\t", i, asic_nonce_num[chainIndex][i]);
			writeLogFile(logstr);
        }
        else
        {
            sprintf(logstr,"asic[%02d]=%02d\t", i, asic_nonce_num[chainIndex][i]);
			writeLogFile(logstr);
        }

        if(i % 8 == 7)
        {
            sprintf(logstr,"\n");
			writeLogFile(logstr);
        }
#endif
        if(ASIC_NUM == 1)
        {
            for(; j < 128; j++)
            {
                if(j < 32)
                {
                    if(OpenCoreNum1 & 0x00000001)
                    {
                        if(asic_nonce_num[chainIndex][j] < passCount)
                        {
                            ret = (~0x00000001) & ret;
                        }
                        OpenCoreNum1 = OpenCoreNum1 >> 1;
                    }
                    else
                    {
                        OpenCoreNum1 = OpenCoreNum1 >> 1;
                    }
                }
                else if((j >= 32) && (j < 64))
                {
                    if(OpenCoreNum2 & 0x00000001)
                    {
                        if(asic_nonce_num[chainIndex][j] < passCount)
                        {
                            ret = (~0x00000001) & ret;
                        }
                        OpenCoreNum2 = OpenCoreNum2 >> 1;
                    }
                    else
                    {
                        OpenCoreNum2 = OpenCoreNum2 >> 1;
                    }
                }
                else if((j >= 64) && (j < 96))
                {
                    if(OpenCoreNum3 & 0x00000001)
                    {
                        if(asic_nonce_num[chainIndex][j] < passCount)
                        {
                            ret = (~0x00000001) & ret;
                        }
                        OpenCoreNum3 = OpenCoreNum3 >> 1;
                    }
                    else
                    {
                        OpenCoreNum3 = OpenCoreNum3 >> 1;
                    }
                }
                else
                {
                    if(OpenCoreNum4 & 0x00000001)
                    {
                        if(asic_nonce_num[chainIndex][j] < passCount)
                        {
                            ret = (~0x00000001) & ret;
                        }
                        OpenCoreNum4 = OpenCoreNum4 >> 1;
                    }
                    else
                    {
                        OpenCoreNum4 = OpenCoreNum4 >> 1;
                    }
                }
            }
        }
        else
        {
            if(asic_nonce_num[chainIndex][i] < passCount)
            {
                ret = (~0x00000001) & ret;
            }
        }
    }

    {
        n = passCount/ASIC_CORE_NUM;
#ifdef LOG_CHIPS_CORE_DETAIL
        sprintf(logstr,"\n\n\nBelow ASIC's core didn't receive all the nonce, they should receive %d nonce each!\n\n", n);
		writeLogFile(logstr);
#endif
        for(i = 0; i < loop; i++)
        {
#ifdef FORCE_8xPATTENT_TEST
			last_result[chainIndex][i] = get_TestRet_ByBadCoreNum(chainIndex,i,passCount);
#else
        	last_result[chainIndex][i] = get_TestRet_ByCore(chainIndex,i,passCount);

#ifdef DEBUG_STOP_WHEN_ASICNUM_NOTENOUGH
			last_opencore_result[chainIndex][i] = get_OpenCoreResult(chainIndex,i,passCount);
#endif
#endif
			
            if(asic_nonce_num[chainIndex][i] < passCount)
            {
#ifdef LOG_CHIPS_CORE_DETAIL
                sprintf(logstr,"asic[%02d]=%02d\n", i, asic_nonce_num[chainIndex][i]);
				writeLogFile(logstr);
#endif
                for(m=0; m<ASIC_CORE_NUM; m++)
                {
                    if(asic_core_nonce_num[chainIndex][i][m] != n)
                    {
#ifdef LOG_CHIPS_CORE_DETAIL
                        sprintf(logstr,"[%03d]=%d\t", m, asic_core_nonce_num[chainIndex][i][m]);
						writeLogFile(logstr);
#endif
                    }
                }
#ifdef LOG_CHIPS_CORE_DETAIL
                sprintf(logstr,"\n\n\n");
                writeLogFile(logstr);
#endif
            }
        }
    }

    sprintf(logstr,"\n\n");
	writeLogFile(logstr);
	
    for(i = 0; i < loop; i++)
    {
        sprintf(logstr,"freq[%02d]=%s\t", i, freq_pll_1385[last_freq[chainIndex][i]].freq);
		writeLogFile(logstr);
		
        if(i % 8 == 7)
        {
            sprintf(logstr,"\n");
			writeLogFile(logstr);
        }
    }

    sprintf(logstr,"\n\n");
	writeLogFile(logstr);

    if(valid_nonce_num[chainIndex] < validnonce)
    {
        ret = (~0x00000001) & ret;
    }
    
    sprintf(logstr,"total valid nonce number:%d\n", valid_nonce_num[chainIndex]);
	writeLogFile(logstr);
    sprintf(logstr,"total send work number:%d\n", send_work_num[chainIndex]);
	writeLogFile(logstr);
    sprintf(logstr,"require valid nonce number:%d\n", validnonce);
	writeLogFile(logstr);

	sprintf(logstr,"repeated_nonce_num:%d\n", repeated_nonce_num[chainIndex]);
	writeLogFile(logstr);
	sprintf(logstr,"err_nonce_num:%d\n", err_nonce_num[chainIndex]);
	writeLogFile(logstr);
	sprintf(logstr,"last_nonce_num:%d\n", last_nonce_num[chainIndex]);
	writeLogFile(logstr);
    return ret;
}

int get_temperature(int chainIndex)
{
    int temperature = 0;

    switch(chainIndex)
    {
        case 0:
        case 1:
        case 2:
        case 3:
            temperature = get_temperature_0_3();
            temperature = (temperature >> chainIndex*8) & 0x000000ff;
            break;

        case 4:
        case 5:
        case 6:
        case 7:
            temperature = get_temperature_4_7();
            temperature = (temperature >> (chainIndex-4)*8) & 0x000000ff;
            break;

        case 8:
        case 9:
        case 10:
        case 11:
            temperature = get_temperature_8_11();
            temperature = (temperature >> (chainIndex-8)*8) & 0x000000ff;
            break;

        case 12:
        case 13:
        case 14:
        case 15:
            temperature = get_temperature_12_15();
            temperature = (temperature >> (chainIndex-12)*8) & 0x000000ff;
            break;

        default:
            printf("Chain = %d, but it is wrong! \n", chainIndex);
            break;
    }

    return temperature;
}

unsigned int get_pic_iic()
{
    int ret = -1;
    ret = *(axi_fpga_addr + IIC_COMMAND);
    //applog(LOG_DEBUG,"%s: IIC_COMMAND is 0x%x\n", __FUNCTION__, ret);
    return ret;
}

unsigned char set_pic_iic(unsigned int data)
{
    unsigned int ret=0;
    unsigned char ret_data = 0;

    *((unsigned int *)(axi_fpga_addr + IIC_COMMAND)) = data & 0x7fffffff;
    //applog(LOG_DEBUG,"%s: set IIC_COMMAND is 0x%x\n", __FUNCTION__, data & 0x7fffffff);

    while(1)
    {
        ret = get_pic_iic();
        if(ret & 0x80000000)
        {
            ret_data = (unsigned char)(ret & 0x000000ff);
            return ret_data;
        }
        else
        {
            //applog(LOG_DEBUG,"%s: waiting write pic iic\n", __FUNCTION__);
            usleep(1000);
        }
    }
}

unsigned char write_pic_iic(bool read, bool reg_addr_valid, unsigned char reg_addr, unsigned char chain, unsigned char data)
{
    unsigned int value = 0;
    unsigned char ret = 0;

#if 0
	while(1)
    {
        ret = get_pic_iic();
		//printf("iic command ret = 0x%08x\n", ret);
        if(ret & 0x80000000)
        {
        	usleep(10*1000);
			break;
		}
		else if(counter++>3)
		{
			//printf("^^^^^^^^^^^^^^^^^^^^^\n");
			break;
		}
		usleep(10*1000);
	}
#endif
    if(read)
    {
        value |= IIC_READ;
    }

    if(reg_addr_valid)
    {
        value |= IIC_REG_ADDR_VALID;
        value |= IIC_REG_ADDR(reg_addr);
    }

    value |= IIC_ADDR_HIGH_4_BIT;

    value |= IIC_CHAIN_NUMBER(chain);

    value |= data;

    ret = set_pic_iic(value);

    return ret;
}

void send_pic_command(unsigned char chain)
{
	pthread_mutex_lock(&iic_mutex);
    write_pic_iic(false, false, 0x0, chain, PIC_COMMAND_1);
    write_pic_iic(false, false, 0x0, chain, PIC_COMMAND_2);
	pthread_mutex_unlock(&iic_mutex);
}

void get_pic_iic_flash_addr_pointer(unsigned char chain, unsigned char *addr_H, unsigned char *addr_L);
void set_pic_iic_flash_addr_pointer(unsigned char chain, unsigned char addr_H, unsigned char addr_L)
{
	unsigned char check_addr_H, check_addr_L;
	char logstr[256];
	int err_count=0;

#ifdef ENABLE_CHECK_PIC_FLASH_ADDR
	do{
#endif
	    send_pic_command(chain);

		pthread_mutex_lock(&iic_mutex);
	    write_pic_iic(false, false, 0x0, chain, SET_PIC_FLASH_POINTER);
	    write_pic_iic(false, false, 0x0, chain, addr_H);
	    write_pic_iic(false, false, 0x0, chain, addr_L);
		pthread_mutex_unlock(&iic_mutex);


		// we need check this address, because some PIC lost data of flash!!!
		get_pic_iic_flash_addr_pointer(chain,&check_addr_H,&check_addr_L);
		if(check_addr_H!=addr_H || check_addr_L!=addr_L)
		{
			sprintf(logstr,"Error of set PIC FLASH addr: addr_H=%x(%x) addr_L=%x(%x) on Chain[%d]\n",addr_H,check_addr_H,addr_L,check_addr_L,chain);
			writeLogFile(logstr);

#ifndef ENABLE_CHECK_PIC_FLASH_ADDR
		}
#else
			reset_iic_pic(chain);
			sleep(5);
			
			err_count++;

			if(err_count>3)
			{
				if(conf.force_freq)
				{
					// set error flag
					sprintf(search_failed_info,"J%d:5",chain+1);
					
					saveSearchFailedFlagInfo();
					searchStatus=SEARCH_FAILED;
					while(1)
					{
						processTEST();
						sleep(1);
					}
				}
				break;
			}
			
#ifdef DEBUG_TEST_PIC_FLASH_RW
			while(1)sleep(1);	// we need wait here forever!!! for DEBUG
#endif
		}
		else break;
	}while(1);
#endif
}

void send_data_to_pic_iic(unsigned char chain, unsigned char command, unsigned char *buf, unsigned char length)
{
    int i=0;

	pthread_mutex_lock(&iic_mutex);
    write_pic_iic(false, false, 0x0, chain, command);
    for(i=0; i<length; i++)
    {
        write_pic_iic(false, false, 0x0, chain, *(buf + i));
    }
	pthread_mutex_unlock(&iic_mutex);
}

void get_data_from_pic_iic(unsigned char chain, unsigned char command, unsigned char *buf, unsigned char length)
{
    int i=0;

	pthread_mutex_lock(&iic_mutex);
    write_pic_iic(false, false, 0x0, chain, command);
    for(i=0; i<length; i++)
    {
        *(buf + i) = write_pic_iic(true, false, 0x0, chain, 0);
    }
	pthread_mutex_unlock(&iic_mutex);
}

void send_data_to_pic_flash(unsigned char chain, unsigned char *buf)
{
    send_pic_command(chain);
    send_data_to_pic_iic(chain, SEND_DATA_TO_IIC, buf, 16);
}

void get_data_from_pic_flash(unsigned char chain, unsigned char *buf)
{
    send_pic_command(chain);
    get_data_from_pic_iic(chain, READ_DATA_FROM_IIC, buf, 16);
}

unsigned char erase_pic_flash(unsigned char chain)
{
    send_pic_command(chain);

	pthread_mutex_lock(&iic_mutex);
    write_pic_iic(false, false, 0x0, chain, ERASE_IIC_FLASH);
	usleep(200000);
	pthread_mutex_unlock(&iic_mutex);
	
/*    while(1)
    {
        usleep(3000);
        ret = write_pic_iic(true, false, 0x0, chain, 0);
        if(ret == 0x0)
        {
            printf("erase done\n");
            return ret;
        }
    }

    */
    return 0;
}

unsigned char erase_pic_flash_all(unsigned char chain)
{
    unsigned int i=0, erase_loop = 0;
    unsigned char start_addr_h = PIC_FLASH_POINTER_START_ADDRESS_H, start_addr_l = PIC_FLASH_POINTER_START_ADDRESS_L;
    unsigned char end_addr_h = PIC_FLASH_POINTER_END_ADDRESS_H, end_addr_l = PIC_FLASH_POINTER_END_ADDRESS_L;
    unsigned int pic_flash_length=0;

    set_pic_iic_flash_addr_pointer(chain, PIC_FLASH_POINTER_START_ADDRESS_H, PIC_FLASH_POINTER_START_ADDRESS_L);

    pic_flash_length = (((unsigned int)end_addr_h << 8) + end_addr_l) - (((unsigned int)start_addr_h << 8) + start_addr_l) + 1;
    erase_loop = pic_flash_length/PIC_FLASH_SECTOR_LENGTH;
    printf("%s: erase_loop = %d\n", __FUNCTION__, erase_loop);

    for(i=0; i<erase_loop; i++)
    {
        erase_pic_flash(chain);
    }

	return 0;
}

unsigned char write_data_into_pic_flash(unsigned char chain)
{
    send_pic_command(chain);

	pthread_mutex_lock(&iic_mutex);
    write_pic_iic(false, false, 0x0, chain, WRITE_DATA_INTO_PIC);
	
	usleep(200000);
	pthread_mutex_unlock(&iic_mutex);

    return 0;
}

unsigned char read_pic_badcore_num(unsigned char chain,unsigned char *buf1)
{
	int i;
	set_pic_iic_flash_addr_pointer(chain, PIC_FLASH_POINTER_BADCORE_START_ADDRESS_H, PIC_FLASH_POINTER_BADCORE_START_ADDRESS_L);
	
	for(i=0;i<4;i++)
	{
    	get_data_from_pic_flash(chain, buf1+i*16);
		usleep(200000);
	}
	return 0;
}

unsigned char flash_pic_badcore_num(unsigned char chain,unsigned char *buf1)
{
    unsigned char buf[16] = {0};
    unsigned int i=0;
    unsigned char start_addr_h = PIC_FLASH_POINTER_BADCORE_START_ADDRESS_H, start_addr_l = PIC_FLASH_POINTER_BADCORE_START_ADDRESS_L;
    unsigned char end_addr_h = PIC_FLASH_POINTER_BADCORE_END_ADDRESS_H, end_addr_l = PIC_FLASH_POINTER_BADCORE_END_ADDRESS_L;
    unsigned int pic_flash_length=0;

    set_pic_iic_flash_addr_pointer(chain, PIC_FLASH_POINTER_BADCORE_START_ADDRESS_H, PIC_FLASH_POINTER_BADCORE_START_ADDRESS_L);

    pic_flash_length = (((unsigned int)end_addr_h << 8) + end_addr_l) - (((unsigned int)start_addr_h << 8) + start_addr_l) + 1;

    for(i=0; i<(pic_flash_length/PIC_FLASH_SECTOR_LENGTH)*4; i++)
    {
        memcpy(buf, buf1+i*16, 16);
        send_data_to_pic_flash(chain, buf);
        write_data_into_pic_flash(chain);
    }
	return 0;
}


unsigned char erase_pic_badcore_num(unsigned char chain)
{
    unsigned int i=0, erase_loop = 0;
    unsigned char start_addr_h = PIC_FLASH_POINTER_BADCORE_START_ADDRESS_H, start_addr_l = PIC_FLASH_POINTER_BADCORE_START_ADDRESS_L;
    unsigned char end_addr_h = PIC_FLASH_POINTER_BADCORE_END_ADDRESS_H, end_addr_l = PIC_FLASH_POINTER_BADCORE_END_ADDRESS_L;
    unsigned int pic_flash_length=0;

    set_pic_iic_flash_addr_pointer(chain, PIC_FLASH_POINTER_BADCORE_START_ADDRESS_H, PIC_FLASH_POINTER_BADCORE_START_ADDRESS_L);
		
    pic_flash_length = (((unsigned int)end_addr_h << 8) + end_addr_l) - (((unsigned int)start_addr_h << 8) + start_addr_l) + 1;
    erase_loop = pic_flash_length/PIC_FLASH_SECTOR_LENGTH;
    printf("%s: erase_loop = %d\n", __FUNCTION__, erase_loop);

    for(i=0; i<erase_loop; i++)
    {
        erase_pic_flash(chain);
    }

	return 0;
}

unsigned char read_pic_freq(unsigned char chain,unsigned char *buf1)
{
	int i;
	set_pic_iic_flash_addr_pointer(chain, PIC_FLASH_POINTER_FREQ_START_ADDRESS_H, PIC_FLASH_POINTER_FREQ_START_ADDRESS_L);
	
	for(i=0;i<8;i++)
	{
    	get_data_from_pic_flash(chain, buf1+i*16);
		usleep(200000);
	}
	return 0;
}

unsigned char flash_pic_freq(unsigned char chain,unsigned char *buf1)
{
    unsigned char buf[16] = {0};
    unsigned int i=0;
    unsigned char start_addr_h = PIC_FLASH_POINTER_FREQ_START_ADDRESS_H, start_addr_l = PIC_FLASH_POINTER_FREQ_START_ADDRESS_L;
    unsigned char end_addr_h = PIC_FLASH_POINTER_FREQ_END_ADDRESS_H, end_addr_l = PIC_FLASH_POINTER_FREQ_END_ADDRESS_L;
    unsigned int pic_flash_length=0;

    set_pic_iic_flash_addr_pointer(chain, PIC_FLASH_POINTER_FREQ_START_ADDRESS_H, PIC_FLASH_POINTER_FREQ_START_ADDRESS_L);

    pic_flash_length = (((unsigned int)end_addr_h << 8) + end_addr_l) - (((unsigned int)start_addr_h << 8) + start_addr_l) + 1;

    for(i=0; i<(pic_flash_length/PIC_FLASH_SECTOR_LENGTH)*4; i++)
    {
        memcpy(buf, buf1+i*16, 16);
        send_data_to_pic_flash(chain, buf);
        write_data_into_pic_flash(chain);
    }
	return 0;
}


unsigned char erase_pic_freq(unsigned char chain)
{
    unsigned int i=0, erase_loop = 0;
    unsigned char start_addr_h = PIC_FLASH_POINTER_FREQ_START_ADDRESS_H, start_addr_l = PIC_FLASH_POINTER_FREQ_START_ADDRESS_L;
    unsigned char end_addr_h = PIC_FLASH_POINTER_FREQ_END_ADDRESS_H, end_addr_l = PIC_FLASH_POINTER_FREQ_END_ADDRESS_L;
    unsigned int pic_flash_length=0;

    set_pic_iic_flash_addr_pointer(chain, PIC_FLASH_POINTER_FREQ_START_ADDRESS_H, PIC_FLASH_POINTER_FREQ_START_ADDRESS_L);

    pic_flash_length = (((unsigned int)end_addr_h << 8) + end_addr_l) - (((unsigned int)start_addr_h << 8) + start_addr_l) + 1;
    erase_loop = pic_flash_length/PIC_FLASH_SECTOR_LENGTH;
    printf("%s: erase_loop = %d\n", __FUNCTION__, erase_loop);

    for(i=0; i<erase_loop; i++)
    {
        erase_pic_flash(chain);
    }

	return 0;
}

unsigned char jump_to_app_from_loader(unsigned char chain)
{
    printf("%s\n", __FUNCTION__);

    send_pic_command(chain);

	pthread_mutex_lock(&iic_mutex);
    write_pic_iic(false, false, 0x0, chain, JUMP_FROM_LOADER_TO_APP);
    usleep(1000000);
	pthread_mutex_unlock(&iic_mutex);
 
    return 0;
}

#ifdef T9_18
int dsPIC33EP16GS202_jump_to_app_from_loader(unsigned char which_iic)
{
	unsigned char length = 0x04, crc_data[2] = {0xff}, read_back_data[2] = {0xff};
	unsigned short crc = 0;
	char logstr[256];
	int retry_count=0;

#ifdef DEBUG_IGNORE_T9_18_PIC_CMD
	return 0;
#endif

	printf("\n--- %s\n", __FUNCTION__);
	
	crc = length + JUMP_FROM_LOADER_TO_APP;
	crc_data[0] = (unsigned char)((crc >> 8) & 0x00ff);
	crc_data[1] = (unsigned char)((crc >> 0) & 0x00ff);
	printf("--- %s: crc_data[0] = 0x%x, crc_data[1] = 0x%x\n", __FUNCTION__, crc_data[0], crc_data[1]);

	while(retry_count++<3)
	{
		pthread_mutex_lock(&iic_mutex);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, PIC_COMMAND_1);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, PIC_COMMAND_2);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, length);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, JUMP_FROM_LOADER_TO_APP);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, crc_data[0]);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, crc_data[1]);
		usleep(100*1000);
		read_back_data[0] = T9_plus_write_pic_iic(true, false, 0x0, which_iic, 0);
		read_back_data[1] = T9_plus_write_pic_iic(true, false, 0x0, which_iic, 0);
		pthread_mutex_unlock(&iic_mutex);
		
		printf("--- %s: read_back_data[0] = 0x%x, read_back_data[1] = 0x%x\n", __FUNCTION__, read_back_data[0], read_back_data[1]);

		if((read_back_data[0] != JUMP_FROM_LOADER_TO_APP) || (read_back_data[1] != 1))
		{
			sprintf(logstr,"%s failed on Chain[%d]! try again...\n", __FUNCTION__,which_iic);
			writeLogFile(logstr);
			sleep(1);
		//	return 0;	// error
		}
		else
		{
			sleep(3);
			printf("\n--- %s ok\n\n", __FUNCTION__);
			return 1;	// ok
		}
	}
	return 0;
}

int dsPIC33EP16GS202_reset_pic(unsigned char which_iic)
{
	unsigned char length = 0x04, crc_data[2] = {0xff}, read_back_data[2] = {0xff};
	unsigned short crc = 0;
	char logstr[256];
	int retry_count=0;
	
#ifdef DEBUG_IGNORE_T9_18_PIC_CMD
	return 0;
#endif

	printf("\n--- %s\n", __FUNCTION__);

	crc = length + RESET_PIC;
	crc_data[0] = (unsigned char)((crc >> 8) & 0x00ff);
	crc_data[1] = (unsigned char)((crc >> 0) & 0x00ff);
	printf("--- %s: which_iic=%d crc_data[0] = 0x%x, crc_data[1] = 0x%x\n", __FUNCTION__,which_iic, crc_data[0], crc_data[1]);

	while(retry_count++<3)
	{
		pthread_mutex_lock(&iic_mutex);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, PIC_COMMAND_1);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, PIC_COMMAND_2);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, length);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, RESET_PIC);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, crc_data[0]);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, crc_data[1]);
		usleep(400*1000);
		read_back_data[0] = T9_plus_write_pic_iic(true, false, 0x0, which_iic, 0);
		read_back_data[1] = T9_plus_write_pic_iic(true, false, 0x0, which_iic, 0);
		pthread_mutex_unlock(&iic_mutex);
		
		printf("--- %s: read_back_data[0] = 0x%x, read_back_data[1] = 0x%x\n", __FUNCTION__, read_back_data[0], read_back_data[1]);
		usleep(100*1000);
		
		if((read_back_data[0] != RESET_PIC) || (read_back_data[1] != 1))
		{
			sprintf(logstr,"%s failed on Chain[%d]! try again...\n", __FUNCTION__,which_iic);
			writeLogFile(logstr);
			sleep(1);
		//	return 0;	// error
		}
		else
		{
			printf("\n--- %s ok\n\n", __FUNCTION__);
			return 1;	// ok
		}
	}

	return 0;
}

unsigned char reset_iic_pic(unsigned char chain)
{
	if(fpga_version>=0xE)
	{
		if(chain<1 || chain>3)	// T9+ only chain[1] [2] [3] can control the PIC with new FPGA which can support S9 too.
			return 1;

		return dsPIC33EP16GS202_reset_pic(chain);
	}
	else
	{
		if(chain%3 != 0)
			return 1;
		
		return dsPIC33EP16GS202_reset_pic(chain/3);
	}
}
#else
unsigned char reset_iic_pic(unsigned char chain)
{
    printf("%s\n", __FUNCTION__);

    send_pic_command(chain);

	pthread_mutex_lock(&iic_mutex);
    write_pic_iic(false, false, 0x0, chain, RESET_PIC);
    usleep(1000000);
	pthread_mutex_unlock(&iic_mutex);
	
    return 0;
}
#endif

void get_pic_iic_flash_addr_pointer(unsigned char chain, unsigned char *addr_H, unsigned char *addr_L)
{
    printf("%s\n", __FUNCTION__);
    send_pic_command(chain);

	pthread_mutex_lock(&iic_mutex);
    write_pic_iic(false, false, 0x0, chain, GET_PIC_FLASH_POINTER);
    *addr_H = write_pic_iic(true, false, 0x0, chain, 0);
    *addr_L = write_pic_iic(true, false, 0x0, chain, 0);
	pthread_mutex_unlock(&iic_mutex);
	
    printf("%s: *addr_H = 0x%02x, *addr_L = 0x%02x\n", __FUNCTION__, *addr_H, *addr_L);
}

unsigned int get_Hardware_version()
{
	unsigned int value = *((unsigned int *)(axi_fpga_addr + HARDWARE_VERSION));
	return value;
}

#ifdef T9_18
extern int chain_vol_value[BITMAIN_MAX_CHAIN_NUM];
void set_Hardware_version(unsigned int value)
{
	*((unsigned int *)(axi_fpga_addr + HARDWARE_VERSION)) = value;
}

unsigned char write_EEPROM_iic(bool read, bool reg_addr_valid, unsigned char reg_addr, unsigned char which_iic, unsigned char data)
{
	unsigned int value = 0x00000000, counter = 0;
	unsigned char ret = 0;

	while(1)
    {
        ret = get_iic();
		//printf("iic command ret = 0x%08x\n", ret);
        if(ret & 0x80000000)
        {        	
			break;
		}
		else if(counter++>10)
		{
			//printf("^^^^^^^^^^^^^^^^^^^^^\n");
			break;
		}
		usleep(1*1000);
	}

	if(read)
	{
		value |= IIC_READ;
	}

	if(reg_addr_valid)
	{
		value |= IIC_REG_ADDR_VALID;
		value |= IIC_REG_ADDR(reg_addr);
	}

	value |= EEPROM_ADDR_HIGH_4_BIT;

	if(fpga_version>=0xE && fpga_version<=0xF)//  decrease one , because chain[1] [2] [3] use PIC [0] [1] [2]
		which_iic--;

	value |= IIC_SELECT(which_iic);

	value |= data;

	ret = set_iic(value);

	return ret;
}


void AT24C02_write_one_byte(unsigned char address, unsigned char data, unsigned char which_iic)
{
	pthread_mutex_lock(&iic_mutex);
	write_EEPROM_iic(false, true, address, which_iic, data);
	pthread_mutex_unlock(&iic_mutex);
}

unsigned char AT24C02_read_one_byte(unsigned char address, unsigned char which_iic)
{
	unsigned char data = 0;
	
	pthread_mutex_lock(&iic_mutex);
	data = write_EEPROM_iic(true, true, address, which_iic, 0);
	pthread_mutex_unlock(&iic_mutex);

	return data;
}

void AT24C02_write_bytes(unsigned char address, unsigned char *buf, unsigned char which_iic, unsigned int length)
{
	unsigned int i = 0;

	printf("--- %s\n", __FUNCTION__);
	
	if((address + length) > EEPROM_LENGTH)
	{
		printf("\n--- %s: address + length = %d > EEPROM_LENGTH(%d)\n", __FUNCTION__, address + length, EEPROM_LENGTH);
		return;
	}

	for(i=0; i<length; i++)
	{
		AT24C02_write_one_byte(address+i, *(buf + i), which_iic);
	}
}

void AT24C02_read_bytes(unsigned char address, unsigned char *buf, unsigned char which_iic, unsigned int length)
{
	unsigned int i = 0;

	printf("--- %s\n", __FUNCTION__);
	
	if((address + length) > EEPROM_LENGTH)
	{
		printf("\n--- %s: address + length = %d > EEPROM_LENGTH(%d)\n", __FUNCTION__, address + length, EEPROM_LENGTH);
		return;
	}

	for(i=0; i<length; i++)
	{
		*(buf + i) =  AT24C02_read_one_byte(address+i, which_iic);
	}
}

unsigned char read_freq_badcores(unsigned char chain, unsigned char *buf)
{
	if(fpga_version>=0xE)
	{
		if(chain<1 || chain>3)
			return 0;
	
		AT24C02_read_bytes(FREQ_BADCORE_ADDR,buf,chain,128);
	}
	else
	{
		if(chain%3 != 0)
			return 0;
	
		AT24C02_read_bytes(FREQ_BADCORE_ADDR,buf,chain/3,128);
	}
	return 128;
}

unsigned char save_freq_badcores(unsigned char chain, unsigned char *buf)
{
	if(fpga_version>=0xE)
	{
		if(chain<1 || chain>3)
			return 0;
		
		AT24C02_write_bytes(FREQ_BADCORE_ADDR,buf,chain,128);
	}
	else
	{
		if(chain%3 != 0)
			return 0;
		
		AT24C02_write_bytes(FREQ_BADCORE_ADDR,buf,chain/3,128);
	}
	return 128;
}

void AT24C02_save_voltage(unsigned char which_iic, unsigned char voltage)
{
	printf("\n--- %s\n", __FUNCTION__);

	AT24C02_write_one_byte(VOLTAGE_ADDR, voltage, which_iic);

	printf("%s: voltage = 0x%02x\n", __FUNCTION__, voltage);
}

unsigned char AT24C02_read_voltage(unsigned char which_iic)
{
	return AT24C02_read_one_byte(VOLTAGE_ADDR, which_iic);
}

int getHighestVoltageValue(int chainIndex)
{
	int startIndex;	
	int i;
	int maxVolValue=0;
	char logstr[256];

	if(fpga_version>=0xE)
	{
		switch(chainIndex)
		{
		case 1:
		case 8:
		case 9:
			maxVolValue=chain_vol_value[1];
			for(i=8;i<10;i++)
			{
				if(maxVolValue<chain_vol_value[i])
					maxVolValue=chain_vol_value[i];
			}
			break;
		case 2:
		case 10:
		case 11:
			maxVolValue=chain_vol_value[2];
			for(i=10;i<12;i++)
			{
				if(maxVolValue<chain_vol_value[i])
					maxVolValue=chain_vol_value[i];
			}
			break;
		case 3:
		case 12:
		case 13:
			maxVolValue=chain_vol_value[3];
			for(i=12;i<14;i++)
			{
				if(maxVolValue<chain_vol_value[i])
					maxVolValue=chain_vol_value[i];
			}
			break;
		default:
			sprintf(logstr,"T9+ FATAL ERROR chainindex=%d\n",chainIndex);
			writeLogFile(logstr);
			break;
		}
	}
	else
	{
		startIndex=(chainIndex/3)*3; // get start chainindex, 0,1,2 = 0    3,4,5 = 3  ...
		maxVolValue=chain_vol_value[chainIndex];
		
		for(i=startIndex;i<startIndex+3;i++)
		{
			if(maxVolValue<chain_vol_value[i])
				maxVolValue=chain_vol_value[i];
		}
	}

	return maxVolValue;
}

int set_Voltage_S9_plus_plus_BM1387_54(unsigned char which_iic, unsigned char pic_voltage)
{
	double temp_voltage = 0;
	unsigned char length = 0x07, crc_data[2] = {0xff}, read_back_data[2] = {0xff};
	unsigned short crc = 0;
	unsigned char voltage1 = pic_voltage, voltage2 = 0, voltage3 = 0;
	int i;
	char logstr[256];
	int retry_count=0;

#ifdef DEBUG_IGNORE_T9_18_PIC_CMD
	return 0;
#endif

	printf("voltage1 = %d\n", voltage1);

	if((voltage1 > 127) || (voltage2 > 127) || (voltage3 > 127))
	{
		printf("\n--- %s voltage1(%d) > 127 \n\n", __FUNCTION__, voltage1);
		return 0; // because 4017 just have 127 level
	}

	if(T9_18_chain_voltage[which_iic]==pic_voltage)
		return 1;

	T9_18_chain_voltage[which_iic]=pic_voltage;
	
	crc = length + SET_VOLTAGE + voltage1 + voltage2 + voltage3;
	crc_data[0] = (unsigned char)((crc >> 8) & 0x00ff);
	crc_data[1] = (unsigned char)((crc >> 0) & 0x00ff);
	//printf("--- %s: crc_data[0] = 0x%x, crc_data[1] = 0x%x\n", __FUNCTION__, crc_data[0], crc_data[1]);

	while(retry_count++<3)
	{
		pthread_mutex_lock(&iic_mutex);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, PIC_COMMAND_1);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, PIC_COMMAND_2);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, length);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, SET_VOLTAGE);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, voltage1);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, voltage2);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, voltage3);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, crc_data[0]);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, crc_data[1]);
		usleep(100*1000);
		read_back_data[0] = T9_plus_write_pic_iic(true, false, 0x0, which_iic, 0);
		read_back_data[1] = T9_plus_write_pic_iic(true, false, 0x0, which_iic, 0);
		pthread_mutex_unlock(&iic_mutex);

		//printf("--- %s: read_back_data[0] = 0x%x, read_back_data[1] = 0x%x\n", __FUNCTION__, read_back_data[0], read_back_data[1]);

		if((read_back_data[0] != SET_VOLTAGE) || (read_back_data[1] != 1))
		{
			sprintf(logstr,"%s failed on Chain[%d]!\n\n", __FUNCTION__,which_iic);
			writeLogFile(logstr);
			sleep(1);
		//	return 0;	// error
		}
		else
		{
			printf("\n--- %s ok!\n\n", __FUNCTION__);

			AT24C02_save_voltage(which_iic, voltage1);
		//	dsPIC33EP16GS202_enable_pic_dc_dc(which_iic, 1);	// open pic dc-dc
			sleep(5);
			return 1;	// ok
		}
	}

	return 0;
}

void set_voltage_T9_18_into_PIC(unsigned char chain, unsigned char voltage)
{
	if(fpga_version>=0xE)
	{
	//	if(chain%<1 || chain>3)
	//		return;

		int new_T9_PLUS_chainIndex,new_T9_PLUS_chainOffset;	// only used by new T9+ FPGA
		getPICChainIndexOffset(chain,&new_T9_PLUS_chainIndex,&new_T9_PLUS_chainOffset);
		
		set_Voltage_S9_plus_plus_BM1387_54(new_T9_PLUS_chainIndex,voltage);
	}
	else
	{
	//	if(chain%3 != 0)
	//		return;

		set_Voltage_S9_plus_plus_BM1387_54(chain/3,voltage);
	}
}

void set_pic_voltage_T9_18(unsigned char chain)
{
	char logstr[256];
	unsigned char vol_pic;
	// before call this function, must set chain_vol_value[chain] value at first! because T9_18 will only set the highest voltage for 3 chains!
	int highestVoltageValue=getHighestVoltageValue(chain);
	vol_pic=getPICvoltageFromValue(highestVoltageValue);

	sprintf(logstr,"set voltage=%d on chain[%d], the real voltage=%d\n",chain_vol_value[chain],chain,highestVoltageValue);
	writeLogFile(logstr);
	
	set_voltage_T9_18_into_PIC(chain,vol_pic);
}

void set_pic_voltage(unsigned char chain, unsigned char voltage)
{
	set_pic_voltage_T9_18(chain);
}
#else
void set_pic_voltage(unsigned char chain, unsigned char voltage)
{
    printf("%s\n", __FUNCTION__);
    send_pic_command(chain);

	pthread_mutex_lock(&iic_mutex);
    write_pic_iic(false, false, 0x0, chain, SET_VOLTAGE);
    write_pic_iic(false, false, 0x0, chain, voltage);
    usleep(100000);
	pthread_mutex_unlock(&iic_mutex);
}
#endif

void set_voltage_setting_time(unsigned char chain, unsigned char *time)
{
    int i=0;
    printf("\n--- %s\n", __FUNCTION__);
    send_pic_command(chain);
    send_data_to_pic_iic(chain, SET_VOLTAGE_TIME, time, 6);
    for(i=0; i<6; i++)
    {
        printf("%s: time[%d] = 0x%02x\n", __FUNCTION__, i, *(time + i));
    }
    usleep(100000);
}

void set_hash_board_id_number(unsigned char chain, unsigned char *id)
{
    printf("\n--- %s\n", __FUNCTION__);
    send_pic_command(chain);
    send_data_to_pic_iic(chain, SET_HASH_BOARD_ID, id, 12);
    usleep(100000);
}

void get_hash_board_id_number(unsigned char chain, unsigned char *id)
{
    printf("%s\n", __FUNCTION__);
    send_pic_command(chain);
    get_data_from_pic_iic(chain, GET_HASH_BOARD_ID, id, 12);
}

void write_host_MAC_and_time(unsigned char chain, unsigned char *buf)
{
    unsigned int i=0;
    printf("\n--- %s\n", __FUNCTION__);
    send_pic_command(chain);
    send_data_to_pic_iic(chain, SET_HOST_MAC_ADDRESS, buf, 12);
    for(i=0; i<12; i++)
    {
        printf("%s: buf[%d] = 0x%02x\n", __FUNCTION__, i, *(buf + i));
    }
    usleep(100000);
}

#ifdef T9_18
int dsPIC33EP16GS202_enable_pic_dc_dc(unsigned char which_iic, unsigned char enable)
{
	unsigned char length = 0x05, crc_data[2] = {0xff}, read_back_data[2] = {0xff};
	unsigned short crc = 0;
	char logstr[256];
	int retry_count=0;

#ifdef DEBUG_IGNORE_T9_18_PIC_CMD
	return 0;
#endif

	printf("\n--- %s\n", __FUNCTION__);

	crc = length + ENABLE_VOLTAGE + enable;
	crc_data[0] = (unsigned char)((crc >> 8) & 0x00ff);
	crc_data[1] = (unsigned char)((crc >> 0) & 0x00ff);
	printf("--- %s: crc_data[0] = 0x%x, crc_data[1] = 0x%x\n", __FUNCTION__, crc_data[0], crc_data[1]);

	while(retry_count++<3)
	{
		pthread_mutex_lock(&iic_mutex);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, PIC_COMMAND_1);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, PIC_COMMAND_2);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, length);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, ENABLE_VOLTAGE);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, enable);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, crc_data[0]);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, crc_data[1]);
		usleep(200*1000);
		read_back_data[0] = T9_plus_write_pic_iic(true, false, 0x0, which_iic, 0);
		read_back_data[1] = T9_plus_write_pic_iic(true, false, 0x0, which_iic, 0);
		pthread_mutex_unlock(&iic_mutex);

		printf("--- %s: read_back_data[0] = 0x%x, read_back_data[1] = 0x%x\n", __FUNCTION__, read_back_data[0], read_back_data[1]);

		if((read_back_data[0] != ENABLE_VOLTAGE) || (read_back_data[1] != 1))
		{
			sprintf(logstr,"%s failed on Chain[%d]!\n", __FUNCTION__,which_iic);
			writeLogFile(logstr);
			sleep(1);
	//		return 0;	// error
		}
		else
		{
			printf("\n--- %s ok\n\n", __FUNCTION__);
			sleep(5);
			return 1;	// ok
		}
	}
	return 0;
}

void enable_pic_dac(unsigned char chain)
{
	if(fpga_version>=0xE)
	{
		if(chain<1 || chain>3)
			return;

		dsPIC33EP16GS202_enable_pic_dc_dc(chain, 1);	// open pic dc-dc
	}
	else
	{
		if(chain%3!=0) // only enable DC when enable the first chain of 3 chains, like 0,  3,  6 ...
			return;

		dsPIC33EP16GS202_enable_pic_dc_dc(chain/3, 1);	// open pic dc-dc
	}
}

void disable_pic_dac(unsigned char chain)
{
	if(fpga_version>=0xE)
	{
		int new_T9_PLUS_chainIndex,new_T9_PLUS_chainOffset;
		getPICChainIndexOffset(chain,&new_T9_PLUS_chainIndex,&new_T9_PLUS_chainOffset);
		
	    if(chain!=9 && chain!=11 && chain!=13) // only disable DC when close the last chain of 3 chains, like 8,  11,  13 ...
			return;

		dsPIC33EP16GS202_enable_pic_dc_dc(new_T9_PLUS_chainIndex, 0);	// open pic dc-dc
	}
	else
	{
	    if(chain%3 !=2)	// only disable DC when close the last chain of 3 chains, like 2,  5,  8 ...
			return;

		dsPIC33EP16GS202_enable_pic_dc_dc(chain/3, 0);	// open pic dc-dc
	}
}

unsigned int get_iic()
{
	int ret = -1;
	ret = *(axi_fpga_addr + IIC_COMMAND);

	//applog(LOG_DEBUG,"%s: IIC_COMMAND is 0x%x\n", __FUNCTION__, ret);
	return ret;
}

unsigned char set_iic(unsigned int data)
{
	unsigned int ret=0;
	unsigned char ret_data = 0;

	*((unsigned int *)(axi_fpga_addr + IIC_COMMAND)) = data & 0x7fffffff;
	//printf("%s: set IIC_COMMAND is 0x%08x\n", __FUNCTION__, data & 0x7fffffff);

	while(1)
	{
		ret = get_iic();
		if(ret & 0x80000000)
		{
		//	printf("%s: get ret = 0x%08x\n", __FUNCTION__,ret);
			
			ret_data = (unsigned char)(ret & 0x000000ff);
			return ret_data;
		}
		else
		{
		//	printf("%s: waiting write pic iic ret=0x%08x\n", __FUNCTION__, ret);
			usleep(1000);
		}
	}
}

unsigned char T9_plus_write_pic_iic(bool read, bool reg_addr_valid, unsigned char reg_addr, unsigned char which_iic, unsigned char data)
{
	unsigned int value = 0x00000000, counter = 0;
	unsigned int ret = 0;
	
	while(1)
    {
        ret = get_iic();
		//printf("iic command ret = 0x%08x\n", ret);
        if(ret & 0x80000000)
        {
			break;
		}
#if 0
		else if(counter++>3)	// Attention:  can not wait for more than 3 times, because PIC will timeout when get no data, in fact PIC recveived data ,but FPGA's PIC flag is not ready!? maybe a bug
		{
		//	printf("^^^^^^^^^^^^^^^^^^^^^\n");
			break;
		}
#endif
		usleep(1*1000);
	}
	
	if(read)
	{
		value |= IIC_READ;
	}

	if(reg_addr_valid)
	{
		value |= IIC_REG_ADDR_VALID;
		value |= IIC_REG_ADDR(reg_addr);
	}

	value |= IIC_ADDR_HIGH_4_BIT;

	if(fpga_version>=0xE && fpga_version<=0xF)//  decrease one , because chain[1] [2] [3] use PIC [0] [1] [2]
		which_iic--;

	value |= IIC_SELECT(which_iic);

	value |= data;

	return set_iic(value);
}

int dsPIC33EP16GS202_erase_pic_app_program(unsigned char which_iic)
{
	unsigned char length = 0x04, crc_data[2] = {0xff}, read_back_data[2] = {0xff};
	unsigned short crc = 0;
	char logstr[256];
	int retry_count=0;

#ifdef DEBUG_IGNORE_T9_18_PIC_CMD
	return 0;
#endif

	printf("\n--- %s\n", __FUNCTION__);
	
	crc = length + ERASE_PIC_APP_PROGRAM;
	crc_data[0] = (unsigned char)((crc >> 8) & 0x00ff);
	crc_data[1] = (unsigned char)((crc >> 0) & 0x00ff);
	printf("--- %s: crc_data[0] = 0x%x, crc_data[1] = 0x%x\n", __FUNCTION__, crc_data[0], crc_data[1]);

	while(retry_count++<3)
	{
		pthread_mutex_lock(&iic_mutex);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, PIC_COMMAND_1);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, PIC_COMMAND_2);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, length);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, ERASE_PIC_APP_PROGRAM);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, crc_data[0]);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, crc_data[1]);
		usleep(100*1000);
		read_back_data[0] = T9_plus_write_pic_iic(true, false, 0x0, which_iic, 0);
		read_back_data[1] = T9_plus_write_pic_iic(true, false, 0x0, which_iic, 0);
		pthread_mutex_unlock(&iic_mutex);
		
		printf("--- %s: read_back_data[0] = 0x%x, read_back_data[1] = 0x%x\n", __FUNCTION__, read_back_data[0], read_back_data[1]);
		usleep(200*1000);
		
		if((read_back_data[0] != ERASE_PIC_APP_PROGRAM) || (read_back_data[1] != 1))
		{
			sprintf(logstr,"%s failed on Chain[%d]!\n", __FUNCTION__,which_iic);
			writeLogFile(logstr);
			sleep(1);
		//	return 0;	// error
		}
		else
		{
			printf("\n--- %s ok\n\n", __FUNCTION__);
			return 1;	// ok
		}
	}
	return 0;
}

int dsPIC33EP16GS202_send_data_to_pic(unsigned char which_iic, unsigned char *buf)
{
	unsigned char length = 0x14, crc_data[2] = {0xff}, read_back_data[2] = {0xff}, i;
	unsigned short crc = 0;
	char logstr[256];
	int retry_count=0;

#ifdef DEBUG_IGNORE_T9_18_PIC_CMD
	return 0;
#endif

	printf("\n--- %s\n", __FUNCTION__);
	
	crc = length + SEND_DATA_TO_IIC;
	for(i=0; i<16; i++)
	{
		crc += *(buf + i);
	}
	crc_data[0] = (unsigned char)((crc >> 8) & 0x00ff);
	crc_data[1] = (unsigned char)((crc >> 0) & 0x00ff);
	printf("--- %s: crc_data[0] = 0x%x, crc_data[1] = 0x%x\n", __FUNCTION__, crc_data[0], crc_data[1]);

	while(retry_count++<3)
	{
		pthread_mutex_lock(&iic_mutex);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, PIC_COMMAND_1);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, PIC_COMMAND_2);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, length);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, SEND_DATA_TO_IIC);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, *(buf + 0));
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, *(buf + 1));
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, *(buf + 2));
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, *(buf + 3));
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, *(buf + 4));
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, *(buf + 5));
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, *(buf + 6));
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, *(buf + 7));
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, *(buf + 8));
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, *(buf + 9));
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, *(buf + 10));
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, *(buf + 11));
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, *(buf + 12));
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, *(buf + 13));
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, *(buf + 14));
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, *(buf + 15));	
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, crc_data[0]);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, crc_data[1]);
		usleep(200*1000);
		read_back_data[0] = T9_plus_write_pic_iic(true, false, 0x0, which_iic, 0);
		read_back_data[1] = T9_plus_write_pic_iic(true, false, 0x0, which_iic, 0);
		pthread_mutex_unlock(&iic_mutex);
		
		printf("--- %s: read_back_data[0] = 0x%x, read_back_data[1] = 0x%x\n", __FUNCTION__, read_back_data[0], read_back_data[1]);
		usleep(100*1000);
		
		if((read_back_data[0] != SEND_DATA_TO_IIC) || (read_back_data[1] != 1))
		{
			sprintf(logstr,"%s failed on Chain[%d]!\n", __FUNCTION__,which_iic);
			writeLogFile(logstr);
			sleep(1);
		//	return 0;	// error
		}
		else
		{
			printf("\n--- %s ok\n\n", __FUNCTION__);
			return 1;	// ok
		}
	}
	return 0;
}

int dsPIC33EP16GS202_update_pic_app_program(unsigned char which_iic)
{
	unsigned char program_data[14080] = {0};
	FILE * pic_program_file;
	unsigned int filesize = 0,i=0,j;
	unsigned char data_read[7]={0,0,0,0,0,0,'\0'}, buf[16]={0};
	unsigned int data_int = 0;
	struct stat statbuff; 	
	unsigned int data_len = 3520, loop = 880;
	unsigned int pic_flash_length=0;
	int ret = 0;
	
	printf("\n--- update pic program\n");

	// read upgrade file first, if it is wrong, don't erase pic, but just return;
	pic_program_file = fopen(DSPIC33EP16GS202_PIC_PROGRAM, "r");
	if(!pic_program_file)
	{
		printf("\n%s: open hash_s8_app.txt failed\n", __FUNCTION__);
		return -1;
	}
	fseek(pic_program_file,0,SEEK_SET);
	memset(program_data, 0x0, 14080);
	
	for(i=0; i<data_len; i++)
	{
		fgets(data_read, MAX_CHAR_NUM - 1 , pic_program_file);
		//printf("data_read[0]=%c, data_read[1]=%c, data_read[2]=%c, data_read[3]=%c, data_read[4]=%c, data_read[5]=%c\n",
		//	data_read[0], data_read[1], data_read[2], data_read[3], data_read[4], data_read[5]);
		
		data_int = strtoul(data_read, NULL, 16);
		//printf("data_int = 0x%08x\n", data_int);
		program_data[4*i + 0] = (unsigned char)((data_int >> 24) & 0x000000ff);
		program_data[4*i + 1] = (unsigned char)((data_int >> 16) & 0x000000ff);
		program_data[4*i + 2] = (unsigned char)((data_int >> 8) & 0x000000ff);
		program_data[4*i + 3] = (unsigned char)((data_int >> 0) & 0x000000ff);
		
		//printf("program_data[%d]=0x%02x, program_data[%d]=0x%02x, program_data[%d]=0x%02x, program_data[%d]=0x%02x\n\n", 
		//	4*i + 0, program_data[4*i + 0], 4*i + 1, program_data[4*i + 1], 4*i + 2, program_data[4*i + 2], 4*i + 3, program_data[4*i + 3]);
	}

	fclose(pic_program_file);


	// after read upgrade file correct, erase pic
	ret = dsPIC33EP16GS202_reset_pic(which_iic);
	if(ret == 0)
	{
		printf("!!! %s: reset pic error!\n\n", __FUNCTION__);
		return 0;
	}
	
	ret = dsPIC33EP16GS202_erase_pic_app_program(which_iic);
	if(ret == 0)
	{
		printf("!!! %s: erase flash error!\n\n", __FUNCTION__);
		return 0;
	}

	for(i=0; i<loop; i++)
	{
		memcpy(buf, program_data+i*16, 16);
		/**/
		printf("send pic program time: %d\n",i);
		for(j=0;j<16;j++)
		{
			printf("buf[%d] = 0x%02x\n", j, *(buf+j));
		}
		printf("\n");

		ret = dsPIC33EP16GS202_send_data_to_pic(which_iic, buf);
		if(ret == 0)
		{
			printf("!!! %s: send flash data error!\n\n", __FUNCTION__);
			return 0;
		}
		//usleep(200*1000);
	}

	ret = dsPIC33EP16GS202_reset_pic(which_iic);
	if(ret == 0)
	{
		printf("!!! %s: reset pic error!\n\n", __FUNCTION__);
		return 0;
	}

	return 1;
}


int dsPIC33EP16GS202_pic_heart_beat(unsigned char which_iic)
{
	unsigned char length = 0x04, crc_data[2] = {0xff}, read_back_data[6] = {0xff};
	unsigned short crc = 0;
	char logstr[256];
	int retry_count=0;

#ifdef DEBUG_IGNORE_T9_18_PIC_CMD
	return 0;
#endif

	crc = length + SEND_HEART_BEAT;
	crc_data[0] = (unsigned char)((crc >> 8) & 0x00ff);
	crc_data[1] = (unsigned char)((crc >> 0) & 0x00ff);
	printf("--- %s: crc_data[0] = 0x%x, crc_data[1] = 0x%x\n", __FUNCTION__, crc_data[0], crc_data[1]);

	while(retry_count++<3)
	{
		pthread_mutex_lock(&iic_mutex);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, PIC_COMMAND_1);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, PIC_COMMAND_2);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, length);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, SEND_HEART_BEAT);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, crc_data[0]);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, crc_data[1]);
		usleep(500*1000);
		read_back_data[0] = T9_plus_write_pic_iic(true, false, 0x0, which_iic, 0);
		read_back_data[1] = T9_plus_write_pic_iic(true, false, 0x0, which_iic, 0);
		read_back_data[2] = T9_plus_write_pic_iic(true, false, 0x0, which_iic, 0);
		read_back_data[3] = T9_plus_write_pic_iic(true, false, 0x0, which_iic, 0);
		read_back_data[4] = T9_plus_write_pic_iic(true, false, 0x0, which_iic, 0);
		read_back_data[5] = T9_plus_write_pic_iic(true, false, 0x0, which_iic, 0);
		pthread_mutex_unlock(&iic_mutex);

		printf("--- %s: read_back_data[0] = 0x%x, read_back_data[1] = 0x%x, read_back_data[2] = 0x%x, read_back_data[3] = 0x%x, read_back_data[4] = 0x%x, read_back_data[5] = 0x%x\n", 
				__FUNCTION__, read_back_data[0], read_back_data[1], read_back_data[2], read_back_data[3], read_back_data[4], read_back_data[5]);
		
		if((read_back_data[1] != SEND_HEART_BEAT) || (read_back_data[2] != 1))
		{
			sprintf(logstr,"%s failed on Chain[%d]!\n", __FUNCTION__,which_iic);
			writeLogFile(logstr);
			sleep(1);
		//	return 0;	// error
		}
		else
		{
			printf("\n--- %s ok, HeartBeatReturnWord = %d\n\n", __FUNCTION__, read_back_data[3]);
			return 1;	// ok
		}
	}

	return 0;
}

void pic_heart_beat(unsigned char chain)
{
	if(fpga_version>=0xE)
	{
		if(chain<1 || chain>3) // only enable DC when enable the first chain of 3 chains, like 0,  3,  6 ...
			return;

		dsPIC33EP16GS202_pic_heart_beat(chain);
	}
	else
	{
		if(chain%3!=0) // only enable DC when enable the first chain of 3 chains, like 0,  3,  6 ...
			return;

		dsPIC33EP16GS202_pic_heart_beat(chain/3);
	}
}
#else
void enable_pic_dac(unsigned char chain)
{
    printf("\n--- %s\n", __FUNCTION__);
    send_pic_command(chain);

	pthread_mutex_lock(&iic_mutex);
    write_pic_iic(false, false, 0x0, chain, ENABLE_VOLTAGE);
    write_pic_iic(false, false, 0x0, chain, 1);
	pthread_mutex_unlock(&iic_mutex);
}

void disable_pic_dac(unsigned char chain)
{
    //printf("\n--- %s\n", __FUNCTION__);
    send_pic_command(chain);
	pthread_mutex_lock(&iic_mutex);
    write_pic_iic(false, false, 0x0, chain, ENABLE_VOLTAGE);
    write_pic_iic(false, false, 0x0, chain, 0);
	pthread_mutex_unlock(&iic_mutex);
}

void pic_heart_beat(unsigned char chain)
{
    printf("--- send pic heart beat ... \n");
    send_pic_command(chain);
	pthread_mutex_lock(&iic_mutex);
    write_pic_iic(false, false, 0x0, chain, SEND_HEART_BEAT);
	pthread_mutex_unlock(&iic_mutex);
}
#endif

void update_pic_program(unsigned char chain)
{
    unsigned char program_data[10*MAX_CHAR_NUM] = {0};
    FILE * pic_program_file;
    unsigned int i=0,j;
    unsigned char data_read[5]= {0,0,0,0,'\0'}, buf[16]= {0};
    unsigned int data_int = 0;
    unsigned char start_addr_h = PIC_FLASH_POINTER_START_ADDRESS_H, start_addr_l = PIC_FLASH_POINTER_START_ADDRESS_L;
    unsigned char end_addr_h = PIC_FLASH_POINTER_END_ADDRESS_H, end_addr_l = PIC_FLASH_POINTER_END_ADDRESS_L;
    unsigned int pic_flash_length=0;

    printf("\n--- update pic program\n");

    // read upgrade file first, if it is wrong, don't erase pic, but just return;
    pic_program_file = fopen(PIC_PROGRAM, "r");
    if(!pic_program_file)
    {
        printf("\n%s: open hash_s8_app.txt failed\n", __FUNCTION__);
        return;
    }
    fseek(pic_program_file,0,SEEK_SET);
    memset(program_data, 0x0, 10*MAX_CHAR_NUM);

    pic_flash_length = (((unsigned int)end_addr_h << 8) + end_addr_l) - (((unsigned int)start_addr_h << 8) + start_addr_l) + 1;
    printf("pic_flash_length = %d\n", pic_flash_length);

    for(i=0; i<pic_flash_length; i++)
    {
        fgets((char *)data_read, MAX_CHAR_NUM - 1 , pic_program_file);
        //printf("data_read[0]=%c, data_read[1]=%c, data_read[2]=%c, data_read[3]=%c\n", data_read[0], data_read[1], data_read[2], data_read[3]);
        data_int = strtoul((char *)data_read, NULL, 16);
        //printf("data_int = 0x%04x\n", data_int);
        program_data[2*i + 0] = (unsigned char)((data_int >> 8) & 0x000000ff);
        program_data[2*i + 1] = (unsigned char)(data_int & 0x000000ff);
        //printf("program_data[%d]=0x%02x, program_data[%d]=0x%02x\n\n", 2*i + 0, program_data[2*i + 0], 2*i + 1, program_data[2*i + 1]);
    }

    fclose(pic_program_file);

    // after read upgrade file correct, erase pic
    reset_iic_pic(chain);
    erase_pic_flash_all(chain);

    // write data into pic
    set_pic_iic_flash_addr_pointer(chain, PIC_FLASH_POINTER_START_ADDRESS_H, PIC_FLASH_POINTER_START_ADDRESS_L);

    for(i=0; i<pic_flash_length/PIC_FLASH_SECTOR_LENGTH*4; i++)
    {
        memcpy(buf, program_data+i*16, 16);
        /*
        printf("send pic program time: %d\n",i);
        for(j=0; j<16; j++)
        {
            printf("buf[%d] = 0x%02x\n", j, *(buf+j));
        }
        printf("\n");
		*/

        send_data_to_pic_flash(chain, buf);
        write_data_into_pic_flash(chain);
    }

	usleep(500000);
}

#ifdef T9_18
int dsPIC33EP16GS202_get_pic_sw_version(unsigned char which_iic, unsigned char *version)
{
	unsigned char length = 0x04, crc_data[2] = {0xff}, read_back_data[5] = {0xff};
	unsigned short crc = 0;
	char logstr[256];
	int retry_count=0;

#ifdef DEBUG_IGNORE_T9_18_PIC_CMD
	return 0;
#endif

	printf("\n--- %s\n", __FUNCTION__);

	*version = 0xff;
	
	crc = length + GET_PIC_SOFTWARE_VERSION;
	crc_data[0] = (unsigned char)((crc >> 8) & 0x00ff);
	crc_data[1] = (unsigned char)((crc >> 0) & 0x00ff);
	printf("--- %s: crc_data[0] = 0x%x, crc_data[1] = 0x%x\n", __FUNCTION__, crc_data[0], crc_data[1]);

	while(retry_count++<3)
	{
		pthread_mutex_lock(&iic_mutex);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, PIC_COMMAND_1);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, PIC_COMMAND_2);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, length);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, GET_PIC_SOFTWARE_VERSION);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, crc_data[0]);
		T9_plus_write_pic_iic(false, false, 0x0, which_iic, crc_data[1]);
		usleep(100*1000);
		read_back_data[0] = T9_plus_write_pic_iic(true, false, 0x0, which_iic, 0);
		read_back_data[1] = T9_plus_write_pic_iic(true, false, 0x0, which_iic, 0);
		read_back_data[2] = T9_plus_write_pic_iic(true, false, 0x0, which_iic, 0);
		read_back_data[3] = T9_plus_write_pic_iic(true, false, 0x0, which_iic, 0);
		read_back_data[4] = T9_plus_write_pic_iic(true, false, 0x0, which_iic, 0);	
		pthread_mutex_unlock(&iic_mutex);
		
		printf("--- %s: read_back_data[0] = 0x%x, read_back_data[1] = 0x%x\n", __FUNCTION__, read_back_data[0], read_back_data[1]);
		usleep(100*1000);
		
		if((read_back_data[1] != GET_PIC_SOFTWARE_VERSION) || (read_back_data[0] != 5))
		{
			sprintf(logstr,"%s failed on Chain[%d]!\n", __FUNCTION__,which_iic);
			writeLogFile(logstr);
			// return 0;	// error
			sleep(1);
		}
		else
		{
			crc = read_back_data[0] + read_back_data[1] + read_back_data[2];
			if(((unsigned char)((crc >> 8) & 0x00ff) != read_back_data[3]) || ((unsigned char)((crc >> 0) & 0x00ff) != read_back_data[4]))
			{
				printf("\n--- %s failed!\n\n", __FUNCTION__);
				//return 0;	// error
				sleep(1);
			}
			else
			{
				*version = read_back_data[2];
				printf("\n--- %s ok\n\n", __FUNCTION__);
				return 1;	// ok
			}
		}
	}

	return 0;
}

void get_pic_software_version(unsigned char chain, unsigned char *version)
{
	if(fpga_version>=0xE)
	{
		int new_T9_PLUS_chainIndex,new_T9_PLUS_chainOffset;
		getPICChainIndexOffset(chain,&new_T9_PLUS_chainIndex,&new_T9_PLUS_chainOffset);
		dsPIC33EP16GS202_get_pic_sw_version(new_T9_PLUS_chainIndex,version);
	}
	else
	{
    	dsPIC33EP16GS202_get_pic_sw_version(chain/3,version);
	}
}

unsigned char get_pic_voltage(unsigned char chain)
{
	if(fpga_version>=0xE)
	{
		int new_T9_PLUS_chainIndex,new_T9_PLUS_chainOffset;
		getPICChainIndexOffset(chain,&new_T9_PLUS_chainIndex,&new_T9_PLUS_chainOffset);
		return AT24C02_read_voltage(new_T9_PLUS_chainIndex);
	}
	else
	{
		return AT24C02_read_voltage(chain/3);
	}
}
#else
void get_pic_software_version(unsigned char chain, unsigned char *version)
{
    int i=0;

    printf("%s\n", __FUNCTION__);

    send_pic_command(chain);
	pthread_mutex_lock(&iic_mutex);
    write_pic_iic(false, false, 0x0, chain, GET_PIC_SOFTWARE_VERSION);

    for(i=0; i<PIC_SOFTWARE_VERSION_LENGTH; i++)
    {
        *(version + i) = write_pic_iic(true, false, 0x0, chain, 0);
        printf("%s: version[%d] = 0x%x\n", __FUNCTION__, i, *(version + i));
    }
	pthread_mutex_unlock(&iic_mutex);
}

unsigned char get_pic_voltage(unsigned char chain)
{
    unsigned char ret=0;

    printf("%s\n", __FUNCTION__);

    send_pic_command(chain);
	pthread_mutex_lock(&iic_mutex);
    write_pic_iic(false, false, 0x0, chain, GET_VOLTAGE);
    ret = write_pic_iic(true, false, 0x0, chain, 0);
	pthread_mutex_unlock(&iic_mutex);
	
    printf("%s: voltage = 0x%x\n", __FUNCTION__, ret);
    return ret;
}
#endif

void get_pic_voltage_setting_time(unsigned char chain, unsigned char *time)
{
    int i=0;

    printf("%s\n", __FUNCTION__);

    send_pic_command(chain);

	pthread_mutex_lock(&iic_mutex);
    write_pic_iic(false, false, 0x0, chain, GET_DATE);

    for(i=0; i<PIC_VOLTAGE_TIME_LENGTH; i++)
    {
        *(time + i) = write_pic_iic(true, false, 0x0, chain, 0);
        printf("%s: time[%d] = 0x%02x\n", __FUNCTION__, i, *(time + i));
    }
	pthread_mutex_unlock(&iic_mutex);
}

void get_pic_mac_and_time(unsigned char chain, unsigned char which_mac, unsigned char *mac, unsigned char *date)
{
    int i=0;
    unsigned char buf[16] = {0};

    printf("%s\n", __FUNCTION__);

    if(which_mac > 1)
    {
        printf("%s: There are only 2 mac address, they are MAC0 to MAC1\n", __FUNCTION__);
        return;
    }

    send_pic_command(chain);

	pthread_mutex_lock(&iic_mutex);
    write_pic_iic(false, false, 0x0, chain, GET_WHICH_MAC);
    write_pic_iic(false, false, 0x0, chain, which_mac);
    usleep(100000);
	pthread_mutex_unlock(&iic_mutex);

    send_pic_command(chain);
	
	pthread_mutex_lock(&iic_mutex);
    write_pic_iic(false, false, 0x0, chain, GET_MAC);

    for(i=0; i<16; i++)
    {
        *(buf + i) = write_pic_iic(true, false, 0x0, chain, 0);
        //printf("%s: buf[%d] = 0x%02x\n", __FUNCTION__, i, *(buf + i));
    }
	pthread_mutex_unlock(&iic_mutex);
	
    *(mac + 0) = *(buf + 0);
    *(mac + 1) = *(buf + 2);
    *(mac + 2) = *(buf + 4);
    *(mac + 3) = *(buf + 6);
    *(mac + 4) = *(buf + 8);
    *(mac + 5) = *(buf + 10);
    for(i=0; i<6; i++)
    {
        printf("%s: mac[%d] = 0x%02x\n", __FUNCTION__, i, *(mac + i));
    }

    *(date + 0) = *(buf + 1);
    *(date + 1) = *(buf + 3);
    *(date + 2) = *(buf + 5);
    *(date + 3) = *(buf + 7);
    *(date + 4) = *(buf + 12);
    *(date + 5) = *(buf + 14);
    for(i=0; i<6; i++)
    {
        printf("%s: date[%d] = 0x%02x\n", __FUNCTION__, i, *(date + i));
    }
}

void set_temperature_offset_value(unsigned char chain, unsigned char *value)
{
    send_pic_command(chain);
    send_data_to_pic_iic(chain, WR_TEMP_OFFSET_VALUE, value, 8);
    usleep(100000);
}

#ifdef T9_18
// only the first chain of hashboard has temp sensor!!!
void get_temperature_offset_value(unsigned char chain, unsigned char *value)
{
	if(fpga_version>=0xE)
	{
		int new_T9_PLUS_chainIndex,new_T9_PLUS_chainOffset;
		getPICChainIndexOffset(chain,&new_T9_PLUS_chainIndex,&new_T9_PLUS_chainOffset);
		AT24C02_read_bytes(SENSOR_OFFSET_ADDR, value, new_T9_PLUS_chainIndex, 8);
	}
	else
	{
		AT24C02_read_bytes(SENSOR_OFFSET_ADDR, value, (chain/3), 8);
	}
}
#else
void get_temperature_offset_value(unsigned char chain, unsigned char *value)
{
    send_pic_command(chain);
    get_data_from_pic_iic(chain, RD_TEMP_OFFSET_VALUE, value, 8);
}
#endif

static bool sock_connecting(void)
{
    return errno == EINPROGRESS;
}

static inline bool interrupted(void)
{
    return (errno == EINTR);
}

void get_time(void)
{
    time_t timep, now, set_timep;
    struct tm *p, *timenow, set_time;

    int i,ret;
    long sock;
    struct sockaddr_in serv;
    struct timeval tm_socket, tv;
    fd_set set;
    int error=-1;
	unsigned int len;
    int selret;
    unsigned int IsGetTime = 0;

    printf("\n--- get time\n");
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0)
    {
        printf("Socket initialisation failed\n");
        printf("Can't get time\n");
    }
    else
    {
        printf("Socket initialisation ok\n");
    }

    memset(&serv, 0, sizeof(serv));
    serv.sin_family = AF_INET;
    serv.sin_addr.s_addr = inet_addr("202.108.22.5");
    serv.sin_port = htons(80);

    ret = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, ret | O_NONBLOCK);
    if(connect(sock, (struct sockaddr *)&serv, sizeof(struct sockaddr)) == -1)
    {
        tm_socket.tv_sec  = 1;
        tm_socket.tv_usec = 0;

        if (!sock_connecting())
        {
            close(sock);
            printf("Socket isn't connecting\n");
        }

    retry:
        FD_ZERO(&set);
        FD_SET(sock, &set);

        selret = select(sock+1, NULL, &set, NULL, &tm_socket);

        if(selret > 0 && FD_ISSET(sock, &set))
        {
            len = sizeof(error);
            i=getsockopt(sock, SOL_SOCKET, SO_ERROR, &error, &len);
            if(!i && !error)
            {
                ret = fcntl(sock, F_GETFL, 0);
                fcntl(sock, F_SETFL, ret & ~O_NONBLOCK);
                printf("Socket connect ok\n");
                time(&timep);
                p=localtime(&timep);
                printf("year:%04d, month:%02d, day:%02d\n", (1900+p->tm_year),(1+p->tm_mon), p->tm_mday);
                printf("hour:%02d, minute:%02d, second:%02d\n", p->tm_hour, p->tm_min, p->tm_sec);
                change_time_format(p, time_data);
                IsGetTime = 1;
            }
        }

        if(selret < 0 && interrupted())
        {
            goto retry;
        }

        close(sock);
    }
    else
    {
        ret = fcntl(sock, F_GETFL, 0);
        fcntl(sock, F_SETFL, ret & ~O_NONBLOCK);
        printf("Socket connect ok ...\n");
        time(&timep);
        p=localtime(&timep);
        printf("year:%04d, month:%02d, day:%02d\n", (1900+p->tm_year),(1+p->tm_mon), p->tm_mday);
        printf("hour:%02d, minute:%02d, second:%02d\n", p->tm_hour, p->tm_min, p->tm_sec);
        change_time_format(p, time_data);
        IsGetTime = 1;
    }

    if(IsGetTime == 0)
    {
        if(gIsSetTime)
        {
            printf("%s: Start set system date\n", __FUNCTION__);

            set_time.tm_year = conf.year - 1900;
            set_time.tm_mon = conf.month - 1;
            set_time.tm_mday = conf.date;
            set_time.tm_min = conf.minute;
            set_time.tm_sec = conf.second;
            if((conf.hour >= 0) && (conf.hour <= 15))
            {
                set_time.tm_hour = conf.hour + 8;
            }
            else if((conf.hour >= 16) && (conf.hour <= 23))
            {
                set_time.tm_hour = conf.hour - 16;
            }
            else
            {
                printf("%s: hour value in Config.ini is error!\n", __FUNCTION__);
            }

            set_timep = mktime(&set_time);
            tv.tv_sec = set_timep;
            tv.tv_usec = 0;

            if(settimeofday (&tv, (struct timezone *) 0) < 0)
            {
                printf("Set system datatime error!\n");
            }
            else
            {
                printf("Set system datatime ok!\n");
            }


            gIsSetTime = 0;
        }

        time(&now);
        timenow = localtime(&now);
        printf("UTC time is : %s\n", asctime(timenow));
        change_time_format(timenow, time_data);

        /*
        printf("%s: year = 0x%02x\n", __FUNCTION__, *(time_data + 0));
        printf("%s: month = 0x%02x\n", __FUNCTION__, *(time_data + 1));
        printf("%s: day = 0x%02x\n", __FUNCTION__, *(time_data + 2));
        printf("%s: hour = 0x%02x\n", __FUNCTION__, *(time_data + 3));
        printf("%s: minute = 0x%02x\n", __FUNCTION__, *(time_data + 4));
        printf("%s: second = 0x%02x\n", __FUNCTION__, *(time_data + 5));
        */
    }
}

void enable_read_write_NCT218()
{
    unsigned char buf[9] = {0,0,0,0,0,0,0,0,0};
    unsigned int cmd_buf[3] = {0,0,0};
    unsigned int ret, value, which_sensor;
	int i;

    printf("\n--- %s\n", __FUNCTION__);

    if((Conf.CommandMode == VIL) && (conf.GetTempFrom == 1))
    {
        for(i=0; i<4; i++)
        {
            if(i == 0)
            {
                which_sensor = conf.TempSensor1;
            }

            if(i == 1)
            {
                which_sensor = conf.TempSensor2;
            }

            if(i == 2)
            {
                which_sensor = conf.TempSensor3;
            }

            if(i == 3)
            {
                which_sensor = conf.TempSensor4;
            }

            if(which_sensor)
            {
                for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
                {
                    if(cgpu.chain_exist[i] == 1)
                    {
                        if(!cgpu.CommandMode)   // vil mode
                        {
                            buf[0] = VIL_COMMAND_TYPE | SET_CONFIG;
                            buf[1] = 0x09;
                            buf[2] = CHIP_ADDR_INTERVAL*(which_sensor-1);
                            buf[3] = MISC_CONTROL;
                            buf[4] = 0x40;
                            buf[5] = INV_CLKO | cgpu.temp_sel;
                            buf[6] = (cgpu.baud & 0x1f) | (cgpu.rfs << 6);
                            buf[7] = cgpu.tfs << 5;
                            buf[8] = CRC5(buf, 8*8);

                            cmd_buf[0] = buf[0]<<24 | buf[1]<<16 | buf[2]<<8 | buf[3];
                            cmd_buf[1] = buf[4]<<24 | buf[5]<<16 | buf[6]<<8 | buf[7];
                            cmd_buf[2] = buf[8]<<24;
                            //printf("%s: cmd_buf[0]=0x%x, cmd_buf[1]=0x%x, cmd_buf[2]=0x%x\n", __FUNCTION__, cmd_buf[0], cmd_buf[1], cmd_buf[2]);

                            set_BC_command_buffer(cmd_buf);
                            ret = get_BC_write_command();
                            value = BC_COMMAND_BUFFER_READY | BC_COMMAND_EN_CHAIN_ID| (i << 16) | (ret & 0xfff0ffff);
                            set_BC_write_command(value);
                        }
                    }
                }
            }
        }
    }
}

void set_config(int chainIndex, unsigned char mode, unsigned char asic_addr, unsigned char reg_addr, unsigned int reg_data)
{
    unsigned char buf[9] = {0,0,0,0,0,0,0,0,0};
    unsigned int cmd_buf[3] = {0,0,0};
    unsigned int ret, value;

    //printf("\n--- %s\n", __FUNCTION__);

    buf[0] = VIL_COMMAND_TYPE | SET_CONFIG;
    if(mode)
        buf[0] |= VIL_ALL;

    buf[1] = 9;
    buf[2] = asic_addr;
    buf[3] = reg_addr;
    buf[4] = (unsigned char)((reg_data >> 24) & 0xff);
    buf[5] = (unsigned char)((reg_data >> 16) & 0xff);
    buf[6] = (unsigned char)((reg_data >> 8) & 0xff);
    buf[7] = (unsigned char)((reg_data >> 0) & 0xff);
    buf[8] = CRC5(buf, 8*8);

    cmd_buf[0] = buf[0]<<24 | buf[1]<<16 | buf[2]<<8 | buf[3];
    cmd_buf[1] = buf[4]<<24 | buf[5]<<16 | buf[6]<<8 | buf[7];
    cmd_buf[2] = buf[8]<<24;
    //printf("%s: cmd_buf[0]=0x%x, cmd_buf[1]=0x%x, cmd_buf[2]=0x%x\n", __FUNCTION__, cmd_buf[0], cmd_buf[1], cmd_buf[2]);

    set_BC_command_buffer(cmd_buf);
    ret = get_BC_write_command();
    value = BC_COMMAND_BUFFER_READY | BC_COMMAND_EN_CHAIN_ID| (chainIndex << 16) | (ret & 0xfff0ffff);
    set_BC_write_command(value);
}


void enable_read_temperature_from_asic(int chainIndex, unsigned char addr)
{
    unsigned char buf[9] = {0,0,0,0,0,0,0,0,0};
    unsigned int cmd_buf[3] = {0,0,0};
    unsigned int ret, value,i;

   // printf("\n--- %s\n", __FUNCTION__);

	i=chainIndex;
//    for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
    {
        if(cgpu.chain_exist[i] == 1
#ifndef ALL_CHAIN_INTEST
			&& (!testDone[i])
#endif
			)
        {
            if(!cgpu.CommandMode)   // vil mode
            {
                buf[0] = VIL_COMMAND_TYPE | SET_CONFIG;
                buf[1] = 0x09;
                buf[2] = addr;
                buf[3] = MISC_CONTROL;
                buf[4] = 0x40;
                buf[5] = INV_CLKO | cgpu.temp_sel;
                buf[6] = (cgpu.baud & 0x1f) | (cgpu.rfs << 6);
                buf[7] = cgpu.tfs << 5;
                buf[8] = CRC5(buf, 8*8);

                cmd_buf[0] = buf[0]<<24 | buf[1]<<16 | buf[2]<<8 | buf[3];
                cmd_buf[1] = buf[4]<<24 | buf[5]<<16 | buf[6]<<8 | buf[7];
                cmd_buf[2] = buf[8]<<24;
                //printf("%s: cmd_buf[0]=0x%x, cmd_buf[1]=0x%x, cmd_buf[2]=0x%x\n", __FUNCTION__, cmd_buf[0], cmd_buf[1], cmd_buf[2]);

                set_BC_command_buffer(cmd_buf);
                ret = get_BC_write_command();
                value = BC_COMMAND_BUFFER_READY | BC_COMMAND_EN_CHAIN_ID| (i << 16) | (ret & 0xfff0ffff);
                set_BC_write_command(value);
            }
        }
    }
}

signed char get_temperature_offset_value_from_asic(int chainIndex)
{
    unsigned int ret, which_sensor, read_temperature_time = 0;
    signed char local_temp=0, remote_temp=0, temp_offset_value=0;
	char logstr[256];
	
    printf("\n--- %s\n", __FUNCTION__);

    which_sensor = temp_chip_index[chainIndex];

    enable_read_temperature_from_asic(chainIndex,CHIP_ADDR_INTERVAL*(which_sensor-1));
    check_one_asic_reg(chainIndex, MISC_CONTROL, CHIP_ADDR_INTERVAL*(which_sensor-1));

    do
    {
        ret = check_one_asic_reg(chainIndex, GENERAL_I2C_COMMAND, CHIP_ADDR_INTERVAL*(which_sensor-1));
        read_temperature_time++;
    }
    while((ret & 0x80000000) && (read_temperature_time < read_loop));
    read_temperature_time = 0;

    set_config(chainIndex, 0, CHIP_ADDR_INTERVAL*(which_sensor-1), GENERAL_I2C_COMMAND, REGADDRVALID | DEVICEADDR | REGADDR(EXTERNAL_TEMPERATURE_VALUE_HIGH_BYTE) | (DATA(0) & (~RW)));
    do
    {
        ret = check_one_asic_reg(chainIndex, GENERAL_I2C_COMMAND, CHIP_ADDR_INTERVAL*(which_sensor-1));
        read_temperature_time++;
    }
    while((ret & 0x80000000) && (read_temperature_time < read_loop));
    read_temperature_time = 0;

    remote_temp = (signed char)(ret & 0xff);
    
    sprintf(logstr,"Chain[%d] ASIC temperature is %d\n",chainIndex, remote_temp);
	writeLogFile(logstr);
	
    set_config(chainIndex, 0, CHIP_ADDR_INTERVAL*(which_sensor-1), GENERAL_I2C_COMMAND, REGADDRVALID | DEVICEADDR | REGADDR(LOCAL_TEMPERATURE_VALUE) | (DATA(0) & (~RW)));
    do
    {
        ret = check_one_asic_reg(chainIndex, GENERAL_I2C_COMMAND, CHIP_ADDR_INTERVAL*(which_sensor-1));
        read_temperature_time++;
    }
    while((ret & 0x80000000) && (read_temperature_time < read_loop));
    read_temperature_time = 0;

    local_temp = (signed char)(ret & 0xff);

	sprintf(logstr,"Chain[%d] PCB temperature is %d\n",chainIndex, local_temp);
	writeLogFile(logstr);
	
    temp_offset_value = local_temp - remote_temp;

	if(remote_temp==0)
		chip_temp_offset[chainIndex]+=30;	//read error, maybe default -70 is too large
	else
		chip_temp_offset[chainIndex]+=temp_offset_value;
	
    sprintf(logstr,"Chain[%d] new temp_offset_value = %d\n", chainIndex, chip_temp_offset[chainIndex]);
	writeLogFile(logstr);

	return chip_temp_offset[chainIndex];
}

void set_default_temperature_offset_value(int chainIndex)
{
    unsigned int which_sensor = 0, read_temperature_time = 0;
    unsigned int data = 0, ret = 0;
    signed char offset = 0;
	char logstr[256];

    data = 0;
	which_sensor = temp_chip_index[chainIndex];

	sprintf(logstr,"set default temp offset=%d on chip[%d] of chain[%d]\n",chip_temp_offset[chainIndex],which_sensor,chainIndex);
	writeLogFile(logstr);
	
    data = 0x000000ff & DATA(chip_temp_offset[chainIndex]);
    data |= REGADDRVALID | DEVICEADDR | REGADDR(EXTERNAL_TEMPERATURE_OFFSET_HIGH_BYTE) | RW;

    enable_read_temperature_from_asic(chainIndex,CHIP_ADDR_INTERVAL*(which_sensor-1));
    check_one_asic_reg(chainIndex, MISC_CONTROL, CHIP_ADDR_INTERVAL*(which_sensor-1));

    do
    {
        ret = check_one_asic_reg(chainIndex, GENERAL_I2C_COMMAND, CHIP_ADDR_INTERVAL*(which_sensor-1));
        read_temperature_time++;
    }
    while((ret & 0x80000000) && (read_temperature_time < read_loop));
    read_temperature_time = 0;

    set_config(chainIndex, 0, CHIP_ADDR_INTERVAL*(which_sensor-1), GENERAL_I2C_COMMAND, data);
    do
    {
        ret = check_one_asic_reg(chainIndex, GENERAL_I2C_COMMAND, CHIP_ADDR_INTERVAL*(which_sensor-1));
        read_temperature_time++;
    }
    while((ret & 0x80000000) && (read_temperature_time < read_loop));
    read_temperature_time = 0;

    // read back offset value
    set_config(chainIndex, 0, CHIP_ADDR_INTERVAL*(which_sensor-1), GENERAL_I2C_COMMAND, REGADDRVALID | DEVICEADDR | REGADDR(EXTERNAL_TEMPERATURE_OFFSET_HIGH_BYTE) | (DATA(0) & (~RW)));
    do
    {
        ret = check_one_asic_reg(chainIndex, GENERAL_I2C_COMMAND, CHIP_ADDR_INTERVAL*(which_sensor-1));
        read_temperature_time++;
    }
    while((ret & 0x80000000) && (read_temperature_time < read_loop));
    read_temperature_time = 0;

    offset = (signed char)(ret & 0xff);
    sprintf(logstr,"chain[%d] ret = 0x%x, read back offset is %d, %d\n",chainIndex, ret, offset,chip_temp_offset[chainIndex]);
	writeLogFile(logstr);
}

signed char read_asic_temperature(int chainIndex)
{
	char logstr[256];
    unsigned int ret = 0, which_sensor, read_temperature_time = 0;
    signed char remote_temp=0;
//    signed char temp_offset_value=0;
//	signed char local_temp=0;

    which_sensor = temp_chip_index[chainIndex];

    enable_read_temperature_from_asic(chainIndex,CHIP_ADDR_INTERVAL*(which_sensor-1));
    check_one_asic_reg(chainIndex, MISC_CONTROL, CHIP_ADDR_INTERVAL*(which_sensor-1));

    do
    {
        ret = check_one_asic_reg(chainIndex, GENERAL_I2C_COMMAND, CHIP_ADDR_INTERVAL*(which_sensor-1));
        read_temperature_time++;
        //printf("In check GENERAL_I2C_COMMAND whether is busy\n");
    }
    while((ret & 0x80000000) && (read_temperature_time < read_loop));
    read_temperature_time = 0;

    set_config(chainIndex, 0, CHIP_ADDR_INTERVAL*(which_sensor-1), GENERAL_I2C_COMMAND, REGADDRVALID | DEVICEADDR | REGADDR(EXTERNAL_TEMPERATURE_VALUE_HIGH_BYTE) | (DATA(0) & (~RW)));
    do
    {
        ret = check_one_asic_reg(chainIndex, GENERAL_I2C_COMMAND, CHIP_ADDR_INTERVAL*(which_sensor-1));
        read_temperature_time++;
        //printf("In check remote temperature\n");
    }
    while((ret & 0x80000000) && (read_temperature_time < read_loop));
    read_temperature_time = 0;

    if((ret & 0x80000000) != 0x80000000)
    {
        remote_temp = (signed char)(ret & 0xff);
        sprintf(logstr,"ASIC temperature is %d on Chain[%d]\n", remote_temp, chainIndex);
		writeLogFile(logstr);
    }
    else
    {
    	remote_temp=-23;	// a magic number to indicate error!
    }
	return remote_temp;

/*
    set_config(chainIndex, 0, CHIP_ADDR_INTERVAL*(which_sensor-1), GENERAL_I2C_COMMAND, REGADDRVALID | DEVICEADDR | REGADDR(LOCAL_TEMPERATURE_VALUE) | (DATA(0) & (~RW)));
    do
    {
        ret = check_one_asic_reg(chainIndex, GENERAL_I2C_COMMAND, CHIP_ADDR_INTERVAL*(which_sensor-1));
        read_temperature_time++;
        //printf("In check local temperature\n");
    }
    while((ret & 0x80000000) && (read_temperature_time < read_loop));
    read_temperature_time = 0;

    if((ret & 0x80000000) != 0x80000000)
    {
        local_temp = (signed char)(ret & 0xff);
        temp_offset_value = local_temp - remote_temp;

		if(which_sensor == Conf.StartSensor)
        {
            if(Conf.StartTemp <= local_temp)
            {
                if(!gStartTest)
                {
                    printf("\nBegin test!!! Start sensor is %d, ASIC temperature is %d on Chain[%d]\n", which_sensor, local_temp, chainIndex);
                }
                gStartTest = true;
            }
            else
            {
                printf("\nASIC temperature is %d, waiting it raise to start temperature %d on Chain[%d]\n", local_temp, Conf.StartTemp, chainIndex);
            }
        }

		//printf("Local temperature is %d on Chain[%d]\n", local_temp, chainIndex);
        //printf("ECT218 temperature is %d\n", local_temp);
        //printf("temp_offset_value = %d\n\n", temp_offset_value);
    }
    else
    {
        //printf("%s: do not read out ECT218 temperature\n", __FUNCTION__);
    }

	return local_temp;
*/
}

bool check_fan()
{
    int i=0, j=0;
    unsigned char fan_id = 0;
    unsigned int fan_speed;
	char logstr[256];

    for(j=0; j < 2; j++)    //means check for twice to make sure find out all fan
    {
        for(i=0; i < BITMAIN_MAX_FAN_NUM; i++)
        {
            if(get_fan_speed(&fan_id, &fan_speed) != -1)
            {
            	fan_speed_value[fan_id] = fan_speed * 60 * 2;
                if((fan_speed > 0) && (fan_exist[fan_id] == 0))
                {
                    fan_exist[fan_id] = 1;
                    fan_num++;
                }
                else if((fan_speed == 0) && (fan_exist[fan_id] == 1))
                {
                    fan_exist[fan_id] = 0;
                    fan_num--;
                }
            }
        }

		if(fan_num>=MINER_FAN_NUM)
		{
			sprintf(logstr,"check FAN OK: fan num=%d\n",fan_num);
			writeLogFile(logstr);
	
			return true;
		}
    }

	sprintf(logstr,"check FAN ERROR: fan num=%d , ought to be %d\n",fan_num,MINER_FAN_NUM);
	writeLogFile(logstr);
	
	return false;
}

void set_PWM(unsigned char pwm_percent)
{
    uint16_t pwm_high_value = 0, pwm_low_value = 0;
    int temp_pwm_percent = 0;
	unsigned int	pwm_value;
	
    temp_pwm_percent = pwm_percent;

    if(temp_pwm_percent < MIN_PWM_PERCENT)
    {
        temp_pwm_percent = MIN_PWM_PERCENT;
    }

    if(temp_pwm_percent > MAX_PWM_PERCENT)
    {
        temp_pwm_percent = MAX_PWM_PERCENT;
    }

    pwm_high_value = temp_pwm_percent * PWM_SCALE / 100;
    pwm_low_value  = (100 - temp_pwm_percent) * PWM_SCALE / 100;
    pwm_value = (pwm_high_value << 16) | pwm_low_value;

    set_fan_control(pwm_value);
}

void set_hcnt(unsigned int hcnt)
{
	unsigned char buf[9] = {0};
	unsigned int cmd_buf[3] = {0,0,0};
	unsigned int ret, value,i;

    printf("\n--- %s\n", __FUNCTION__);

	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
	{
		if(cgpu.chain_exist[i] == 1 
#ifndef ALL_CHAIN_INTEST
			&& (!testDone[i])
#endif
			)
		{
			{
				/* 20160510
				buf[0] = VIL_COMMAND_TYPE | VIL_ALL | SET_CONFIG;
				buf[1] = 0x09;
				buf[2] = 0;
				buf[3] = MISC_CONTROL;
				buf[4] = 0x40;
				buf[5] = INV_CLKO | cgpu.temp_sel;
				buf[6] = (bauddiv & 0x1f) | (cgpu.rfs << 6);
				buf[7] = cgpu.tfs << 5;
				buf[8] = CRC5(buf, 8*8);
				*/

				buf[0] = VIL_COMMAND_TYPE | VIL_ALL | SET_CONFIG;
				buf[1] = 0x09;
				buf[2] = 0;
				buf[3] = HASH_COUNTING_NUMBER;
				buf[4] = hcnt>>24;
				buf[5] = hcnt>>16;
				buf[6] = hcnt>>8;
				buf[7] = hcnt;
				buf[8] = CRC5(buf, 8*8);

				cmd_buf[0] = buf[0]<<24 | buf[1]<<16 | buf[2]<<8 | buf[3];
				cmd_buf[1] = buf[4]<<24 | buf[5]<<16 | buf[6]<<8 | buf[7];
				cmd_buf[2] = buf[8]<<24;
                printf("%s: cmd_buf[0]=0x%x, cmd_buf[1]=0x%x, cmd_buf[2]=0x%x\n", __FUNCTION__, cmd_buf[0], cmd_buf[1], cmd_buf[2]);

				set_BC_command_buffer(cmd_buf);
				ret = get_BC_write_command();
				value = BC_COMMAND_BUFFER_READY | BC_COMMAND_EN_CHAIN_ID| (i << 16) | (ret & 0xFFF0FFFF);
				set_BC_write_command(value);
			}
		}
	}
}

void set_tickmask(unsigned int tickmask)
{
	unsigned char buf[9] = {0};
	unsigned int cmd_buf[3] = {0,0,0};
	unsigned int ret, value,i;

    printf("\n--- %s\n", __FUNCTION__);

	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
	{
		if(cgpu.chain_exist[i] == 1 
#ifndef ALL_CHAIN_INTEST
			&& (!testDone[i])
#endif
			)
		{
			//first step: send new bauddiv to ASIC, but FPGA doesn't change its bauddiv, it uses old bauddiv to send BC command to ASIC
			if(cgpu.CommandMode)	// fil mode
			{
				;
			}
			else	// vil mode
			{
				/* 20160510
				buf[0] = VIL_COMMAND_TYPE | VIL_ALL | SET_CONFIG;
				buf[1] = 0x09;
				buf[2] = 0;
				buf[3] = MISC_CONTROL;
				buf[4] = 0x40;
				buf[5] = INV_CLKO | cgpu.temp_sel;
				buf[6] = (bauddiv & 0x1f) | (cgpu.rfs << 6);
				buf[7] = cgpu.tfs << 5;
				buf[8] = CRC5(buf, 8*8);
				*/

				buf[0] = VIL_COMMAND_TYPE | VIL_ALL | SET_CONFIG;
				buf[1] = 0x09;
				buf[2] = 0;
				buf[3] = TICKET_MASK;
				buf[4] = tickmask>>24;
				buf[5] = tickmask>>16;
				buf[6] = tickmask>>8;
				buf[7] = tickmask;
				buf[8] = CRC5(buf, 8*8);

				cmd_buf[0] = buf[0]<<24 | buf[1]<<16 | buf[2]<<8 | buf[3];
				cmd_buf[1] = buf[4]<<24 | buf[5]<<16 | buf[6]<<8 | buf[7];
				cmd_buf[2] = buf[8]<<24;
                printf("%s: cmd_buf[0]=0x%x, cmd_buf[1]=0x%x, cmd_buf[2]=0x%x\n", __FUNCTION__, cmd_buf[0], cmd_buf[1], cmd_buf[2]);

				set_BC_command_buffer(cmd_buf);
				ret = get_BC_write_command();
				value = BC_COMMAND_BUFFER_READY | BC_COMMAND_EN_CHAIN_ID| (i << 16) | (ret & 0xFFF0FFFF);
				set_BC_write_command(value);
			}
		}
	}
}

unsigned char getPICvoltageFromValue(int vol_value)	// vol_value = 940  means 9.4V
{
#ifdef S9_PLUS
#ifdef S9_PLUS_VOLTAGE2
	unsigned char temp_voltage=1250.809516-127.817623*(vol_value*1.0)/100;
#else
	unsigned char temp_voltage=824.784-73.1705*((vol_value*1.0)/100.0);
#endif
#endif

#ifdef R4
	unsigned char temp_voltage = 1608.420446 - 170.423497*(vol_value*1.0)/100.0;
#endif

#ifdef S9_63
	unsigned char temp_voltage = 1608.420446 - 170.423497*(vol_value*1.0)/100.0;
#endif

#ifdef T9_18
	unsigned char temp_voltage = 364.0704 / (4.75*(vol_value*1.0)/100 - 32.79) - 30.72;
#endif

	return temp_voltage;
}

int getVolValueFromPICvoltage(unsigned char vol_pic)
{
#ifdef S9_PLUS
#ifdef S9_PLUS_VOLTAGE2
	int vol_value = ((1250.809516 - vol_pic)/127.817623)*100.0;
#else
	int vol_value = ((824.784 - vol_pic)/73.1705)*100.0;
#endif
#endif

#ifdef S9_63
	int vol_value = ((1608.420446 - vol_pic) *100.0)/170.423497;
#endif

#ifdef R4
	int vol_value = ((1608.420446 - vol_pic) *100.0)/170.423497;
#endif

#ifdef T9_18
	int vol_value = ((364.0704/(vol_pic+30.72))+32.79)*100/4.75;
#endif

	vol_value=(vol_value/10)*10;	//remove the last one number

	return vol_value;
}

unsigned int PHY_MEM_NONCE2_JOBID_ADDRESS=PHY_MEM_NONCE2_JOBID_ADDRESS_XILINX_1GB;	// set to XILINX as default
bool isC5_Board()
{
  FILE *fd;
  char board_type[32];
  int isC5=0;
  
  memset(board_type,'\0',32);
  
	fd=fopen("/usr/bin/ctrl_bd","rb");
	if(fd)
	{
		fread(board_type,1,32,fd);
		fclose(fd);
		
		if(strstr(board_type,"XILINX"))
			{
				isC5=0;
			}
			else isC5=1;
	}
	else
	{
		isC5=1;
	}
	
	if(isC5)
		return true;
	else return false;
}

