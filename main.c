/*
 * Copyright 2016-2017 Fazio Bai <yang.bai@bitmain.com>
 * Copyright 2016-2017 Clement Duan <kai.duan@bitmain.com>
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
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
#include <arpa/inet.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <sys/sysinfo.h>

#include <openssl/hmac.h>
#include <openssl/aes.h>
#include <openssl/sha.h>
#include <openssl/evp.h>
#include <openssl/bn.h>
#include <openssl/ecdsa.h>
#include <openssl/obj_mac.h>
#include <openssl/rand.h>
#include <openssl/bn.h>

#include <curl/curl.h>


#include "driver-bitmain.h"


#define RUNNING true
#define STOPED false
pthread_mutex_t stats_lock;
#define START_KEY_DEV "/sys/class/gpio/gpio235/value"
#define RED_LED_DEV "/sys/class/leds/hps_led2/brightness"
#define GREEN_LED_DEV "/sys/class/leds/hps_led0/brightness"


#define CHECK_CHAIN
#define Sliding_rheostat_50k

#ifdef Sliding_rheostat_50k
unsigned int pic_voltage1 = 27; // 10V
unsigned int pic_voltage2 = 21; // 10.25V
unsigned int pic_voltage3 = 16; // 10.50V
#endif

#ifdef Sliding_rheostat_100k
unsigned int pic_voltage1 = 13; // 10V
unsigned int pic_voltage2 = 11; // 10.25V
unsigned int pic_voltage3 = 8; // 10.50V
#endif

#ifdef R4
int R4_MAX_VOLTAGE;
int START_VOLTAGE;
int RETRY_VOLTAGE;
int HIGHEST_FREQ_INDEX;
#endif

int fpga_version;	// very important for T9+

int temp_chip_index[BITMAIN_MAX_CHAIN_NUM]={0};

int searchStatus;
bool isChipNumOK_Once=false;

bool isFailedOnTestPatten=false;
bool IsSomeBoardHasNoFreq=false;

char search_failed_info[64];
void saveSearchFailedFlagInfo();

unsigned char chain_pic_buf[BITMAIN_MAX_CHAIN_NUM][128];

bool StartSendFlag[BITMAIN_MAX_CHAIN_NUM];

int chain_DataCount[BITMAIN_MAX_CHAIN_NUM];
int chain_ValidNonce[BITMAIN_MAX_CHAIN_NUM];
int chain_PassCount[BITMAIN_MAX_CHAIN_NUM];
	
int chain_vol_value[BITMAIN_MAX_CHAIN_NUM];	// the searching vol 
int chain_vol_added[BITMAIN_MAX_CHAIN_NUM];	// how many vol added , recorded in PIC

int last_opencore_result[BITMAIN_MAX_CHAIN_NUM][256];
int last_result[BITMAIN_MAX_CHAIN_NUM][256];
int last_freq[BITMAIN_MAX_CHAIN_NUM][256];
int last_success_freq[BITMAIN_MAX_CHAIN_NUM][256];	// success freq recores and will not clear them

bool isNoBoardError=false;
bool search_over[BITMAIN_MAX_CHAIN_NUM];
bool testDone[BITMAIN_MAX_CHAIN_NUM];

unsigned char vol_oldvalue[BITMAIN_MAX_CHAIN_NUM];
signed char board_temp[BITMAIN_MAX_CHAIN_NUM]={0};

int TEST_MODE_OK_NUM=0;

char voltage_char[30] = {0};
char lcd_buffer[16] = {' ','P','i','c',' ','V','o','l',':',' ',' ',' ',' ',' ',' ',' '};
unsigned char time_data[6]= {'H','H','H','H','H','H'};

bool first_freq = true;
int result = 0;
bool search_freq_result[BITMAIN_MAX_CHAIN_NUM];	// set true as default
int search_freq_chances[BITMAIN_MAX_CHAIN_NUM];	// give each board SEARCH_FREQ_CHANCE_NUM chances to search freq.

struct cgpu_info cgpu;
volatile bool gBegin_get_nonce = false;

extern pthread_mutex_t reg_mutex;
pthread_mutex_t iic_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t log_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t fpga_mutex = PTHREAD_MUTEX_INITIALIZER;

static pthread_mutex_t temp_work_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t read_temp_mutex = PTHREAD_MUTEX_INITIALIZER;

extern unsigned int *nonce2_jobid_address;      // the value should be filled in NONCE2_AND_JOBID_STORE_ADDRESS
unsigned int send_work_num[BITMAIN_MAX_CHAIN_NUM];

#ifndef T9_18
unsigned char pic_badcore_num[BITMAIN_MAX_CHAIN_NUM][64];
#endif

int temp_asic_core_enabled_flag[BITMAIN_MAX_CHAIN_NUM][256][256]; // only used to try fix enabled flag, temp value
int asic_core_enabled_flag[BITMAIN_MAX_CHAIN_NUM][256][256];	// [chainindex][chipindex][coreindex]=0  disabled, 1:enabled
int last_asic_core_enabled_flag[BITMAIN_MAX_CHAIN_NUM][256][256];	// last asic core flags

int chain_badcore_num[BITMAIN_MAX_CHAIN_NUM][256];	// only used in test mode , not in search mode, because after reboot, we only can get bad core num, but can not know which core is bad!!!

int asic_nonce_num[BITMAIN_MAX_CHAIN_NUM][256];
int asic_core_nonce_num[BITMAIN_MAX_CHAIN_NUM][256][256];  // 1st: which asic, 2nd: which core
int last_nonce_num[BITMAIN_MAX_CHAIN_NUM];
int repeated_nonce_num[BITMAIN_MAX_CHAIN_NUM];
uint32_t repeated_nonce_id[BITMAIN_MAX_CHAIN_NUM][256];
int valid_nonce_num[BITMAIN_MAX_CHAIN_NUM];    // all the received nonce in one test
int err_nonce_num[BITMAIN_MAX_CHAIN_NUM];
int total_valid_nonce_num=0;

volatile bool start_receive = false;
bool start_pic_heart=false;

#define NO_CHIP_DOWN_FREQ_MAX_COUNTER	2	// if there is no chip down freq , but test mode failed for 3 times, we need force to down freq on all chips!
int testModeChainHasNoChipDownFreqCounter[BITMAIN_MAX_CHAIN_NUM];
bool testModeHasTriedAcceptBadCore[BITMAIN_MAX_CHAIN_NUM][256];
int testModeOKCounter[BITMAIN_MAX_CHAIN_NUM];

struct configuration Conf;  //store information that read from Config.ini
struct _CONFIG conf;        //store the information that handled from Config.ini
extern unsigned int *axi_fpga_addr;
struct reg_buf *reg_value_buf = NULL;
unsigned int gtest = 0;
extern unsigned char hash_board_id[12];
extern unsigned int hash_board_id_recorder;
extern unsigned int CHIP_ADDRESS;
extern unsigned int GOLDEN_NONCE_COUNTER;
extern unsigned int PLL_PARAMETER;
extern unsigned int START_NONCE_OFFSET;
extern unsigned int HASH_COUNTING_NUMBER;
extern unsigned int TICKET_MASK;
extern unsigned int MISC_CONTROL;
extern unsigned int HASH_RATE;
extern unsigned int GENERAL_I2C_COMMAND;
extern unsigned int SECURITY_I2C_COMMAND;
extern unsigned int SIGNATURE_INPUT;
extern unsigned int SIGNATURE_NONCE;
extern unsigned int SIGNATURE_ID;
extern unsigned int SECURITY_CONTROL_AND_STATUS;
extern unsigned int JOB_INFORMATION;

bool gIsReadTemp = false;
bool gReadingTemp = false;
bool gStartTest = false;
bool ExitFlag=false;
bool showExit;
bool receiveExit;
bool picheartExit;
bool sendExit[BITMAIN_MAX_CHAIN_NUM];

signed char chip_temp_offset[BITMAIN_MAX_CHAIN_NUM]={0};

typedef enum{
	SEARCH_BASE_FREQ_V89_200=0,
	SEARCH_BASE_FREQ_V89_300,
	SEARCH_BASE_FREQ_V89_400,
	SEARCH_BASE_FREQ_V89_400_2,		// speical for 8.9V 400M, only used in search mode 2, this state will not switch voltage, just up freq 100M step
	SEARCH_BASE_FREQ_V89_500,
	SEARCH_BASE_FREQ_V89_600,
	SEARCH_BASE_FREQ_V89_650,
	SEARCH_BASE_FREQ_V86_400,
	SEARCH_BASE_FREQ_V86_500,
	SEARCH_BASE_FREQ_V86_600,
	SEARCH_BASE_FREQ_V86_650,
	SEARCH_BASE_FREQ,	// search one freq as start, if found , changed to ALLCHIP_FREQ_UP, if failed, down all chips freq one step and test
	ALLCHIP_FREQ_UP,	// All chips up one step freq, if pass then continue up, if failed then go to FAILED_CHIP_DOWN
	FAILED_CHIP_DOWN,	// failed chip down one step, successful chip keep no change.  test OK, then 
	DOWN_CHIP_ONEBYONE,	// need down chip in DC area one by one, to test OK
	SUCCESS_CHIP_UP,	// successful chip with Fmax+1 freq, keep go up one step
	DOWN_VOLTAGE_TEST,
	SEARCH_OVER,
}SEARCH_WORK_STATE;

int Fmax[BITMAIN_MAX_CHAIN_NUM];
int base_freq_index[BITMAIN_MAX_CHAIN_NUM];	// the start freq index of hashboard
int searchFreqMode[BITMAIN_MAX_CHAIN_NUM];

extern void set_PWM(unsigned char pwm_percent);
extern void writeLogFile(char *logstr);
extern void ClearForceFreq();
int GetTotalRate();
int ConvirtTotalRate(int totalRate);
int GetBoardRate(int chainIndex);
static int get_mac(char * device, unsigned char *mac);

int getVoltageLimitedFromHashrate(int hashrate_GHz)
{
	int vol_value;

#ifdef R4
	vol_value=R4_MAX_VOLTAGE;
#endif

#ifdef S9_PLUS
	if(hashrate_GHz>=12500)
		vol_value=840;
	else if(hashrate_GHz>=12000)
		vol_value=850;
	else if(hashrate_GHz>=11500)
		vol_value=870;
	else if(hashrate_GHz>=11000)
		vol_value=890;
	else if(hashrate_GHz>=10500)
		vol_value=910;
	else if(hashrate_GHz>=10000)
		vol_value=930;
	else if(hashrate_GHz>=9500)
		vol_value=960;
	else if(hashrate_GHz>=9000)
		vol_value=970;
	else
		vol_value=970;
#endif

#ifdef S9_63
	if(hashrate_GHz>=14500)
		vol_value=870;
	else if(hashrate_GHz>=14000)
		vol_value=880;
	else if(hashrate_GHz>=13500)
		vol_value=900;
	else if(hashrate_GHz>=13000)
		vol_value=910;
	else if(hashrate_GHz>=12500)
		vol_value=930;
	else
		vol_value=940;
#endif

#ifdef T9_18
	if(hashrate_GHz>=12000)
		vol_value=810;
	else if(hashrate_GHz>=11500)
		vol_value=830;
	else if(hashrate_GHz>=11000)
		vol_value=850;
	else if(hashrate_GHz>=10500)
		vol_value=870;
	else if(hashrate_GHz>=10000)
		vol_value=890;
	else if(hashrate_GHz>=9500)
		vol_value=920;
	else if(hashrate_GHz>=9000)
		vol_value=930;
	else
		vol_value=930;
#endif

	return vol_value;
}

int getLimitedHashrateByVoltage(int vol_value)	// hashrate must be less than this return value, can not equal!!!
{
#ifdef R4
	return 14500;
#endif

#ifdef S9_PLUS
	switch(vol_value)
	{
	case 970:
		return 9500;
	case 960:
	case 950:
	case 940:
		return 10000;
	case 930:
	case 920:
		return 10500;
	case 910:
	case 900:
		return 11000;
	case 890:
	case 880:
		return 11500;
	case 870:
	case 860:
		return 12000;
	case 850:
		return 12500;
	default:
		return 13000;
	}
#endif

#ifdef S9_63
	switch(vol_value)
	{
	case 940:
		return 12500;
	case 930:
	case 920:
		return 13000;
	case 910:
		return 13500;
	case 900:
	case 890:
		return 14000;
	case 880:
		return 14500;
	case 870:
		return 15000;
	default:
		return 15500;
	}
#endif

#ifdef T9_18
	switch(vol_value)
	{
	case 930:
		return 9500;
	case 920:
	case 910:
	case 900:
		return 10000;
	case 890:
	case 880:
		return 10500;
	case 870:
	case 860:
		return 11000;
	case 850:
	case 840:
		return 11500;
	case 830:
	case 820:
		return 12000;
	default:
		return 12500;
	}
#endif
}

int getNextSearchBaseFreq(int cur_freq)
{
	if(cur_freq<0)
		cur_freq=66;	//600M
	else if(cur_freq>=66)
		cur_freq=58;	//550M
	else if(cur_freq>=58)
		cur_freq=44;	//500M
	else if(cur_freq>=44)
		cur_freq=28;	//450M
	else if(cur_freq>=28)
		cur_freq=12;	//400M
	else cur_freq--;

	return cur_freq;
}

#ifdef T9_18
void getPICChainIndexOffset(int chainIndex, int *pChain, int *pOffset)
{
	int new_T9_PLUS_chainIndex,new_T9_PLUS_chainOffset;
	switch(chainIndex)
	{
	case 1:
		new_T9_PLUS_chainIndex=1;
		new_T9_PLUS_chainOffset=0;
		break;
	case 8:
		new_T9_PLUS_chainIndex=1;
		new_T9_PLUS_chainOffset=1;
		break;
	case 9:
		new_T9_PLUS_chainIndex=1;
		new_T9_PLUS_chainOffset=2;
		break;
	case 2:
		new_T9_PLUS_chainIndex=2;
		new_T9_PLUS_chainOffset=0;
		break;
	case 10:
		new_T9_PLUS_chainIndex=2;
		new_T9_PLUS_chainOffset=1;
		break;
	case 11:
		new_T9_PLUS_chainIndex=2;
		new_T9_PLUS_chainOffset=2;
		break;
	case 3:
		new_T9_PLUS_chainIndex=3;
		new_T9_PLUS_chainOffset=0;
		break;
	case 12:
		new_T9_PLUS_chainIndex=3;
		new_T9_PLUS_chainOffset=1;
		break;
	case 13:
		new_T9_PLUS_chainIndex=3;
		new_T9_PLUS_chainOffset=2;
		break;
	default:
		new_T9_PLUS_chainIndex=0;
		new_T9_PLUS_chainOffset=0;
		break;
	}

	*pChain=new_T9_PLUS_chainIndex;
	*pOffset=new_T9_PLUS_chainOffset;
}

int getChainPICMagicNumber(int chainIndex)
{
	if(fpga_version>=0xE)
	{
		int new_T9_PLUS_chainIndex,new_T9_PLUS_chainOffset;
		getPICChainIndexOffset(chainIndex,&new_T9_PLUS_chainIndex,&new_T9_PLUS_chainOffset);
		return chain_pic_buf[new_T9_PLUS_chainIndex][0];
	}
	else
	{
		return chain_pic_buf[((chainIndex/3)*3)][0];
	}
}

int getOneChipFreqIndex(int chainIndex, int chipIndex)
{
	if(fpga_version>=0xE)
	{
		int new_T9_PLUS_chainIndex,new_T9_PLUS_chainOffset;
		getPICChainIndexOffset(chainIndex,&new_T9_PLUS_chainIndex,&new_T9_PLUS_chainOffset);
		return chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+chipIndex];
	}
	else
	{
		return chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+chipIndex];
	}
}

void jump_to_app_CheckAndRestorePIC(int chainIndex)
{
	unsigned char pic_version;
	char logstr[256];
	int try_count=0;

	if(fpga_version>=0xE)
	{
		if(chainIndex<1 || chainIndex>3)
			return;

		sprintf(logstr,"chain[%d] PIC jump to app\n",chainIndex);
		writeLogFile(logstr);
		
		dsPIC33EP16GS202_jump_to_app_from_loader(chainIndex);

		get_pic_software_version(chainIndex,&pic_version);
		sprintf(logstr,"Check chain[%d] PIC fw version=0x%02x\n",chainIndex,pic_version);
		writeLogFile(logstr);

#ifdef ENABLE_RESTORE_PIC_APP
#ifndef DEBUG_PIC_UPGRADE
		while(pic_version!=PIC_VERSION && try_count<2)
#endif
		{
			try_count++;
			sprintf(logstr,"chain[%d] PIC need restore ...\n",chainIndex);
			writeLogFile(logstr);

			dsPIC33EP16GS202_update_pic_app_program(chainIndex);
			dsPIC33EP16GS202_jump_to_app_from_loader(chainIndex);
			
			get_pic_software_version(chainIndex,&pic_version);
			sprintf(logstr,"After restore: chain[%d] PIC fw version=0x%02x\n",chainIndex,pic_version);
			writeLogFile(logstr);
		}
#endif
	}
	else
	{
		if(chainIndex%3 != 0)
			return;

		sprintf(logstr,"chain[%d] PIC jump to app\n",chainIndex);
		writeLogFile(logstr);
		
		dsPIC33EP16GS202_jump_to_app_from_loader(chainIndex/3);

		get_pic_software_version(chainIndex,&pic_version);
		sprintf(logstr,"Check chain[%d] PIC fw version=0x%02x\n",chainIndex,pic_version);
		writeLogFile(logstr);

#ifdef ENABLE_RESTORE_PIC_APP
#ifndef DEBUG_PIC_UPGRADE
		while(pic_version!=PIC_VERSION && try_count<2)
#endif
		{
			try_count++;
			sprintf(logstr,"chain[%d] PIC need restore ...\n",chainIndex);
			writeLogFile(logstr);

			dsPIC33EP16GS202_update_pic_app_program(chainIndex/3);
			dsPIC33EP16GS202_jump_to_app_from_loader(chainIndex/3);
			
			get_pic_software_version(chainIndex,&pic_version);
			sprintf(logstr,"After restore: chain[%d] PIC fw version=0x%02x\n",chainIndex,pic_version);
			writeLogFile(logstr);
		}
#endif
	}
}
#else
// this check and restore PIC function must be called after reset_iic_pic process, that means only can be called when in bootloader mode!
void jump_to_app_CheckAndRestorePIC(int chainIndex)	// check PIC app is OK or not, if not right, write flash to restore app and jump to app mode again!
{
	unsigned char pic_version;
	char logstr[256];
	int try_count=0;

	jump_to_app_from_loader(chainIndex);
	get_pic_software_version(chainIndex,&pic_version);
	sprintf(logstr,"Check chain[%d] PIC fw version=0x%02x\n",chainIndex,pic_version);
	writeLogFile(logstr);

#ifdef ENABLE_RESTORE_PIC_APP
#ifndef DEBUG_PIC_UPGRADE
	while(pic_version!=PIC_VERSION && try_count<2)
#endif
	{
		try_count++;
		sprintf(logstr,"chain[%d] PIC need restore ...\n",chainIndex);
		writeLogFile(logstr);

		update_pic_program(chainIndex);

		jump_to_app_from_loader(chainIndex);
		
		get_pic_software_version(chainIndex,&pic_version);
		sprintf(logstr,"After restore: chain[%d] PIC fw version=0x%02x\n",chainIndex,pic_version);
		writeLogFile(logstr);
	}
#endif
}
#endif

void InitAsicCoreEnabledFlag()
{
	int i,j,k;
	for(i = 0; i < BITMAIN_MAX_CHAIN_NUM; i++)
	{
		for(j = 0; j < 256; j++)
		{
			for(k=0; k<256; k++)
			{
				asic_core_enabled_flag[i][j][k]=1;	// set enabled as default value
				last_asic_core_enabled_flag[i][j][k]=1;	// set enabled as default value
			}

			chain_badcore_num[i][j]=0;	// set init bad core num is 0
		}
	}
}

void ResetAsicCoreEnabledFlag(int chainIndex)
{
	int j,k;

	for(j = 0; j < 256; j++)
	{
		for(k=0; k<256; k++)
		{
			asic_core_enabled_flag[chainIndex][j][k]=1;	// set enabled as default value
			last_asic_core_enabled_flag[chainIndex][j][k]=1;	// set enabled as default value
		}

		chain_badcore_num[chainIndex][j]=0;	// set init bad core num is 0
	}
}

void PrintAsicCoreEnabledFlag(int chainIndex)
{
	int i,m;
	char logstr[256];

	sprintf(logstr,"Chain[%d] Bad Core info:\n",chainIndex);
	writeLogFile(logstr);
	
	for(i = 0; i < ASIC_NUM; i++)
	{
		for(m=0; m<ASIC_CORE_NUM; m++)
		{
			if(asic_core_enabled_flag[chainIndex][i][m] <= 0)
			{
				sprintf(logstr,"ASIC[%d] Core[%d] is bad!\n",i,m);
				writeLogFile(logstr);
			}
		}
	}
	sprintf(logstr,"\n");
	writeLogFile(logstr);
}

void UpdateAsicCoreEnabledFlagByResult(int chainIndex)
{
	int i,m,j,k;
	int n = chain_PassCount[chainIndex]/ASIC_CORE_NUM;

	for(j = 0; j < 256; j++)
	{
		for(k=0; k<256; k++)
		{
			asic_core_enabled_flag[chainIndex][j][k]=1;	// set enabled as default value
		}
	}

	for(i = 0; i < ASIC_NUM; i++)
	{
		if(asic_nonce_num[chainIndex][i] < chain_PassCount[chainIndex])
		{
			for(m=0; m<ASIC_CORE_NUM; m++)
			{
				if(asic_core_nonce_num[chainIndex][i][m] < n)
				{
					asic_core_enabled_flag[chainIndex][i][m]=0;
				}
			}
		}
	}
}

void UpdateSingleAsicCoreEnabledFlagByResult(int chainIndex, int chipIndex)
{
	int m,k;
	int n = chain_PassCount[chainIndex]/ASIC_CORE_NUM;

	for(k=0; k<256; k++)
	{
		asic_core_enabled_flag[chainIndex][chipIndex][k]=1;	// set enabled as default value
	}
	
	if(asic_nonce_num[chainIndex][chipIndex] < chain_PassCount[chainIndex])
	{
		for(m=0; m<ASIC_CORE_NUM; m++)
		{
			if(asic_core_nonce_num[chainIndex][chipIndex][m] < n)
			{
				asic_core_enabled_flag[chainIndex][chipIndex][m]=0;
			}
		}
	}
}


void copyAsicCoreEnabledFlagFromTemp(int chainIndex)
{
	int i,m;
	for(i = 0; i < ASIC_NUM; i++)
	{
		for(m=0; m<ASIC_CORE_NUM; m++)
		{
			asic_core_enabled_flag[chainIndex][i][m]=temp_asic_core_enabled_flag[chainIndex][i][m];
		}
	}
}

void copyAsicCoreEnabledFlagFromLast(int chainIndex)
{
	int i,m;
	for(i = 0; i < ASIC_NUM; i++)
	{
		for(m=0; m<ASIC_CORE_NUM; m++)
		{
			asic_core_enabled_flag[chainIndex][i][m]=last_asic_core_enabled_flag[chainIndex][i][m];
		}
	}
}

void SaveAsicCoreEnabledFlagByResultToLastRecord(int chainIndex)
{
	int i,m,j,k;
	int n = chain_PassCount[chainIndex]/ASIC_CORE_NUM;

	for(j = 0; j < 256; j++)
	{
		for(k=0; k<256; k++)
		{
			last_asic_core_enabled_flag[chainIndex][j][k]=1;	// set enabled as default value
		}
	}
	
	for(i = 0; i < ASIC_NUM; i++)
	{
		if(asic_nonce_num[chainIndex][i] < chain_PassCount[chainIndex])
		{
			for(m=0; m<ASIC_CORE_NUM; m++)
			{
				if(asic_core_nonce_num[chainIndex][i][m] < n)
				{
					last_asic_core_enabled_flag[chainIndex][i][m]=0;
				}
			}
		}
	}
}

void SaveAsicCoreEnabledFlagByResultToTempRecord(int chainIndex)
{
	int i,j,k,m;
	int n = chain_PassCount[chainIndex]/ASIC_CORE_NUM;

	for(j = 0; j < 256; j++)
	{
		for(k=0; k<256; k++)
		{
			temp_asic_core_enabled_flag[chainIndex][j][k]=1;	// set enabled as default value
		}
	}
	
	for(i = 0; i < ASIC_NUM; i++)
	{
		if(asic_nonce_num[chainIndex][i] < chain_PassCount[chainIndex])
		{
			for(m=0; m<ASIC_CORE_NUM; m++)
			{
				if(asic_core_nonce_num[chainIndex][i][m] < n)
					temp_asic_core_enabled_flag[chainIndex][i][m]=0;
			}
		}
	}
}

bool isSameLastAsicCoreEnabledFlagWithTemp(int chainIndex)
{
	int i,m;
	
	for(i = 0; i < ASIC_NUM; i++)
	{
		for(m=0; m<ASIC_CORE_NUM; m++)
		{
			if(temp_asic_core_enabled_flag[chainIndex][i][m] != last_asic_core_enabled_flag[chainIndex][i][m])
				return false;
		}
	}

	return true;
}

int getChainAsicLastBadCoreNum(int chainIndex, int asicIndex)
{
	int m;
	int badcore=0;

	if(asicIndex>=ASIC_NUM)
		return 0;
	
	for(m=0; m<ASIC_CORE_NUM; m++)
	{
		if(last_asic_core_enabled_flag[chainIndex][asicIndex][m]<=0)
		{
			badcore++;
		}
	}
	return badcore;
}

int getChainAsicTempBadCoreNum(int chainIndex, int asicIndex)
{
	int m;
	int badcore=0;

	if(asicIndex>=ASIC_NUM)
		return 0;
	
	for(m=0; m<ASIC_CORE_NUM; m++)
	{
		if(temp_asic_core_enabled_flag[chainIndex][asicIndex][m]<=0)
		{
			badcore++;
		}
	}
	return badcore;
}

bool isLastAsicGoodCoreNumLessThanTempAsicGoodCoreNum(int chainIndex)
{
	int i,m;
	int tempGoodCoreNum=0;
	int lastGoodCoreNum=0;

	if(Fmax[chainIndex]>ACCEPT_BADCORE_FREQ_INDEX && chain_vol_value[chainIndex]<RETRY_VOLTAGE)
		return false;
		
	for(i = 0; i < ASIC_NUM; i++)
	{
		for(m=0; m<ASIC_CORE_NUM; m++)
		{
			if(temp_asic_core_enabled_flag[chainIndex][i][m] > 0)
				tempGoodCoreNum++;
		}
	}

	for(i = 0; i < ASIC_NUM; i++)
	{
		for(m=0; m<ASIC_CORE_NUM; m++)
		{
			if(last_asic_core_enabled_flag[chainIndex][i][m] > 0)
				lastGoodCoreNum++;
		}
	}

	if(lastGoodCoreNum<=tempGoodCoreNum)
	{
		SaveAsicCoreEnabledFlagByResultToLastRecord(chainIndex);
		return true;
	}
	else return false;
}

int getChainAsicBadCoreNum(int chainIndex, int asicIndex)
{
	int m;
	int badcore=0;

	if(asicIndex>=ASIC_NUM)
		return 0;
	
	for(m=0; m<ASIC_CORE_NUM; m++)
	{
		if(asic_core_enabled_flag[chainIndex][asicIndex][m]<=0)
		{
			badcore++;
		}
	}
	return badcore;
}

int getChainAsicGoodCoreNum(int chainIndex, int asicIndex)
{
	int m;
	int okcore=0;

	if(asicIndex>=ASIC_NUM)
		return 0;
	
	for(m=0; m<ASIC_CORE_NUM; m++)
	{
		if(asic_core_enabled_flag[chainIndex][asicIndex][m]>0)
		{
			okcore++;
		}
	}

	// decrease bad core num, when search freq mode, chain_badcore_num is all 0, but in test mode, has value!!!
	okcore-=chain_badcore_num[chainIndex][asicIndex];
	return okcore;
}

int getChainBadCoreNumUserMode(int chainIndex)
{
	int m;
	int badcore=0;
	int asicIndex;

	for(asicIndex=0;asicIndex<ASIC_NUM;asicIndex++)
	{
		// decrease bad core num, when search freq mode, chain_badcore_num is all 0, but in test mode, has value!!!
		badcore+=chain_badcore_num[chainIndex][asicIndex];
	}
	return badcore;
}


int getChainLastBadCoreNum(int chainIndex)
{
	int i,m;
	int badcore=0;

	for(i=0; i<ASIC_NUM; i++)
		for(m=0; m<ASIC_CORE_NUM; m++)
		{
			if(last_asic_core_enabled_flag[chainIndex][i][m]<=0)
			{
				badcore++;
			}
		}
		
	return badcore;
}

int getChainTempBadCoreNum(int chainIndex)
{
	int i,m;
	int badcore=0;

	for(i=0; i<ASIC_NUM; i++)
		for(m=0; m<ASIC_CORE_NUM; m++)
		{
			if(temp_asic_core_enabled_flag[chainIndex][i][m]<=0)
			{
				badcore++;
			}
		}
		
	return badcore;
}

int getChainBadCoreNum(int chainIndex)
{
	int i,m;
	int badcore=0;

	for(i=0; i<ASIC_NUM; i++)
		for(m=0; m<ASIC_CORE_NUM; m++)
		{
			if(asic_core_enabled_flag[chainIndex][i][m]<=0)
			{
				badcore++;
			}
		}
		
	return badcore;
}

bool isChainAsicLastBadCoreCanAccepted(int chainIndex)
{
	int i;
	char logstr[256];
	bool ret=true;
	int badcoreNum;

	if(Fmax[chainIndex]>ACCEPT_BADCORE_FREQ_INDEX && chain_vol_value[chainIndex]<RETRY_VOLTAGE)
		return false;
	
	for(i = 0; i < ASIC_NUM; i++)
	{
		badcoreNum=getChainAsicLastBadCoreNum(chainIndex,i);

		if(badcoreNum>0)
		{
			sprintf(logstr,"Chain[%d] Chip[%d] has %d bad cores.\n",chainIndex,i,badcoreNum);
			writeLogFile(logstr);
		}
		
		if(badcoreNum>MAX_BAD_CORE_NUM)
			ret=false;	//set failed flag
	}
	return ret;
}

bool isChainAsicTempBadCoreCanAccepted(int chainIndex)
{
	int i;
	char logstr[256];
	bool ret=true;
	int badcoreNum;

	if(Fmax[chainIndex]>ACCEPT_BADCORE_FREQ_INDEX && chain_vol_value[chainIndex]<RETRY_VOLTAGE)
		return false;
	
	for(i = 0; i < ASIC_NUM; i++)
	{
		badcoreNum=getChainAsicTempBadCoreNum(chainIndex,i);

		if(badcoreNum>0)
		{
			sprintf(logstr,"Chain[%d] Chip[%d] has %d bad cores.\n",chainIndex,i,badcoreNum);
			writeLogFile(logstr);
		}
		
		if(badcoreNum>MAX_BAD_CORE_NUM)
			ret=false;	//set failed flag
	}
	return ret;
}


bool isChainAsicBadCoreCanAccepted(int chainIndex)
{
	int i;
	char logstr[256];
	bool ret=true;
	int badcoreNum;
	
	for(i = 0; i < ASIC_NUM; i++)
	{
		badcoreNum=getChainAsicBadCoreNum(chainIndex,i);

		if(badcoreNum>0)
		{
			sprintf(logstr,"Chain[%d] Chip[%d] has %d bad cores.\n",chainIndex,i,badcoreNum);
			writeLogFile(logstr);
		}
		
		if(badcoreNum>MAX_BAD_CORE_NUM)
			ret=false;	//set failed flag
	}
	return ret;
}

bool isChainSingleAsicBadCoreCanAccepted(int chainIndex, int chipIndex)
{
	char logstr[256];
	int badcoreNum;
	
	badcoreNum=getChainAsicBadCoreNum(chainIndex,chipIndex);

	if(badcoreNum>0)
	{
		sprintf(logstr,"Chain[%d] Chip[%d] has %d bad cores.\n",chainIndex,chipIndex,badcoreNum);
		writeLogFile(logstr);
	}
	
	if(badcoreNum>MAX_BAD_CORE_NUM)
		return false;	//set failed flag

	return true;
}

int getChainRateByTempAsicCoreEnabledFlag(int chainIndex, int freqIndex)
{
	int okCoreNum;
	int freq_value=atoi(freq_pll_1385[freqIndex].freq);
	okCoreNum=ASIC_NUM*ASIC_CORE_NUM-getChainTempBadCoreNum(chainIndex);

	return (freq_value*okCoreNum)/1000;
}

int getChainRateByAsicCoreEnabledFlag(int chainIndex, int freqIndex)
{
	int okCoreNum;
	int freq_value=atoi(freq_pll_1385[freqIndex].freq);
	okCoreNum=ASIC_NUM*ASIC_CORE_NUM-getChainBadCoreNum(chainIndex);

	return (freq_value*okCoreNum)/1000;
}

bool tryFixAsicCoreEnabledFlagByResult_searchMode(int chainIndex, int freqIndex)
{
	int i,j,k;
	int lastRate;
	int curRate;
	// save current enabled flag
	for(j=0;j<256;j++)
		for(k=0;k<256;k++)
			temp_asic_core_enabled_flag[chainIndex][j][k]=asic_core_enabled_flag[chainIndex][j][k];

	UpdateAsicCoreEnabledFlagByResult(chainIndex);
	if(isChainAsicBadCoreCanAccepted(chainIndex))
	{
		return true;	// that is ok, so we use this new enabled flag 
	}
	else
	{
		// restore old enabled flag
		for(j=0;j<256;j++)
			for(k=0;k<256;k++)
				asic_core_enabled_flag[chainIndex][j][k]=temp_asic_core_enabled_flag[chainIndex][j][k];
		return false;
	}
}

bool tryFixAsicCoreEnabledFlagByResult_testMode(int chainIndex)
{
	int i,j,k;

	// save current enabled flag
	for(j=0;j<256;j++)
		for(k=0;k<256;k++)
			temp_asic_core_enabled_flag[chainIndex][j][k]=asic_core_enabled_flag[chainIndex][j][k];

	UpdateAsicCoreEnabledFlagByResult(chainIndex);
	if(isChainAsicBadCoreCanAccepted(chainIndex))
	{
		return true;	// that is ok, so we use this new enabled flag 
	}
	else
	{
		// restore old enabled flag
		for(j=0;j<256;j++)
			for(k=0;k<256;k++)
				asic_core_enabled_flag[chainIndex][j][k]=temp_asic_core_enabled_flag[chainIndex][j][k];
		return false;
	}
}

bool tryFixSingleAsicCoreEnabledFlagByResult_testMode(int chainIndex, int chipIndex)
{
	int k;

	// save current enabled flag
	for(k=0;k<256;k++)
		temp_asic_core_enabled_flag[chainIndex][chipIndex][k]=asic_core_enabled_flag[chainIndex][chipIndex][k];

	UpdateSingleAsicCoreEnabledFlagByResult(chainIndex,chipIndex);
	if(isChainSingleAsicBadCoreCanAccepted(chainIndex,chipIndex))
	{
		return true;	// that is ok, so we use this new enabled flag 
	}
	else
	{
		// restore old enabled flag
		for(k=0;k<256;k++)
			asic_core_enabled_flag[chainIndex][chipIndex][k]=temp_asic_core_enabled_flag[chainIndex][chipIndex][k];
		return false;
	}
}


static int reset_work_data(void)
{
    int i, j;
	int asicNum=calculate_asic_number(ASIC_NUM);
    for(i =0 ; i < asicNum; i++)
    {
        for(j = 0; j < conf.dataCount; j++)
        {
            cgpu.results[i][j] = 0;
        }
        cgpu.result_array[i] = 0;
    }
    cgpu.index = 0;
    cgpu.valid_nonce = 0;
    cgpu.err_nonce = 0;
    cgpu.repeated_nonce = 0;
    return 0;
}

int cgpu_init(void)
{
    int ret = 0;
    memset(&cgpu, 0, sizeof(struct cgpu_info));

    ret = bitmain_axi_init();
    if(ret < 0)
    {
        printf("open axi driver error\n");
        return -1;
    }

    return 0;
}

bool isMD5fileSame(char *md5File1, char *md5File2)
{
	char buffer1[256],buffer2[256];
	FILE *fd1,*fd2;
	int rlen;
	fd1=fopen(md5File1,"rb");
	if(!fd1)
		return false;
	memset(buffer1,'\0',sizeof(buffer1));
	rlen=fread(buffer1, 1, 256, fd1);
	fclose(fd1);
	if(rlen<32)
		return false;
	
	fd2=fopen(md5File2,"rb");
	if(!fd2)
		return false;
	memset(buffer2,'\0',sizeof(buffer2));
	rlen=fread(buffer2, 1, 256, fd2);
	fclose(fd2);

	if(rlen<32)
		return false;

	// md5 is 32 chars
	buffer1[32]='\0';
	buffer2[32]='\0';
	
	if(strcmp(buffer1,buffer2)==0)
		return true;

	return false;
}


/* holder for curl fetch */
struct curl_fetch_st {
	char *payload;
	size_t size;
};

/* callback for curl fetch */
size_t curl_callback(void *contents, size_t size, size_t nmemb, void *userp) {
	size_t realsize = size * nmemb;                             /* calculate buffer size */
	struct curl_fetch_st *p = (struct curl_fetch_st *) userp;   /* cast pointer to fetch struct */

	/* expand buffer */
	p->payload = (char *)realloc(p->payload, p->size + realsize + 1);

	/* check buffer */
	if (p->payload == NULL) {
		/* this isn't good */
		fprintf(stderr, "ERROR: Failed to expand buffer in curl_callback");
		/* free buffer */
		free(p->payload);
		/* return */
		return -1;
	}

	/* copy contents to buffer */
	memcpy(&(p->payload[p->size]), contents, realsize);

	/* set new buffer size */
	p->size += realsize;

	/* ensure null termination */
	p->payload[p->size] = 0;

	/* return size */
	return realsize;
}

/* fetch and return url body via curl */
CURLcode curl_fetch_url(CURL *ch, const char *url, struct curl_fetch_st *fetch, const char *crt_dir, const char *crt_path) {
	CURLcode rcode;                   /* curl result code */

	/* init payload */
	fetch->payload = (char *)calloc(1, sizeof(fetch->payload));

	/* check payload */
	if (fetch->payload == NULL) {
		/* log error */
		fprintf(stderr, "ERROR: Failed to allocate payload in curl_fetch_url");
		/* return error */
		return CURLE_FAILED_INIT;
	}

	/* init size */
	fetch->size = 0;

	/* set url to fetch */
	curl_easy_setopt(ch, CURLOPT_URL, url);

	/* set calback function */
	curl_easy_setopt(ch, CURLOPT_WRITEFUNCTION, curl_callback);

	/* pass fetch struct pointer */
	curl_easy_setopt(ch, CURLOPT_WRITEDATA, (void *)fetch);

	/* set default user agent */
	curl_easy_setopt(ch, CURLOPT_USERAGENT, "Mozilla/5.0 (Windows NT 6.2; WOW64; rv:25.0) Gecko/20100101 Firefox/25.0");

	/* set timeout */
	curl_easy_setopt(ch, CURLOPT_TIMEOUT, 5);

	/* enable location redirects */
	curl_easy_setopt(ch, CURLOPT_FOLLOWLOCATION, 1);

	/* set maximum allowed redirects */
	curl_easy_setopt(ch, CURLOPT_MAXREDIRS, 1);

	curl_easy_setopt(ch, CURLOPT_SSL_VERIFYHOST, 0L);
	curl_easy_setopt(ch, CURLOPT_CAPATH, crt_dir);
	curl_easy_setopt(ch, CURLOPT_VERBOSE, 0L);
	curl_easy_setopt(ch, CURLOPT_SSLVERSION, CURL_SSLVERSION_DEFAULT);

	/* set the file with the certs vaildating the server */
	curl_easy_setopt(ch, CURLOPT_CAINFO, crt_path);

	/* disconnect if we can't validate server's cert */
	curl_easy_setopt(ch, CURLOPT_SSL_VERIFYPEER, 0L);

	/* fetch the url */
	rcode = curl_easy_perform(ch);

	/* return */
	return rcode;
}

int http_download(char *pUrl, char *file_path)
{
	int ret;
	FILE *fd;
	char logstr[256];
    char url[1024];
	CURL *ch;                                               /* curl handle */
	CURLcode rcode;                                         /* curl result code */

	struct curl_fetch_st curl_fetch;                        /* curl fetch struct */
	struct curl_fetch_st *cf = &curl_fetch;                 /* pointer to fetch struct */
	struct curl_slist *headers = NULL;                      /* http headers to send with request */
	
	/* url to test site */
	sprintf(url, "%s",pUrl);

	/* init curl handle */
	if ((ch = curl_easy_init()) == NULL) {
		/* log error */
	//	sprintf(logstr, "ERROR: Failed to create curl handle in fetch_session\n");
	//	writeLogFile(logstr);
		
		/* return error */
		return -1;
	}

	/* set content type */
	headers = curl_slist_append(headers, "Accept: application/zip");
	headers = curl_slist_append(headers, "Content-Type: application/zip");

	/* set curl options */
	curl_easy_setopt(ch, CURLOPT_CUSTOMREQUEST, "GET");	// GET  or  POST
	curl_easy_setopt(ch, CURLOPT_HTTPHEADER, headers);

	/* fetch page and capture return code */
	rcode = curl_fetch_url(ch, url, cf, NULL, NULL);
	
	/* cleanup curl handle */
	curl_easy_cleanup(ch);

	/* free headers */
	curl_slist_free_all(headers);

	/* check return code */
	if (rcode != CURLE_OK || cf->size < 1) {
		/* log error */
	//	sprintf(logstr,"ERROR: Failed to fetch url (%s) - curl said: %s \n",	url, curl_easy_strerror(rcode));
	//	writeLogFile(logstr);
		/* return error */
		return -1;
	}

	/* check payload */
	if (cf->payload != NULL) {
		fd=fopen(file_path,"wb");
		if(!fd)
		{
			sprintf(logstr,"Fatal Error: can not open %s \n",file_path);
			writeLogFile(logstr);
			
			while(1)sleep(1);
		}
		
		fwrite(cf->payload,1,cf->size,fd);
		fclose(fd);

		ret=cf->size;
		
		/* free payload */
		free(cf->payload);
	}
	else {
		/* error */
	//	sprintf(logstr, "ERROR: Failed to populate payload\n");
	//	writeLogFile(logstr);
		
		/* free payload */
		free(cf->payload);
		/* return */
		return -1;
	}

	return ret;
}

int http_get(char *pUrl, char **ret_str)
{
	int ret=-1;
	char logstr[256];
    char url[1024];
	CURL *ch;                                               /* curl handle */
	CURLcode rcode;                                         /* curl result code */

	struct curl_fetch_st curl_fetch;                        /* curl fetch struct */
	struct curl_fetch_st *cf = &curl_fetch;                 /* pointer to fetch struct */
	struct curl_slist *headers = NULL;                      /* http headers to send with request */
	
	/* url to test site */
	sprintf(url, "%s",pUrl);

	/* init curl handle */
	if ((ch = curl_easy_init()) == NULL) {
		/* log error */
	//	sprintf(logstr, "ERROR: Failed to create curl handle in fetch_session\n");
	//	writeLogFile(logstr);
		
		/* return error */
		return -1;
	}

	/* set content type */
	headers = curl_slist_append(headers, "Accept: application/zip");
	headers = curl_slist_append(headers, "Content-Type: application/zip");

	/* set curl options */
	curl_easy_setopt(ch, CURLOPT_CUSTOMREQUEST, "GET");	// GET  or  POST
	curl_easy_setopt(ch, CURLOPT_HTTPHEADER, headers);

	/* fetch page and capture return code */
	rcode = curl_fetch_url(ch, url, cf, NULL, NULL);
	
	/* cleanup curl handle */
	curl_easy_cleanup(ch);

	/* free headers */
	curl_slist_free_all(headers);

	/* check return code */
	if (rcode != CURLE_OK || cf->size < 1) {
		/* log error */
	//	sprintf(logstr,"ERROR: Failed to fetch url (%s) - curl said: %s \n",	url, curl_easy_strerror(rcode));
	//	writeLogFile(logstr);
		/* return error */
		return -1;
	}

	/* check payload */
	if (cf->payload != NULL) {
		*ret_str=(char *)malloc(cf->size+1);
		memcpy(*ret_str,cf->payload,cf->size);
		(*ret_str)[cf->size]='\0';
		ret=cf->size;
		/* free payload */
		free(cf->payload);
	}
	else {
		/* error */
	//	sprintf(logstr, "ERROR: Failed to populate payload\n");
	//	writeLogFile(logstr);
		/* free payload */
		free(cf->payload);
		/* return */
		return -1;
	}

	return ret;
}

unsigned int swap_uint32(unsigned int a)
{
	unsigned int b;
	b=((a&0xff)<<24)+(((a>>8)&0xff)<<16)+(((a>>16)&0xff)<<8)+(((a>>24)&0xff));
	return b;
}

#ifdef NEED_AUTH_SEARCHFREQ
bool PostRate(int hashrate)
{
	char logstr[256];
	char url[1024];
	unsigned char minerMAC[6];
	char *ret_str=NULL;
	int remote_flag;
	
	get_mac("eth0",minerMAC);

	sprintf(url,"http://bindminers.bitmain.com:6060/minerauth/postrate.asp?minerMAC=%02x%02x%02x%02x%02x%02x&hashrate=%d",
			minerMAC[0],minerMAC[1],minerMAC[2],minerMAC[3],minerMAC[4],minerMAC[5],hashrate);
	
	if(http_get(url,&ret_str)>0)
	{
		sprintf(logstr,"server return: ");
		writeLogFile(logstr);
		writeLogFile(ret_str);	// ret_str is very larger than logstr's length, so we print independently
		sprintf(logstr,"\n");
		writeLogFile(logstr);
		
		remote_flag=atoi(ret_str);
		if(23==remote_flag)	// magic number is 23, means server get the hashrate OK!
		{
			sprintf(logstr,"hashrate=%d posted OK!\n",hashrate);
			writeLogFile(logstr);
			if(ret_str)
				free(ret_str);
			return true;
		}
	}

	sprintf(logstr,"post hashrate FAILED!\n");
	writeLogFile(logstr);

	if(ret_str)
		free(ret_str);
	return false;
}


bool isAuthToRun()
{
	char logstr[256];
	char url[1024];
	unsigned char minerMAC[6];
	unsigned char random_bytes[6];
	unsigned int ran_int1;
	unsigned int ran_int2;
	unsigned int ran_int3;
	unsigned int local_flag,remote_flag;
	char *ret_str=NULL;
	
	get_mac("eth0",minerMAC);
	
    RAND_seed(minerMAC, 6);
	RAND_bytes(random_bytes, 6);

	memcpy(&ran_int1,minerMAC,4);
	memcpy(((unsigned char *)&ran_int2),minerMAC+4,2);
	memcpy(((unsigned char *)&ran_int2)+2,random_bytes,2);
	memcpy(&ran_int3,random_bytes+2,4);

	ran_int1=swap_uint32(ran_int1);
	ran_int2=swap_uint32(ran_int2);
	ran_int3=swap_uint32(ran_int3);

	local_flag=((ran_int1 ^ ran_int2) ^ ran_int3);

	sprintf(logstr,"try getauth on MAC=%02x%02x%02x%02x%02x%02x reqID=%02x%02x%02x%02x%02x%02x\n",minerMAC[0],minerMAC[1],minerMAC[2],minerMAC[3],minerMAC[4],minerMAC[5],random_bytes[0],random_bytes[1],random_bytes[2],random_bytes[3],random_bytes[4],random_bytes[5]);
	writeLogFile(logstr);
	
	sprintf(url,"http://bindminers.bitmain.com:6060/minerauth/doauth.asp?minerMAC=%02x%02x%02x%02x%02x%02x&reqID=%02x%02x%02x%02x%02x%02x",
		minerMAC[0],minerMAC[1],minerMAC[2],minerMAC[3],minerMAC[4],minerMAC[5],random_bytes[0],random_bytes[1],random_bytes[2],random_bytes[3],random_bytes[4],random_bytes[5]);

	if(http_get(url,&ret_str)>0)
	{
		sprintf(logstr,"getauth return: ");
		writeLogFile(logstr);
		writeLogFile(ret_str);	// ret_str is very larger than logstr's length, so we print independently
		sprintf(logstr,"\n");
		writeLogFile(logstr);
		
		remote_flag=(unsigned int)strtoul(ret_str,NULL,10);
	//	sprintf(logstr,"remote_flag=%u local_flag=%u\n",remote_flag,local_flag);
	//	writeLogFile(logstr);
		
		if(local_flag==remote_flag)
		{
			sprintf(logstr,"getauth OK!\n");
			writeLogFile(logstr);
			if(ret_str)
				free(ret_str);
			return true;
		}
	}

	sprintf(logstr,"getauth FAILED!\n");
	writeLogFile(logstr);

	if(ret_str)
		free(ret_str);
	return false;
}
#endif

int run_cmd(char *cmdline)
{
	char logstr[1024];
	char buffer[256];
	FILE *fp;
	int rlen;

	memset(buffer, '\0', sizeof(buffer));
	if((fp = popen(cmdline,"r")) == NULL)
		return 0;

	while(fgets(buffer, sizeof(buffer), fp)!=NULL)
	{
		sprintf(logstr,"ret=%s\n",buffer);
		writeLogFile(logstr);

		memset(buffer, '\0', sizeof(buffer));
		usleep(1000);
	}
	pclose(fp);
	return 0;
}

int create_shell(char *cmdline)
{
	char buffer[1024];
	FILE *fp;
	int rlen;

	fp=fopen("/etc/config/run.sh","wb");
	sprintf(buffer,"#!/bin/sh\n");
	fwrite(buffer,1,strlen(buffer),fp);

	sprintf(buffer,"%s\n",cmdline);
	fwrite(buffer,1,strlen(buffer),fp);

	fclose(fp);

	system("chmod +x /etc/config/run.sh");
	return 0;
}


bool is32xPattenReady=false;

void *download_testpatten_func(void * arg)
{
	char cmd[1024];
	char url[256];
	char file_path[256];
	int i;
	char md5file1[256],md5file2[256];
	char logstr[256];
	int failed_counter=0;

	sprintf(cmd,"/bin/mkdir %s -p",TESTPATTEN_DIR);
	system(cmd);
	
	for(i=0;i<64;i++)
	{
	//	sprintf(logstr,"check %d test patten file md5...\n",i+1);
	//	writeLogFile(logstr);
		
		sprintf(cmd,"/bin/rm %s/md5.txt -f",TESTPATTEN_DIR);
		system(cmd);
		
		sprintf(cmd,"/usr/bin/md5sum %s/minertest64_%02d.bin > %s/md5.txt",TESTPATTEN_DIR,i+1,TESTPATTEN_DIR);
		system(cmd);

		sprintf(md5file1,"%s/md5.txt",TESTPATTEN_DIR);
		sprintf(md5file2,"%s/minertest64_%02d.txt",TESTPATTEN_DIR,i+1);
		if(isMD5fileSame(md5file1,md5file2))
		{
		//	sprintf(logstr,"%d test patten file md5 OK!\n",i+1);
		//	writeLogFile(logstr);
		}
		else
		{
			failed_counter=0;
			
			do{
			//	sprintf(logstr,"%d test patten file need download ...\n",i+1);
			//	writeLogFile(logstr);

				sprintf(cmd,"/bin/rm %s/minertest64_%02d.bin -f",TESTPATTEN_DIR,i+1);
				system(cmd);

				sprintf(cmd,"/bin/rm %s/minertest64_%02d.txt -f",TESTPATTEN_DIR,i+1);
				system(cmd);

				sprintf(url,"%s%02d.bin",TESTPATTEN_DOWNLOAD_URL_PREFIX,i+1);
				sprintf(file_path,"%s/minertest64_%02d.bin",TESTPATTEN_DIR,i+1);
				http_download(url,file_path);

				sprintf(url,"%s%02d.txt",TESTPATTEN_DOWNLOAD_URL_PREFIX,i+1);
				sprintf(file_path,"%s/minertest64_%02d.txt",TESTPATTEN_DIR,i+1);
				http_download(url,file_path);

			//	sprintf(logstr,"calculate %d test patten file md5...\n",i+1);
			//	writeLogFile(logstr);

				sprintf(cmd,"/bin/rm %s/md5.txt -f",TESTPATTEN_DIR);
				system(cmd);
				
				sprintf(cmd,"/usr/bin/md5sum %s/minertest64_%02d.bin > %s/md5.txt",TESTPATTEN_DIR,i+1,TESTPATTEN_DIR);
				system(cmd);

				sprintf(md5file1,"%s/md5.txt",TESTPATTEN_DIR);
				sprintf(md5file2,"%s/minertest64_%02d.txt",TESTPATTEN_DIR,i+1);
				if(isMD5fileSame(md5file1,md5file2))
				{
				//	sprintf(logstr,"%d test patten file md5 OK!\n",i+1);
				//	writeLogFile(logstr);

					failed_counter=0;
				}
				else
				{
				//	sprintf(logstr,"%d test patten file md5 ERROR!\n",i+1);
				//	writeLogFile(logstr);

					failed_counter=23;	// magic number 
					sleep(10);
				}
			}while(failed_counter>0);
		}
	}

	sprintf(logstr,"32xPatten download and ready!\n");
	writeLogFile(logstr);
	is32xPattenReady=true;
	return 0;
}

void *show_status_func(void * arg)
{
    int count = 0;
    int i;
	signed char top_temp=0,temp=0,low_temp;
	int pwm_percent=100;
	char logstr[256];
	
    while(!ExitFlag)
    {
    	count++;
        if(gIsReadTemp)
        {
        	if(count<30)
        	{
        		usleep(100*1000);
				continue;
        	}
			
        	count=0;
			top_temp=0;
			temp=0;
			low_temp=125;


			pthread_mutex_lock(&temp_work_mutex);
			usleep(10000);
				
			for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
			{
				if(cgpu.chain_exist[i]==0 
#ifndef ALL_CHAIN_INTEST
					|| testDone[i]
#endif
					)
					continue;

				if(gIsReadTemp)
				{
#ifdef T9_18
					if(fpga_version>=0xE)
					{
						if(i!=8 && i!=10 && i!=12)
							continue;
					}
					else
					{
						if(i%3!=1)	// only 1,4,7 chain hash temp chips
							continue;
					}
#endif
					pthread_mutex_lock(&read_temp_mutex);
	            	temp=read_asic_temperature(i);

					if(board_temp[i]<temp)
						board_temp[i]=temp;

					pthread_mutex_unlock(&read_temp_mutex);
				}
				
				if(top_temp<temp)
					top_temp=temp;

				if(low_temp>temp)
					low_temp=temp;
			}

			pthread_mutex_unlock(&temp_work_mutex);

#ifdef T9_18
			if(fpga_version>=0xE)
			{
				// copy chain[8] temp to chain[1] and chain[9]
				for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
				{
					if(cgpu.chain_exist[i]==0 
#ifndef ALL_CHAIN_INTEST
						|| testDone[i]
#endif
						)
						continue;

					switch(i)
					{
					case 1:
					case 9:
						board_temp[i]=board_temp[8];
						break;
					case 2:
					case 11:
						board_temp[i]=board_temp[10];
						break;
					case 3:
					case 13:
						board_temp[i]=board_temp[12];
						break;
					default:
						break;
					}
					
				}
			}
			else
			{
				// copy chain[1] temp to chain[0] and chain[2]
				for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
				{
					if(cgpu.chain_exist[i]==0 
#ifndef ALL_CHAIN_INTEST
						|| testDone[i]
#endif
						)
						continue;

					if(i%3!=1)
						board_temp[i]=board_temp[((i/3)*3)+1];
				}
			}
#endif
			sprintf(logstr,"get top_temp=%d low_temp=%d\n",top_temp,low_temp);
			writeLogFile(logstr);

       		if(top_temp==0 || low_temp==0)
			{
				pwm_percent=100;
			}
    		else if(low_temp<75)
    		{
    			if(pwm_percent>30)
    			{
    				pwm_percent=30;
    			}
    			else if(pwm_percent>10)
    			{
        			pwm_percent-=10;
    			}
				else
				{
					pwm_percent=0;
				}
    		}
			else
			{
				if(low_temp>75)
					pwm_percent=30+(low_temp-75);
			}

			if(top_temp>85)
			{
				pwm_percent=100;
			}
			
			set_PWM(pwm_percent);
			sprintf(logstr,"set fan PWM=%d\n",pwm_percent);
			writeLogFile(logstr);
        }
		else
		{
			count = 0;
			top_temp=0;
			temp=0;
			pwm_percent=100;
		}

        usleep(100*1000);
    }

	showExit=true;
	return 0;
}

void set_fan_by_temp()
{
    int i;
	signed char top_temp=0,temp=0,low_temp;
	int pwm_percent=100;
	char logstr[256];
	
	top_temp=0;
	temp=0;
	low_temp=125;
	
	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
	{
		if(cgpu.chain_exist[i]==0
#ifndef ALL_CHAIN_INTEST
			|| testDone[i]
#endif
			)
			continue;

#ifdef T9_18
		if(fpga_version>=0xE)
		{
			if(i!=8 && i!=10 && i!=12)	// only 8,10,12 chain has temp chips
				continue;
		}
		else
		{
			if(i%3!=1)	// only 1,4,7 chain has temp chips
				continue;
		}
#endif
    	temp=read_asic_temperature(i);

		if(board_temp[i]<temp)
			board_temp[i]=temp;
		
		if(top_temp<temp)
			top_temp=temp;

		if(low_temp>temp)
			low_temp=temp;
	}

#ifdef T9_18
	// copy chain[1] temp to chain[0] and chain[2]
	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
	{
		if(cgpu.chain_exist[i]==0
#ifndef ALL_CHAIN_INTEST
			|| testDone[i]
#endif
			)
			continue;

		if(fpga_version>=0xE)
		{
			switch(i)
			{
			case 1:
			case 9:
				board_temp[i]=board_temp[8];
				break;
			case 2:
			case 11:
				board_temp[i]=board_temp[10];
				break;
			case 3:
			case 13:
				board_temp[i]=board_temp[12];
				break;
			default:
				break;
			}
		}
		else
		{
			if(i%3!=1)
				board_temp[i]=board_temp[((i/3)*3)+1];
		}
	}
#endif

	sprintf(logstr,"get top_temp=%d low_temp=%d\n",top_temp,low_temp);
	writeLogFile(logstr);

	if(top_temp==0 || low_temp==0)
	{
		pwm_percent=100;
	}
	else if(low_temp<75)
	{
		if(pwm_percent>30)
		{
			pwm_percent=30;
		}
		else if(pwm_percent>10)
		{
			pwm_percent-=10;
		}
		else
		{
			pwm_percent=0;
		}
	}
	else
	{
		if(low_temp>75)
			pwm_percent=30+(low_temp-75);
	}

	if(top_temp>85)
	{
		pwm_percent=100;
	}
	
	set_PWM(pwm_percent);
	sprintf(logstr,"set fan PWM=%d\n",pwm_percent);
	writeLogFile(logstr);

	return 0;
}

void send_pic_heart_once()
{
	int i;
	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
	{
		if(cgpu.chain_exist[i]==0)
			continue;
		
		pic_heart_beat(i);
	}
}

void *pic_heart_beat_func(void *arg)
{
    int i=0,j=0;
    while(!ExitFlag)
    {
    	if(!start_pic_heart)
    	{
    		j=0;
    		usleep(100000);
    		continue;
    	}
		
        if(j>100)
    	{
	    	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
	    	{
	    		if(cgpu.chain_exist[i]==0 || (!start_pic_heart))
					continue;
				
	        	pic_heart_beat(i);
	    	}
            j=0;
    	}
        usleep(100000);
		j++;
    }

    picheartExit=true;
	return 0;
}

static uint32_t get_plldata_i(int type,int freq)
{
    uint32_t i;
    char freq_str[10];
    sprintf(freq_str,"%d", freq);

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

    return i;
    //printf("PLL1 %#x, PLL2 %#x, vilpll %#x\n",*reg_data, *reg_data2, *vil_data);
}

int send_func_all()
{
    int which_asic[BITMAIN_MAX_CHAIN_NUM];
    int i,j;
    unsigned int work_fifo_ready = 0;
    int index[BITMAIN_MAX_CHAIN_NUM];
    struct work * works, *work;
    unsigned char data_fil[TW_WRITE_COMMAND_LEN] = {0xff};
    unsigned char data_vil[TW_WRITE_COMMAND_LEN_VIL] = {0xff};
    struct vil_work_1387 work_vil_1387;
    unsigned int buf[TW_WRITE_COMMAND_LEN/sizeof(unsigned int)]= {0};
    unsigned int buf_vil[TW_WRITE_COMMAND_LEN_VIL/sizeof(unsigned int)]= {0};
	int chainIndex;
	char logstr[256];
	bool isSendOver=false;
	int wait_counter=0;
	bool sendStartFlag[BITMAIN_MAX_CHAIN_NUM];
	
	for(i=0;i<BITMAIN_MAX_CHAIN_NUM;i++)
	{
		index[i]=0;
		which_asic[i]=0;
		sendStartFlag[i]=StartSendFlag[i];
	}

	while(!isSendOver)
	{
	    // send work
	    for(chainIndex=0;chainIndex<BITMAIN_MAX_CHAIN_NUM;chainIndex++)
	    {
	    	if(cgpu.chain_exist[chainIndex] == 0 || (!sendStartFlag[chainIndex]))
				continue;
			
	        while(which_asic[chainIndex] < ASIC_NUM)
	        {
	            work_fifo_ready = get_buffer_space();
	            if(work_fifo_ready & (0x1 << chainIndex))   // work fifo is not full, we can send work
	            {
	            	wait_counter=0;	// clear wait fifo counter
					
	                if(cgpu.CommandMode)    // fil mode
	                {
	                    memset(buf, 0x0, TW_WRITE_COMMAND_LEN/sizeof(unsigned int));

	                    // get work for sending to asic
	                    works = cgpu.works[which_asic[chainIndex]]; // which ASIC
	                    work = works + index[chainIndex];      // which test data for the ASIC

	                    // parse work data
	                    memset(data_fil, 0x0, TW_WRITE_COMMAND_LEN);
	                    data_fil[0] = NORMAL_BLOCK_MARKER;
	                    data_fil[1] = chainIndex | 0x80; //set chain id and enable it
	                    for(i=0; i<MIDSTATE_LEN; i++)
	                    {
	                        data_fil[i+4] = work->midstate[i];
	                    }
	                    for(i=0; i<DATA2_LEN; i++)
	                    {
	                        data_fil[i+40] = work->data[i];
	                    }

	                    // send work
	                    //printf("\n");
	                    for(j=0; j<TW_WRITE_COMMAND_LEN/sizeof(unsigned int); j++)
	                    {
	                        buf[j] = (data_fil[4*j + 0] << 24) | (data_fil[4*j + 1] << 16) | (data_fil[4*j + 2] << 8) | data_fil[4*j + 3];
	                        if(j==9)
	                        {
	                            buf[j] = index[chainIndex];
	                        }
	                        //applog(LOG_DEBUG,"%s: buf[%d] = 0x%08x\n", __FUNCTION__, j, buf[j]);
	                    }

						pthread_mutex_lock(&temp_work_mutex);
	                    set_TW_write_command(buf);
						pthread_mutex_unlock(&temp_work_mutex);
	                    which_asic[chainIndex]++;
	                }
	                else    // vil mode
	                {
	                    if(ASIC_TYPE == 1387)
	                    {
	                        //printf("\n--- send work\n");
	                        memset(buf_vil, 0x0, TW_WRITE_COMMAND_LEN_VIL/sizeof(unsigned int));

	                        works = cgpu.works[which_asic[chainIndex]]; // which ASIC
	                        work = works + index[chainIndex];      // which test data for the ASIC

	                        // parse work data
	                        memset(&work_vil_1387, 0, sizeof(struct vil_work_1387));
	                        work_vil_1387.work_type = NORMAL_BLOCK_MARKER;
	                        work_vil_1387.chain_id = 0x80 | chainIndex;
	                        work_vil_1387.reserved1[0]= 0;
	                        work_vil_1387.reserved1[1]= 0;
	                        work_vil_1387.work_count = index[chainIndex];
	                        for(i=0; i<DATA2_LEN; i++)
	                        {
	                            work_vil_1387.data[i] = work->data[i];
	                        }
	                        for(i=0; i<MIDSTATE_LEN; i++)
	                        {
	                            work_vil_1387.midstate[i] = work->midstate[i];
	                        }

	                        // send work
	                        buf_vil[0] = (work_vil_1387.work_type << 24) | (work_vil_1387.chain_id << 16) | (work_vil_1387.reserved1[0] << 8) | work_vil_1387.reserved1[1];
	                        buf_vil[1] = work_vil_1387.work_count;
	                        for(j=2; j<DATA2_LEN/sizeof(int)+2; j++)
	                        {
	                            buf_vil[j] = (work_vil_1387.data[4*(j-2) + 0] << 24) | (work_vil_1387.data[4*(j-2) + 1] << 16) | (work_vil_1387.data[4*(j-2) + 2] << 8) | work_vil_1387.data[4*(j-2) + 3];
	                        }
	                        for(j=5; j<MIDSTATE_LEN/sizeof(unsigned int)+5; j++)
	                        {
	                            buf_vil[j] = (work_vil_1387.midstate[4*(j-5) + 0] << 24) | (work_vil_1387.midstate[4*(j-5) + 1] << 16) | (work_vil_1387.midstate[4*(j-5) + 2] << 8) | work_vil_1387.midstate[4*(j-5) + 3];;
	                        }

							pthread_mutex_lock(&temp_work_mutex);
	                        set_TW_write_command_vil(buf_vil);
							pthread_mutex_unlock(&temp_work_mutex);

	                        which_asic[chainIndex]++;

	                    }
	                    else
	                    {
	                        //printf("\n--- send work\n");
	                        memset(buf_vil, 0x0, TW_WRITE_COMMAND_LEN_VIL/sizeof(unsigned int));
	                        // get work for sending to asic
	                        //work_fil = (struct work *)((void *)cgpu.works[which_asic] + index*sizeof(struct work));
	                        works = cgpu.works[which_asic[chainIndex]]; // which ASIC
	                        work = works + index[chainIndex];      // which test data for the ASIC

	                        // parse work data
	                        memset(data_vil, 0x00, TW_WRITE_COMMAND_LEN_VIL);
	                        data_vil[0] = NORMAL_BLOCK_MARKER;
	                        data_vil[1] = chainIndex | 0x80; //set chain id and enable it
	                        data_vil[4] = 0x01 << 5;                // type
	                        data_vil[5] = sizeof(struct vil_work);  // length
	                        data_vil[6] = index[chainIndex];               // wc_base / work_id
	                        data_vil[7] = 0x01;                     // mid_num

	                        for(i=0; i<MIDSTATE_LEN; i++)
	                        {
	                            data_vil[i+8] = work->midstate[i];
	                        }
	                        for(i=0; i<DATA2_LEN; i++)
	                        {
	                            data_vil[i+40] = work->data[i];
	                        }

	                        // send work
	                        for(j=0; j<TW_WRITE_COMMAND_LEN_VIL/sizeof(unsigned int); j++)
	                        {
	                            buf_vil[j] = (data_vil[4*j + 0] << 24) | (data_vil[4*j + 1] << 16) | (data_vil[4*j + 2] << 8) | data_vil[4*j + 3];
	                            //printf("%s: buf_vil[%d] = 0x%08x\n", __FUNCTION__, j, buf_vil[j]);
	                        }

							pthread_mutex_lock(&temp_work_mutex);
	                        set_TW_write_command_vil(buf_vil);
							pthread_mutex_unlock(&temp_work_mutex);

	                        which_asic[chainIndex]++;
	                    }
	                }
	                send_work_num[chainIndex]++;
	                //printf("%s: send_work_num = %d\n", __FUNCTION__, send_work_num);
	            }
	            else    //work fifo is full, wait for 1ms
	            {
	            	wait_counter++;
	                break;
	            }
	        }
	
			if(which_asic[chainIndex] >= ASIC_NUM)
			{
				which_asic[chainIndex]=0;	// then send from chip[0] ....
				index[chainIndex]++;	// switch to next work
				if(index[chainIndex] >= chain_DataCount[chainIndex])
					sendStartFlag[chainIndex]=false;
			}
			
			if(wait_counter>2000)
			{// timeout on wait for fifo ready
				sprintf(logstr,"Fatal Error: send work timeout\n");
				writeLogFile(logstr);
				break;
			}
	    }
		usleep(5000);
		
		isSendOver=true;
		for(i=0;i<BITMAIN_MAX_CHAIN_NUM;i++)
		{
			if(cgpu.chain_exist[i] == 0 || (!StartSendFlag[i]))
				continue;
			
	    	if(index[i] < chain_DataCount[i])
	    	{
	    		isSendOver=false;

			//	printf("index[%d]=%d < chain_DataCount[%d]=%d\n",i,index[i],i,chain_DataCount[i]);
	    	}
		}
	}

	for(i=0;i<BITMAIN_MAX_CHAIN_NUM;i++)
	{
		if(cgpu.chain_exist[i] == 0 || (!StartSendFlag[i]))
			continue;
		
		StartSendFlag[i]=false;	// when send over , must set this flag to false!!!
		sprintf(logstr,"get send work num :%d on Chain[%d]\n", send_work_num[i],i);
		writeLogFile(logstr);

		sendExit[i]=true;
	}

	return 0;
}

void *send_func(void * arg)
{
    int which_asic;
    int i,j;
    unsigned int work_fifo_ready = 0;
    int index;
    struct work * works, *work;
    unsigned char data_fil[TW_WRITE_COMMAND_LEN] = {0xff};
    unsigned char data_vil[TW_WRITE_COMMAND_LEN_VIL] = {0xff};
    struct vil_work_1387 work_vil_1387;
    unsigned int buf[TW_WRITE_COMMAND_LEN/sizeof(unsigned int)]= {0};
    unsigned int buf_vil[TW_WRITE_COMMAND_LEN_VIL/sizeof(unsigned int)]= {0};
    unsigned int number=0;
	int chainIndex=(int)arg;
	char logstr[256];

	printf("%s: start on chain[%d]...\n", __FUNCTION__, chainIndex);
	number = ASIC_NUM;
	index=0;

	while(!ExitFlag)
	{
		if(!StartSendFlag[chainIndex])
		{
			number = ASIC_NUM;
			index=0;
	
			usleep(100000);
			continue;
		}
		
	    // send work
	    while(index < chain_DataCount[chainIndex] && StartSendFlag[chainIndex])      // send work for dataCount loop which is set in config file
	    {
	        for(which_asic=0; which_asic < number && StartSendFlag[chainIndex];)    // send work to every asic
	        {
	            work_fifo_ready = get_buffer_space();
	            if(work_fifo_ready & (0x1 << chainIndex))   // work fifo is not full, we can send work
	            {
		                if(cgpu.CommandMode)    // fil mode
		                {
		                    memset(buf, 0x0, TW_WRITE_COMMAND_LEN/sizeof(unsigned int));

		                    // get work for sending to asic
		                    works = cgpu.works[which_asic]; // which ASIC
		                    work = works + index;      // which test data for the ASIC

		                    // parse work data
		                    memset(data_fil, 0x0, TW_WRITE_COMMAND_LEN);
		                    data_fil[0] = NORMAL_BLOCK_MARKER;
		                    data_fil[1] = chainIndex | 0x80; //set chain id and enable it
		                    for(i=0; i<MIDSTATE_LEN; i++)
		                    {
		                        data_fil[i+4] = work->midstate[i];
		                    }
		                    for(i=0; i<DATA2_LEN; i++)
		                    {
		                        data_fil[i+40] = work->data[i];
		                    }

		                    // send work
		                    //printf("\n");
		                    for(j=0; j<TW_WRITE_COMMAND_LEN/sizeof(unsigned int); j++)
		                    {
		                        buf[j] = (data_fil[4*j + 0] << 24) | (data_fil[4*j + 1] << 16) | (data_fil[4*j + 2] << 8) | data_fil[4*j + 3];
		                        if(j==9)
		                        {
		                            buf[j] = index;
		                        }
		                        //applog(LOG_DEBUG,"%s: buf[%d] = 0x%08x\n", __FUNCTION__, j, buf[j]);
		                    }

							pthread_mutex_lock(&temp_work_mutex);
		                    set_TW_write_command(buf);
							pthread_mutex_unlock(&temp_work_mutex);
		                    which_asic++;
		                }
		                else    // vil mode
		                {
		                    if(ASIC_TYPE == 1387)
		                    {
		                        //printf("\n--- send work\n");
		                        memset(buf_vil, 0x0, TW_WRITE_COMMAND_LEN_VIL/sizeof(unsigned int));

		                        works = cgpu.works[which_asic]; // which ASIC
		                        work = works + index;      // which test data for the ASIC

		                        // parse work data
		                        memset(&work_vil_1387, 0, sizeof(struct vil_work_1387));
		                        work_vil_1387.work_type = NORMAL_BLOCK_MARKER;
		                        work_vil_1387.chain_id = 0x80 | chainIndex;
		                        work_vil_1387.reserved1[0]= 0;
		                        work_vil_1387.reserved1[1]= 0;
		                        work_vil_1387.work_count = index;
		                        for(i=0; i<DATA2_LEN; i++)
		                        {
		                            work_vil_1387.data[i] = work->data[i];
		                        }
		                        for(i=0; i<MIDSTATE_LEN; i++)
		                        {
		                            work_vil_1387.midstate[i] = work->midstate[i];
		                        }

		                        // send work
		                        buf_vil[0] = (work_vil_1387.work_type << 24) | (work_vil_1387.chain_id << 16) | (work_vil_1387.reserved1[0] << 8) | work_vil_1387.reserved1[1];
		                        buf_vil[1] = work_vil_1387.work_count;
		                        for(j=2; j<DATA2_LEN/sizeof(int)+2; j++)
		                        {
		                            buf_vil[j] = (work_vil_1387.data[4*(j-2) + 0] << 24) | (work_vil_1387.data[4*(j-2) + 1] << 16) | (work_vil_1387.data[4*(j-2) + 2] << 8) | work_vil_1387.data[4*(j-2) + 3];
		                        }
		                        for(j=5; j<MIDSTATE_LEN/sizeof(unsigned int)+5; j++)
		                        {
		                            buf_vil[j] = (work_vil_1387.midstate[4*(j-5) + 0] << 24) | (work_vil_1387.midstate[4*(j-5) + 1] << 16) | (work_vil_1387.midstate[4*(j-5) + 2] << 8) | work_vil_1387.midstate[4*(j-5) + 3];;
		                        }

								pthread_mutex_lock(&temp_work_mutex);
		                        set_TW_write_command_vil(buf_vil);
								pthread_mutex_unlock(&temp_work_mutex);

		                        which_asic++;

		                    }
		                    else
		                    {
		                        //printf("\n--- send work\n");
		                        memset(buf_vil, 0x0, TW_WRITE_COMMAND_LEN_VIL/sizeof(unsigned int));
		                        // get work for sending to asic
		                        //work_fil = (struct work *)((void *)cgpu.works[which_asic] + index*sizeof(struct work));
		                        works = cgpu.works[which_asic]; // which ASIC
		                        work = works + index;      // which test data for the ASIC

		                        // parse work data
		                        memset(data_vil, 0x00, TW_WRITE_COMMAND_LEN_VIL);
		                        data_vil[0] = NORMAL_BLOCK_MARKER;
		                        data_vil[1] = chainIndex | 0x80; //set chain id and enable it
		                        data_vil[4] = 0x01 << 5;                // type
		                        data_vil[5] = sizeof(struct vil_work);  // length
		                        data_vil[6] = index;               // wc_base / work_id
		                        data_vil[7] = 0x01;                     // mid_num

		                        for(i=0; i<MIDSTATE_LEN; i++)
		                        {
		                            data_vil[i+8] = work->midstate[i];
		                        }
		                        for(i=0; i<DATA2_LEN; i++)
		                        {
		                            data_vil[i+40] = work->data[i];
		                        }

		                        // send work
		                        for(j=0; j<TW_WRITE_COMMAND_LEN_VIL/sizeof(unsigned int); j++)
		                        {
		                            buf_vil[j] = (data_vil[4*j + 0] << 24) | (data_vil[4*j + 1] << 16) | (data_vil[4*j + 2] << 8) | data_vil[4*j + 3];
		                            //printf("%s: buf_vil[%d] = 0x%08x\n", __FUNCTION__, j, buf_vil[j]);
		                        }

								pthread_mutex_lock(&temp_work_mutex);
		                        set_TW_write_command_vil(buf_vil);
								pthread_mutex_unlock(&temp_work_mutex);

		                        which_asic++;
		                    }
		                }
		                send_work_num[chainIndex]++;
	                //printf("%s: send_work_num = %d\n", __FUNCTION__, send_work_num);
	            }
	            else    //work fifo is full, wait for 1ms
	            {
	                usleep(5000);
	            }
	        }
	        index++;
	    }

		StartSendFlag[chainIndex]=false;	// when send over , must set this flag to false!!!

		sprintf(logstr,"get send work num :%d on Chain[%d]\n", send_work_num[chainIndex],chainIndex);
		writeLogFile(logstr);
	}
	
	sendExit[chainIndex]=true;
	return 0;
}

static uint32_t last_nonce[BITMAIN_MAX_CHAIN_NUM], llast_nonce[BITMAIN_MAX_CHAIN_NUM];
static unsigned int work_id[BITMAIN_MAX_CHAIN_NUM];
static unsigned int m_nonce[BITMAIN_MAX_CHAIN_NUM];
void *receive_func(void *arg)
{
    unsigned int j=0, n=0, nonce_number = 0, read_loop=0;
    unsigned int buf[2] = {0,0};
    
    uint8_t which_asic_nonce = 0;
    uint8_t which_core_nonce = 0;
    uint8_t whose_nonce = 0, nonce_index=0;
    unsigned int OpenCoreNum1 = conf.OpenCoreNum1;
    unsigned int OpenCoreNum2 = conf.OpenCoreNum2;
    unsigned int OpenCoreNum3 = conf.OpenCoreNum3;
    unsigned int OpenCoreNum4 = conf.OpenCoreNum4;
	char logstr[256];
	int chainIndex;
	int asicNum=calculate_asic_number(ASIC_NUM);

    memset(repeated_nonce_id, 0xff, sizeof(repeated_nonce_id));
	memset(last_nonce,0x00,sizeof(last_nonce));
	memset(llast_nonce,0x00,sizeof(llast_nonce));
	memset(work_id,0x00,sizeof(work_id));
	memset(m_nonce,0x00,sizeof(m_nonce));

    while(!ExitFlag)
    {
        if(!start_receive)
        {
        	j=0;
			n=0;
			nonce_number = 0;
			read_loop=0;
		    buf[0]=0;
			buf[1]=0;
		    
		    which_asic_nonce = 0;
		    which_core_nonce = 0;
		    whose_nonce = 0;
			nonce_index=0;
		    OpenCoreNum1 = conf.OpenCoreNum1;
		    OpenCoreNum2 = conf.OpenCoreNum2;
		    OpenCoreNum3 = conf.OpenCoreNum3;
		    OpenCoreNum4 = conf.OpenCoreNum4;
			
		    memset(repeated_nonce_id, 0xff, sizeof(repeated_nonce_id));
			memset(last_nonce,0x00,sizeof(last_nonce));
			memset(llast_nonce,0x00,sizeof(llast_nonce));
			memset(work_id,0x00,sizeof(work_id));
			memset(m_nonce,0x00,sizeof(m_nonce));
			
        	usleep(100000);
            continue;
        }
		
        read_loop = 0;

        nonce_number = get_nonce_number_in_fifo() & MAX_NONCE_NUMBER_IN_FIFO;
        //applog(LOG_DEBUG,"%s: --- nonce_number = %d\n", __FUNCTION__, nonce_number);
        if(nonce_number>0)
        {
            read_loop = nonce_number;
            //applog(LOG_DEBUG,"%s: read_loop = %d\n", __FUNCTION__, read_loop);

            for(j=0; j<read_loop; j++)
            {
                get_return_nonce(buf);
                //printf("%s: buf[0] = 0x%08x\n", __FUNCTION__, buf[0]);
                //printf("%s: buf[1] = 0x%08x\n", __FUNCTION__, buf[1]);
                if(buf[0] & WORK_ID_OR_CRC) //nonce
                {
                    if(gBegin_get_nonce)
                    {
                        if(buf[0] & NONCE_INDICATOR)
                        {
                        	chainIndex=CHAIN_NUMBER(buf[0]);
							if(chainIndex<0 || chainIndex>=BITMAIN_MAX_CHAIN_NUM || cgpu.chain_exist[chainIndex] == 0 || testDone[chainIndex])
							{
								sprintf(logstr,"Error chain index=%d of nonce!!!\n",chainIndex);
								writeLogFile(logstr);
								continue;
							}

                            if(cgpu.CommandMode)    // fil mode
                            {
                                work_id[chainIndex] = (buf[0] >> 16) & 0x00007fff;
                            }
                            else    // vil mode
                            {
                                if(ASIC_TYPE == 1387)
                                {
                                    work_id[chainIndex] = (buf[0] >> 16) & 0x00007fff;
                                }
                                else
                                {
                                    work_id[chainIndex] = (buf[0] >> 24) & 0x0000007f;
                                }
                            }

                            if((buf[1] == last_nonce[chainIndex]) || (buf[1] == llast_nonce[chainIndex]))
                            {
                                last_nonce_num[chainIndex]++;
                                continue;
                            }

                            if(ASIC_NUM == 1)
                            {
                                if(ASIC_CORE_NUM <= 64)
                                {
                                    which_core_nonce = (buf[1] & 0x0000003f);
                                    whose_nonce = which_core_nonce;
                                }
                                else if((ASIC_CORE_NUM <= 128) && (ASIC_CORE_NUM > 64))
                                {
                                    which_core_nonce = (buf[1] & 0x0000007f);
                                    if(which_core_nonce <= 56)
                                    {
                                        whose_nonce = which_core_nonce;
                                    }
                                    else if((which_core_nonce >= 64) && (which_core_nonce < 128))
                                    {
                                        whose_nonce = which_core_nonce - 7;
                                    }
                                }
                                else
                                {
                                    printf("%s: ASIC_CORE_NUM = %d, but it is error\n", __FUNCTION__, ASIC_CORE_NUM);
                                }
                                nonce_index = 0;
                                OpenCoreNum1 = conf.OpenCoreNum1;
                                OpenCoreNum2 = conf.OpenCoreNum2;
                                OpenCoreNum3 = conf.OpenCoreNum3;
                                OpenCoreNum4 = conf.OpenCoreNum4;

                                for(n=0; n<whose_nonce; n++)
                                {
                                    if(n < 32)
                                    {
                                        if(OpenCoreNum1 & 0x00000001)
                                        {
                                            nonce_index++;
                                            OpenCoreNum1 = OpenCoreNum1 >> 1;
                                        }
                                        else
                                        {
                                            OpenCoreNum1 = OpenCoreNum1 >> 1;
                                        }
                                    }
                                    else if((n >= 32) && (n < 64))
                                    {
                                        if(OpenCoreNum2 & 0x00000001)
                                        {
                                            nonce_index++;
                                            OpenCoreNum2 = OpenCoreNum2 >> 1;
                                        }
                                        else
                                        {
                                            OpenCoreNum2 = OpenCoreNum2 >> 1;
                                        }
                                    }
                                    else if((n >= 64) && (n < 96))
                                    {
                                        if(OpenCoreNum3 & 0x00000001)
                                        {
                                            nonce_index++;
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
                                            nonce_index++;
                                            OpenCoreNum4 = OpenCoreNum4 >> 1;
                                        }
                                        else
                                        {
                                            OpenCoreNum4 = OpenCoreNum4 >> 1;
                                        }
                                    }
                                }
                                //printf("%s: nonce_index = 0x%08x\n", __FUNCTION__, nonce_index);
                            }
                            else
                            {
                                if(CHIP_ADDR_INTERVAL != 0)
                                {
                                    which_asic_nonce = (buf[1] >> 24) / CHIP_ADDR_INTERVAL;
                                    if(which_asic_nonce >= ASIC_NUM)
                                    {
                                        continue;
                                    }
                                }
                                else
                                {
                                    which_asic_nonce = (buf[1] >> 24) / 4;
                                    if(which_asic_nonce >= asicNum)
                                    {
                                        continue;
                                    }
                                }
                                whose_nonce = which_asic_nonce;
                                nonce_index = which_asic_nonce;
                            }
                            //printf("%s: whose_nonce = 0x%08x\n", __FUNCTION__, whose_nonce);

                            llast_nonce[chainIndex] = last_nonce[chainIndex];
                            last_nonce[chainIndex] = buf[1];

							if(work_id[chainIndex]>=MAX_WORK)
								continue;
							
                            m_nonce[chainIndex] = (cgpu.works[nonce_index] + work_id[chainIndex])->nonce;
                            //printf("%s: m_nonce = 0x%08x\n", __FUNCTION__, m_nonce);

                            if(buf[1] == m_nonce[chainIndex])
                            {
                                //printf("%s: repeated_nonce_id[which_asic] = 0x%08x\n", __FUNCTION__, repeated_nonce_id[which_asic]);

                                if(work_id[chainIndex] != repeated_nonce_id[chainIndex][whose_nonce])
                                {
                                    repeated_nonce_id[chainIndex][whose_nonce] = work_id[chainIndex];
                                    asic_nonce_num[chainIndex][whose_nonce]++;
                                    valid_nonce_num[chainIndex]++;

									total_valid_nonce_num++;	// used to check and wait all nonce back...

                                    if(ASIC_NUM != 1)
                                    {
                                        if(ASIC_CORE_NUM <= 64)
                                        {
                                            which_core_nonce = (buf[1] & 0x0000003f);
                                        }
                                        else if((ASIC_CORE_NUM <= 128) && (ASIC_CORE_NUM > 64))
                                        {
                                            which_core_nonce = (buf[1] & 0x0000007f);
                                            if(which_core_nonce <= 56)
                                            {
                                                which_core_nonce = which_core_nonce;
                                            }
                                            else if((which_core_nonce >= 64) && (which_core_nonce < 128))
                                            {
                                                which_core_nonce = which_core_nonce - 7;
                                            }
                                        }
                                        else
                                        {
                                            printf("%s: ASIC_CORE_NUM = %d, but it is error\n", __FUNCTION__, ASIC_CORE_NUM);
                                        }
                                        asic_core_nonce_num[chainIndex][whose_nonce][which_core_nonce]++;
                                    }

                                }
                                else
                                {
                                    repeated_nonce_num[chainIndex]++;
                                    //printf("repeat nonce 0x%08x\n", buf[1]);
                                }
                            }
                            else
                            {
                                err_nonce_num[chainIndex]++;
                                //printf("error nonce 0x%08x\n", buf[1]);
                            }
                            //printf("\n");
                        }
                    }
                }
                else    //reg value
                {
                	pthread_mutex_lock(&reg_mutex);
                    if(reg_value_buf->reg_value_num < MAX_NONCE_NUMBER_IN_FIFO)
                    {
                        reg_value_buf->reg_buffer[reg_value_buf->p_wr].reg_value    = buf[1];
                        reg_value_buf->reg_buffer[reg_value_buf->p_wr].crc          = (buf[0] >> 24) & 0x1f;
                        reg_value_buf->reg_buffer[reg_value_buf->p_wr].chain_number = CHAIN_NUMBER(buf[0]);
	
                        reg_value_buf->p_wr++;
                        reg_value_buf->reg_value_num++;
                        if(reg_value_buf->p_wr >= MAX_NONCE_NUMBER_IN_FIFO)
                        {
                            reg_value_buf->p_wr = 0;
                        }
                    }
                    else
                    {
                        applog(LOG_DEBUG,"%s: reg_value_buf buffer is full!\n", __FUNCTION__);
                    }
					pthread_mutex_unlock(&reg_mutex);
                }
            }
        }
		else usleep(1000);
    }

	receiveExit=true;
	return 0;
}

static int get_mac(char * device, unsigned char *mac)
{
    struct ifreq ifreq;
    int sock = 0;

    sock = socket(AF_INET,SOCK_STREAM,0);
    if(sock < 0)
    {
        perror("error sock");
        return 2;
    }
    strcpy(ifreq.ifr_name,device);
    if(ioctl(sock,SIOCGIFHWADDR,&ifreq) < 0)
    {
        perror("error ioctl");
        close(sock);
        return 3;
    }
    int i = 0;
    for(i = 0; i < 6; i++)
    {
        mac[i]=(unsigned char)ifreq.ifr_hwaddr.sa_data[i];
    }

    close(sock);
    return 0;
}

void up10_vol_toPIC(int chainIndex)
{
	int i;
	unsigned char buf[128]= {0};
	char logstr[256];
	unsigned char temp_voltage = 0;
	int vol_final;
	int total_rate=GetTotalRate();
	int vol_limited;

	vol_limited=getVoltageLimitedFromHashrate(total_rate);
	
	if(chain_vol_value[chainIndex]<vol_limited)
	{
		chain_vol_value[chainIndex]=chain_vol_value[chainIndex]+10;
		chain_vol_added[chainIndex]++;

		temp_voltage = getPICvoltageFromValue(chain_vol_value[chainIndex]);
		vol_final=chain_vol_value[chainIndex];

		sprintf(logstr,"up10_vol_toPIC fix hash_voltage pic value = %d [%d] of Chain[%d], voladded=%d\n", temp_voltage, vol_final, chainIndex, chain_vol_added[chainIndex]);
		writeLogFile(logstr);
		
	    set_pic_voltage(chainIndex, temp_voltage);

 #ifdef T9_18
 		if(fpga_version>=0xE)
 		{
 			int new_T9_PLUS_chainIndex,new_T9_PLUS_chainOffset;
			getPICChainIndexOffset(chainIndex,&new_T9_PLUS_chainIndex,&new_T9_PLUS_chainOffset);

			chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+2]=chain_vol_added[chainIndex]&0x3f;
 		}
		else
		{
 			chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+2]=chain_vol_added[chainIndex]&0x3f;
		}
 #else
		chain_pic_buf[chainIndex][10]=chain_vol_added[chainIndex]&0x3f;
 #endif
	}
	else
	{
		return;
	}
}

void Force_up10_vol_toPIC(int chainIndex)
{
	int i;
	unsigned char buf[128]= {0};
	char logstr[256];
	unsigned char temp_voltage = 0;
	int vol_final;

	chain_vol_value[chainIndex]=chain_vol_value[chainIndex]+10;
	chain_vol_added[chainIndex]++;

	temp_voltage = getPICvoltageFromValue(chain_vol_value[chainIndex]);
	vol_final=chain_vol_value[chainIndex];

	sprintf(logstr,"Force_up10_vol_toPIC fix hash_voltage pic value = %d [%d] of Chain[%d], voladded=%d\n", temp_voltage, vol_final, chainIndex, chain_vol_added[chainIndex]);
	writeLogFile(logstr);
	
    set_pic_voltage(chainIndex, temp_voltage);

#ifdef T9_18
	if(fpga_version>=0xE)
	{
		int new_T9_PLUS_chainIndex,new_T9_PLUS_chainOffset;
		getPICChainIndexOffset(chainIndex,&new_T9_PLUS_chainIndex,&new_T9_PLUS_chainOffset);

		chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+2]=chain_vol_added[chainIndex]&0x3f;
	}
	else
	{
		chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+2]=chain_vol_added[chainIndex]&0x3f;
	}
#else
	chain_pic_buf[chainIndex][10]=chain_vol_added[chainIndex]&0x3f;
#endif
}


void fix_vol_toPIC(int chainIndex)
{
	int i;
	unsigned char buf[128]= {0};
	unsigned char badcore_buf[64]= {0};
	int freq_index;
	char logstr[256];
	unsigned char temp_voltage = 0;
//	unsigned char temp_offset=0;
	int vol_final;
	int base_freq;
	int total_rate=GetTotalRate();
	int voltage_limited;
//	int board_rate=GetBoardRate(chainIndex);
	bool hasChipCanDown=false;
	bool hasChipDownFreq=false;

	voltage_limited=getVoltageLimitedFromHashrate(total_rate);
	if(chain_vol_value[chainIndex]<voltage_limited)
	{
		chain_vol_value[chainIndex]=chain_vol_value[chainIndex]+10;
		chain_vol_added[chainIndex]++;

		temp_voltage = getPICvoltageFromValue(chain_vol_value[chainIndex]);
		vol_final=chain_vol_value[chainIndex];

		sprintf(logstr,"TestMode fix hash_voltage pic value = %d [%d] of Chain[%d], voladded=%d\n", temp_voltage, vol_final, chainIndex, chain_vol_added[chainIndex]);
		writeLogFile(logstr);
		
	    set_pic_voltage(chainIndex, temp_voltage);

#ifdef T9_18
		if(fpga_version>=0xE)
		{
			int new_T9_PLUS_chainIndex,new_T9_PLUS_chainOffset;
			getPICChainIndexOffset(chainIndex,&new_T9_PLUS_chainIndex,&new_T9_PLUS_chainOffset);
			chain_pic_buf[new_T9_PLUS_chainOffset][7+new_T9_PLUS_chainOffset*31+2]=chain_vol_added[chainIndex]&0x3f;
		}
		else
		{
			chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+2]=chain_vol_added[chainIndex]&0x3f;
		}
#else
		chain_pic_buf[chainIndex][10]=chain_vol_added[chainIndex]&0x3f;
#endif
	}
	else
	{
#ifdef T9_18
		if(fpga_version>=0xE)
		{
			int new_T9_PLUS_chainIndex,new_T9_PLUS_chainOffset;
			getPICChainIndexOffset(chainIndex,&new_T9_PLUS_chainIndex,&new_T9_PLUS_chainOffset);
			base_freq=chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31];
		}
		else
		{
			base_freq=chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31];
		}
#else
		base_freq=((chain_pic_buf[chainIndex][6]&0x0f)<<4)+(chain_pic_buf[chainIndex][8]&0x0f);
#endif
		sprintf(logstr,"TestMode fix all freq down one step on Chain[%d] larger than base freq=%s, voladded=%d\n", chainIndex, freq_pll_1385[base_freq].freq, chain_vol_added[chainIndex]);
		writeLogFile(logstr);

#if FINAL_TESTPATTEN_MODE == 1
		// first all failed chips must down freq or accept badcores (freq must be > base freq)
		for(freq_index = 0; freq_index < ASIC_NUM; freq_index++)
		{
			if(!last_result[chainIndex][freq_index])
			{
#ifdef T9_18
				if(fpga_version>=0xE)
				{
					int new_T9_PLUS_chainIndex,new_T9_PLUS_chainOffset; // only used by new T9+ FPGA
					getPICChainIndexOffset(chainIndex,&new_T9_PLUS_chainIndex,&new_T9_PLUS_chainOffset);
					
					if(chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]>ACCEPT_BADCORE_FREQ_INDEX) // decrease one step, if larger than base freq.
					{
						// has failed chip with freq > ACCEPT_BADCORE_FREQ_INDEX
						chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]-=1;
						hasChipCanDown=true;

						hasChipDownFreq=true;
					}
					else if(chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]>base_freq) // decrease one step, if larger than base freq.
					{
						if(!testModeHasTriedAcceptBadCore[chainIndex][freq_index])
						{
							if(tryFixSingleAsicCoreEnabledFlagByResult_testMode(chainIndex,freq_index))
							{
								// record: this chain's chip has tried accepting bad cores, next time ,do not try
								testModeHasTriedAcceptBadCore[chainIndex][freq_index]=true;	
								hasChipCanDown=true;	// set flag that we can continue. not failed.
							}
							else
							{
								testModeHasTriedAcceptBadCore[chainIndex][freq_index]=false;
								chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]-=1;
								hasChipCanDown=true;

								hasChipDownFreq=true;
							}
						}
						else
						{
							testModeHasTriedAcceptBadCore[chainIndex][freq_index]=false;
							chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]-=1;
							hasChipCanDown=true;

							hasChipDownFreq=true;
						}
					}

					// update the success freq values
					last_success_freq[chainIndex][freq_index]=chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index];	// used to write rate file
				}
				else
				{
					if(chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]>ACCEPT_BADCORE_FREQ_INDEX) // decrease one step, if larger than base freq.
					{
						// has failed chip with freq > ACCEPT_BADCORE_FREQ_INDEX
						chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]-=1;
						hasChipCanDown=true;

						hasChipDownFreq=true;
					}
					else if(chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]>base_freq) // decrease one step, if larger than base freq.
					{
						if(!testModeHasTriedAcceptBadCore[chainIndex][freq_index])
						{
							if(tryFixSingleAsicCoreEnabledFlagByResult_testMode(chainIndex,freq_index))
							{
								// record: this chain's chip has tried accepting bad cores, next time ,do not try
								testModeHasTriedAcceptBadCore[chainIndex][freq_index]=true;
								hasChipCanDown=true;	// set flag that we can continue. not failed.
							}
							else
							{
								testModeHasTriedAcceptBadCore[chainIndex][freq_index]=false;
								chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]-=1;
								hasChipCanDown=true;

								hasChipDownFreq=true;
							}
						}
						else
						{
							testModeHasTriedAcceptBadCore[chainIndex][freq_index]=false;
							chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]-=1;
							hasChipCanDown=true;

							hasChipDownFreq=true;
						}
					}

					// update the success freq values
					last_success_freq[chainIndex][freq_index]=chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index];	// used to write rate file
				}
#else
				if(chain_pic_buf[chainIndex][freq_index*2+3]>ACCEPT_BADCORE_FREQ_INDEX) // decrease one step, if larger than base freq.
				{
					chain_pic_buf[chainIndex][freq_index*2+3]-=1;
					hasChipCanDown=true;

					hasChipDownFreq=true;
				}
				else if(chain_pic_buf[chainIndex][freq_index*2+3]>base_freq)
				{
					if(!testModeHasTriedAcceptBadCore[chainIndex][freq_index])
					{
						if(tryFixSingleAsicCoreEnabledFlagByResult_testMode(chainIndex,freq_index))
						{
							// record: this chain's chip has tried accepting bad cores, next time ,do not try
							testModeHasTriedAcceptBadCore[chainIndex][freq_index]=true;
							hasChipCanDown=true;	// set flag that we can continue. not failed.
						}
						else
						{
							testModeHasTriedAcceptBadCore[chainIndex][freq_index]=false;
							chain_pic_buf[chainIndex][freq_index*2+3]-=1;
							hasChipCanDown=true;

							hasChipDownFreq=true;
						}
					}
					else
					{
						testModeHasTriedAcceptBadCore[chainIndex][freq_index]=false;
						chain_pic_buf[chainIndex][freq_index*2+3]-=1;
						hasChipCanDown=true;

						hasChipDownFreq=true;
					}
				}

				// update the success freq values
				last_success_freq[chainIndex][freq_index]=chain_pic_buf[chainIndex][freq_index*2+3];	// used to write rate file
#endif
			}
		}

		// second all failed chips must down freq(freq must be > LOWEST_FREQ_INDEX)
		if(!hasChipCanDown)
		{
			for(freq_index = 0; freq_index < ASIC_NUM; freq_index++)
			{
				if(!last_result[chainIndex][freq_index])
				{
#ifdef T9_18
					if(fpga_version>=0xE)
					{
						int new_T9_PLUS_chainIndex,new_T9_PLUS_chainOffset; // only used by new T9+ FPGA
						getPICChainIndexOffset(chainIndex,&new_T9_PLUS_chainIndex,&new_T9_PLUS_chainOffset);
						
						if(chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]>LOWEST_FREQ_INDEX) // decrease one step, if larger than base freq.
						{
							if(!testModeHasTriedAcceptBadCore[chainIndex][freq_index])
							{
								if(tryFixSingleAsicCoreEnabledFlagByResult_testMode(chainIndex,freq_index))
								{
									// record: this chain's chip has tried accepting bad cores, next time ,do not try
									testModeHasTriedAcceptBadCore[chainIndex][freq_index]=true;
									hasChipCanDown=true;	// set flag that we can continue. not failed.
								}
								else
								{
									testModeHasTriedAcceptBadCore[chainIndex][freq_index]=false;
									chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]-=1;
									hasChipCanDown=true;

									hasChipDownFreq=true;
								}
							}
							else
							{
								testModeHasTriedAcceptBadCore[chainIndex][freq_index]=false;
								chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]-=1;
								hasChipCanDown=true;

								hasChipDownFreq=true;
							}
						}

						// update the success freq values
						last_success_freq[chainIndex][freq_index]=chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index];	// used to write rate file
					}
					else
					{
						if(chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]>LOWEST_FREQ_INDEX) // decrease one step, if larger than base freq.
						{
							if(!testModeHasTriedAcceptBadCore[chainIndex][freq_index])
							{
								if(tryFixSingleAsicCoreEnabledFlagByResult_testMode(chainIndex,freq_index))
								{
									// record: this chain's chip has tried accepting bad cores, next time ,do not try
									testModeHasTriedAcceptBadCore[chainIndex][freq_index]=true;
									hasChipCanDown=true;	// set flag that we can continue. not failed.
								}
								else
								{
									testModeHasTriedAcceptBadCore[chainIndex][freq_index]=false;
									chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]-=1;
									hasChipCanDown=true;

									hasChipDownFreq=true;
								}
							}
							else
							{
								testModeHasTriedAcceptBadCore[chainIndex][freq_index]=false;
								chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]-=1;
								hasChipCanDown=true;

								hasChipDownFreq=true;
							}
						}

						// update the success freq values
						last_success_freq[chainIndex][freq_index]=chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index];	// used to write rate file
					}
#else
					if(chain_pic_buf[chainIndex][freq_index*2+3]>LOWEST_FREQ_INDEX) // decrease one step, if larger than base freq.
					{
						if(!testModeHasTriedAcceptBadCore[chainIndex][freq_index])
						{
							if(tryFixSingleAsicCoreEnabledFlagByResult_testMode(chainIndex,freq_index))
							{
								// record: this chain's chip has tried accepting bad cores, next time ,do not try
								testModeHasTriedAcceptBadCore[chainIndex][freq_index]=true;
								hasChipCanDown=true;	// set flag that we can continue. not failed.
							}
							else
							{
								testModeHasTriedAcceptBadCore[chainIndex][freq_index]=false;
								chain_pic_buf[chainIndex][freq_index*2+3]-=1;
								hasChipCanDown=true;

								hasChipDownFreq=true;
							}
						}
						else
						{
							testModeHasTriedAcceptBadCore[chainIndex][freq_index]=false;
							chain_pic_buf[chainIndex][freq_index*2+3]-=1;
							hasChipCanDown=true;

							hasChipDownFreq=true;
						}
					}

					// update the success freq values
					last_success_freq[chainIndex][freq_index]=chain_pic_buf[chainIndex][freq_index*2+3];	// used to write rate file
#endif
				}
			}
		}

		// third all chips must down freq without check result without accept badcores. (freq must be > LOWEST_FREQ_INDEX)
		if(!hasChipCanDown)
		{
			for(freq_index = 0; freq_index < ASIC_NUM; freq_index++)
			{
#ifdef T9_18
				if(fpga_version>=0xE)
				{
					int new_T9_PLUS_chainIndex,new_T9_PLUS_chainOffset; // only used by new T9+ FPGA
					getPICChainIndexOffset(chainIndex,&new_T9_PLUS_chainIndex,&new_T9_PLUS_chainOffset);
					
					if(chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]>LOWEST_FREQ_INDEX) // decrease one step, if larger than base freq.
					{
						// has failed chip with freq > LOWEST_FREQ_INDEX
						chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]-=1;
						hasChipCanDown=true;

						hasChipDownFreq=true;
					}

					// update the success freq values
					last_success_freq[chainIndex][freq_index]=chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index];	// used to write rate file
				}
				else
				{
					if(chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]>LOWEST_FREQ_INDEX) // decrease one step, if larger than base freq.
					{
						// has failed chip with freq > LOWEST_FREQ_INDEX
						chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]-=1;
						hasChipCanDown=true;

						hasChipDownFreq=true;
					}

					// update the success freq values
					last_success_freq[chainIndex][freq_index]=chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index];	// used to write rate file
				}
#else
				if(chain_pic_buf[chainIndex][freq_index*2+3]>LOWEST_FREQ_INDEX) // decrease one step, if larger than base freq.
				{
					chain_pic_buf[chainIndex][freq_index*2+3]-=1;
					hasChipCanDown=true;

					hasChipDownFreq=true;
				}

				// update the success freq values
				last_success_freq[chainIndex][freq_index]=chain_pic_buf[chainIndex][freq_index*2+3];	// used to write rate file
#endif
			}
		}

		// we check : if there is no chip down freq (only some chip accept badcores), and test process has continue for NO_CHIP_DOWN_FREQ_MAX_COUNTER times
		// avoid to loop forever, we force to down freq on chips!!!
		if(!hasChipDownFreq)
		{
			testModeChainHasNoChipDownFreqCounter[chainIndex]++;
			if(testModeChainHasNoChipDownFreqCounter[chainIndex]>=NO_CHIP_DOWN_FREQ_MAX_COUNTER)
			{
				testModeChainHasNoChipDownFreqCounter[chainIndex]=0;

				// there is NO_CHIP_DOWN_FREQ_MAX_COUNTER times, no chip down freq, maybe bad cores changes but still test failed. force to down freq on all failed chips!
				for(freq_index = 0; freq_index < ASIC_NUM; freq_index++)
				{
					if(!last_result[chainIndex][freq_index])
					{
#ifdef T9_18
						if(fpga_version>=0xE)
						{
							int new_T9_PLUS_chainIndex,new_T9_PLUS_chainOffset; // only used by new T9+ FPGA
							getPICChainIndexOffset(chainIndex,&new_T9_PLUS_chainIndex,&new_T9_PLUS_chainOffset);
							
							if(chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]>LOWEST_FREQ_INDEX) // decrease one step, if larger than base freq.
							{
								testModeHasTriedAcceptBadCore[chainIndex][freq_index]=false;
								chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]-=1;
								hasChipCanDown=true;

								hasChipDownFreq=true;
							}

							// update the success freq values
							last_success_freq[chainIndex][freq_index]=chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index];	// used to write rate file
						}
						else
						{
							if(chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]>LOWEST_FREQ_INDEX) // decrease one step, if larger than base freq.
							{
								testModeHasTriedAcceptBadCore[chainIndex][freq_index]=false;
								chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]-=1;
								hasChipCanDown=true;

								hasChipDownFreq=true;
							}

							// update the success freq values
							last_success_freq[chainIndex][freq_index]=chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index];	// used to write rate file
						}
#else
						if(chain_pic_buf[chainIndex][freq_index*2+3]>LOWEST_FREQ_INDEX) // decrease one step, if larger than base freq.
						{
							testModeHasTriedAcceptBadCore[chainIndex][freq_index]=false;
							chain_pic_buf[chainIndex][freq_index*2+3]-=1;
							hasChipCanDown=true;

							hasChipDownFreq=true;
						}

						// update the success freq values
						last_success_freq[chainIndex][freq_index]=chain_pic_buf[chainIndex][freq_index*2+3];	// used to write rate file
#endif
					}
				}

				if(!hasChipDownFreq)
				{
					hasChipCanDown=false;	// we set this to false; means must be some chip down freq, if no chip can down, then post failed code!
					
					// maybe still has no chip down freq, then we force to down freq on all chips without check result!!!
					for(freq_index = 0; freq_index < ASIC_NUM; freq_index++)
					{
#ifdef T9_18
						if(fpga_version>=0xE)
						{
							int new_T9_PLUS_chainIndex,new_T9_PLUS_chainOffset; // only used by new T9+ FPGA
							getPICChainIndexOffset(chainIndex,&new_T9_PLUS_chainIndex,&new_T9_PLUS_chainOffset);
							
							if(chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]>LOWEST_FREQ_INDEX) // decrease one step, if larger than base freq.
							{
								testModeHasTriedAcceptBadCore[chainIndex][freq_index]=false;
								chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]-=1;
								hasChipCanDown=true;

								hasChipDownFreq=true;
							}

							// update the success freq values
							last_success_freq[chainIndex][freq_index]=chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index];	// used to write rate file
						}
						else
						{
							if(chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]>LOWEST_FREQ_INDEX) // decrease one step, if larger than base freq.
							{
								testModeHasTriedAcceptBadCore[chainIndex][freq_index]=false;
								chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]-=1;
								hasChipCanDown=true;

								hasChipDownFreq=true;
							}

							// update the success freq values
							last_success_freq[chainIndex][freq_index]=chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index];	// used to write rate file
						}
#else
						if(chain_pic_buf[chainIndex][freq_index*2+3]>LOWEST_FREQ_INDEX) // decrease one step, if larger than base freq.
						{
							testModeHasTriedAcceptBadCore[chainIndex][freq_index]=false;
							chain_pic_buf[chainIndex][freq_index*2+3]-=1;
							hasChipCanDown=true;

							hasChipDownFreq=true;
						}

						// update the success freq values
						last_success_freq[chainIndex][freq_index]=chain_pic_buf[chainIndex][freq_index*2+3];	// used to write rate file
#endif
					}
				}
			}
		}
		else testModeChainHasNoChipDownFreqCounter[chainIndex]=0;
#elif  FINAL_TESTPATTEN_MODE == 2
		if(tryFixAsicCoreEnabledFlagByResult_testMode(chainIndex))
		{
			sprintf(logstr,"fix bad core num of Chain[%d]\n", chainIndex);
			writeLogFile(logstr);

			PrintAsicCoreEnabledFlag(chainIndex);
			hasChipCanDown=true;	// set flag that we can continue. not failed.
		}
		else
		{  
			for(freq_index = 0; freq_index < ASIC_NUM; freq_index++)
			{
#ifdef T9_18
				if(fpga_version>=0xE)
				{
					int new_T9_PLUS_chainIndex,new_T9_PLUS_chainOffset;	// only used by new T9+ FPGA
					getPICChainIndexOffset(chainIndex,&new_T9_PLUS_chainIndex,&new_T9_PLUS_chainOffset);
					
					if(chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]>LOWEST_FREQ_INDEX && (!last_result[chainIndex][freq_index])) // decrease one step, if larger than base freq.
					{
						chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]-=1;
						hasChipCanDown=true;
					}

					// update the success freq values
					last_success_freq[chainIndex][freq_index]=chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index];	// used to write rate file
				}
				else
				{
					if(chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]>LOWEST_FREQ_INDEX && (!last_result[chainIndex][freq_index])) // decrease one step, if larger than base freq.
					{
						chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]-=1;
						hasChipCanDown=true;
					}

					// update the success freq values
					last_success_freq[chainIndex][freq_index]=chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index];	// used to write rate file
				}
#else
				if(chain_pic_buf[chainIndex][freq_index*2+3]>LOWEST_FREQ_INDEX && (!last_result[chainIndex][freq_index])) // decrease one step, if larger than base freq.
				{
					chain_pic_buf[chainIndex][freq_index*2+3]-=1;
					hasChipCanDown=true;
				}

				// update the success freq values
				last_success_freq[chainIndex][freq_index]=chain_pic_buf[chainIndex][freq_index*2+3];	// used to write rate file
#endif
			}
		}

		if(!hasChipCanDown)
		{
			for(freq_index = 0; freq_index < ASIC_NUM; freq_index++)
			{
#ifdef T9_18
				if(fpga_version>=0xE)
				{
					int new_T9_PLUS_chainIndex,new_T9_PLUS_chainOffset;	// only used by new T9+ FPGA
					getPICChainIndexOffset(chainIndex,&new_T9_PLUS_chainIndex,&new_T9_PLUS_chainOffset);
					
					if(chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]>LOWEST_FREQ_INDEX) // decrease one step, if larger than base freq.
					{
						chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]-=1;
						hasChipCanDown=true;
					}

					// update the success freq values
					last_success_freq[chainIndex][freq_index]=chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index];	// used to write rate file
				}
				else
				{
					if(chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]>LOWEST_FREQ_INDEX) // decrease one step, if larger than base freq.
					{
						chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]-=1;
						hasChipCanDown=true;
					}

					// update the success freq values
					last_success_freq[chainIndex][freq_index]=chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index];	// used to write rate file
				}
#else
				if(chain_pic_buf[chainIndex][freq_index*2+3]>LOWEST_FREQ_INDEX) // decrease one step, if larger than base freq.
				{
					chain_pic_buf[chainIndex][freq_index*2+3]-=1;
					hasChipCanDown=true;
				}

				// update the success freq values
				last_success_freq[chainIndex][freq_index]=chain_pic_buf[chainIndex][freq_index*2+3];	// used to write rate file
#endif
			}
		}
#elif  FINAL_TESTPATTEN_MODE == 3
		if(chain_vol_value[chainIndex]<RETRY_VOLTAGE)	// if current voltage is second level search freq voltage, we will not down freq here
		{
			// for S9 , low temp will cause low hashrate, so we need force down freq lager than base freq without accept badcores at first
			for(freq_index = 0; freq_index < ASIC_NUM; freq_index++)
			{
#ifdef T9_18
				if(fpga_version>=0xE)
				{
					int new_T9_PLUS_chainIndex,new_T9_PLUS_chainOffset;	// only used by new T9+ FPGA
					getPICChainIndexOffset(chainIndex,&new_T9_PLUS_chainIndex,&new_T9_PLUS_chainOffset);
					
					if(chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]>base_freq) 
					{
						chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]-=1;
						hasChipCanDown=true;
						hasChipDownFreq=true;
					}

					// update the success freq values
					last_success_freq[chainIndex][freq_index]=chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index];	// used to write rate file
				}
				else
				{
					if(chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]>base_freq) 
					{
						chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]-=1;
						hasChipCanDown=true;
						hasChipDownFreq=true;
					}

					// update the success freq values
					last_success_freq[chainIndex][freq_index]=chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index];	// used to write rate file
				}
#else
				if(chain_pic_buf[chainIndex][freq_index*2+3]>base_freq) 
				{
					chain_pic_buf[chainIndex][freq_index*2+3]-=1;
					hasChipCanDown=true;
					hasChipDownFreq=true;
				}

				// update the success freq values
				last_success_freq[chainIndex][freq_index]=chain_pic_buf[chainIndex][freq_index*2+3];	// used to write rate file
#endif
			}
		}
		/////////////////////////////// BELOW is same as FINAL_TESTPATTEN_MODE 1 ///////////////////
		// first all failed chips must down freq or accept badcores (freq must be > base freq)
		if(!hasChipCanDown)
		{
			for(freq_index = 0; freq_index < ASIC_NUM; freq_index++)
			{
				if(!last_result[chainIndex][freq_index])
				{
#ifdef T9_18
					if(fpga_version>=0xE)
					{
						int new_T9_PLUS_chainIndex,new_T9_PLUS_chainOffset; // only used by new T9+ FPGA
						getPICChainIndexOffset(chainIndex,&new_T9_PLUS_chainIndex,&new_T9_PLUS_chainOffset);
						
						if(chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]>ACCEPT_BADCORE_FREQ_INDEX && chain_vol_value[chainIndex]<RETRY_VOLTAGE) // decrease one step, if larger than base freq.
						{
							// has failed chip with freq > ACCEPT_BADCORE_FREQ_INDEX
							chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]-=1;
							hasChipCanDown=true;

							hasChipDownFreq=true;
						}
						else if(chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]>base_freq) // decrease one step, if larger than base freq.
						{
							if(!testModeHasTriedAcceptBadCore[chainIndex][freq_index])
							{
								if(tryFixSingleAsicCoreEnabledFlagByResult_testMode(chainIndex,freq_index))
								{
									// record: this chain's chip has tried accepting bad cores, next time ,do not try
									testModeHasTriedAcceptBadCore[chainIndex][freq_index]=true;	
									hasChipCanDown=true;	// set flag that we can continue. not failed.
								}
								else
								{
									testModeHasTriedAcceptBadCore[chainIndex][freq_index]=false;
									chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]-=1;
									hasChipCanDown=true;

									hasChipDownFreq=true;
								}
							}
							else
							{
								testModeHasTriedAcceptBadCore[chainIndex][freq_index]=false;
								chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]-=1;
								hasChipCanDown=true;

								hasChipDownFreq=true;
							}
						}

						// update the success freq values
						last_success_freq[chainIndex][freq_index]=chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index];	// used to write rate file
					}
					else
					{
						if(chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]>ACCEPT_BADCORE_FREQ_INDEX && chain_vol_value[chainIndex]<RETRY_VOLTAGE) // decrease one step, if larger than base freq.
						{
							// has failed chip with freq > ACCEPT_BADCORE_FREQ_INDEX
							chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]-=1;
							hasChipCanDown=true;

							hasChipDownFreq=true;
						}
						else if(chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]>base_freq) // decrease one step, if larger than base freq.
						{
							if(!testModeHasTriedAcceptBadCore[chainIndex][freq_index])
							{
								if(tryFixSingleAsicCoreEnabledFlagByResult_testMode(chainIndex,freq_index))
								{
									// record: this chain's chip has tried accepting bad cores, next time ,do not try
									testModeHasTriedAcceptBadCore[chainIndex][freq_index]=true;
									hasChipCanDown=true;	// set flag that we can continue. not failed.
								}
								else
								{
									testModeHasTriedAcceptBadCore[chainIndex][freq_index]=false;
									chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]-=1;
									hasChipCanDown=true;

									hasChipDownFreq=true;
								}
							}
							else
							{
								testModeHasTriedAcceptBadCore[chainIndex][freq_index]=false;
								chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]-=1;
								hasChipCanDown=true;

								hasChipDownFreq=true;
							}
						}

						// update the success freq values
						last_success_freq[chainIndex][freq_index]=chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index];	// used to write rate file
					}
#else
					if(chain_pic_buf[chainIndex][freq_index*2+3]>ACCEPT_BADCORE_FREQ_INDEX && chain_vol_value[chainIndex]<RETRY_VOLTAGE) // decrease one step, if larger than base freq.
					{
						chain_pic_buf[chainIndex][freq_index*2+3]-=1;
						hasChipCanDown=true;

						hasChipDownFreq=true;
					}
					else if(chain_pic_buf[chainIndex][freq_index*2+3]>base_freq)
					{
						if(!testModeHasTriedAcceptBadCore[chainIndex][freq_index])
						{
							if(tryFixSingleAsicCoreEnabledFlagByResult_testMode(chainIndex,freq_index))
							{
								// record: this chain's chip has tried accepting bad cores, next time ,do not try
								testModeHasTriedAcceptBadCore[chainIndex][freq_index]=true;
								hasChipCanDown=true;	// set flag that we can continue. not failed.
							}
							else
							{
								testModeHasTriedAcceptBadCore[chainIndex][freq_index]=false;
								chain_pic_buf[chainIndex][freq_index*2+3]-=1;
								hasChipCanDown=true;

								hasChipDownFreq=true;
							}
						}
						else
						{
							testModeHasTriedAcceptBadCore[chainIndex][freq_index]=false;
							chain_pic_buf[chainIndex][freq_index*2+3]-=1;
							hasChipCanDown=true;

							hasChipDownFreq=true;
						}
					}

					// update the success freq values
					last_success_freq[chainIndex][freq_index]=chain_pic_buf[chainIndex][freq_index*2+3];	// used to write rate file
#endif
				}
			}
		}

		// second all failed chips must down freq(freq must be > LOWEST_FREQ_INDEX)
		if(!hasChipCanDown)
		{
			for(freq_index = 0; freq_index < ASIC_NUM; freq_index++)
			{
				if(!last_result[chainIndex][freq_index])
				{
#ifdef T9_18
					if(fpga_version>=0xE)
					{
						int new_T9_PLUS_chainIndex,new_T9_PLUS_chainOffset; // only used by new T9+ FPGA
						getPICChainIndexOffset(chainIndex,&new_T9_PLUS_chainIndex,&new_T9_PLUS_chainOffset);
						
						if(chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]>LOWEST_FREQ_INDEX) // decrease one step, if larger than base freq.
						{
							if(!testModeHasTriedAcceptBadCore[chainIndex][freq_index])
							{
								if(tryFixSingleAsicCoreEnabledFlagByResult_testMode(chainIndex,freq_index))
								{
									// record: this chain's chip has tried accepting bad cores, next time ,do not try
									testModeHasTriedAcceptBadCore[chainIndex][freq_index]=true;
									hasChipCanDown=true;	// set flag that we can continue. not failed.
								}
								else
								{
									testModeHasTriedAcceptBadCore[chainIndex][freq_index]=false;
									chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]-=1;
									hasChipCanDown=true;

									hasChipDownFreq=true;
								}
							}
							else
							{
								testModeHasTriedAcceptBadCore[chainIndex][freq_index]=false;
								chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]-=1;
								hasChipCanDown=true;

								hasChipDownFreq=true;
							}
						}

						// update the success freq values
						last_success_freq[chainIndex][freq_index]=chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index];	// used to write rate file
					}
					else
					{
						if(chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]>LOWEST_FREQ_INDEX) // decrease one step, if larger than base freq.
						{
							if(!testModeHasTriedAcceptBadCore[chainIndex][freq_index])
							{
								if(tryFixSingleAsicCoreEnabledFlagByResult_testMode(chainIndex,freq_index))
								{
									// record: this chain's chip has tried accepting bad cores, next time ,do not try
									testModeHasTriedAcceptBadCore[chainIndex][freq_index]=true;
									hasChipCanDown=true;	// set flag that we can continue. not failed.
								}
								else
								{
									testModeHasTriedAcceptBadCore[chainIndex][freq_index]=false;
									chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]-=1;
									hasChipCanDown=true;

									hasChipDownFreq=true;
								}
							}
							else
							{
								testModeHasTriedAcceptBadCore[chainIndex][freq_index]=false;
								chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]-=1;
								hasChipCanDown=true;

								hasChipDownFreq=true;
							}
						}

						// update the success freq values
						last_success_freq[chainIndex][freq_index]=chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index];	// used to write rate file
					}
#else
					if(chain_pic_buf[chainIndex][freq_index*2+3]>LOWEST_FREQ_INDEX) // decrease one step, if larger than base freq.
					{
						if(!testModeHasTriedAcceptBadCore[chainIndex][freq_index])
						{
							if(tryFixSingleAsicCoreEnabledFlagByResult_testMode(chainIndex,freq_index))
							{
								// record: this chain's chip has tried accepting bad cores, next time ,do not try
								testModeHasTriedAcceptBadCore[chainIndex][freq_index]=true;
								hasChipCanDown=true;	// set flag that we can continue. not failed.
							}
							else
							{
								testModeHasTriedAcceptBadCore[chainIndex][freq_index]=false;
								chain_pic_buf[chainIndex][freq_index*2+3]-=1;
								hasChipCanDown=true;

								hasChipDownFreq=true;
							}
						}
						else
						{
							testModeHasTriedAcceptBadCore[chainIndex][freq_index]=false;
							chain_pic_buf[chainIndex][freq_index*2+3]-=1;
							hasChipCanDown=true;

							hasChipDownFreq=true;
						}
					}

					// update the success freq values
					last_success_freq[chainIndex][freq_index]=chain_pic_buf[chainIndex][freq_index*2+3];	// used to write rate file
#endif
				}
			}
		}

		// third all chips must down freq without check result without accept badcores. (freq must be > LOWEST_FREQ_INDEX)
		if(!hasChipCanDown)
		{
			for(freq_index = 0; freq_index < ASIC_NUM; freq_index++)
			{
#ifdef T9_18
				if(fpga_version>=0xE)
				{
					int new_T9_PLUS_chainIndex,new_T9_PLUS_chainOffset; // only used by new T9+ FPGA
					getPICChainIndexOffset(chainIndex,&new_T9_PLUS_chainIndex,&new_T9_PLUS_chainOffset);
					
					if(chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]>LOWEST_FREQ_INDEX) // decrease one step, if larger than base freq.
					{
						// has failed chip with freq > LOWEST_FREQ_INDEX
						chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]-=1;
						hasChipCanDown=true;

						hasChipDownFreq=true;
					}

					// update the success freq values
					last_success_freq[chainIndex][freq_index]=chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index];	// used to write rate file
				}
				else
				{
					if(chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]>LOWEST_FREQ_INDEX) // decrease one step, if larger than base freq.
					{
						// has failed chip with freq > LOWEST_FREQ_INDEX
						chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]-=1;
						hasChipCanDown=true;

						hasChipDownFreq=true;
					}

					// update the success freq values
					last_success_freq[chainIndex][freq_index]=chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index];	// used to write rate file
				}
#else
				if(chain_pic_buf[chainIndex][freq_index*2+3]>LOWEST_FREQ_INDEX) // decrease one step, if larger than base freq.
				{
					chain_pic_buf[chainIndex][freq_index*2+3]-=1;
					hasChipCanDown=true;

					hasChipDownFreq=true;
				}

				// update the success freq values
				last_success_freq[chainIndex][freq_index]=chain_pic_buf[chainIndex][freq_index*2+3];	// used to write rate file
#endif
			}
		}

		// we check : if there is no chip down freq (only some chip accept badcores), and test process has continue for NO_CHIP_DOWN_FREQ_MAX_COUNTER times
		// avoid to loop forever, we force to down freq on chips!!!
		if(!hasChipDownFreq)
		{
			testModeChainHasNoChipDownFreqCounter[chainIndex]++;
			if(testModeChainHasNoChipDownFreqCounter[chainIndex]>=NO_CHIP_DOWN_FREQ_MAX_COUNTER)
			{
				testModeChainHasNoChipDownFreqCounter[chainIndex]=0;

				// there is NO_CHIP_DOWN_FREQ_MAX_COUNTER times, no chip down freq, maybe bad cores changes but still test failed. force to down freq on all failed chips!
				for(freq_index = 0; freq_index < ASIC_NUM; freq_index++)
				{
					if(!last_result[chainIndex][freq_index])
					{
#ifdef T9_18
						if(fpga_version>=0xE)
						{
							int new_T9_PLUS_chainIndex,new_T9_PLUS_chainOffset; // only used by new T9+ FPGA
							getPICChainIndexOffset(chainIndex,&new_T9_PLUS_chainIndex,&new_T9_PLUS_chainOffset);
							
							if(chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]>LOWEST_FREQ_INDEX) // decrease one step, if larger than base freq.
							{
								testModeHasTriedAcceptBadCore[chainIndex][freq_index]=false;
								chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]-=1;
								hasChipCanDown=true;

								hasChipDownFreq=true;
							}

							// update the success freq values
							last_success_freq[chainIndex][freq_index]=chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index];	// used to write rate file
						}
						else
						{
							if(chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]>LOWEST_FREQ_INDEX) // decrease one step, if larger than base freq.
							{
								testModeHasTriedAcceptBadCore[chainIndex][freq_index]=false;
								chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]-=1;
								hasChipCanDown=true;

								hasChipDownFreq=true;
							}

							// update the success freq values
							last_success_freq[chainIndex][freq_index]=chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index];	// used to write rate file
						}
#else
						if(chain_pic_buf[chainIndex][freq_index*2+3]>LOWEST_FREQ_INDEX) // decrease one step, if larger than base freq.
						{
							testModeHasTriedAcceptBadCore[chainIndex][freq_index]=false;
							chain_pic_buf[chainIndex][freq_index*2+3]-=1;
							hasChipCanDown=true;

							hasChipDownFreq=true;
						}

						// update the success freq values
						last_success_freq[chainIndex][freq_index]=chain_pic_buf[chainIndex][freq_index*2+3];	// used to write rate file
#endif
					}
				}

				if(!hasChipDownFreq)
				{
					hasChipCanDown=false;	// we set this to false; means must be some chip down freq, if no chip can down, then post failed code!
					
					// maybe still has no chip down freq, then we force to down freq on all chips without check result!!!
					for(freq_index = 0; freq_index < ASIC_NUM; freq_index++)
					{
#ifdef T9_18
						if(fpga_version>=0xE)
						{
							int new_T9_PLUS_chainIndex,new_T9_PLUS_chainOffset; // only used by new T9+ FPGA
							getPICChainIndexOffset(chainIndex,&new_T9_PLUS_chainIndex,&new_T9_PLUS_chainOffset);
							
							if(chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]>LOWEST_FREQ_INDEX) // decrease one step, if larger than base freq.
							{
								testModeHasTriedAcceptBadCore[chainIndex][freq_index]=false;
								chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]-=1;
								hasChipCanDown=true;

								hasChipDownFreq=true;
							}

							// update the success freq values
							last_success_freq[chainIndex][freq_index]=chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index];	// used to write rate file
						}
						else
						{
							if(chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]>LOWEST_FREQ_INDEX) // decrease one step, if larger than base freq.
							{
								testModeHasTriedAcceptBadCore[chainIndex][freq_index]=false;
								chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]-=1;
								hasChipCanDown=true;

								hasChipDownFreq=true;
							}

							// update the success freq values
							last_success_freq[chainIndex][freq_index]=chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index];	// used to write rate file
						}
#else
						if(chain_pic_buf[chainIndex][freq_index*2+3]>LOWEST_FREQ_INDEX) // decrease one step, if larger than base freq.
						{
							testModeHasTriedAcceptBadCore[chainIndex][freq_index]=false;
							chain_pic_buf[chainIndex][freq_index*2+3]-=1;
							hasChipCanDown=true;

							hasChipDownFreq=true;
						}

						// update the success freq values
						last_success_freq[chainIndex][freq_index]=chain_pic_buf[chainIndex][freq_index*2+3];	// used to write rate file
#endif
					}
				}
			}
		}
		else testModeChainHasNoChipDownFreqCounter[chainIndex]=0;
		//////////////////////////////// END OF FINAL_TESTPATTEN_MODE 1 ///////////////////////////
		
#elif  FINAL_TESTPATTEN_MODE == 0
		for(freq_index = 0; freq_index < ASIC_NUM; freq_index++)
		{
#ifdef T9_18
			if(fpga_version>=0xE)
			{
				int new_T9_PLUS_chainIndex,new_T9_PLUS_chainOffset;	// only used by new T9+ FPGA
				getPICChainIndexOffset(chainIndex,&new_T9_PLUS_chainIndex,&new_T9_PLUS_chainOffset);
				
				if(chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]>base_freq) 
				{
					chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]-=1;
					hasChipCanDown=true;
				}

				// update the success freq values
				last_success_freq[chainIndex][freq_index]=chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index];	// used to write rate file
			}
			else
			{
				if(chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]>base_freq) 
				{
					chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]-=1;
					hasChipCanDown=true;
				}

				// update the success freq values
				last_success_freq[chainIndex][freq_index]=chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index];	// used to write rate file
			}
#else
			if(chain_pic_buf[chainIndex][freq_index*2+3]>base_freq) 
			{
				chain_pic_buf[chainIndex][freq_index*2+3]-=1;
				hasChipCanDown=true;
			}

			// update the success freq values
			last_success_freq[chainIndex][freq_index]=chain_pic_buf[chainIndex][freq_index*2+3];	// used to write rate file
#endif
		}

		// down all freq, if larger than 300M
		if(!hasChipCanDown)
		{
#ifndef FORCE_8xPATTENT_TEST
			// check anyone chip's freq , if <= ACCEPT_BADCORE_FREQ_INDEX, we will enable accept bad core numbers
			freq_index=0;
#ifdef T9_18
			if(getOneChipFreqIndex(chainIndex,freq_index)<=ACCEPT_BADCORE_FREQ_INDEX)
#else
			if(chain_pic_buf[chainIndex][freq_index*2+3]<=ACCEPT_BADCORE_FREQ_INDEX)
#endif
			{
				if(tryFixAsicCoreEnabledFlagByResult_testMode(chainIndex))
				{
					sprintf(logstr,"fix bad core num of Chain[%d]\n", chainIndex);
					writeLogFile(logstr);

					PrintAsicCoreEnabledFlag(chainIndex);
					hasChipCanDown=true;	// set flag that we can continue. not failed.
				}
				else
				{  
					for(freq_index = 0; freq_index < ASIC_NUM; freq_index++)
					{
#ifdef T9_18
						if(fpga_version>=0xE)
						{
							int new_T9_PLUS_chainIndex,new_T9_PLUS_chainOffset;	// only used by new T9+ FPGA
							getPICChainIndexOffset(chainIndex,&new_T9_PLUS_chainIndex,&new_T9_PLUS_chainOffset);
							
							if(chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]>LOWEST_FREQ_INDEX) // decrease one step, if larger than base freq.
							{
								chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]-=1;
								hasChipCanDown=true;
							}

							// update the success freq values
							last_success_freq[chainIndex][freq_index]=chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index];	// used to write rate file
						}
						else
						{
							if(chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]>LOWEST_FREQ_INDEX) // decrease one step, if larger than base freq.
							{
								chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]-=1;
								hasChipCanDown=true;
							}

							// update the success freq values
							last_success_freq[chainIndex][freq_index]=chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index];	// used to write rate file
						}
#else
						if(chain_pic_buf[chainIndex][freq_index*2+3]>LOWEST_FREQ_INDEX) // decrease one step, if larger than base freq.
						{
							chain_pic_buf[chainIndex][freq_index*2+3]-=1;
							hasChipCanDown=true;
						}

						// update the success freq values
						last_success_freq[chainIndex][freq_index]=chain_pic_buf[chainIndex][freq_index*2+3];	// used to write rate file
#endif
					}
				}
			}
			else
#endif
			{
				for(freq_index = 0; freq_index < ASIC_NUM; freq_index++)
				{
#ifdef T9_18
					if(fpga_version>=0xE)
					{
						int new_T9_PLUS_chainIndex,new_T9_PLUS_chainOffset;	// only used by new T9+ FPGA
						getPICChainIndexOffset(chainIndex,&new_T9_PLUS_chainIndex,&new_T9_PLUS_chainOffset);
						
						if(chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]>LOWEST_FREQ_INDEX) // decrease one step, if larger than base freq.
						{
							chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index]-=1;
							hasChipCanDown=true;
						}

						// update the success freq values
						last_success_freq[chainIndex][freq_index]=chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index];	// used to write rate file
					}
					else
					{
						if(chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]>LOWEST_FREQ_INDEX) // decrease one step, if larger than base freq.
						{
							chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index]-=1;
							hasChipCanDown=true;
						}

						// update the success freq values
						last_success_freq[chainIndex][freq_index]=chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index];	// used to write rate file
					}
#else
					if(chain_pic_buf[chainIndex][freq_index*2+3]>LOWEST_FREQ_INDEX) // decrease one step, if larger than base freq.
					{
						chain_pic_buf[chainIndex][freq_index*2+3]-=1;
						hasChipCanDown=true;
					}

					// update the success freq values
					last_success_freq[chainIndex][freq_index]=chain_pic_buf[chainIndex][freq_index*2+3];	// used to write rate file
#endif
				}
			}
		}
#endif	//FINAL_TESTPATTEN_MODE

		// if there is no chip can down freq step, means error! because the base freq failed on test mode too
		if(!hasChipCanDown)
		{
			testModeOKCounter[chainIndex]=TEST_MODE_OK_NUM;	// set a number >= TEST_MODE_OK_NUM, means will not test this board again!
			isFailedOnTestPatten=true;

			//failed on test patten
			sprintf(logstr,"Failed on TEST PATTEN on chain[%d]\n",chainIndex);
			writeLogFile(logstr);

			sprintf(search_failed_info,"T:%d",chainIndex+1);
			saveSearchFailedFlagInfo();
			
			searchStatus=SEARCH_FAILED;
			while(1)
			{
				processTEST();
				sleep(1);
			}
		}
	}

#ifndef FORCE_8xPATTENT_TEST
	////////////// prepare bad core num //////////////////
	sprintf(logstr,"save fixed Chain[%d] bad core info:\n", chainIndex);
	writeLogFile(logstr);
#ifdef T9_18		
	for(i = 0; i <  ASIC_NUM/2; i++)
	{
		if(fpga_version>=0xE)
		{
			int new_T9_PLUS_chainIndex,new_T9_PLUS_chainOffset;	// only used by new T9+ FPGA
			getPICChainIndexOffset(chainIndex,&new_T9_PLUS_chainIndex,&new_T9_PLUS_chainOffset);

			chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+22+i]=(getChainAsicBadCoreNum(chainIndex,2*i)<<4)+getChainAsicBadCoreNum(chainIndex,2*i+1);
		}
		else
		{
			chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+22+i]=(getChainAsicBadCoreNum(chainIndex,2*i)<<4)+getChainAsicBadCoreNum(chainIndex,2*i+1);
		}

		if(getChainAsicBadCoreNum(chainIndex,2*i)>0)
		{
			sprintf(logstr,"ASIC[%d] bad core num=%d\n", 2*i, getChainAsicBadCoreNum(chainIndex,2*i));
			writeLogFile(logstr);
		}
		if(getChainAsicBadCoreNum(chainIndex,2*i+1)>0)
		{
			sprintf(logstr,"ASIC[%d] bad core num=%d\n", 2*i+1, getChainAsicBadCoreNum(chainIndex,2*i+1));
			writeLogFile(logstr);
		}
	}
#else
	for(i = 0; i < 32; i++) // 32 words (32*2 = 64 bytes, each byte need store two asic bad core num)
	{

		pic_badcore_num[chainIndex][i*2]=0;
		pic_badcore_num[chainIndex][i*2 + 1]=(getChainAsicBadCoreNum(chainIndex,2*i)<<4)+getChainAsicBadCoreNum(chainIndex,2*i+1);

		if(getChainAsicBadCoreNum(chainIndex,2*i)>0)
		{
			sprintf(logstr,"ASIC[%d] bad core num=%d\n", 2*i, getChainAsicBadCoreNum(chainIndex,2*i));
			writeLogFile(logstr);
		}
		if(getChainAsicBadCoreNum(chainIndex,2*i+1)>0)
		{
			sprintf(logstr,"ASIC[%d] bad core num=%d\n", 2*i+1, getChainAsicBadCoreNum(chainIndex,2*i+1));
			writeLogFile(logstr);
		}
	}

	pic_badcore_num[chainIndex][0] = BADCORE_MAGIC;
#endif
	sprintf(logstr,"\n");
	writeLogFile(logstr);
#endif

	usleep(5000000);
}

#ifdef T9_18
void save_fixed_FreqAndCoreNum_toPIC()
{
	int i,j;
	unsigned char buf[128]= {0};
	char logstr[256];

	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
    {
    	if(cgpu.chain_exist[i]==0)
			continue;

		if(fpga_version>=0xE)
		{
			int new_T9_PLUS_chainIndex,new_T9_PLUS_chainOffset;	// only used by new T9+ FPGA
			getPICChainIndexOffset(i,&new_T9_PLUS_chainIndex,&new_T9_PLUS_chainOffset);
			
			///////////// backup voltage //////////////////
			chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+1]=(chain_vol_value[i]/10);

			if(i==9 || i==11 || i==13)	// only write to E2PROM at the last sub chain of 3 chains!!!
			{
				// write success_freq into PIC
				do{
					// flash_pic_freq(k,chain_pic_buf[k]);
					save_freq_badcores(new_T9_PLUS_chainIndex,chain_pic_buf[new_T9_PLUS_chainIndex]);
					// read_pic_freq(k,buf);
					read_freq_badcores(new_T9_PLUS_chainIndex,buf);

					if(memcmp(buf,chain_pic_buf[new_T9_PLUS_chainIndex],128)!=0)
					{
						sprintf(logstr,"Error: flash write error! on Chain[%d]\n",new_T9_PLUS_chainIndex);
						writeLogFile(logstr);

						sprintf(logstr,"try to write again! on Chain[%d]\n",new_T9_PLUS_chainIndex);
						writeLogFile(logstr);
					}
					else
					{
						sprintf(logstr,"Chain[%d] read buf : ",new_T9_PLUS_chainIndex);
						writeLogFile(logstr);
						for(j=0;j<128;j++)
						{
							sprintf(logstr,"0x%x ",buf[j]);
							writeLogFile(logstr);
						}
						sprintf(logstr,"\n");
						writeLogFile(logstr);
						break;
					}
				}while(1);
			}
		}
		else
		{
			///////////// backup voltage //////////////////
			chain_pic_buf[((i/3)*3)][7+(i%3)*31+1]=(chain_vol_value[i]/10);

			if(i%3==2)	// only write to E2PROM at the last chain of 3 chains!!!
			{
				// write success_freq into PIC
				do{
					// flash_pic_freq(k,chain_pic_buf[k]);
					save_freq_badcores((i/3)*3,chain_pic_buf[(i/3)*3]);
					// read_pic_freq(k,buf);
					read_freq_badcores((i/3)*3,buf);

					if(memcmp(buf,chain_pic_buf[(i/3)*3],128)!=0)
					{
						sprintf(logstr,"Error: flash write error! on Chain[%d]\n",(i/3)*3);
						writeLogFile(logstr);

						sprintf(logstr,"try to write again! on Chain[%d]\n",(i/3)*3);
						writeLogFile(logstr);
					}
					else
					{
						sprintf(logstr,"Chain[%d] read buf : ",(i/3)*3);
						writeLogFile(logstr);
						for(j=0;j<128;j++)
						{
							sprintf(logstr,"0x%x ",buf[j]);
							writeLogFile(logstr);
						}
						sprintf(logstr,"\n");
						writeLogFile(logstr);
						break;
					}
				}while(1);
			}
		}
	}
}

#else
void save_fixed_FreqAndCoreNum_toPIC()
{
	int i,j;
	unsigned char buf[128]= {0};
	unsigned char badcore_buf[64]= {0};
	char logstr[256];

	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
    {
    	if(cgpu.chain_exist[i]==0)
			continue;
		
        disable_pic_dac(i);
		sprintf(logstr,"disable_pic_dac on chain[%d]\n",i);
		writeLogFile(logstr);
	}

	// must stop PIC HEART before write freq and badcore into PIC FLASH
	start_pic_heart=false;
	pthread_mutex_lock(&iic_mutex);	// hold mutex to ensure the PIC HEART thread finished on call on pic heart function
	sleep(1);
	pthread_mutex_unlock(&iic_mutex);
	
	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
    {
		if(cgpu.chain_exist[i]==0)
			continue;

		///////////// backup voltage //////////////////
		chain_pic_buf[i][36] = ((chain_vol_value[i]/10)>>4)&0xf;
		chain_pic_buf[i][38] = (chain_vol_value[i]/10)&0xf;
		chain_pic_buf[i][40] = 0x23;	//magic number for backup voltage
	
		// write success_freq into PIC
		do{
			reset_iic_pic(i);
		    usleep(500*1000);

			erase_pic_badcore_num(i);
			flash_pic_badcore_num(i,pic_badcore_num[i]);
			read_pic_badcore_num(i,badcore_buf);
			
			erase_pic_freq(i);
			flash_pic_freq(i,chain_pic_buf[i]);
			read_pic_freq(i,buf);

			if(memcmp(buf,chain_pic_buf[i],128)!=0
				|| memcmp(badcore_buf,pic_badcore_num[i],64)!=0	)
			{
				sprintf(logstr,"Error: flash write error! on Chain[%d]\n",i);
				writeLogFile(logstr);

				sprintf(logstr,"try to write again! on Chain[%d]\n",i);
				writeLogFile(logstr);
			}
			else
			{
				sprintf(logstr,"Chain[%d] read buf : ",i);
				writeLogFile(logstr);
				for(j=0;j<128;j++)
				{
					sprintf(logstr,"0x%x ",buf[j]);
					writeLogFile(logstr);
				}
				sprintf(logstr,"\n");
				writeLogFile(logstr);

				sprintf(logstr,"Chain[%d] read badcore_buf : ",i);
				writeLogFile(logstr);
				for(j=0;j<64;j++)
				{
					sprintf(logstr,"0x%x ",badcore_buf[j]);
					writeLogFile(logstr);
				}
				sprintf(logstr,"\n");
				writeLogFile(logstr);
				break;
			}
		}while(1);

		jump_to_app_from_loader(i);
	}
}

void test_PIC_flash_rw()
{
	int i,j;
	unsigned char buf[128]= {0};
	unsigned char badcore_buf[64]= {0};
	char logstr[256];
	int counter=0;

	// must stop PIC HEART before write freq and badcore into PIC FLASH
	start_pic_heart=false;
	pthread_mutex_lock(&iic_mutex);	// hold mutex to ensure the PIC HEART thread finished on call on pic heart function
	sleep(1);
	pthread_mutex_unlock(&iic_mutex);
	
	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
    {
		if(cgpu.chain_exist[i]==0)
			continue;

		///////////// fill in test data //////////////////
		memset(chain_pic_buf[i],0x13,128);
		memset(pic_badcore_num[i],0x13,64);

		// write success_freq into PIC
		do{
			reset_iic_pic(i);
		    usleep(500*1000);

			// do set FLASH address for 100 times
			for(j=0;j<100;j++)
				set_pic_iic_flash_addr_pointer(i, PIC_FLASH_POINTER_BADCORE_START_ADDRESS_H, PIC_FLASH_POINTER_BADCORE_START_ADDRESS_L);
			
			erase_pic_badcore_num(i);
			flash_pic_badcore_num(i,pic_badcore_num[i]);
			read_pic_badcore_num(i,badcore_buf);

			// do set FLASH address for 100 times
			for(j=0;j<100;j++)
				set_pic_iic_flash_addr_pointer(i, PIC_FLASH_POINTER_FREQ_START_ADDRESS_H, PIC_FLASH_POINTER_FREQ_START_ADDRESS_L);
			
			erase_pic_freq(i);
			flash_pic_freq(i,chain_pic_buf[i]);
			read_pic_freq(i,buf);

			if(memcmp(buf,chain_pic_buf[i],128)!=0
				|| memcmp(badcore_buf,pic_badcore_num[i],64)!=0	)
			{
				sprintf(logstr,"Error: flash write error! on Chain[%d]\n",i);
				writeLogFile(logstr);

				sprintf(logstr,"Chain[%d] read buf : ",i);
				writeLogFile(logstr);
				for(j=0;j<128;j++)
				{
					sprintf(logstr,"0x%x ",buf[j]);
					writeLogFile(logstr);
				}
				sprintf(logstr,"\n");
				writeLogFile(logstr);

				sprintf(logstr,"Chain[%d] read badcore_buf : ",i);
				writeLogFile(logstr);
				for(j=0;j<64;j++)
				{
					sprintf(logstr,"0x%x ",badcore_buf[j]);
					writeLogFile(logstr);
				}
				sprintf(logstr,"\n");
				writeLogFile(logstr);
				
				sprintf(logstr,"try to write again! on Chain[%d]\n",i);
				writeLogFile(logstr);

#ifdef DEBUG_TEST_PIC_FLASH_RW
				while(1)sleep(1);	// we need wait here forever!!! for DEBUG
#endif
			}
			else
			{
				sprintf(logstr,"test_PIC_flash_rw OK! wait for 1 seconds and test again\n");
				writeLogFile(logstr);
				sleep(1);

				counter++;

				if(counter>10)
					break;
			}
		}while(1);
	}
}

#endif

void save_freq_toPIC(int chainIndex, int *freq_ok)
{
	int i;
	unsigned char buf[128]= {0};
	unsigned char badcore_buf[64]= {0};
	int freq_index;
	char logstr[256];
	unsigned char temp_voltage = 0;
	unsigned char temp_offset=0;
	int vol_final;
	unsigned char minerMAC[6];

	get_mac("eth0",minerMAC);

#ifdef R4
	temp_voltage = getPICvoltageFromValue(chain_vol_value[chainIndex]);
	vol_final=chain_vol_value[chainIndex];

	sprintf(logstr,"save hash_voltage pic value = %d [%d] of Chain[%d], voladded=%d\n", temp_voltage, vol_final, chainIndex, chain_vol_added[chainIndex]);
	writeLogFile(logstr);
#else
	chain_vol_added[chainIndex]=0;

	temp_voltage = getPICvoltageFromValue(chain_vol_value[chainIndex]);
	vol_final=chain_vol_value[chainIndex];
	
	sprintf(logstr,"special fix hash_voltage pic value = %d [%d] of Chain[%d], voladded=%d\n", temp_voltage, vol_final, chainIndex, chain_vol_added[chainIndex]);
	writeLogFile(logstr);
#endif

    set_pic_voltage(chainIndex, temp_voltage);
	
	// write success_freq into PIC
	printf("\nwrite freq index into pic\n");

#ifdef T9_18
	if(fpga_version>=0xE)
	{
		int new_T9_PLUS_chainIndex,new_T9_PLUS_chainOffset;	// only used by new T9+ FPGA
		getPICChainIndexOffset(chainIndex,&new_T9_PLUS_chainIndex,&new_T9_PLUS_chainOffset);
		
		chain_pic_buf[new_T9_PLUS_chainIndex][0] = FREQ_MAGIC;

		sprintf(logstr,"\nWrite freq to PIC : \n");
		writeLogFile(logstr);
		for(freq_index = 0; freq_index < ASIC_NUM; freq_index++)
		{
			chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index] = freq_ok[freq_index] & 0xff;
			sprintf(logstr,"Freq[%02d]=%s\t", freq_index, freq_pll_1385[freq_ok[freq_index]].freq);
			writeLogFile(logstr);

			if((freq_index%7)==0)
			{
				sprintf(logstr,"\n");
				writeLogFile(logstr);
			}
		}
		sprintf(logstr,"\n");
		writeLogFile(logstr);

		// save base freq of this hashboard
		chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31] = base_freq_index[chainIndex];

		// chain_pic_buf[chainIndex][10] is the vol added value
		chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+2]=chain_vol_added[chainIndex]&0x3f;

		// save MAC into PIC 
		chain_pic_buf[new_T9_PLUS_chainIndex][1] = minerMAC[0];
		chain_pic_buf[new_T9_PLUS_chainIndex][2] = minerMAC[1];
		chain_pic_buf[new_T9_PLUS_chainIndex][3] = minerMAC[2];
		chain_pic_buf[new_T9_PLUS_chainIndex][4] = minerMAC[3];
		chain_pic_buf[new_T9_PLUS_chainIndex][5] = minerMAC[4];
		chain_pic_buf[new_T9_PLUS_chainIndex][6] = minerMAC[5];
		////////////// end of MAC //////////////
		
		////////////// prepare bad core num //////////////////
		sprintf(logstr,"Chain[%d] bad core info:\n", chainIndex);
		writeLogFile(logstr);
				
		for(i = 0; i < ASIC_NUM/2; i++)	// 9 bytes for 18 asics
		{
			chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+22+i]=(getChainAsicBadCoreNum(chainIndex,2*i)<<4)+getChainAsicBadCoreNum(chainIndex,2*i+1);

			if(getChainAsicBadCoreNum(chainIndex,2*i)>0)
			{
				sprintf(logstr,"ASIC[%d] bad core num=%d\n", 2*i, getChainAsicBadCoreNum(chainIndex,2*i));
				writeLogFile(logstr);
			}
			
			if(getChainAsicBadCoreNum(chainIndex,2*i+1)>0)
			{
				sprintf(logstr,"ASIC[%d] bad core num=%d\n", 2*i+1, getChainAsicBadCoreNum(chainIndex,2*i+1));
				writeLogFile(logstr);
			}
		}

		sprintf(logstr,"\n");
		writeLogFile(logstr);
	}
	else
	{
		chain_pic_buf[((chainIndex/3)*3)][0] = FREQ_MAGIC;

		sprintf(logstr,"\nWrite freq to PIC : \n");
		writeLogFile(logstr);
		for(freq_index = 0; freq_index < ASIC_NUM; freq_index++)
		{
			chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+4+freq_index] = freq_ok[freq_index] & 0xff;
			sprintf(logstr,"Freq[%02d]=%s\t", freq_index, freq_pll_1385[freq_ok[freq_index]].freq);
			writeLogFile(logstr);

			if((freq_index%7)==0)
			{
				sprintf(logstr,"\n");
				writeLogFile(logstr);
			}
		}
		sprintf(logstr,"\n");
		writeLogFile(logstr);

		// save base freq of this hashboard
		chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31] = base_freq_index[chainIndex];

		// chain_pic_buf[chainIndex][10] is the vol added value
		chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+2]=chain_vol_added[chainIndex]&0x3f;

		// save MAC into PIC 
		chain_pic_buf[((chainIndex/3)*3)][1] = minerMAC[0];
		chain_pic_buf[((chainIndex/3)*3)][2] = minerMAC[1];
		chain_pic_buf[((chainIndex/3)*3)][3] = minerMAC[2];
		chain_pic_buf[((chainIndex/3)*3)][4] = minerMAC[3];
		chain_pic_buf[((chainIndex/3)*3)][5] = minerMAC[4];
		chain_pic_buf[((chainIndex/3)*3)][6] = minerMAC[5];
		////////////// end of MAC //////////////
		
		////////////// prepare bad core num //////////////////
		sprintf(logstr,"Chain[%d] bad core info:\n", chainIndex);
		writeLogFile(logstr);
				
		for(i = 0; i < ASIC_NUM/2; i++)	// 9 bytes for 18 asics
		{
			chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31+22+i]=(getChainAsicBadCoreNum(chainIndex,2*i)<<4)+getChainAsicBadCoreNum(chainIndex,2*i+1);

			if(getChainAsicBadCoreNum(chainIndex,2*i)>0)
			{
				sprintf(logstr,"ASIC[%d] bad core num=%d\n", 2*i, getChainAsicBadCoreNum(chainIndex,2*i));
				writeLogFile(logstr);
			}
			
			if(getChainAsicBadCoreNum(chainIndex,2*i+1)>0)
			{
				sprintf(logstr,"ASIC[%d] bad core num=%d\n", 2*i+1, getChainAsicBadCoreNum(chainIndex,2*i+1));
				writeLogFile(logstr);
			}
		}

		sprintf(logstr,"\n");
		writeLogFile(logstr);
	}
#else
	chain_pic_buf[chainIndex][0] = Conf.freq_gap&0x3f;
	chain_pic_buf[chainIndex][1] = FREQ_MAGIC;

	sprintf(logstr,"\nWrite freq to PIC : \n");
	writeLogFile(logstr);
	for(freq_index = 0; freq_index < ASIC_NUM; freq_index++)
	{
		chain_pic_buf[chainIndex][(freq_index+1)*2] = 0x0;
		chain_pic_buf[chainIndex][(freq_index+1)*2 + 1] = freq_ok[freq_index] & 0xff;

		sprintf(logstr,"Freq[%02d]=%s\t", freq_index, freq_pll_1385[freq_ok[freq_index]].freq);
		writeLogFile(logstr);

		if((freq_index%7)==0)
		{
			sprintf(logstr,"\n");
			writeLogFile(logstr);
		}
	}
	sprintf(logstr,"\n");
	writeLogFile(logstr);

	temp_offset=(unsigned char)chip_temp_offset[chainIndex];	// temp offset is not used now, set 0
	chain_pic_buf[chainIndex][2] = (temp_offset>>4)&0xf;
	chain_pic_buf[chainIndex][4] = (temp_offset)&0xf;

	// save base freq of this hashboard
	chain_pic_buf[chainIndex][6] = (base_freq_index[chainIndex]>>4)&0xf;
	chain_pic_buf[chainIndex][8] = (base_freq_index[chainIndex])&0xf;

	// chain_pic_buf[chainIndex][10] is the vol added value
	chain_pic_buf[chainIndex][10]=chain_vol_added[chainIndex]&0x3f;

	// save MAC into PIC 
	chain_pic_buf[chainIndex][12] = (minerMAC[0]>>4)&0xf;
	chain_pic_buf[chainIndex][14] = (minerMAC[0])&0xf;

	chain_pic_buf[chainIndex][16] = (minerMAC[1]>>4)&0xf;
	chain_pic_buf[chainIndex][18] = (minerMAC[1])&0xf;

	chain_pic_buf[chainIndex][20] = (minerMAC[2]>>4)&0xf;
	chain_pic_buf[chainIndex][22] = (minerMAC[2])&0xf;

	chain_pic_buf[chainIndex][24] = (minerMAC[3]>>4)&0xf;
	chain_pic_buf[chainIndex][26] = (minerMAC[3])&0xf;

	chain_pic_buf[chainIndex][28] = (minerMAC[4]>>4)&0xf;
	chain_pic_buf[chainIndex][30] = (minerMAC[4])&0xf;

	chain_pic_buf[chainIndex][32] = (minerMAC[5]>>4)&0xf;
	chain_pic_buf[chainIndex][34] = (minerMAC[5])&0xf;
	////////////// end of MAC //////////////
	
	////////////// prepare bad core num //////////////////
	sprintf(logstr,"Chain[%d] bad core info:\n", chainIndex);
	writeLogFile(logstr);
			
	for(i = 0; i < 32; i++)	// 32 words (32*2 = 64 bytes, each byte need store two asic bad core num)
	{
		pic_badcore_num[chainIndex][i*2]=0;
		pic_badcore_num[chainIndex][i*2 + 1]=(getChainAsicBadCoreNum(chainIndex,2*i)<<4)+getChainAsicBadCoreNum(chainIndex,2*i+1);

		if(getChainAsicBadCoreNum(chainIndex,2*i)>0)
		{
			sprintf(logstr,"ASIC[%d] bad core num=%d\n", 2*i, getChainAsicBadCoreNum(chainIndex,2*i));
			writeLogFile(logstr);
		}
		if(getChainAsicBadCoreNum(chainIndex,2*i+1)>0)
		{
			sprintf(logstr,"ASIC[%d] bad core num=%d\n", 2*i+1, getChainAsicBadCoreNum(chainIndex,2*i+1));
			writeLogFile(logstr);
		}
	}
	pic_badcore_num[chainIndex][0] = BADCORE_MAGIC;

	sprintf(logstr,"\n");
	writeLogFile(logstr);
#endif
}

int GetBandValue(int minFreq)
{
	int ret;
	uint32_t rBaudrate;
	int temp_corenum = calculate_core_number(ASIC_CORE_NUM);
	conf.timeout = 0x1000000/temp_corenum*CHIP_ADDR_INTERVAL/minFreq*95/100;
	rBaudrate = 1000000 * 5/3 / conf.timeout * (64*8);//64*8 need send bit, ratio=2/3
	ret = 25000000/rBaudrate/8 - 1;
	return ret;
}

static int get_pll_index(int freq)
{
	
	int i;
	char freq_str[10];
	sprintf(freq_str,"%d", freq);

	for(i=0; i < sizeof(freq_pll_1385)/sizeof(freq_pll_1385[0]); i++)
	{
		if( memcmp(freq_pll_1385[i].freq, freq_str, sizeof(freq_pll_1385[i].freq)) == 0)
			break;
	}


	if(i == sizeof(freq_pll_1385)/sizeof(freq_pll_1385[0]))
	{
		i = -1;
	}

	return i;

}

static int GetDefaultFreq()
{
	FILE *fd;
	char confData[2048];
	char *pStr,*pEnd;
	char freqStr[32];
	
	fd=fopen("/etc/bmminer.conf.factory","rb");
	if(fd)
	{
		memset(confData,'\0',sizeof(confData));
		fread(confData,1,sizeof(confData),fd);
		fclose(fd);

		pStr=strstr(confData,"bitmain-freq");
		if(pStr)
		{
			pStr=strstr(pStr,":");
			if(pStr)
			{
				pStr=strstr(pStr,"\"");
				pStr++;
				pEnd=strstr(pStr,"\"");

				memcpy(freqStr,pStr,pEnd-pStr);
				freqStr[pEnd-pStr]='\0';

				return atoi(freqStr);
			}
		}
	}

	return 550;	//550M as error default
}

#if 0
static void DownOneChipFreqOneStep()
{
	int i,j;
	uint8_t voltage_array[BITMAIN_MAX_CHAIN_NUM] = {0};
	uint8_t chain_voltage[BITMAIN_MAX_CHAIN_NUM] = {0};
	char logstr[256];
	uint8_t tmp_vol;
	int board_rate=0;
	int max_freq=0,max_freq_chipIndex=0,max_rate_chainIndex=0;
	
	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
	{
		if(cgpu.chain_exist[i] == 1)
			chain_voltage[i]=getPICvoltageFromValue(chain_vol_value[i]);
		else chain_voltage[i]=0;
	}

	memcpy(voltage_array,chain_voltage,sizeof(chain_voltage));

	// desc order for voltage value
	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
    {
        for(j=i+1; j<BITMAIN_MAX_CHAIN_NUM; j++)
        {
        	if(voltage_array[i]>voltage_array[j])
        	{
        		tmp_vol=voltage_array[i];
				voltage_array[i]=voltage_array[j];
				voltage_array[j]=tmp_vol;
        	}
        }
	}

	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
	{
		if(voltage_array[i]>0)
		{
			board_rate=0;
			max_rate_chainIndex=-1;
			//find the highest rate board with this voltage
			for(j=0; j<BITMAIN_MAX_CHAIN_NUM; j++)
			{
				if(cgpu.chain_exist[j] == 1 && chain_voltage[j]==voltage_array[i])
				{
					if(board_rate==0 || board_rate<GetBoardRate(j))
					{
						board_rate=GetBoardRate(j);
						max_rate_chainIndex=j;
					}
				}
			}
			if(max_rate_chainIndex<0)
				continue;
			
			max_freq=0;
			max_freq_chipIndex=-1;
			for(j = 0; j < ASIC_NUM; j ++)
			{
				if(max_freq==0 || max_freq<last_success_freq[max_rate_chainIndex][j])
				{
					max_freq_chipIndex=j;
					max_freq=last_success_freq[max_rate_chainIndex][j];
				}
			}
			if(max_freq_chipIndex<0)
				continue;

			//down one step on highest chip, but freq must be > 250M
			if(max_freq<=LOWEST_FREQ_INDEX)
				continue;
			else
			{
				//down one step
				last_success_freq[max_rate_chainIndex][max_freq_chipIndex]-=1;	// down one step
				// update PIC buffer
				chain_pic_buf[max_rate_chainIndex][max_freq_chipIndex*2+3]=last_success_freq[max_rate_chainIndex][max_freq_chipIndex];
				return;
			}
		}
	}
}
#else
static void DownOneChipFreqOneStep()
{
	int j;
	uint8_t chain_voltage[BITMAIN_MAX_CHAIN_NUM] = {0};
	char logstr[256];
	uint8_t tmp_vol;
	int board_rate=0;
	int max_freq=0,max_freq_chipIndex=0,max_rate_chainIndex=0;

	board_rate=0;
	max_rate_chainIndex=-1;
	//find the highest rate board with this voltage
	for(j=0; j<BITMAIN_MAX_CHAIN_NUM; j++)
	{
		if(cgpu.chain_exist[j] == 1)
		{
			if(board_rate==0 || board_rate<GetBoardRate(j))
			{
				board_rate=GetBoardRate(j);
				max_rate_chainIndex=j;
			}
		}
	}
	if(max_rate_chainIndex<0)
	{
		sprintf(logstr,"Fatal Error: DownOneChipFreqOneStep has Wrong chain index=%d\n",max_rate_chainIndex);
		writeLogFile(logstr);
		while(1)sleep(1);
	}
	
	max_freq=0;
	max_freq_chipIndex=-1;
	for(j = 0; j < ASIC_NUM; j ++)
	{
		if(max_freq==0 || max_freq<last_success_freq[max_rate_chainIndex][j])
		{
			max_freq_chipIndex=j;
			max_freq=last_success_freq[max_rate_chainIndex][j];
		}
	}
	if(max_freq_chipIndex<0)
	{
		sprintf(logstr,"Fatal Error: DownOneChipFreqOneStep Chain[%d] has Wrong chip index=%d\n",max_freq_chipIndex);
		writeLogFile(logstr);
		while(1)sleep(1);
	}

	//down one step on highest chip, but freq must be > 250M
	if(max_freq<=LOWEST_FREQ_INDEX)
	{
		sprintf(logstr,"Fatal Error: DownOneChipFreqOneStep Chain[%d] has no chip can down freq!!!\n",max_rate_chainIndex);
		writeLogFile(logstr);
		while(1)sleep(1);
	}
	else
	{
		//down one step
		last_success_freq[max_rate_chainIndex][max_freq_chipIndex]-=1;	// down one step
		// update PIC buffer
#ifdef T9_18
		if(fpga_version>=0xE)
		{
			int new_T9_PLUS_chainIndex,new_T9_PLUS_chainOffset;	// only used by new T9+ FPGA
			getPICChainIndexOffset(max_rate_chainIndex,&new_T9_PLUS_chainIndex,&new_T9_PLUS_chainOffset);
			
			chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+max_freq_chipIndex+4]=last_success_freq[max_rate_chainIndex][max_freq_chipIndex];
		}
		else
		{
			chain_pic_buf[((max_rate_chainIndex/3)*3)][7+(max_rate_chainIndex%3)*31+max_freq_chipIndex+4]=last_success_freq[max_rate_chainIndex][max_freq_chipIndex];
		}
#else
		chain_pic_buf[max_rate_chainIndex][max_freq_chipIndex*2+3]=last_success_freq[max_rate_chainIndex][max_freq_chipIndex];
#endif
		return;
	}
}

#endif
static void ProcessFixFreqForChips(int CHECK_VOLTAGE_RATE)
{
	int i,j;
	static int last_record_freq[BITMAIN_MAX_CHAIN_NUM][256];
	
	if(GetTotalRate()>=CHECK_VOLTAGE_RATE)
	{
		//record the current freq 
		for(i = 0; i < BITMAIN_MAX_CHAIN_NUM; i++)
		{
			if(cgpu.chain_exist[i] == 1)
			{
				for(j = 0; j < ASIC_NUM; j ++)
				{
					last_record_freq[i][j]=last_success_freq[i][j];
				}
			}
		}
		
		do{
			//do:  down one step
			DownOneChipFreqOneStep();
		}while(GetTotalRate()>=CHECK_VOLTAGE_RATE);
	}
}

static void editVoltageforBoard()	// if can up 0.1V, then we up 0.1V, if hashrate is 14T , and hashboard 
{
    int i;
	char logstr[256];

	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
    {
    	if(cgpu.chain_exist[i]==0)
			continue;

		sprintf(logstr,"try to add more 0.1V voltage on chain[%d]\n",i);
		writeLogFile(logstr);

        up10_vol_toPIC(i);	// by board hashrate limit
	}
}

static void forceAddVoltageforBoard()	// if can up 0.1V, then we up 0.1V, if hashrate is 14T , and hashboard 
{
    int i;
	char logstr[256];

	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
    {
    	if(cgpu.chain_exist[i]==0)
			continue;

		sprintf(logstr,"Force to add more 0.1V voltage on chain[%d]\n",i);
		writeLogFile(logstr);

        Force_up10_vol_toPIC(i);	// by board hashrate limit
	}
}

void opencore_onebyone_onChain(int chainIndex)
{
	int i;
	for(i=0;i<ASIC_NUM;i++)
	{
		open_core_onChain(chainIndex,ASIC_CORE_NUM,i+1,true);
		usleep(100000);
	}
}

#ifdef USE_PREINIT_OPENCORE
void getAsicNum_preOpenCore(int chainIndex)
{
	char logstr[256];
	int i,j;
	int each_asic_freq;
	int freq_test=PRE_OPENCORE_FREQ;
	int freq_value=atoi(freq_pll_1385[freq_test].freq);
	unsigned char vol_pic;
	
	for(j=0;j<2;j++)
	{
		for(i=0;i<6;i++)
		{
			software_set_address_onChain(chainIndex);

			for(each_asic_freq = 0; each_asic_freq < ASIC_NUM; each_asic_freq ++)
		    {
		        set_frequency_with_addr_plldatai(freq_test,0, each_asic_freq * CHIP_ADDR_INTERVAL,chainIndex);
		    }

			if(ASIC_NUM != 1)
		    {
			    int temp_corenum = calculate_core_number(ASIC_CORE_NUM);
		    	conf.timeout = 0x1000000/temp_corenum*CHIP_ADDR_INTERVAL/freq_value*TIMEOUT_PERCENT/100;	// 7% timeout
		        set_time_out_control(conf.timeout);
		    }

			//open core
			open_core_onChain(chainIndex,ASIC_CORE_NUM,i+1,true);	// open 1 cores

			sprintf(logstr,"PreOpenCore: open %d cores Done!\n",i+1);
			writeLogFile(logstr);

			cgpu.chain_asic_num[chainIndex]=0;
			check_asic_reg_oneChain(chainIndex,CHIP_ADDRESS);
			
			sprintf(logstr,"PreOpenCore: chain[%d] get asicNum = %d\n",chainIndex, cgpu.chain_asic_num[chainIndex]);
			writeLogFile(logstr);

			if(cgpu.chain_asic_num[chainIndex]==ASIC_NUM)
				return;
		}
		
#ifdef USE_NEW_RESET_FPGA
		set_reset_hashboard(chainIndex,1);
		sleep(RESET_KEEP_TIME);
		set_reset_hashboard(chainIndex,0);
		sleep(1);
#else
		reset_one_hashboard(chainIndex);
#endif
	}
}
#endif


#ifdef PATTEN_TEST_ONE_BY_ONE
static void doHeatBoard(bool need_fix, int test_ChainIndex)
#else
static void doHeatBoard(bool need_fix)
#endif
{
    int i,j, freq_index = 0;
	unsigned char vol_pic;
    unsigned char buf[128]= {0};
	char logstr[256];
#ifdef NEED_PRE_HEAT
	int need_heat=0;
#endif
	int wait_count=0;
	int min_Freq;
	int max_Freq;
	int last_send_num;
	int last_recv_num;
	int vol_value;
	
    start_pic_heart=false;
	pthread_mutex_lock(&iic_mutex);
    // init fpga
    printf("clement2 init_fpga\n");
	reset_fpga();

	pthread_mutex_unlock(&iic_mutex);
	send_pic_heart_once();
	start_pic_heart=true;

    // reset global arg
    reset_global_arg();
    start_receive = true;

#ifdef ENABLE_TEMP_PROCESS
	gIsReadTemp = false;
#endif

	gStartTest=false;

#ifdef USE_NEW_RESET_FPGA
	set_reset_allhashboard(1);
	sleep(RESET_KEEP_TIME);
	set_reset_allhashboard(0);
	sleep(1);
#else
    reset_hashboard();
#endif

	reset_crc_count();
	sprintf(logstr,"CRC error counter=%d\n",get_crc_count());
	writeLogFile(logstr);
	
#ifdef PATTEN_TEST_ONE_BY_ONE
	for(i=0;i<BITMAIN_MAX_CHAIN_NUM;i++)
	{
		if(test_ChainIndex==i)
			testDone[i]=false;
		else testDone[i]=true;
	}
#endif

    //set CommandMode
    printf("\n--- set command mode\n");
    if(conf.CommandMode)    // fil mode
    {
        // after reset fpga, it should be fil mode as default
        cgpu.CommandMode = 1;
        printf("set command mode to FIL\n");
    }
    else                    // vil mode
    {
    	printf("clement2 set_dhash_acc_control vil\n");
        set_dhash_acc_control((get_dhash_acc_control() & (~OPERATION_MODE)) | VIL_MODE | (VIL_MIDSTATE_NUMBER(1) & (~NEW_BLOCK) & (~RUN_BIT)));
        cgpu.CommandMode = 0;
        sprintf(logstr,"set command mode to VIL\n");
		writeLogFile(logstr);
    }

#ifdef USE_PREINIT_OPENCORE
	set_baud(DEFAULT_BAUD_VALUE);
	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
	{
		if(cgpu.chain_exist[i] == 1 
#ifndef ALL_CHAIN_INTEST
			&& (!testDone[i])
#endif
			)
		{
			getAsicNum_preOpenCore(i);
			
			sprintf(logstr,"check chain[%d]: asicNum = %d\n",i, cgpu.chain_asic_num[i]);
			writeLogFile(logstr);

			if(cgpu.chain_asic_num[i]!=ASIC_NUM)
			{
				isNoBoardError=true;	// we want to exit, and watchdog will run again.
				
				sprintf(logstr,"The AsicNum=%d on chain[%d]\n",cgpu.chain_asic_num[i],i);
				writeLogFile(logstr);

				testDone[i]=true;	// set test over flag!
				search_over[i]=true;
				searchFreqMode[i]=SEARCH_OVER;	// will not search freq on this chain!!!

				search_freq_result[i]=false;	// no board, we do not try search again!

				// just stop ,we do not want to search ....
				if(isChipNumOK_Once)
					sprintf(search_failed_info,"J%d:3",i+1);
				else sprintf(search_failed_info,"J%d:2",i+1);
				
				saveSearchFailedFlagInfo();
				searchStatus=SEARCH_FAILED;
				while(1)
				{
					processTEST();
					sleep(1);
				}
			}
		}
	}
#else
    //check asic number
    if(Conf.CheckChain)
    {
        sprintf(logstr,"\n--- check asic number\n");
		writeLogFile(logstr);

		printf("clement2 check_asic_reg vil\n");
		check_asic_reg(CHIP_ADDRESS);
		
        for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
        {
            if(cgpu.chain_exist[i] == 1 
#ifndef ALL_CHAIN_INTEST
				&& (!testDone[i])
#endif
				)
            {
            	int noboard_retry_count=0;
                sprintf(logstr,"check chain[%d]: asicNum = %d\n",i, cgpu.chain_asic_num[i]);
				writeLogFile(logstr);

#ifdef DEBUG_STOP_WHEN_ASICNUM_NOTENOUGH
				while(cgpu.chain_asic_num[i] != ASIC_NUM)
				{
					int each_asic_freq;
					int freq_test=8;	//300M

					for(j=0; j<256; j++)
		    		{
		    	    	last_result[i][j] = 0 ;
						last_opencore_result[i][j] = 0;
					}

					chain_DataCount[i]=TESTMODE_PATTEN_NUM_8X;	// when seaching base freq, we use 8*144 patten on chip
					chain_ValidNonce[i]=TESTMODE_NONCE_NUM_8X;
					chain_PassCount[i]=TESTMODE_PATTEN_NUM_8X;
					
					sprintf(logstr,"start software set address on Chain[%d]...\n",i);
					writeLogFile(logstr);

					software_set_address_onChain(i);

					sprintf(logstr,"Done: software set address on Chain[%d], wait for 30s...\n",i);
					writeLogFile(logstr);
					sleep(30);

					sprintf(logstr,"start set freq=300M for %d times...\n",noboard_retry_count+1);
					writeLogFile(logstr);

					for(each_asic_freq = 0; each_asic_freq < ASIC_NUM; each_asic_freq ++)
		            {
		                set_frequency_with_addr_plldatai(freq_test,0, each_asic_freq * CHIP_ADDR_INTERVAL,i);
		            }

					min_Freq=300;

					sprintf(logstr,"Done: set freq=300M on Chain[%d], wait for 30s...\n",i);
					writeLogFile(logstr);
					sleep(30);
/*
					conf.baud=BAUD_LEVEL;
					sprintf(logstr,"start: set_baud=%d, ought to be %d\n",conf.baud,GetBandValue(min_Freq));
					writeLogFile(logstr);
					set_baud(conf.baud);

					sprintf(logstr,"Done: set_baud=%d, wait for 30s...\n",conf.baud);
					writeLogFile(logstr);
					sleep(30);
*/
					sprintf(logstr,"start: set_time_out...\n");
					writeLogFile(logstr);
					
					if(ASIC_NUM != 1)
				    {
					    int temp_corenum = calculate_core_number(ASIC_CORE_NUM);
				    	conf.timeout = 0x1000000/temp_corenum*CHIP_ADDR_INTERVAL/min_Freq*TIMEOUT_PERCENT/100;	// 7% timeout

						sprintf(logstr,"The min freq=%d\n",min_Freq);
						writeLogFile(logstr);
						
						sprintf(logstr,"set real timeout %d, need sleep=%d\n", conf.timeout, conf.timeout*(conf.ValidNonce1+conf.passCount1));
						writeLogFile(logstr);
						
				        set_time_out_control(conf.timeout);
				    }
					sprintf(logstr,"Done: set_time_out, wait for 30s...\n");
					writeLogFile(logstr);
					sleep(30);

					sprintf(logstr,"start: open core on Chain[%d]...\n",i);
					writeLogFile(logstr);
					//open core
					open_core_onChain(i,ASIC_CORE_NUM,noboard_retry_count+1,true);	// open 2,3,4 cores

					sprintf(logstr,"Done: open core, wait for 30s...\n");
					writeLogFile(logstr);
					sleep(30);
					
				    if(ASIC_NUM != 1)
				    {
					    int temp_corenum = calculate_core_number(ASIC_CORE_NUM);
				    	conf.timeout = 0x1000000/temp_corenum*CHIP_ADDR_INTERVAL/min_Freq*TIMEOUT_PERCENT/100;	// 7% timeout

						sprintf(logstr,"The min freq=%d\n",min_Freq);
						writeLogFile(logstr);
						
						sprintf(logstr,"set real timeout %d, need sleep=%d\n", conf.timeout, conf.timeout*(conf.ValidNonce1+conf.passCount1));
						writeLogFile(logstr);
						
				        set_time_out_control(conf.timeout);
				    }

					sprintf(logstr,"TEST: open %d cores Done!\n",noboard_retry_count+1);
					writeLogFile(logstr);
					
				//	sprintf(logstr,"TEST: wait for 10s...\n");
				//	writeLogFile(logstr);
				//	sleep(10);

					cgpu.chain_asic_num[i]=0;
					check_asic_reg_oneChain(i,CHIP_ADDRESS);
					
					sprintf(logstr,"TEST: retry check chain[%d]: asicNum = %d, wait for 3mins to test voltage\n",i, cgpu.chain_asic_num[i]);
					writeLogFile(logstr);
					sleep(180);

					reset_nonce_arg();
					reset_work_data();

					// before the first time for sending work, reset the FPGA's nonce fifo
				    if(!gBegin_get_nonce)
				    {
				        //printf("\n--- clear nonce fifo before send work\n");
				        printf("clement2 set_nonce_fifo_interrupt\n");
				        set_nonce_fifo_interrupt(get_nonce_fifo_interrupt() | FLUSH_NONCE3_FIFO);
				        gBegin_get_nonce = true;
				    }

					sprintf(logstr,"start send works on chain[%d]\n",i);
					writeLogFile(logstr);

					StartSendFlag[i]=true;

#ifdef USE_SINGLE_SEND_THREAD
					send_func_all();
#else
					sprintf(logstr,"wait send works done on chain[%d]\n",i);
					writeLogFile(logstr);

					last_send_num=0;
					wait_count=0;
					while(wait_count<SEND_WAIT_TIMEOUT && send_work_num[i]<chain_ValidNonce[i])
					{
						if(last_send_num!=send_work_num[i])
						{
							wait_count=0;
							last_send_num=send_work_num[i];
						}
						else wait_count++;

						usleep(100000);
					}
					if(wait_count>=SEND_WAIT_TIMEOUT)
					{
						sprintf(logstr,"Some Error to cause sending works timeout on chain[%d] sendExit=%d, %d/%d\n",i,sendExit[i],send_work_num[i],chain_ValidNonce[i]);
						writeLogFile(logstr);
					}
#endif
					StartSendFlag[i]=false;

					sprintf(logstr,"wait recv nonce on chain[%d]\n",i);
					writeLogFile(logstr);

					last_recv_num=0;
					wait_count=0;
					while(wait_count < RECV_WAIT_TIMEOUT && valid_nonce_num[i]<chain_ValidNonce[i])
					{
						if(last_recv_num!=valid_nonce_num[i])
						{
							wait_count=0;
							last_recv_num=valid_nonce_num[i];
						}
						else wait_count++;

						usleep(100000);
					}

					gBegin_get_nonce=false;
					
					sprintf(logstr,"get nonces on chain[%d]\n",i);
					writeLogFile(logstr);

				    get_result(i, chain_PassCount[i], chain_ValidNonce[i]);
					print_OpenCoreDetails(i,chain_PassCount[i],noboard_retry_count+1);
					
					if(last_opencore_pass(i,noboard_retry_count+1))
					{
						sprintf(logstr,"OPEN CORE on chain[%d]: OK! \n",i);
						writeLogFile(logstr);

					//	break;	// we jump out
						cgpu.chain_asic_num[i]=0;	// force to try again!
					}
					else
					{
						sprintf(logstr,"OPEN CORE on chain[%d]: FAILED!\n",i);
						writeLogFile(logstr);

						cgpu.chain_asic_num[i]=0;	// force to try again!
					}

				//	sprintf(logstr,"TEST: wait for test voltage, wait for 3mins...\n");
				//	writeLogFile(logstr);
				//	sleep(180);

					noboard_retry_count++;
					if(noboard_retry_count>=3)
					{
						testDone[i]=true;	// set test over flag! only this time, next time will retry again...
						sprintf(logstr,"Error: The AsicNum=%d on chain[%d]!\n",cgpu.chain_asic_num[i],i);
						writeLogFile(logstr);

						isFailedOnTestPatten=true;	// set failed flag!
						search_freq_result[i]=false;

						if(isChipNumOK_Once)
							sprintf(search_failed_info,"J%d:3",i+1);
						else sprintf(search_failed_info,"J%d:2",i+1);
						
						saveSearchFailedFlagInfo();
						searchStatus=SEARCH_FAILED;
						while(1)
						{
							processTEST();
							sleep(1);
						}
					}
				}
#endif

				while(cgpu.chain_asic_num[i] != ASIC_NUM)
                {
                	noboard_retry_count++;
					if(noboard_retry_count>NOBOARD_RETRY_COUNT)
					{
	                	testDone[i]=true;	// set test over flag! only this time, next time will retry again...
						sprintf(logstr,"Error: The AsicNum=%d on chain[%d]!\n",cgpu.chain_asic_num[i],i);
						writeLogFile(logstr);

						isFailedOnTestPatten=true;	// set failed flag!
						search_freq_result[i]=false;

						if(isChipNumOK_Once)
							sprintf(search_failed_info,"J%d:3",i+1);
						else sprintf(search_failed_info,"J%d:2",i+1);
						
						saveSearchFailedFlagInfo();
						searchStatus=SEARCH_FAILED;
						while(1)
						{
							processTEST();
							sleep(1);
						}
					}
					else
					{
#ifdef DEBUG_STOP_WHEN_ASICNUM_NOTENOUGH
#if 0 //def DEBUG_STOP_WHEN_ASICNUM_NOTENOUGH
						sprintf(logstr,"Will wait for test voltage, for 3 mins...\n");
						writeLogFile(logstr);
						sleep(180);
#endif

						if(((noboard_retry_count-1)%3)==0 && noboard_retry_count>1)
						{
							sprintf(logstr,"Chain[%d] will retry getAsicNum %d times!, close DC and open DC and reset\n",i,NOBOARD_RETRY_COUNT-noboard_retry_count+1);
							writeLogFile(logstr);

#ifdef USE_NEW_RESET_FPGA
							set_reset_hashboard(i,1);
#endif

							disable_pic_dac(i);
							sleep(1);
							enable_pic_dac(i);
							sleep(1);

#ifdef USE_NEW_RESET_FPGA
							set_reset_hashboard(i,0);
#endif

							sprintf(logstr,"DC closed and opened again! wait for test voltage, for 30s...\n");
							writeLogFile(logstr);
							sleep(30);
						}
						else
						{
							sprintf(logstr,"Chain[%d] will retry getAsicNum %d times!, NOT close DC, only reset\n",i,NOBOARD_RETRY_COUNT-noboard_retry_count+1);
							writeLogFile(logstr);
						}

#ifdef USE_NEW_RESET_FPGA
						set_reset_hashboard(i,1);
						sleep(RESET_KEEP_TIME);
						set_reset_hashboard(i,0);
						sleep(1);
#else
						reset_one_hashboard(i);
#endif

						sprintf(logstr,"Chain[%d] reset over! wait for test voltage, for 30s...\n",i);
						writeLogFile(logstr);
						sleep(30);
							
						cgpu.chain_asic_num[i]=0;
						check_asic_reg_oneChain(i,CHIP_ADDRESS);
						
						sprintf(logstr,"retry check chain[%d]: asicNum = %d\n",i, cgpu.chain_asic_num[i]);
						writeLogFile(logstr);
						
						sprintf(logstr,"After Retry to get asic number,  wait for test voltage, for 30s...\n");
						writeLogFile(logstr);
						sleep(30);
#else
#ifdef USE_NEW_RESET_FPGA
						set_reset_hashboard(i,1);
#endif

						disable_pic_dac(i);
						sleep(1);
						enable_pic_dac(i);
						sleep(1);

#ifdef USE_NEW_RESET_FPGA
						set_reset_hashboard(i,1);
						sleep(RESET_KEEP_TIME);
						set_reset_hashboard(i,0);
						sleep(1);
#else
						reset_one_hashboard(i);
#endif						
						cgpu.chain_asic_num[i]=0;
						check_asic_reg_oneChain(i,CHIP_ADDRESS);
						
						sprintf(logstr,"retry check chain[%d]: asicNum = %d\n",i, cgpu.chain_asic_num[i]);
						writeLogFile(logstr);
#endif
					}
                }
            }
        }
    }
#endif

#ifdef DEBUG_STOP_WHEN_ASICNUM_NOTENOUGH
	// reset nonce arg  again, because maybe it will test in get asicnum
    reset_nonce_arg();
#endif

	printf("clement software_set_address\n");
	software_set_address();

	//set baud
	//conf.baud=GetBandValue(min_Freq);
	conf.baud=BAUD_LEVEL;
	sprintf(logstr,"set_baud=%d\n",conf.baud);
	writeLogFile(logstr);
	set_baud(conf.baud);

#ifdef FORCE_8xPATTENT_TEST
	// need do temp offset test once !!!! when in FORCE_8xPATTENT_TEST mode
	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
	{
		if(cgpu.chain_exist[i]==0 
#ifndef ALL_CHAIN_INTEST
			|| testDone[i]
#endif
			)
			continue;

#ifdef T9_18
		if(fpga_version>=0xE)
		{
			if(i!=8 && i!=10 && i!=12)	// only 8,10,12 has temp chip!!!
				continue;
		}
		else
		{
			if(i%3!=1)	// only 1,4,7 has temp chip!!!
				continue;
		}
#endif

		if(chip_temp_offset[i]==-123)
		{
			for(j=0;j<16;j++)
			{
				set_default_temperature_offset_value(i);	// set default
				get_temperature_offset_value_from_asic(i);	// get offset value again inside of this function
			}
		}
		
		set_default_temperature_offset_value(i);	// set new offset value into chip
		get_temperature_offset_value_from_asic(i);	// get offset value again inside of this function
		
		sprintf(logstr,"first time set temp offset of chain[%d] : %d\n",i,chip_temp_offset[i]); // print new offset value
		writeLogFile(logstr);
	}
#else
	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
	{
		if(cgpu.chain_exist[i]==0 
#ifndef ALL_CHAIN_INTEST
			|| testDone[i]
#endif
			)
			continue;

#ifdef T9_18
		if(fpga_version>=0xE)
		{
			if(i!=8 && i!=10 && i!=12)	// only 8,10,12 has temp chip!!!
				continue;
		}
		else
		{
			if(i%3!=1)	// only 1,4,7 has temp chip!!!
				continue;
		}
#endif

		if(chip_temp_offset[i]==-123)
		{
			for(j=0;j<16;j++)
			{
				set_default_temperature_offset_value(i);	// set default
				get_temperature_offset_value_from_asic(i);	// get offset value again inside of this function
			}
		}
		
		sprintf(logstr,"set temp offset of chain[%d] : %d\n",i,chip_temp_offset[i]);	// print new offset value
		writeLogFile(logstr);
		
		set_default_temperature_offset_value(i);	// set new offset value into chip
		get_temperature_offset_value_from_asic(i);	// get offset value again inside of this function
	}
#endif

#if ((defined USE_LOWFREQ_OPENCORE) && (defined USE_PREINIT_OPENCORE))
#ifdef ENABLE_HIGH_VOLTAGE_OPENCORE
	// set highest voltage to open core
	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
	{
		unsigned char vol_pic;
		if(cgpu.chain_exist[i]==0 
#ifndef ALL_CHAIN_INTEST
			|| testDone[i]
#endif
			)
			continue;

		vol_pic=getPICvoltageFromValue(HIGHEST_VOLTAGE_LIMITED_HW);
		
#ifdef T9_18
		set_voltage_T9_18_into_PIC(i, vol_pic);
#else
		set_pic_voltage(i, vol_pic);
#endif
		sleep(1);

#ifdef USE_OPENCORE_ONEBYONE
		opencore_onebyone_onChain(i);
#else
		open_core_onChain(i,ASIC_CORE_NUM,ASIC_CORE_NUM,true);
#endif

#ifdef USE_OPENCORE_TWICE
		open_core_onChain(i,ASIC_CORE_NUM,ASIC_CORE_NUM,true);
#endif

		vol_pic=getPICvoltageFromValue(chain_vol_value[i]);

#ifdef T9_18
		if(fpga_version>=0xE)
		{
			if(i==9 
#ifndef ALL_CHAIN_INTEST
				|| (i==1 && testDone[8] && testDone[9]) || (i==8 && testDone[9])
#endif
				)
				set_pic_voltage(i, vol_pic);
			else if(i==11 
#ifndef ALL_CHAIN_INTEST
				|| (i==2 && testDone[10] && testDone[11]) || (i==10 && testDone[11])
#endif
				)
				set_pic_voltage(i, vol_pic);
			else if(i==13 
#ifndef ALL_CHAIN_INTEST
				|| (i==3 && testDone[12] && testDone[13]) || (i==12 && testDone[13])
#endif
				)
				set_pic_voltage(i, vol_pic);
		}
		else
		{
			if(i%3==2 
#ifndef ALL_CHAIN_INTEST
				|| (i%3==0 && testDone[(i/3)*3+1] && testDone[(i/3)*3+2]) || (i%3==1 && testDone[(i/3)*3+2])
#endif
				)
				set_pic_voltage(i, vol_pic);
		}
#else
		set_pic_voltage(i, vol_pic);
#endif
	}
#else
	//open core
	open_core(true);
#endif
#endif

	min_Freq=700;
	max_Freq=300;
	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
    {
    	int each_asic_freq = 0;

		if(cgpu.chain_exist[i]==0 
#ifndef ALL_CHAIN_INTEST
			|| testDone[i]
#endif
			)
			continue;

		if(last_freq[i][0]>0)	// last_freq[i][0] = 0 means use conf freq
		{
			sprintf(logstr,"Set Freq of PIC for Test Patten on Chain[%d]\n",i);
			writeLogFile(logstr);
	
			//set freq from pic freq records
			for(each_asic_freq = 0; each_asic_freq < ASIC_NUM; each_asic_freq ++)
        	{
        	    set_frequency_with_addr_plldatai(last_freq[i][each_asic_freq],0, each_asic_freq * CHIP_ADDR_INTERVAL,i);

				if(min_Freq>atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq))
					min_Freq=atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq);

				if(max_Freq<atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq))
					max_Freq=atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq);
        	}
		}
		else
		{
			sprintf(logstr,"Error: Set Freq=%d of config for Test Patten on Chain[%d]\n",conf.freq,i);
			writeLogFile(logstr);

			//set freq from config file as start
			printf("clement2 set_frequency=%d\n",conf.freq);
			set_frequency(i, conf.freq);

			if(min_Freq>conf.freq)
				min_Freq=conf.freq;

			if(max_Freq<conf.freq)
				max_Freq=conf.freq;
		}
	}

	if(first_freq)
    {
        first_freq = false;
#ifdef NEED_PRE_HEAT
		need_heat=1;	// first time to run
#endif
    }

#if 1
	if(ASIC_NUM != 1)
    {
	    int temp_corenum = calculate_core_number(ASIC_CORE_NUM);
    	conf.timeout = 0x1000000/temp_corenum*CHIP_ADDR_INTERVAL/min_Freq*TIMEOUT_PERCENT/100;	// 7% timeout

		sprintf(logstr,"The min freq=%d\n",min_Freq);
		writeLogFile(logstr);
		
		sprintf(logstr,"set real timeout %d, need sleep=%d\n", conf.timeout, conf.timeout*(conf.ValidNonce1+conf.passCount1));
		writeLogFile(logstr);
		
        set_time_out_control(conf.timeout);
    }
#else
    //set timeout for open core
    printf("\n--- set timeout %d\n", conf.OpenCoreGap);
    set_time_out_control(conf.OpenCoreGap);
#endif

#ifdef ENABLE_HIGH_VOLTAGE_OPENCORE
	// set highest voltage to open core
	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
	{
		unsigned char vol_pic;
		if(cgpu.chain_exist[i]==0 
#ifndef ALL_CHAIN_INTEST
			|| testDone[i]
#endif
			)
			continue;

		vol_pic=getPICvoltageFromValue(HIGHEST_VOLTAGE_LIMITED_HW);
#ifdef T9_18
		sprintf(logstr,"set voltage=%d[%d] on chain[%d] to open core...\n",HIGHEST_VOLTAGE_LIMITED_HW,vol_pic,i);
		writeLogFile(logstr);
		
		set_voltage_T9_18_into_PIC(i, vol_pic);
#else
		set_pic_voltage(i, vol_pic);
#endif

		sleep(1);

#ifdef USE_OPENCORE_ONEBYONE
		opencore_onebyone_onChain(i);
#else
		open_core_onChain(i,ASIC_CORE_NUM,ASIC_CORE_NUM,true);
#endif

#ifdef USE_OPENCORE_TWICE
		open_core_onChain(i,ASIC_CORE_NUM,ASIC_CORE_NUM,true);
#endif

		vol_pic=getPICvoltageFromValue(chain_vol_value[i]);

#ifdef T9_18
		if(fpga_version>=0xE)
		{
			if(i==9 
#ifndef ALL_CHAIN_INTEST
				|| (i==1 && testDone[8] && testDone[9]) || (i==8 && testDone[9])
#endif
				)
			{
				sprintf(logstr,"set working voltage=%d[%d] on chain[%d]\n",chain_vol_value[i],vol_pic,i);
				writeLogFile(logstr);
		
				set_pic_voltage(i, vol_pic);
			}
			else if(i==11 
#ifndef ALL_CHAIN_INTEST
				|| (i==2 && testDone[10] && testDone[11]) || (i==10 && testDone[11])
#endif
				)
			{
				sprintf(logstr,"set working voltage=%d[%d] on chain[%d]\n",chain_vol_value[i],vol_pic,i);
				writeLogFile(logstr);
			
				set_pic_voltage(i, vol_pic);
			}
			else if(i==13 
#ifndef ALL_CHAIN_INTEST
				|| (i==3 && testDone[12] && testDone[13]) || (i==12 && testDone[13])
#endif
				)
			{
				sprintf(logstr,"set working voltage=%d[%d] on chain[%d]\n",chain_vol_value[i],vol_pic,i);
				writeLogFile(logstr);
			
				set_pic_voltage(i, vol_pic);
			}
		}
		else
		{
			if(i%3==2 
#ifndef ALL_CHAIN_INTEST
				|| (i%3==0 && testDone[(i/3)*3+1] && testDone[(i/3)*3+2]) || (i%3==1 && testDone[(i/3)*3+2])
#endif
				)
				set_pic_voltage(i, vol_pic);
		}
#else
		set_pic_voltage(i, vol_pic);
#endif
	}
#else
	//open core
	open_core(true);
#endif

#ifdef SET_TICKETMASK_BEFORE_TEST
//	sprintf(logstr,"DEBUG before test: check TICKET_MASK:\n");
//	writeLogFile(logstr);
//	check_asic_reg(TICKET_MASK);
	sleep(5);
	set_tickmask(0);
	set_hcnt(0);
	sleep(5);
#endif

	for(i=0; i < BITMAIN_MAX_CHAIN_NUM; i++)
	{
		if(cgpu.chain_exist[i]==0 
#ifndef ALL_CHAIN_INTEST
			|| testDone[i]
#endif
			)
			continue;
		
		board_temp[i]=0;
	}
	
#ifdef ENABLE_TEMP_PROCESS
	gIsReadTemp = true;
#endif

#if 0
    if(ASIC_NUM != 1)
    {
	    int temp_corenum = calculate_core_number(ASIC_CORE_NUM);
    	conf.timeout = 0x1000000/temp_corenum*CHIP_ADDR_INTERVAL/min_Freq*TIMEOUT_PERCENT/100;	// 7% timeout

		sprintf(logstr,"The min freq=%d\n",min_Freq);
		writeLogFile(logstr);
		
		sprintf(logstr,"set real timeout %d, need sleep=%d\n", conf.timeout, conf.timeout*(conf.ValidNonce1+conf.passCount1));
		writeLogFile(logstr);
		
        set_time_out_control(conf.timeout);
    }
#endif

    reset_work_data();
	
#ifdef NEED_PRE_HEAT
	if(need_heat)
	{
		set_PWM(10);
		sprintf(logstr,"wait for becoming heat...\n");
		writeLogFile(logstr);
		while(!gStartTest)
			usleep(1000000);
		sprintf(logstr,"becoming heat Over!\n");
		writeLogFile(logstr);
		set_PWM(80);
	}
#endif

	// before the first time for sending work, reset the FPGA's nonce fifo
    if(!gBegin_get_nonce)
    {
        //printf("\n--- clear nonce fifo before send work\n");
        printf("clement2 set_nonce_fifo_interrupt\n");
        set_nonce_fifo_interrupt(get_nonce_fifo_interrupt() | FLUSH_NONCE3_FIFO);
        gBegin_get_nonce = true;
    }

    for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
    {
    	if(cgpu.chain_exist[i]==0 || testDone[i])
			continue;

		sprintf(logstr,"start send works on chain[%d]\n",i);
		writeLogFile(logstr);

		StartSendFlag[i]=true;
    }

#ifdef USE_SINGLE_SEND_THREAD
	send_func_all();
#else
	for(i=0; i < BITMAIN_MAX_CHAIN_NUM; i++)
    {
    	if(cgpu.chain_exist[i]==0 || testDone[i])
			continue;

		sprintf(logstr,"wait send works done on chain[%d]\n",i);
		writeLogFile(logstr);

		last_send_num=0;
		wait_count=0;
		while(wait_count<SEND_WAIT_TIMEOUT && send_work_num[i]<chain_ValidNonce[i])
		{
			if(last_send_num!=send_work_num[i])
			{
				wait_count=0;
				last_send_num=send_work_num[i];
			}
			else wait_count++;

			usleep(100000);
		}
		if(wait_count>=SEND_WAIT_TIMEOUT)
		{
			sprintf(logstr,"Some Error to cause sending works timeout on chain[%d] sendExit=%d, %d/%d\n",i,sendExit[i],send_work_num[i],chain_ValidNonce[i]);
			writeLogFile(logstr);
		}

		StartSendFlag[i]=false;
    }
#endif

	for(i=0; i < BITMAIN_MAX_CHAIN_NUM; i++)
    {
    	if(cgpu.chain_exist[i]==0 || testDone[i])
			continue;

		sprintf(logstr,"wait recv nonce on chain[%d]\n",i);
		writeLogFile(logstr);

		last_recv_num=0;
		wait_count=0;
		while(wait_count < RECV_WAIT_TIMEOUT && valid_nonce_num[i]<chain_ValidNonce[i])
		{
			if(last_recv_num!=valid_nonce_num[i])
			{
				wait_count=0;
				last_recv_num=valid_nonce_num[i];
			}
			else wait_count++;

			usleep(100000);
		}
	}

#ifdef CHECK_TICKETMASK_AFTER_TEST
	sprintf(logstr,"DEBUG after test: check TICKET_MASK:\n");
	writeLogFile(logstr);
	check_asic_reg(TICKET_MASK);
	check_asic_reg(HASH_COUNTING_NUMBER);
#endif

#ifdef ENABLE_TEMP_PROCESS
	gIsReadTemp = false;
	pthread_mutex_lock(&read_temp_mutex);
	sprintf(logstr,"read temp thread stop!\n");
	writeLogFile(logstr);
	pthread_mutex_unlock(&read_temp_mutex);

	set_fan_by_temp();
#else
	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
	{
		if(cgpu.chain_exist[i]==0 || testDone[i])
			continue;

#ifdef T9_18
		if(fpga_version>=0xE)
		{
			if(i!=8 && i!=10 && i!=12)	// only 8,10,12 chain has temp chips
				continue;
		}
		else
		{
			if(i%3!=1)	// only 1,4,7 chain has temp chips
				continue;
		}
#endif
    	board_temp[i]=read_asic_temperature(i);
	}

#ifdef T9_18
	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
	{
		if(cgpu.chain_exist[i]==0 || testDone[i])
			continue;

		if(fpga_version>=0xE)
		{
			// copy chain[8] temp to chain[1] and chain[9]
			switch(i)
			{
			case 1:
			case 9:
				board_temp[i]=board_temp[8];
				break;
			case 2:
			case 11:
				board_temp[i]=board_temp[10];
				break;
			case 3:
			case 13:
				board_temp[i]=board_temp[12];
				break;
			}
		}
		else
		{
			// copy chain[1] temp to chain[0] and chain[2]
			if(i%3!=1)
				board_temp[i]=board_temp[((i/3)*3)+1];
		}
	}
#endif

#endif

	gBegin_get_nonce=false;
	start_receive=false;
	
	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
	{
		if(cgpu.chain_exist[i]==0 || testDone[i])
			continue;

		sprintf(logstr,"get nonces on chain[%d]\n",i);
		writeLogFile(logstr);

    	result = get_result(i, chain_PassCount[i], chain_ValidNonce[i]);
	}

	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
	{
		if(cgpu.chain_exist[i]==0 || testDone[i])
			continue;

		if(last_all_pass(i))
		{
			// if test mode OK, then we clear testModeChainHasNoChipDownFreqCounter to 0
			testModeChainHasNoChipDownFreqCounter[i]=0;
			// if test mode OK, then we clear testModeHasTriedAcceptBadCore to false, can accept bad cores on next time!
			for(j=0; j<256; j++)
				testModeHasTriedAcceptBadCore[i][j]=false;
			
			if(testModeOKCounter[i]>=TEST_MODE_OK_NUM)
				sprintf(logstr,"Test Patten on chain[%d]: OK! board_temp=%d Done!\n",i,board_temp[i]);
			else sprintf(logstr,"Test Patten on chain[%d]: OK! board_temp=%d testing...\n",i,board_temp[i]);
			writeLogFile(logstr);

#ifdef T9_18
			if(fpga_version>=0xE)
			{
				int new_T9_PLUS_chainIndex,new_T9_PLUS_chainOffset;	// only used by new T9+ FPGA
				getPICChainIndexOffset(i,&new_T9_PLUS_chainIndex,&new_T9_PLUS_chainOffset);
				
				chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+3]=board_temp[i];
			}
			else
			{
				chain_pic_buf[((i/3)*3)][7+(i%3)*31+3]=board_temp[i];
			}
#else
			chain_pic_buf[i][42] = (0x0f&(((unsigned char)board_temp[i])>>4));
			chain_pic_buf[i][44] = (0x0f&((unsigned char)board_temp[i]));
			chain_pic_buf[i][46] = 0x23;	//magic number for board temp
#endif
		}
		else
		{
			if(testModeOKCounter[i]>=TEST_MODE_OK_NUM)
				sprintf(logstr,"Test Patten on chain[%d]: FAILED! board_temp=%d Done!\n",i,board_temp[i]);
			else sprintf(logstr,"Test Patten on chain[%d]: FAILED! board_temp=%d testing...\n",i,board_temp[i]);
			writeLogFile(logstr);

			search_freq_result[i]=false;

			if(need_fix && testModeOKCounter[i]<TEST_MODE_OK_NUM)
				fix_vol_toPIC(i);
		}

		testDone[i]=true;
	}

	sprintf(logstr,"CRC error counter=%d\n",get_crc_count());
	writeLogFile(logstr);
	
#ifndef DEBUG_NOT_CHECK_FAN_NUM
	if(conf.force_freq)
	{
		if(!check_fan())
		{
			sprintf(search_failed_info,"F:1");
			
			saveSearchFailedFlagInfo();
			searchStatus=SEARCH_FAILED;
			while(1)
			{
				processTEST();
				sleep(1);
			}
		}
	}
#endif
}

bool isBoardFreqTooLow(unsigned char *pic_buf)
{
	int freq_index;
	for(freq_index = 0; freq_index < ASIC_NUM; freq_index++)
	{
		if(pic_buf[freq_index*2+3]>LOWEST_FREQ_INDEX)
			return false;
	}
	return true;
}

#ifdef CLEMENT_DEBUG_T9_18_IIC
void ClementTest()
{
	int i,j;
	unsigned char buf[128];
	start_pic_heart=false;

	reset_fpga();

	// check chain
	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
	{
		cgpu.chain_exist[i] = 0;
	}
	
	printf("clement check_chain\n");
	check_chain();

	dsPIC33EP16GS202_reset_pic(0);
	sleep(1);
	
	dsPIC33EP16GS202_jump_to_app_from_loader(0);
	sleep(1);

	start_pic_heart=true;
	set_Voltage_S9_plus_plus_BM1387_54(0,20);

	printf("get_pic_voltage = %d \n",get_pic_voltage(0));
	
	while(1)
	{
		sleep(10);

		memset(chain_pic_buf[0],0x66,128);
		save_freq_badcores(0,chain_pic_buf[0]);
		// read_pic_freq(k,buf);
		read_freq_badcores(0,buf);

		if(memcmp(buf,chain_pic_buf[0],128)!=0)
		{
			printf("Error: flash write error! on Chain[%d]\n",0);

			printf("Chain[%d] read buf : ",0);
			for(j=0;j<128;j++)
			{
				printf("0x%x ",buf[j]);
			}
			printf("\n");
		}
		else
		{
			printf("Chain[%d] read buf : ",(i/3)*3);
			for(j=0;j<128;j++)
			{
				printf("0x%x ",buf[j]);
			}
			printf("\n");

			while(1)sleep(5);
		}
	}
}
#endif

void detectFPGAversion()
{
	char logstr[256];
	unsigned int hw_version;

	reset_fpga();
	
	hw_version=get_Hardware_version();
	sprintf(logstr,"DETECT HW version=%08x\n",hw_version);
	writeLogFile(logstr);

	fpga_version = hw_version & 0x000000ff;
}

void PreparePICandVoltage()
{
	int i, j, freq_index = 0;
	unsigned char vol_pic;
#ifndef T9_18
    unsigned char buf[128]= {0};
	unsigned char badcore_buf[64]= {0};
#endif
	signed char temp_offset[8]={0};
	char logstr[256];
	unsigned char pic_version;
	int vol_value;

#ifdef CLEMENT_DEBUG_T9_18_IIC
	ClementTest();
#endif

	start_pic_heart=false;
	pthread_mutex_lock(&iic_mutex);
    // init fpga
    printf("clement2 init_fpga\n");
	reset_fpga();

	pthread_mutex_unlock(&iic_mutex);

	// check chain
	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
	{
		cgpu.chain_exist[i] = 0;
	}
	
	printf("clement check_chain\n");
	check_chain();

#ifdef USE_NEW_RESET_FPGA
	set_reset_allhashboard(1);
#endif

	// we need check PIC fw at first , before read freq!!!
	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
    {
    	if(cgpu.chain_exist[i]==0 
#ifndef ALL_CHAIN_INTEST
			|| testDone[i]
#endif
			)
			continue;
		
        reset_iic_pic(i);
        usleep(500*1000);

		jump_to_app_CheckAndRestorePIC(i);
    }

	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
    {
    	if(cgpu.chain_exist[i]==0 
#ifndef ALL_CHAIN_INTEST
			|| testDone[i]
#endif
			)
			continue;

#ifndef T9_18
		reset_iic_pic(i);
        usleep(500*1000);
		
		read_pic_freq(i,buf);
		read_pic_badcore_num(i,badcore_buf);
#else
		if(fpga_version>=0xE)
		{
			if(i>=1 && i<=3)	// only 1,2,3 has PIC 
			{
				read_freq_badcores(i,chain_pic_buf[i]);

				sprintf(logstr,"Chain[%d] read_freq_badcores : ",i);
				writeLogFile(logstr);
				for(j=0;j<128;j++)
				{
					sprintf(logstr,"0x%02x ",chain_pic_buf[i][j]);
					writeLogFile(logstr);
				}
				sprintf(logstr,"\n");
				writeLogFile(logstr);
			}
		}
		else
		{
			// read into chain_pic_buf,  only if(i%3==0) to read E2PROM into chain_pic_buf
			if(i%3==0)
				read_freq_badcores(((i/3)*3),chain_pic_buf[((i/3)*3)]);
		}
#endif

#ifdef T9_18
		if(getChainPICMagicNumber(i) == FREQ_MAGIC)
#else
        if(buf[1] == FREQ_MAGIC)
#endif
        {
        	if(conf.force_freq)
        	{	// do not clear PIC flash, no need to do that
#ifdef FORCE_8xPATTENT_TEST
				sprintf(logstr,"Start do 8xPatten on chain[%d]...\n",i);
				writeLogFile(logstr);
#ifdef T9_18
				if(fpga_version>=0xE)
				{
					int new_T9_PLUS_chainIndex,new_T9_PLUS_chainOffset;
					getPICChainIndexOffset(i,&new_T9_PLUS_chainIndex,&new_T9_PLUS_chainOffset);
					
					base_freq_index[i]=chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31];
					
					for(freq_index = 0; freq_index < ASIC_NUM; freq_index++)
					{
						last_success_freq[i][freq_index]=chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index];	// used to write rate file
					}

					chain_vol_added[i]=chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+2];

					sprintf(logstr,"Chain[%d] has core num in PIC\n",i);
					writeLogFile(logstr);
					
					for(j = 0; j < ASIC_NUM; j++)
					{
						if(j%2)
							chain_badcore_num[i][j]=(chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+22+(j/2)]&0x0f);
						else chain_badcore_num[i][j]=(chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+22+(j/2)]>>4)&0x0f;

						if(chain_badcore_num[i][j]>0)
						{
							sprintf(logstr,"Chain[%d] ASIC[%d] has core num=%d\n",i,j,chain_badcore_num[i][j]);
							writeLogFile(logstr);
						}
					}
				}
				else
				{
					base_freq_index[i]=chain_pic_buf[((i/3)*3)][7+(i%3)*31];
					
					for(freq_index = 0; freq_index < ASIC_NUM; freq_index++)
					{
						last_success_freq[i][freq_index]=chain_pic_buf[((i/3)*3)][7+(i%3)*31+4+freq_index];	// used to write rate file
					}

					chain_vol_added[i]=chain_pic_buf[((i/3)*3)][7+(i%3)*31+2];

					sprintf(logstr,"Chain[%d] has core num in PIC\n",i);
					writeLogFile(logstr);
					
					for(j = 0; j < ASIC_NUM; j++)
					{
						if(j%2)
							chain_badcore_num[i][j]=(chain_pic_buf[((i/3)*3)][7+(i%3)*31+22+(j/2)]&0x0f);
						else chain_badcore_num[i][j]=(chain_pic_buf[((i/3)*3)][7+(i%3)*31+22+(j/2)]>>4)&0x0f;

						if(chain_badcore_num[i][j]>0)
						{
							sprintf(logstr,"Chain[%d] ASIC[%d] has core num=%d\n",i,j,chain_badcore_num[i][j]);
							writeLogFile(logstr);
						}
					}
				}
#else
				memcpy(chain_pic_buf[i],buf,128);
				memcpy(pic_badcore_num[i],badcore_buf,64);

				base_freq_index[i]=((buf[6]&0x0f)<<4)+(buf[8]&0x0f);
				
				for(freq_index = 0; freq_index < ASIC_NUM; freq_index++)
				{
					last_success_freq[i][freq_index]=buf[freq_index*2+3];	// used to write rate file
				}

				chain_vol_added[i]=buf[10]&0x3f;

				if(badcore_buf[0]==BADCORE_MAGIC)
				{
					sprintf(logstr,"Chain[%d] has core num in PIC\n",i);
					writeLogFile(logstr);
					
					for(j = 0; j < ASIC_NUM; j++)
					{
						if(j%2)
							chain_badcore_num[i][j]=(badcore_buf[(j/2)*2+1]&0x0f);
						else chain_badcore_num[i][j]=(badcore_buf[(j/2)*2+1]>>4)&0x0f;

						if(chain_badcore_num[i][j]>0)
						{
							sprintf(logstr,"Chain[%d] ASIC[%d] has core num=%d\n",i,j,chain_badcore_num[i][j]);
							writeLogFile(logstr);
						}
					}
				}
				else
				{
					sprintf(logstr,"Chain[%d] has no core num in PIC\n",i);
					writeLogFile(logstr);

					for(j = 0; j < ASIC_NUM; j++)
						chain_badcore_num[i][j]=0;	// fixed to 0
				}
#endif
#else
        		sprintf(logstr,"Start search freq on chain[%d]...\n",i);
				writeLogFile(logstr);
#endif
        	}
			else
			{
				system("cp /www/pages/cgi-bin/minerConfiguration2.cgi /www/pages/cgi-bin/minerConfiguration.cgi -f");
				sprintf(logstr,"has freq in PIC, will disable freq setting.\n");
				writeLogFile(logstr);

#ifdef T9_18
				if(fpga_version>=0xE)
				{
					int new_T9_PLUS_chainIndex,new_T9_PLUS_chainOffset;
					getPICChainIndexOffset(i,&new_T9_PLUS_chainIndex,&new_T9_PLUS_chainOffset);
					
					for(freq_index = 0; freq_index < ASIC_NUM; freq_index++)
					{
						last_success_freq[i][freq_index]=chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+4+freq_index];	// used to write rate file
					}

					testDone[i]=true;
					search_over[i]=true;
					
					sprintf(logstr,"chain[%d] has freq in PIC and will jump over...\n",i);
					writeLogFile(logstr);

					sprintf(logstr,"Chain[%d] has core num in PIC\n",i);
					writeLogFile(logstr);
					
					for(j = 0; j < ASIC_NUM; j++)
					{
						if(j%2)
							chain_badcore_num[i][j]=(chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+22+(j/2)]&0x0f);
						else chain_badcore_num[i][j]=(chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+22+(j/2)]>>4)&0x0f;

						if(chain_badcore_num[i][j]>0)
						{
							sprintf(logstr,"Chain[%d] ASIC[%d] has core num=%d\n",i,j,chain_badcore_num[i][j]);
							writeLogFile(logstr);
						}
					}
				}
				else
				{
					for(freq_index = 0; freq_index < ASIC_NUM; freq_index++)
					{
						last_success_freq[i][freq_index]=chain_pic_buf[((i/3)*3)][7+(i%3)*31+4+freq_index];	// used to write rate file
					}

					testDone[i]=true;
					search_over[i]=true;
					
					sprintf(logstr,"chain[%d] has freq in PIC and will jump over...\n",i);
					writeLogFile(logstr);

					sprintf(logstr,"Chain[%d] has core num in PIC\n",i);
					writeLogFile(logstr);
					
					for(j = 0; j < ASIC_NUM; j++)
					{
						if(j%2)
							chain_badcore_num[i][j]=(chain_pic_buf[((i/3)*3)][7+(i%3)*31+22+(j/2)]&0x0f);
						else chain_badcore_num[i][j]=(chain_pic_buf[((i/3)*3)][7+(i%3)*31+22+(j/2)]>>4)&0x0f;

						if(chain_badcore_num[i][j]>0)
						{
							sprintf(logstr,"Chain[%d] ASIC[%d] has core num=%d\n",i,j,chain_badcore_num[i][j]);
							writeLogFile(logstr);
						}
					}
				}
#else
				// save the PIC content, used to get miner info!!!
				memcpy(chain_pic_buf[i],buf,128);
				memcpy(pic_badcore_num[i],badcore_buf,64);
		
				for(freq_index = 0; freq_index < ASIC_NUM; freq_index++)
				{
					last_success_freq[i][freq_index]=buf[freq_index*2+3];	// used to write rate file
				}

				testDone[i]=true;
				search_over[i]=true;
				
				sprintf(logstr,"chain[%d] has freq in PIC and will jump over...\n",i);
				writeLogFile(logstr);

				if(badcore_buf[0]==BADCORE_MAGIC)
				{
					sprintf(logstr,"Chain[%d] has core num in PIC\n",i);
					writeLogFile(logstr);
					
					for(j = 0; j < ASIC_NUM; j++)
					{
						if(j%2)
							chain_badcore_num[i][j]=(badcore_buf[(j/2)*2+1]&0x0f);
						else chain_badcore_num[i][j]=(badcore_buf[(j/2)*2+1]>>4)&0x0f;

						if(chain_badcore_num[i][j]>0)
						{
							sprintf(logstr,"Chain[%d] ASIC[%d] has core num=%d\n",i,j,chain_badcore_num[i][j]);
							writeLogFile(logstr);
						}
					}
				}
				else
				{
					sprintf(logstr,"Chain[%d] has no core num in PIC\n",i);
					writeLogFile(logstr);

					for(j = 0; j < ASIC_NUM; j++)
						chain_badcore_num[i][j]=0;	// fixed to 0
				}
#endif
			}
        }
		else
		{
			if(conf.force_freq)
			{
#ifdef FORCE_8xPATTENT_TEST
				sprintf(logstr,"Failed: When do 8xPatten on chain[%d], there is no freq in PIC...\n",i);
				writeLogFile(logstr);
				
				searchStatus=SEARCH_FAILED;
				while(1)
				{
					processTEST();
					sleep(1);
				}
#endif
			}
			else
			{
				int default_freq=GetDefaultFreq();
				int default_freq_index=get_pll_index(default_freq);

				sprintf(logstr,"chain[%d] has no freq in PIC! Will use default freq=%d and jump over...\n",i,default_freq);
				writeLogFile(logstr);

				for(freq_index = 0; freq_index < ASIC_NUM; freq_index++)
				{
					last_success_freq[i][freq_index]=default_freq_index;	// used to write rate file
				}

				testDone[i]=true;
				search_over[i]=true;

				IsSomeBoardHasNoFreq=true;
			}
		}

#ifndef T9_18
		jump_to_app_CheckAndRestorePIC(i);
#endif
    }

	// we just show the old voltage on hashboard!
	for(i=0; i < BITMAIN_MAX_CHAIN_NUM; i++)
	{
		if(cgpu.chain_exist[i]==0)
			continue;
		
		vol_pic=get_pic_voltage(i);
		vol_value = getVolValueFromPICvoltage(vol_pic);
		sprintf(logstr,"get PIC voltage=%d on chain[%d], value=%d\n",vol_pic,i,vol_value);
		writeLogFile(logstr);
	}

#ifdef DEBUG_DC_OPEN_CLOSE
	while(1)
	{
#ifdef USE_NEW_RESET_FPGA
		set_reset_allhashboard(1);
		sleep(1);
#endif
		// enable iic pic dac
		for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
	    {
	    	if(cgpu.chain_exist[i]==0)
				continue;
			
	        enable_pic_dac(i);
			sprintf(logstr,"enable_pic_dac on chain[%d]\n",i);
			writeLogFile(logstr);
		}

		sprintf(logstr,"OPEN DC done , wait for 5s...\n");
		writeLogFile(logstr);
		sleep(1);

#ifdef USE_NEW_RESET_FPGA
		set_reset_allhashboard(1);
		sleep(RESET_KEEP_TIME);
		set_reset_allhashboard(0);
		sleep(1);
#else
		reset_hashboard();
#endif
		sprintf(logstr,"RESET HASHBOARD done , wait for 5s...\n");
		writeLogFile(logstr);
		sleep(5);

		// disable iic pic dac
		for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
	    {
	    	if(cgpu.chain_exist[i]==0)
				continue;
			
	        disable_pic_dac(i);
			sprintf(logstr,"disable_pic_dac on chain[%d]\n",i);
			writeLogFile(logstr);
		}
		sprintf(logstr,"CLOSE DC done , wait for 10s...\n");
		writeLogFile(logstr);
		sleep(10);
	}
#endif

#ifdef FORCE_8xPATTENT_TEST
	for(i=0; i < BITMAIN_MAX_CHAIN_NUM; i++)  // here must use i from 0 in for loop, because we use j to get the index as config file's voltage value
	{
		if(cgpu.chain_exist[i]==0)
			continue;
		
		vol_pic=get_pic_voltage(i);
		chain_vol_value[i] = getVolValueFromPICvoltage(vol_pic);
		sprintf(logstr,"get PIC voltage=%d on chain[%d], voladded=0.%d V\n",chain_vol_value[i],i,chain_vol_added[i]);
		writeLogFile(logstr);
	}
#else
	j=0;
	for(i=0; i < BITMAIN_MAX_CHAIN_NUM; i++)  // here must use i from 0 in for loop, because we use j to get the index as config file's voltage value
	{
		if(cgpu.chain_exist[i]==0 
#ifndef ALL_CHAIN_INTEST
			|| testDone[i]
#endif
			)
			continue;
		
		if(chain_vol_value[i]==0)
			chain_vol_value[i]=START_VOLTAGE;	// first time to run, we set voltage to 8.6V

		chain_vol_added[i]=0;
		
		vol_pic=getPICvoltageFromValue(chain_vol_value[i]);

		sprintf(logstr,"set voltage=%d search freq on chain[%d]\n",chain_vol_value[i],i);
		writeLogFile(logstr);

		sprintf(logstr,"now set pic voltage=%d on chain[%d]\n",vol_pic,i);
		writeLogFile(logstr);
		set_pic_voltage(i, vol_pic);

		j++;	// the order number of chain
	}
#endif

	// enable iic pic dac
	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
    {
    	if(cgpu.chain_exist[i]==0
#ifndef ALL_CHAIN_INTEST
			|| testDone[i]
#endif
			)
			continue;
		
        enable_pic_dac(i);
		sprintf(logstr,"enable_pic_dac on chain[%d]\n",i);
		writeLogFile(logstr);
	}
	sleep(1);

	start_pic_heart=true;
	send_pic_heart_once();

#ifndef T9_18
#ifdef DEBUG_TEST_PIC_FLASH_RW
	sprintf(logstr,"start test_PIC_flash_rw : \n");
	writeLogFile(logstr);
	test_PIC_flash_rw();

	while(1)sleep(1);
#endif
#endif

#ifdef USE_NEW_RESET_FPGA
	set_reset_allhashboard(0);
#endif

	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
    {
    	if(cgpu.chain_exist[i]==0)
			continue;
		
#ifdef T9_18
		if(fpga_version>=0xE)
		{
			if(i!=8 && i!=10 && i!=12)	// only 8,10,12 chain has temp chips
				continue;
		}
		else
		{
			if(i%3!=1) 	// only 1,4,7 chain has temp chip
				continue;
		}
#endif

#ifdef DEBUG_S9_INFAN_TEMP
		temp_chip_index[i]=TEMP_ASIC_INDEX;
		chip_temp_offset[i]=-123;

		sprintf(logstr,"chain[%d] debug in fan temp=%d\n",i,TEMP_ASIC_INDEX);
		writeLogFile(logstr);
#else
#ifdef SPEICAL_FIXED_2TEMP_TO_PIC
		temp_offset[0]=32;
		temp_offset[1]=-4;
		temp_offset[2]=62;
		temp_offset[3]=-4;

		set_temperature_offset_value(i,temp_offset);

		sprintf(logstr,"SPECIAL FIX 2 TEMP CHIPS: chain[%d] temp offset record: %d,%d,%d,%d,%d,%d,%d,%d\n",i,temp_offset[0],temp_offset[1],temp_offset[2],temp_offset[3],temp_offset[4],temp_offset[5],temp_offset[6],temp_offset[7]);
		writeLogFile(logstr);
		memset(temp_offset,0x00,8);	// cleart it
#endif
		get_temperature_offset_value(i,temp_offset);

		sprintf(logstr,"chain[%d] temp offset record: %d,%d,%d,%d,%d,%d,%d,%d\n",i,temp_offset[0],temp_offset[1],temp_offset[2],temp_offset[3],temp_offset[4],temp_offset[5],temp_offset[6],temp_offset[7]);
		writeLogFile(logstr);

		if(temp_offset[0]>0)
		{
			temp_chip_index[i]=temp_offset[0];
			chip_temp_offset[i]=temp_offset[1];
		}
		else
		{
#ifdef T9_18
			temp_chip_index[i]=TEMP_ASIC_INDEX;
			chip_temp_offset[i]=-123;
#else
			temp_chip_index[i]=TEMP_ASIC_INDEX;
			chip_temp_offset[i]=-123;	// no temp offset magic number

			if(conf.force_freq)
			{
				sprintf(search_failed_info,"J%d:4",i+1);	// not temp offset in PIC
				saveSearchFailedFlagInfo();
				searchStatus=SEARCH_FAILED;
				while(1)
				{
					processTEST();
					sleep(1);
				}
			}
#endif
		}
#endif
	}
}

#ifdef USE_DOWN_VOLTAGE_LIMIT_FREQ
static bool checkFreqVoltageTooHigh(int chainIndex)
{
	int vol_value=chain_vol_value[chainIndex];
	int j;
	int avg_freq;
	int totalfreq=0;
	char logstr[256];
	
#ifdef R4
	return false;
#endif

	if(last_success_freq[chainIndex][0]>0)
	{
		for(j=0;j<ASIC_NUM;j++)
		{
			totalfreq+=(atoi(freq_pll_1385[last_success_freq[chainIndex][j]].freq));
		}
	}
	else
	{
		sprintf(logstr,"Fatal Error: checkFreqVoltageTooHigh find no last_success_freq on chain[%d]\n",chainIndex);
		writeLogFile(logstr);
		return false;
	}

	avg_freq=(totalfreq/ASIC_NUM);

#ifdef S9_PLUS
	if(vol_value==START_VOLTAGE)
	{
		if(avg_freq>=700)
		{
			sprintf(logstr,"T9 8.3V avg freq=%d on chain[%d]\n",avg_freq,chainIndex);
			writeLogFile(logstr);
			return true;
		}
	}
	else if(vol_value==RETRY_VOLTAGE)
	{
		if(avg_freq>=650)
		{
			sprintf(logstr,"T9 8.6V avg freq=%d on chain[%d]\n",avg_freq,chainIndex);
			writeLogFile(logstr);
			return true;
		}
	}
#endif

#ifdef T9_18
	if(vol_value==START_VOLTAGE)
	{
		if(avg_freq>=700)
		{
			sprintf(logstr,"T9+ 8.1V avg freq=%d on chain[%d]\n",avg_freq,chainIndex);
			writeLogFile(logstr);
			return true;
		}
	}
	else if(vol_value==RETRY_VOLTAGE)
	{
		if(avg_freq>=650)
		{
			sprintf(logstr,"T9+ 8.4V avg freq=%d on chain[%d]\n",avg_freq,chainIndex);
			writeLogFile(logstr);
			return true;
		}
	}
#endif

#ifdef S9_63
	if(vol_value==START_VOLTAGE)
	{
		if(avg_freq>=712)
		{
			sprintf(logstr,"S9 8.6V avg freq=%d on chain[%d]\n",avg_freq,chainIndex);
			writeLogFile(logstr);
			return true;
		}
	}
	else if(vol_value==RETRY_VOLTAGE)
	{
		if(avg_freq>=662)
		{
			sprintf(logstr,"S9 8.9V avg freq=%d on chain[%d]\n",avg_freq,chainIndex);
			writeLogFile(logstr);
			return true;
		}
	}
#endif

	return false;
}
#endif

static void singleBoardTest(void)
{
    int i, j, freq_index = 0;
	unsigned char vol_pic;
    unsigned char buf[128]= {0};
	char logstr[256];
#ifdef NEED_PRE_HEAT
	int need_heat=0;
#endif
	int wait_count=0;
	int min_Freq;
	int max_Freq;
	int last_send_num;
	int last_recv_num;
	int vol_value;
	unsigned char pic_version;
		
	// flag for each chain test is done or not...
	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
	{
		if(search_over[i])
			testDone[i]=true;
		else 
			testDone[i]=false;
	}
	
	start_pic_heart=false;
	pthread_mutex_lock(&iic_mutex);
    // init fpga
    printf("clement2 init_fpga\n");
	reset_fpga();

	pthread_mutex_unlock(&iic_mutex);
	send_pic_heart_once();
	start_pic_heart=true;

    // reset global arg
    reset_global_arg();
    start_receive = true;

#ifdef ENABLE_TEMP_PROCESS
	gIsReadTemp = false;
#endif

	gStartTest=false;

#ifdef USE_NEW_RESET_FPGA
	set_reset_allhashboard(1);
	sleep(RESET_KEEP_TIME);
	set_reset_allhashboard(0);
	sleep(1);
#else
    reset_hashboard();
#endif

	reset_crc_count();
	sprintf(logstr,"CRC error counter=%d\n",get_crc_count());
	writeLogFile(logstr);
		
    //set CommandMode
    printf("\n--- set command mode\n");
    if(conf.CommandMode)    // fil mode
    {
        // after reset fpga, it should be fil mode as default
        cgpu.CommandMode = 1;
        printf("set command mode to FIL\n");
    }
    else                    // vil mode
    {
    	printf("clement2 set_dhash_acc_control vil\n");
        set_dhash_acc_control((get_dhash_acc_control() & (~OPERATION_MODE)) | VIL_MODE | (VIL_MIDSTATE_NUMBER(1) & (~NEW_BLOCK) & (~RUN_BIT)));
        cgpu.CommandMode = 0;
        sprintf(logstr,"set command mode to VIL\n");
		writeLogFile(logstr);
    }

#ifdef USE_PREINIT_OPENCORE
	set_baud(DEFAULT_BAUD_VALUE);
	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
    {
        if(cgpu.chain_exist[i] == 1
#ifndef ALL_CHAIN_INTEST
			&& (!testDone[i])
#endif
			)
        {
        	getAsicNum_preOpenCore(i);
			
            sprintf(logstr,"check chain[%d]: asicNum = %d\n",i, cgpu.chain_asic_num[i]);
			writeLogFile(logstr);
			if(cgpu.chain_asic_num[i]!=ASIC_NUM)
			{
            	isNoBoardError=true;	// we want to exit, and watchdog will run again.
				
                sprintf(logstr,"The AsicNum=%d on chain[%d]\n",cgpu.chain_asic_num[i],i);
				writeLogFile(logstr);

				testDone[i]=true;	// set test over flag!
				search_over[i]=true;
				searchFreqMode[i]=SEARCH_OVER;	// will not search freq on this chain!!!

				search_freq_result[i]=false;	// no board, we do not try search again!

				// just stop ,we do not want to search ....
				if(isChipNumOK_Once)
					sprintf(search_failed_info,"J%d:3",i+1);
				else sprintf(search_failed_info,"J%d:2",i+1);
				
				saveSearchFailedFlagInfo();
				searchStatus=SEARCH_FAILED;
				while(1)
				{
					processTEST();
					sleep(1);
				}
			}
        }
	}
#else
    //check asic number
    if(Conf.CheckChain)
    {
        sprintf(logstr,"\n--- check asic number\n");
		writeLogFile(logstr);

	//	check_asic_reg(CHIP_ADDRESS);
		
        for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
        {
            if(cgpu.chain_exist[i] == 1 
#ifndef ALL_CHAIN_INTEST
				&& (!testDone[i])
#endif
				)
            {
            	int noboard_retry_count=0;
            	check_asic_reg_oneChain(i,CHIP_ADDRESS);
				
                sprintf(logstr,"check chain[%d]: asicNum = %d\n",i, cgpu.chain_asic_num[i]);
				writeLogFile(logstr);

#ifdef DEBUG_STOP_WHEN_ASICNUM_NOTENOUGH
			//	cgpu.chain_asic_num[i]=0;	// force to do the test!!!
				while(cgpu.chain_asic_num[i] != ASIC_NUM)
				{
					int each_asic_freq;
					int freq_test=8;	//300M

					for(j=0; j<256; j++)
		    		{
		    	    	last_result[i][j] = 0 ;
						last_opencore_result[i][j] = 0;
					}

					chain_DataCount[i]=TESTMODE_PATTEN_NUM_8X;	// when seaching base freq, we use 8*144 patten on chip
					chain_ValidNonce[i]=TESTMODE_NONCE_NUM_8X;
					chain_PassCount[i]=TESTMODE_PATTEN_NUM_8X;
					
					sprintf(logstr,"start software set address on Chain[%d]...\n",i);
					writeLogFile(logstr);

					software_set_address_onChain(i);

					sprintf(logstr,"Done: software set address on Chain[%d], wait for 30s...\n",i);
					writeLogFile(logstr);
					sleep(30);

					sprintf(logstr,"start set freq=300M for %d times...\n",noboard_retry_count+1);
					writeLogFile(logstr);

					for(each_asic_freq = 0; each_asic_freq < ASIC_NUM; each_asic_freq ++)
		            {
		                set_frequency_with_addr_plldatai(freq_test,0, each_asic_freq * CHIP_ADDR_INTERVAL,i);
		            }

					min_Freq=300;

					sprintf(logstr,"Done: set freq=300M on Chain[%d], wait for 30s...\n",i);
					writeLogFile(logstr);
					sleep(30);
/*
					conf.baud=BAUD_LEVEL;
					sprintf(logstr,"start: set_baud=%d, ought to be %d\n",conf.baud,GetBandValue(min_Freq));
					writeLogFile(logstr);
					set_baud(conf.baud);

					sprintf(logstr,"Done: set_baud=%d, wait for 30s...\n",conf.baud);
					writeLogFile(logstr);
					sleep(30);
*/
					sprintf(logstr,"start: set_time_out...\n");
					writeLogFile(logstr);
					
					if(ASIC_NUM != 1)
				    {
					    int temp_corenum = calculate_core_number(ASIC_CORE_NUM);
				    	conf.timeout = 0x1000000/temp_corenum*CHIP_ADDR_INTERVAL/min_Freq*TIMEOUT_PERCENT/100;	// 7% timeout

						sprintf(logstr,"The min freq=%d\n",min_Freq);
						writeLogFile(logstr);
						
						sprintf(logstr,"set real timeout %d, need sleep=%d\n", conf.timeout, conf.timeout*(conf.ValidNonce1+conf.passCount1));
						writeLogFile(logstr);
						
				        set_time_out_control(conf.timeout);
				    }
					sprintf(logstr,"Done: set_time_out, wait for 30s...\n");
					writeLogFile(logstr);
					sleep(30);

					sprintf(logstr,"start: open core on Chain[%d]...\n",i);
					writeLogFile(logstr);
					//open core
					open_core_onChain(i,ASIC_CORE_NUM,noboard_retry_count+1,true);	// open 2,3,4 cores

					sprintf(logstr,"Done: open core, wait for 30s...\n");
					writeLogFile(logstr);
					sleep(30);
					
				    if(ASIC_NUM != 1)
				    {
					    int temp_corenum = calculate_core_number(ASIC_CORE_NUM);
				    	conf.timeout = 0x1000000/temp_corenum*CHIP_ADDR_INTERVAL/min_Freq*TIMEOUT_PERCENT/100;	// 7% timeout

						sprintf(logstr,"The min freq=%d\n",min_Freq);
						writeLogFile(logstr);
						
						sprintf(logstr,"set real timeout %d, need sleep=%d\n", conf.timeout, conf.timeout*(conf.ValidNonce1+conf.passCount1));
						writeLogFile(logstr);
						
				        set_time_out_control(conf.timeout);
				    }

					sprintf(logstr,"TEST: open %d cores Done!\n",noboard_retry_count+1);
					writeLogFile(logstr);
					
				//	sprintf(logstr,"TEST: wait for 10s...\n");
				//	writeLogFile(logstr);
				//	sleep(10);

					cgpu.chain_asic_num[i]=0;
					check_asic_reg_oneChain(i,CHIP_ADDRESS);
					
					sprintf(logstr,"TEST: retry check chain[%d]: asicNum = %d, wait for 3mins to test voltage\n",i, cgpu.chain_asic_num[i]);
					writeLogFile(logstr);
					sleep(180);

					reset_nonce_arg();
					reset_work_data();

					// before the first time for sending work, reset the FPGA's nonce fifo
				    if(!gBegin_get_nonce)
				    {
				        //printf("\n--- clear nonce fifo before send work\n");
				        printf("clement2 set_nonce_fifo_interrupt\n");
				        set_nonce_fifo_interrupt(get_nonce_fifo_interrupt() | FLUSH_NONCE3_FIFO);
				        gBegin_get_nonce = true;
				    }

					sprintf(logstr,"start send works on chain[%d]\n",i);
					writeLogFile(logstr);

					StartSendFlag[i]=true;

#ifdef USE_SINGLE_SEND_THREAD
					send_func_all();
#else
					sprintf(logstr,"wait send works done on chain[%d]\n",i);
					writeLogFile(logstr);

					last_send_num=0;
					wait_count=0;
					while(wait_count<SEND_WAIT_TIMEOUT && send_work_num[i]<chain_ValidNonce[i])
					{
						if(last_send_num!=send_work_num[i])
						{
							wait_count=0;
							last_send_num=send_work_num[i];
						}
						else wait_count++;

						usleep(100000);
					}
					if(wait_count>=SEND_WAIT_TIMEOUT)
					{
						sprintf(logstr,"Some Error to cause sending works timeout on chain[%d] sendExit=%d, %d/%d\n",i,sendExit[i],send_work_num[i],chain_ValidNonce[i]);
						writeLogFile(logstr);
					}
#endif
					StartSendFlag[i]=false;

					sprintf(logstr,"wait recv nonce on chain[%d]\n",i);
					writeLogFile(logstr);

					last_recv_num=0;
					wait_count=0;
					while(wait_count < RECV_WAIT_TIMEOUT && valid_nonce_num[i]<chain_ValidNonce[i])
					{
						if(last_recv_num!=valid_nonce_num[i])
						{
							wait_count=0;
							last_recv_num=valid_nonce_num[i];
						}
						else wait_count++;

						usleep(100000);
					}

					gBegin_get_nonce=false;
					
					sprintf(logstr,"get nonces on chain[%d]\n",i);
					writeLogFile(logstr);

				    get_result(i, chain_PassCount[i], chain_ValidNonce[i]);
					print_OpenCoreDetails(i,chain_PassCount[i],noboard_retry_count+1);
					
					if(last_opencore_pass(i,noboard_retry_count+1))
					{
						sprintf(logstr,"OPEN CORE on chain[%d]: OK! \n",i);
						writeLogFile(logstr);

					//	break;	// we jump out
						cgpu.chain_asic_num[i]=0;	// force to try again!
					}
					else
					{
						sprintf(logstr,"OPEN CORE on chain[%d]: FAILED!\n",i);
						writeLogFile(logstr);

						cgpu.chain_asic_num[i]=0;	// force to try again!
					}

				//	sprintf(logstr,"TEST: wait for test voltage, wait for 3mins...\n");
				//	writeLogFile(logstr);
				//	sleep(180);

					noboard_retry_count++;
					if(noboard_retry_count>=3)
					{
						testDone[i]=true;	// set test over flag! only this time, next time will retry again...
						sprintf(logstr,"Error: The AsicNum=%d on chain[%d]!\n",cgpu.chain_asic_num[i],i);
						writeLogFile(logstr);

						isFailedOnTestPatten=true;	// set failed flag!
						search_freq_result[i]=false;

						if(isChipNumOK_Once)
							sprintf(search_failed_info,"J%d:3",i+1);
						else sprintf(search_failed_info,"J%d:2",i+1);
						
						saveSearchFailedFlagInfo();
						searchStatus=SEARCH_FAILED;
						while(1)
						{
							processTEST();
							sleep(1);
						}
					}
				}
#endif

                while(cgpu.chain_asic_num[i] != ASIC_NUM)
                {
                	noboard_retry_count++;

					if(noboard_retry_count>NOBOARD_RETRY_COUNT)
					{
	                	isNoBoardError=true;	// we want to exit, and watchdog will run again.
						
	                    sprintf(logstr,"The AsicNum=%d on chain[%d]\n",cgpu.chain_asic_num[i],i);
						writeLogFile(logstr);

						testDone[i]=true;	// set test over flag!
						search_over[i]=true;
						searchFreqMode[i]=SEARCH_OVER;	// will not search freq on this chain!!!

						search_freq_result[i]=false;	// no board, we do not try search again!

						// just stop ,we do not want to search ....
						if(isChipNumOK_Once)
							sprintf(search_failed_info,"J%d:3",i+1);
						else sprintf(search_failed_info,"J%d:2",i+1);
						
						saveSearchFailedFlagInfo();
						searchStatus=SEARCH_FAILED;
						while(1)
						{
							processTEST();
							sleep(1);
						}
					}
					else
					{
#ifdef DEBUG_STOP_WHEN_ASICNUM_NOTENOUGH
						if(((noboard_retry_count-1)%3)==0 && noboard_retry_count>1)
						{
							sprintf(logstr,"Chain[%d] will retry getAsicNum %d times!, close DC and open DC and reset\n",i,NOBOARD_RETRY_COUNT-noboard_retry_count+1);
							writeLogFile(logstr);

#ifdef USE_NEW_RESET_FPGA
							set_reset_hashboard(i,1);
#endif

							disable_pic_dac(i);
							sleep(1);
							enable_pic_dac(i);
							sleep(1);

#ifdef USE_NEW_RESET_FPGA
							set_reset_hashboard(i,0);
#endif

							sprintf(logstr,"DC closed and opened again! wait for test voltage, for 30s...\n");
							writeLogFile(logstr);
							sleep(30);
						}
						else
						{
							sprintf(logstr,"Chain[%d] will retry getAsicNum %d times!, NOT close DC, only reset\n",i,NOBOARD_RETRY_COUNT-noboard_retry_count+1);
							writeLogFile(logstr);
						}

#ifdef USE_NEW_RESET_FPGA
						set_reset_hashboard(i,1);
						sleep(RESET_KEEP_TIME);
						set_reset_hashboard(i,0);
						sleep(1);
#else
						reset_one_hashboard(i);
#endif
						sprintf(logstr,"Chain[%d] reset over! wait for test voltage, for 30s...\n",i);
						writeLogFile(logstr);
						sleep(30);
						
						cgpu.chain_asic_num[i]=0;
						check_asic_reg_oneChain(i,CHIP_ADDRESS);
						
						sprintf(logstr,"retry check chain[%d]: asicNum = %d\n",i, cgpu.chain_asic_num[i]);
						writeLogFile(logstr);
						
						sprintf(logstr,"After Retry to get asic number,  wait for test voltage, for 30s...\n");
						writeLogFile(logstr);
						sleep(30);
#else
#ifdef USE_NEW_RESET_FPGA
						set_reset_hashboard(i,1);
#endif

						disable_pic_dac(i);
						sleep(1);
						enable_pic_dac(i);
						sleep(1);

#ifdef USE_NEW_RESET_FPGA
						set_reset_hashboard(i,1);
						sleep(RESET_KEEP_TIME);
						set_reset_hashboard(i,0);
						sleep(1);
#else
						reset_one_hashboard(i);
#endif
						cgpu.chain_asic_num[i]=0;
						check_asic_reg_oneChain(i,CHIP_ADDRESS);
						
						sprintf(logstr,"retry check chain[%d]: asicNum = %d\n",i, cgpu.chain_asic_num[i]);
						writeLogFile(logstr);
#endif
					}
                }
            }
        }
    }
#endif

	isChipNumOK_Once=true;	// we get 63 chips at least once, so the failed number is 3 not 2 when we get chip num < 63 later!!!

#ifdef DEBUG_STOP_WHEN_ASICNUM_NOTENOUGH
	// reset nonce arg  again, because maybe it will test in get asicnum
	reset_nonce_arg();
#endif

	printf("clement software_set_address\n");
	software_set_address();

	//set baud
	//conf.baud=GetBandValue(min_Freq);
	conf.baud=BAUD_LEVEL;
	sprintf(logstr,"set_baud=%d\n",conf.baud);
	writeLogFile(logstr);
	set_baud(conf.baud);

#ifdef ENABLE_TEMP_PROCESS
	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
    {
		if(cgpu.chain_exist[i]==0 
#ifndef ALL_CHAIN_INTEST
			|| testDone[i]
#endif
			)
			continue;

#ifdef T9_18
		if(fpga_version>=0xE)
		{
			if(i!=8 && i!=10 && i!=12)	// only 8,10,12 chain has temp chips
				continue;
		}
		else
		{
			if(i%3!=1) 	// only 1,4,7 chain has temp chip
				continue;
		}
#endif

#ifndef DEBUG_READ_TEMP		// we want test read temp here for debug!
		if(chip_temp_offset[i]==-123)
#endif
		{
			chip_temp_offset[i]=DEFAULT_TEMP_OFFSET;
			for(j=0;j<16;j++)
			{
				set_default_temperature_offset_value(i);	// set default
	    		get_temperature_offset_value_from_asic(i);	// get offset value again inside of this function
			}
		}
		
    	set_default_temperature_offset_value(i);	// set new offset value into chip
		get_temperature_offset_value_from_asic(i);	// get offset value again inside of this function
		
		sprintf(logstr,"first time set temp offset of chain[%d] : %d\n",i,chip_temp_offset[i]);	// print new offset value
		writeLogFile(logstr);
	}
#endif

#if ((defined USE_LOWFREQ_OPENCORE) && (defined USE_PREINIT_OPENCORE))
#ifdef ENABLE_HIGH_VOLTAGE_OPENCORE
	// set highest voltage to open core
	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
	{
		unsigned char vol_pic;
		if(cgpu.chain_exist[i]==0 
#ifndef ALL_CHAIN_INTEST
			|| testDone[i]
#endif
			)
			continue;

		vol_pic=getPICvoltageFromValue(HIGHEST_VOLTAGE_LIMITED_HW);
#ifdef T9_18
		set_voltage_T9_18_into_PIC(i, vol_pic);
#else
		set_pic_voltage(i, vol_pic);
#endif

		sleep(1);

#ifdef USE_OPENCORE_ONEBYONE
		opencore_onebyone_onChain(i);
#else
		open_core_onChain(i,ASIC_CORE_NUM,ASIC_CORE_NUM,true);
#endif

#ifdef USE_OPENCORE_TWICE
		open_core_onChain(i,ASIC_CORE_NUM,ASIC_CORE_NUM,true);
#endif

		vol_pic=getPICvoltageFromValue(chain_vol_value[i]);

#ifdef T9_18
		if(fpga_version>=0xE)
		{
			if(i==9 
#ifndef ALL_CHAIN_INTEST
				|| (i==1 && testDone[8] && testDone[9]) || (i==8 && testDone[9])
#endif
				)
				set_pic_voltage(i, vol_pic);
			else if(i==11 
#ifndef ALL_CHAIN_INTEST
				|| (i==2 && testDone[10] && testDone[11]) || (i==10 && testDone[11])
#endif
				)
				set_pic_voltage(i, vol_pic);
			else if(i==13 
#ifndef ALL_CHAIN_INTEST
				|| (i==3 && testDone[12] && testDone[13]) || (i==12 && testDone[13])
#endif
				)
				set_pic_voltage(i, vol_pic);
		}
		else
		{
			if(i%3==2 
#ifndef ALL_CHAIN_INTEST
				|| (i%3==0 && testDone[(i/3)*3+1] && testDone[(i/3)*3+2]) || (i%3==1 && testDone[(i/3)*3+2])
#endif
				)
				set_pic_voltage(i, vol_pic);
		}
#else
		set_pic_voltage(i, vol_pic);
#endif
	}
#else
	//open core
	open_core(true);
#endif
#endif

	min_Freq=700;
	max_Freq=300;
	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
    {
    	int each_asic_freq = 0;
		int dc_area=0;
		bool found_one_chip;

		if(cgpu.chain_exist[i]==0 
#ifndef ALL_CHAIN_INTEST
			|| testDone[i]
#endif
			)
			continue;

#ifdef ALL_CHAIN_INTEST
		// set freq here, for running on null work
		if(testDone[i])
		{
			for(each_asic_freq = 0; each_asic_freq < ASIC_NUM; each_asic_freq ++)
            {
                set_frequency_with_addr_plldatai(last_success_freq[i][each_asic_freq],0, each_asic_freq * CHIP_ADDR_INTERVAL,i);

				if(min_Freq>atoi(freq_pll_1385[last_success_freq[i][each_asic_freq]].freq))
					min_Freq=atoi(freq_pll_1385[last_success_freq[i][each_asic_freq]].freq);

				if(max_Freq<atoi(freq_pll_1385[last_success_freq[i][each_asic_freq]].freq))
					max_Freq=atoi(freq_pll_1385[last_success_freq[i][each_asic_freq]].freq);
            }
		}
#endif

    	switch(searchFreqMode[i])
    	{
#if ((USE_SEARCH_BASEFREQ_MODE == 2) || (USE_SEARCH_BASEFREQ_MODE == 3) || (USE_SEARCH_BASEFREQ_MODE == 4))
		case SEARCH_BASE_FREQ_V89_200:
			chain_DataCount[i]=SEARCH_BASEFREQ_PATTEN_NUM;	// when seaching base freq, we use 8*144 patten on chip
			chain_ValidNonce[i]=SEARCH_BASEFREQ_NONCE_NUM;
			chain_PassCount[i]=SEARCH_BASEFREQ_PATTEN_NUM;

			Fmax[i]=4;	//200M
			chain_vol_value[i]=RETRY_VOLTAGE;
			chain_vol_added[i]=0;

			vol_pic=getPICvoltageFromValue(chain_vol_value[i]);
			set_pic_voltage(i, vol_pic);
			
			sprintf(logstr,"SEARCH_BASE_FREQ_V89_200 mode set freq=%s voltage=%d on chain[%d]\n",freq_pll_1385[Fmax[i]].freq, chain_vol_value[i], i);
			writeLogFile(logstr);

			for(each_asic_freq = 0; each_asic_freq < ASIC_NUM; each_asic_freq ++)
            {
            	last_freq[i][each_asic_freq]=Fmax[i];
                set_frequency_with_addr_plldatai(Fmax[i],0, each_asic_freq * CHIP_ADDR_INTERVAL,i);

				if(min_Freq>atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq))
					min_Freq=atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq);

				if(max_Freq<atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq))
					max_Freq=atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq);
            }
			break;
			
		case SEARCH_BASE_FREQ_V89_300:
			chain_DataCount[i]=SEARCH_BASEFREQ_PATTEN_NUM;	// when seaching base freq, we use 8*144 patten on chip
			chain_ValidNonce[i]=SEARCH_BASEFREQ_NONCE_NUM;
			chain_PassCount[i]=SEARCH_BASEFREQ_PATTEN_NUM;

			Fmax[i]=8;	//300M
			chain_vol_value[i]=RETRY_VOLTAGE;
			chain_vol_added[i]=0;

			vol_pic=getPICvoltageFromValue(chain_vol_value[i]);
			set_pic_voltage(i, vol_pic);
			
			sprintf(logstr,"SEARCH_BASE_FREQ_V89_300 mode set freq=%s voltage=%d on chain[%d]\n",freq_pll_1385[Fmax[i]].freq, chain_vol_value[i], i);
			writeLogFile(logstr);

			for(each_asic_freq = 0; each_asic_freq < ASIC_NUM; each_asic_freq ++)
            {
            	last_freq[i][each_asic_freq]=Fmax[i];
                set_frequency_with_addr_plldatai(Fmax[i],0, each_asic_freq * CHIP_ADDR_INTERVAL,i);

				if(min_Freq>atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq))
					min_Freq=atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq);

				if(max_Freq<atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq))
					max_Freq=atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq);
            }
			break;

		case SEARCH_BASE_FREQ_V89_400_2:
		case SEARCH_BASE_FREQ_V89_400:
			chain_DataCount[i]=SEARCH_BASEFREQ_PATTEN_NUM;	// when seaching base freq, we use 8*144 patten on chip
			chain_ValidNonce[i]=SEARCH_BASEFREQ_NONCE_NUM;
			chain_PassCount[i]=SEARCH_BASEFREQ_PATTEN_NUM;

			Fmax[i]=12;	//400M
			chain_vol_value[i]=RETRY_VOLTAGE;
			chain_vol_added[i]=0;

			vol_pic=getPICvoltageFromValue(chain_vol_value[i]);
			set_pic_voltage(i, vol_pic);
			
			sprintf(logstr,"SEARCH_BASE_FREQ_V89_400 mode set freq=%s voltage=%d on chain[%d]\n",freq_pll_1385[Fmax[i]].freq, chain_vol_value[i], i);
			writeLogFile(logstr);

			for(each_asic_freq = 0; each_asic_freq < ASIC_NUM; each_asic_freq ++)
            {
            	last_freq[i][each_asic_freq]=Fmax[i];
                set_frequency_with_addr_plldatai(Fmax[i],0, each_asic_freq * CHIP_ADDR_INTERVAL,i);

				if(min_Freq>atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq))
					min_Freq=atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq);

				if(max_Freq<atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq))
					max_Freq=atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq);
            }
			break;
			
		case SEARCH_BASE_FREQ_V86_400:
			chain_DataCount[i]=SEARCH_BASEFREQ_PATTEN_NUM;	// when seaching base freq, we use 8*144 patten on chip
			chain_ValidNonce[i]=SEARCH_BASEFREQ_NONCE_NUM;
			chain_PassCount[i]=SEARCH_BASEFREQ_PATTEN_NUM;

			Fmax[i]=12;	//400M
			chain_vol_value[i]=START_VOLTAGE;
			chain_vol_added[i]=0;

			vol_pic=getPICvoltageFromValue(chain_vol_value[i]);
			set_pic_voltage(i, vol_pic);
			
			sprintf(logstr,"SEARCH_BASE_FREQ_V86_400 mode set freq=%s voltage=%d on chain[%d]\n",freq_pll_1385[Fmax[i]].freq, chain_vol_value[i], i);
			writeLogFile(logstr);

			for(each_asic_freq = 0; each_asic_freq < ASIC_NUM; each_asic_freq ++)
            {
            	last_freq[i][each_asic_freq]=Fmax[i];
                set_frequency_with_addr_plldatai(Fmax[i],0, each_asic_freq * CHIP_ADDR_INTERVAL,i);

				if(min_Freq>atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq))
					min_Freq=atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq);

				if(max_Freq<atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq))
					max_Freq=atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq);
            }
			break;

		case SEARCH_BASE_FREQ_V86_500:
			chain_DataCount[i]=SEARCH_BASEFREQ_PATTEN_NUM;	// when seaching base freq, we use 8*144 patten on chip
			chain_ValidNonce[i]=SEARCH_BASEFREQ_NONCE_NUM;
			chain_PassCount[i]=SEARCH_BASEFREQ_PATTEN_NUM;

			Fmax[i]=44;	//500M
			chain_vol_value[i]=START_VOLTAGE;
			chain_vol_added[i]=0;

			vol_pic=getPICvoltageFromValue(chain_vol_value[i]);
			set_pic_voltage(i, vol_pic);
			
			sprintf(logstr,"SEARCH_BASE_FREQ_V86_500 mode set freq=%s voltage=%d on chain[%d]\n",freq_pll_1385[Fmax[i]].freq, chain_vol_value[i], i);
			writeLogFile(logstr);

			for(each_asic_freq = 0; each_asic_freq < ASIC_NUM; each_asic_freq ++)
            {
            	last_freq[i][each_asic_freq]=Fmax[i];
                set_frequency_with_addr_plldatai(Fmax[i],0, each_asic_freq * CHIP_ADDR_INTERVAL,i);

				if(min_Freq>atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq))
					min_Freq=atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq);

				if(max_Freq<atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq))
					max_Freq=atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq);
            }
			break;

		case SEARCH_BASE_FREQ_V86_600:
			chain_DataCount[i]=SEARCH_BASEFREQ_PATTEN_NUM;	// when seaching base freq, we use 8*144 patten on chip
			chain_ValidNonce[i]=SEARCH_BASEFREQ_NONCE_NUM;
			chain_PassCount[i]=SEARCH_BASEFREQ_PATTEN_NUM;

			Fmax[i]=66;	//600M
			chain_vol_value[i]=START_VOLTAGE;
			chain_vol_added[i]=0;

			vol_pic=getPICvoltageFromValue(chain_vol_value[i]);
			set_pic_voltage(i, vol_pic);
			
			sprintf(logstr,"SEARCH_BASE_FREQ_V86_600 mode set freq=%s voltage=%d on chain[%d]\n",freq_pll_1385[Fmax[i]].freq, chain_vol_value[i], i);
			writeLogFile(logstr);

			for(each_asic_freq = 0; each_asic_freq < ASIC_NUM; each_asic_freq ++)
            {
            	last_freq[i][each_asic_freq]=Fmax[i];
                set_frequency_with_addr_plldatai(Fmax[i],0, each_asic_freq * CHIP_ADDR_INTERVAL,i);

				if(min_Freq>atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq))
					min_Freq=atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq);

				if(max_Freq<atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq))
					max_Freq=atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq);
            }
			break;

		case SEARCH_BASE_FREQ_V86_650:
			chain_DataCount[i]=SEARCH_BASEFREQ_PATTEN_NUM;	// when seaching base freq, we use 8*144 patten on chip
			chain_ValidNonce[i]=SEARCH_BASEFREQ_NONCE_NUM;
			chain_PassCount[i]=SEARCH_BASEFREQ_PATTEN_NUM;

			Fmax[i]=74;	//650M
			chain_vol_value[i]=START_VOLTAGE;
			chain_vol_added[i]=0;

			vol_pic=getPICvoltageFromValue(chain_vol_value[i]);
			set_pic_voltage(i, vol_pic);
			
			sprintf(logstr,"SEARCH_BASE_FREQ_V86_650 mode set freq=%s voltage=%d on chain[%d]\n",freq_pll_1385[Fmax[i]].freq, chain_vol_value[i], i);
			writeLogFile(logstr);

			for(each_asic_freq = 0; each_asic_freq < ASIC_NUM; each_asic_freq ++)
            {
            	last_freq[i][each_asic_freq]=Fmax[i];
                set_frequency_with_addr_plldatai(Fmax[i],0, each_asic_freq * CHIP_ADDR_INTERVAL,i);

				if(min_Freq>atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq))
					min_Freq=atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq);

				if(max_Freq<atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq))
					max_Freq=atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq);
            }
			break;

		case SEARCH_BASE_FREQ_V89_500:
			chain_DataCount[i]=SEARCH_BASEFREQ_PATTEN_NUM;	// when seaching base freq, we use 8*144 patten on chip
			chain_ValidNonce[i]=SEARCH_BASEFREQ_NONCE_NUM;
			chain_PassCount[i]=SEARCH_BASEFREQ_PATTEN_NUM;

			Fmax[i]=44;	//500M
			chain_vol_value[i]=RETRY_VOLTAGE;
			chain_vol_added[i]=0;

			vol_pic=getPICvoltageFromValue(chain_vol_value[i]);
			set_pic_voltage(i, vol_pic);
			
			sprintf(logstr,"SEARCH_BASE_FREQ_V89_500 mode set freq=%s voltage=%d on chain[%d]\n",freq_pll_1385[Fmax[i]].freq, chain_vol_value[i], i);
			writeLogFile(logstr);

			for(each_asic_freq = 0; each_asic_freq < ASIC_NUM; each_asic_freq ++)
            {
            	last_freq[i][each_asic_freq]=Fmax[i];
                set_frequency_with_addr_plldatai(Fmax[i],0, each_asic_freq * CHIP_ADDR_INTERVAL,i);

				if(min_Freq>atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq))
					min_Freq=atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq);

				if(max_Freq<atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq))
					max_Freq=atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq);
            }
			break;

		case SEARCH_BASE_FREQ_V89_600:
			chain_DataCount[i]=SEARCH_BASEFREQ_PATTEN_NUM;	// when seaching base freq, we use 8*144 patten on chip
			chain_ValidNonce[i]=SEARCH_BASEFREQ_NONCE_NUM;
			chain_PassCount[i]=SEARCH_BASEFREQ_PATTEN_NUM;

			Fmax[i]=66;	//600M
			chain_vol_value[i]=RETRY_VOLTAGE;
			chain_vol_added[i]=0;

			vol_pic=getPICvoltageFromValue(chain_vol_value[i]);
			set_pic_voltage(i, vol_pic);
			
			sprintf(logstr,"SEARCH_BASE_FREQ_V89_600 mode set freq=%s voltage=%d on chain[%d]\n",freq_pll_1385[Fmax[i]].freq, chain_vol_value[i], i);
			writeLogFile(logstr);

			for(each_asic_freq = 0; each_asic_freq < ASIC_NUM; each_asic_freq ++)
            {
            	last_freq[i][each_asic_freq]=Fmax[i];
                set_frequency_with_addr_plldatai(Fmax[i],0, each_asic_freq * CHIP_ADDR_INTERVAL,i);

				if(min_Freq>atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq))
					min_Freq=atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq);

				if(max_Freq<atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq))
					max_Freq=atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq);
            }
			break;

		case SEARCH_BASE_FREQ_V89_650:
			chain_DataCount[i]=SEARCH_BASEFREQ_PATTEN_NUM;	// when seaching base freq, we use 8*144 patten on chip
			chain_ValidNonce[i]=SEARCH_BASEFREQ_NONCE_NUM;
			chain_PassCount[i]=SEARCH_BASEFREQ_PATTEN_NUM;

			Fmax[i]=74;	//650M
			chain_vol_value[i]=RETRY_VOLTAGE;
			chain_vol_added[i]=0;

			vol_pic=getPICvoltageFromValue(chain_vol_value[i]);
			set_pic_voltage(i, vol_pic);
			
			sprintf(logstr,"SEARCH_BASE_FREQ_V89_650 mode set freq=%s voltage=%d on chain[%d]\n",freq_pll_1385[Fmax[i]].freq, chain_vol_value[i], i);
			writeLogFile(logstr);

			for(each_asic_freq = 0; each_asic_freq < ASIC_NUM; each_asic_freq ++)
            {
            	last_freq[i][each_asic_freq]=Fmax[i];
                set_frequency_with_addr_plldatai(Fmax[i],0, each_asic_freq * CHIP_ADDR_INTERVAL,i);

				if(min_Freq>atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq))
					min_Freq=atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq);

				if(max_Freq<atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq))
					max_Freq=atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq);
            }
			break;
#else
    	case SEARCH_BASE_FREQ:
			chain_DataCount[i]=SEARCH_BASEFREQ_PATTEN_NUM;	// when seaching base freq, we use 8*144 patten on chip
			chain_ValidNonce[i]=SEARCH_BASEFREQ_NONCE_NUM;
			chain_PassCount[i]=SEARCH_BASEFREQ_PATTEN_NUM;

			sprintf(logstr,"SEARCH_BASE_FREQ mode set freq=%s on chain[%d]\n",freq_pll_1385[Fmax[i]].freq, i);
			writeLogFile(logstr);

			for(each_asic_freq = 0; each_asic_freq < ASIC_NUM; each_asic_freq ++)
            {
            	last_freq[i][each_asic_freq]=Fmax[i];
                set_frequency_with_addr_plldatai(Fmax[i],0, each_asic_freq * CHIP_ADDR_INTERVAL,i);

				if(min_Freq>atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq))
					min_Freq=atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq);

				if(max_Freq<atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq))
					max_Freq=atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq);
            }
			break;
#endif

#ifdef USE_DOWN_VOLTAGE_LIMIT_FREQ
		case DOWN_VOLTAGE_TEST:
			chain_DataCount[i]=SEARCH_BASEFREQ_PATTEN_NUM;	// when seaching base freq, we use 8*144 patten on chip
			chain_ValidNonce[i]=SEARCH_BASEFREQ_NONCE_NUM;
			chain_PassCount[i]=SEARCH_BASEFREQ_PATTEN_NUM;
			sprintf(logstr,"DOWN_VOLTAGE_TEST mode on chain[%d] voltage=%d\n", i, chain_vol_value[i]);
			writeLogFile(logstr);

			for(each_asic_freq = 0; each_asic_freq < ASIC_NUM; each_asic_freq ++)
            {
                set_frequency_with_addr_plldatai(last_freq[i][each_asic_freq], 0, each_asic_freq * CHIP_ADDR_INTERVAL,i);

				if(min_Freq>atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq))
					min_Freq=atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq);

				if(max_Freq<atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq))
					max_Freq=atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq);
            }
			break;
#endif

		case ALLCHIP_FREQ_UP:
			chain_DataCount[i]=SEARCH_BASEFREQ_PATTEN_NUM;	// when seaching base freq, we use 8*144 patten on chip
			chain_ValidNonce[i]=SEARCH_BASEFREQ_NONCE_NUM;
			chain_PassCount[i]=SEARCH_BASEFREQ_PATTEN_NUM;
			sprintf(logstr,"ALLCHIP_FREQ_UP mode Fmax+1=%s on chain[%d]\n",freq_pll_1385[Fmax[i]+1].freq, i);
			writeLogFile(logstr);

			for(each_asic_freq = 0; each_asic_freq < ASIC_NUM; each_asic_freq ++)
            {
            	last_freq[i][each_asic_freq]=Fmax[i]+1;
                set_frequency_with_addr_plldatai(Fmax[i]+1, 0, each_asic_freq * CHIP_ADDR_INTERVAL,i);

				if(min_Freq>atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq))
					min_Freq=atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq);

				if(max_Freq<atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq))
					max_Freq=atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq);
            }
			break;
		case FAILED_CHIP_DOWN:
			chain_DataCount[i]=SEARCH_FREQ_PATTEN_NUM;	// only when seaching base freq, we use 8*144 patten on chip
			chain_ValidNonce[i]=SEARCH_FREQ_NONCE_NUM;
			chain_PassCount[i]=SEARCH_FREQ_PATTEN_NUM;
			
			sprintf(logstr,"FAILED_CHIP_DOWN mode Fmax+1=%s on chain[%d]\n",freq_pll_1385[Fmax[i]+1].freq, i);
			writeLogFile(logstr);

			found_one_chip=false;
			for(each_asic_freq = 0; each_asic_freq < ASIC_NUM; each_asic_freq ++)
            {
            	// only down failed and with high step freq chips
            	if((!last_result[i][each_asic_freq]) && last_freq[i][each_asic_freq]==Fmax[i]+1)
            	{
            		found_one_chip=true;
            		last_freq[i][each_asic_freq]=Fmax[i];

				//	sprintf(logstr,"chip[%d] is failed back to freq=%s on Chain[%d]\n",each_asic_freq,freq_pll_1385[Fmax[i]].freq,i);
				//	writeLogFile(logstr);
            	}
            }

			if(found_one_chip)
			{
				for(each_asic_freq = 0; each_asic_freq < ASIC_NUM; each_asic_freq ++)
            	{
            	    set_frequency_with_addr_plldatai(last_freq[i][each_asic_freq], 0, each_asic_freq * CHIP_ADDR_INTERVAL,i);

					if(min_Freq>atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq))
						min_Freq=atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq);

					if(max_Freq<atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq))
						max_Freq=atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq);
            	}
			}
			else
			{
				sprintf(logstr,"FAILED_CHIP_DOWN mode Fmax=%s There is no chip can down on chain[%d], Will enter DOWN_CHIP_ONEBYONE\n",freq_pll_1385[Fmax[i]].freq, i);
				writeLogFile(logstr);
			
				searchFreqMode[i]=DOWN_CHIP_ONEBYONE;

				testDone[i]=true;	// this time ,test done.
			}
			break;
		case DOWN_CHIP_ONEBYONE:
			chain_DataCount[i]=SEARCH_FREQ_PATTEN_NUM;	// when seaching base freq, we use 8*144 patten on chip
			chain_ValidNonce[i]=SEARCH_FREQ_NONCE_NUM;
			chain_PassCount[i]=SEARCH_FREQ_PATTEN_NUM;
			
			sprintf(logstr,"DOWN_CHIP_ONEBYONE mode Fmax=%s on chain[%d]\n",freq_pll_1385[Fmax[i]].freq, i);
			writeLogFile(logstr);

			found_one_chip=false;
			for(each_asic_freq = 0; each_asic_freq < ASIC_NUM/DC_AREA_NUM; each_asic_freq ++)
            {
            	for(dc_area = 0; dc_area < DC_AREA_NUM; dc_area ++)
            	{
	            	if(last_freq[i][each_asic_freq+dc_area*(ASIC_NUM/DC_AREA_NUM)]==Fmax[i]+1)
	            	{
	            		last_freq[i][each_asic_freq+dc_area*(ASIC_NUM/DC_AREA_NUM)]=Fmax[i];

					//	sprintf(logstr,"chip[%d] is back to freq=%s\n",each_asic_freq+dc_area*(ASIC_NUM/DC_AREA_NUM),freq_pll_1385[Fmax[i]].freq);
					//	writeLogFile(logstr);

						found_one_chip=true;
						break;
	            	}
            	}
				
				if(found_one_chip)
					break;
            }

			if(!found_one_chip)
			{
				if(last_success_freq[i][0]>0)
				{
					sprintf(logstr,"DOWN_CHIP_ONEBYONE there is no Fmax+1 chip on chain[%d], record last successful freq, and exit!\n",i);
					writeLogFile(logstr);
					save_freq_toPIC(i,last_success_freq[i]);

					search_over[i]=true;
					searchFreqMode[i]=SEARCH_OVER;
					testDone[i]=true;
				}
				else
				{	// never come here
					sprintf(logstr,"DOWN_CHIP_ONEBYONE Fatal Error: there is no Fmax+1 chip on chain[%d], and no last successful freq, and exit!\n",i);
					writeLogFile(logstr);
					
					search_freq_result[i]=false;

					searchFreqMode[i]=SEARCH_OVER;
					search_over[i]=true;

					testDone[i]=true;
				}
			}
			else
			{
				for(each_asic_freq = 0; each_asic_freq < ASIC_NUM; each_asic_freq ++)
				{
					set_frequency_with_addr_plldatai(last_freq[i][each_asic_freq], 0, each_asic_freq * CHIP_ADDR_INTERVAL,i);
					
					if(min_Freq>atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq))
						min_Freq=atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq);

					if(max_Freq<atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq))
						max_Freq=atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq);
				}
			}
			
			break;
		case SUCCESS_CHIP_UP:
			chain_DataCount[i]=SEARCH_FREQ_PATTEN_NUM;	// when seaching base freq, we use 8*144 patten on chip
			chain_ValidNonce[i]=SEARCH_FREQ_NONCE_NUM;
			chain_PassCount[i]=SEARCH_FREQ_PATTEN_NUM;
			
			sprintf(logstr,"SUCCESS_CHIP_UP mode Fmax=%s on chain[%d]\n",freq_pll_1385[Fmax[i]].freq, i);
			writeLogFile(logstr);

			found_one_chip=false;
			for(each_asic_freq = 0; each_asic_freq < ASIC_NUM; each_asic_freq ++)
            {
            	if(last_freq[i][each_asic_freq]==Fmax[i])
            	{
            		last_freq[i][each_asic_freq]=Fmax[i]+1;

				//	sprintf(logstr,"chip[%d] is up to freq=%s\n",each_asic_freq,freq_pll_1385[Fmax[i]+1].freq);
				//	writeLogFile(logstr);

					found_one_chip=true;
            	}
				
                set_frequency_with_addr_plldatai(last_freq[i][each_asic_freq], 0, each_asic_freq * CHIP_ADDR_INTERVAL,i);
				if(min_Freq>atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq))
					min_Freq=atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq);

				if(max_Freq<atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq))
					max_Freq=atoi(freq_pll_1385[last_freq[i][each_asic_freq]].freq);
            }

			if(!found_one_chip)
			{
				// there is no chips can up
				if(last_success_freq[i][0]>0)
				{
					sprintf(logstr,"SUCCESS_CHIP_UP there is no chip can up on chain[%d], record last successful freq, and exit!\n",i);
					writeLogFile(logstr);
					save_freq_toPIC(i,last_success_freq[i]);
					search_over[i]=true;
					searchFreqMode[i]=SEARCH_OVER;
					testDone[i]=true;
				}
				else
				{	// never come here
					sprintf(logstr,"SUCCESS_CHIP_UP Fatal Error: there is no chip can up on chain[%d], and no last successful freq, and exit!\n",i);
					writeLogFile(logstr);
					
					search_freq_result[i]=false;

					searchFreqMode[i]=SEARCH_OVER;
					search_over[i]=true;
					testDone[i]=true;
				}
			}
			break;
		case SEARCH_OVER:
			// do nothing !!!!
			testDone[i]=true;
			sprintf(logstr,"search over on chain[%d]\n",i);
			writeLogFile(logstr);
			break;
		default:
			searchFreqMode[i]=SEARCH_OVER;
			search_over[i]=true;
			testDone[i]=true;
			sprintf(logstr,"Fatal Error: unkown search mode=%d on chain[%d]\n",searchFreqMode[i],i);
			writeLogFile(logstr);
			break;
    	}
	}

	if(first_freq)
    {
        first_freq = false;
#ifdef NEED_PRE_HEAT
		need_heat=1;	// first time to run
#endif
    }
	
#if 1
	if(ASIC_NUM != 1)
	{
		int temp_corenum = calculate_core_number(ASIC_CORE_NUM);
		conf.timeout = 0x1000000/temp_corenum*CHIP_ADDR_INTERVAL/min_Freq*TIMEOUT_PERCENT/100;	// 7% timeout

		sprintf(logstr,"The min freq=%d\n",min_Freq);
		writeLogFile(logstr);
		
		sprintf(logstr,"set real timeout %d, need sleep=%d\n", conf.timeout, conf.timeout*(conf.ValidNonce1+conf.passCount1));
		writeLogFile(logstr);
		
		set_time_out_control(conf.timeout);
	}
#else
	//set timeout for open core
	printf("\n--- set timeout %d\n", conf.OpenCoreGap);
	set_time_out_control(conf.OpenCoreGap);
#endif

#ifdef ENABLE_HIGH_VOLTAGE_OPENCORE
	// set highest voltage to open core
	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
    {
    	unsigned char vol_pic;
		if(cgpu.chain_exist[i]==0 
#ifndef ALL_CHAIN_INTEST
			|| testDone[i]
#endif
			)
			continue;

		vol_pic=getPICvoltageFromValue(HIGHEST_VOLTAGE_LIMITED_HW);

#ifdef T9_18
		sprintf(logstr,"set voltage=%d[%d] on chain[%d] to open core...\n",HIGHEST_VOLTAGE_LIMITED_HW,vol_pic,i);
		writeLogFile(logstr);
		
		set_voltage_T9_18_into_PIC(i, vol_pic);
#else
		set_pic_voltage(i, vol_pic);
#endif
		sleep(1);

#ifdef USE_OPENCORE_ONEBYONE
		opencore_onebyone_onChain(i);
#else
		open_core_onChain(i,ASIC_CORE_NUM,ASIC_CORE_NUM,true);
#endif

#ifdef USE_OPENCORE_TWICE
		open_core_onChain(i,ASIC_CORE_NUM,ASIC_CORE_NUM,true);
#endif

		vol_pic=getPICvoltageFromValue(chain_vol_value[i]);

#ifdef T9_18
		if(fpga_version>=0xE)
		{
			// when finish open core on one hashbaord, we need switch to normal voltage!!! 
			if(i==9 
#ifndef ALL_CHAIN_INTEST
				|| (i==1 && testDone[8] && testDone[9]) || (i==8 && testDone[9])
#endif
				)
			{
				sprintf(logstr,"set working voltage=%d[%d] on chain[%d]\n",chain_vol_value[i],vol_pic,i);
				writeLogFile(logstr);
		
				set_pic_voltage(i, vol_pic);
			}
			else if(i==11 
#ifndef ALL_CHAIN_INTEST
				|| (i==2 && testDone[10] && testDone[11]) || (i==10 && testDone[11])
#endif
				)
			{
				sprintf(logstr,"set working voltage=%d[%d] on chain[%d]\n",chain_vol_value[i],vol_pic,i);
				writeLogFile(logstr);
				
				set_pic_voltage(i, vol_pic);
			}
			else if(i==13 
#ifndef ALL_CHAIN_INTEST
				|| (i==3 && testDone[12] && testDone[13]) || (i==12 && testDone[13])
#endif
				)
			{
				sprintf(logstr,"set working voltage=%d[%d] on chain[%d]\n",chain_vol_value[i],vol_pic,i);
				writeLogFile(logstr);
				
				set_pic_voltage(i, vol_pic);
			}
		}
		else
		{
			if(i%3==2 
#ifndef ALL_CHAIN_INTEST
				|| (i%3==0 && testDone[(i/3)*3+1] && testDone[(i/3)*3+2]) || (i%3==1 && testDone[(i/3)*3+2])
#endif
				)
				set_pic_voltage(i, vol_pic);
		}
#else
		set_pic_voltage(i, vol_pic);
#endif
	}
#else
	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
	{
		unsigned char vol_pic;
		if(cgpu.chain_exist[i]==0 
#ifndef ALL_CHAIN_INTEST
			|| testDone[i]
#endif
			)
			continue;

		vol_pic=getPICvoltageFromValue(chain_vol_value[i]);
		set_pic_voltage(i, vol_pic);	// T9_18 will ignore vol_pic, will use highest voltage of chain_vol_value[i] 
	}

	sleep(3);

	//open core
	open_core(true);
#endif

#ifdef SET_TICKETMASK_BEFORE_TEST
//	sprintf(logstr,"DEBUG before test: check TICKET_MASK:\n");
//	writeLogFile(logstr);
//	check_asic_reg(TICKET_MASK);
//	check_asic_reg(HASH_COUNTING_NUMBER);
	sleep(5);
	set_tickmask(0);
	set_hcnt(0);
	sleep(5);
#endif

#ifdef ENABLE_TEMP_PROCESS
	gIsReadTemp = true;
#endif

#if 0
    if(ASIC_NUM != 1)
    {
	    int temp_corenum = calculate_core_number(ASIC_CORE_NUM);
    	conf.timeout = 0x1000000/temp_corenum*CHIP_ADDR_INTERVAL/min_Freq*TIMEOUT_PERCENT/100;	// 7% timeout

		sprintf(logstr,"The min freq=%d\n",min_Freq);
		writeLogFile(logstr);
		
		sprintf(logstr,"set real timeout %d, need sleep=%d\n", conf.timeout, conf.timeout*(conf.ValidNonce1+conf.passCount1));
		writeLogFile(logstr);
		
        set_time_out_control(conf.timeout);
    }
#endif

    reset_work_data();
	
#ifdef NEED_PRE_HEAT
	if(need_heat)
	{
		set_PWM(10);
		sprintf(logstr,"wait for becoming heat...\n");
		writeLogFile(logstr);
		while(!gStartTest)
			usleep(1000000);
		sprintf(logstr,"becoming heat Over!\n");
		writeLogFile(logstr);
		set_PWM(80);
	}
#endif

	// before the first time for sending work, reset the FPGA's nonce fifo
    if(!gBegin_get_nonce)
    {
        //printf("\n--- clear nonce fifo before send work\n");
        printf("clement2 set_nonce_fifo_interrupt\n");
        set_nonce_fifo_interrupt(get_nonce_fifo_interrupt() | FLUSH_NONCE3_FIFO);
        gBegin_get_nonce = true;
    }

    for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
    {
    	if(cgpu.chain_exist[i]==0 || testDone[i])
			continue;

		sprintf(logstr,"start send works on chain[%d]\n",i);
		writeLogFile(logstr);

		StartSendFlag[i]=true;
    }

#ifdef USE_SINGLE_SEND_THREAD
	send_func_all();
#else
	for(i=0; i < BITMAIN_MAX_CHAIN_NUM; i++)
    {
    	if(cgpu.chain_exist[i]==0 || testDone[i])
			continue;

		sprintf(logstr,"wait send works done on chain[%d]\n",i);
		writeLogFile(logstr);

		last_send_num=0;
		wait_count=0;
		while(wait_count<SEND_WAIT_TIMEOUT && send_work_num[i]<chain_ValidNonce[i])
		{
			if(last_send_num!=send_work_num[i])
			{
				wait_count=0;
				last_send_num=send_work_num[i];
			}
			else wait_count++;

			usleep(100000);
		}
		if(wait_count>=SEND_WAIT_TIMEOUT)
		{
			sprintf(logstr,"Some Error to cause sending works timeout on chain[%d] sendExit=%d, %d/%d\n",i,sendExit[i],send_work_num[i],chain_ValidNonce[i]);
			writeLogFile(logstr);
		}

		StartSendFlag[i]=false;
    }
#endif

	for(i=0; i < BITMAIN_MAX_CHAIN_NUM; i++)
    {
    	if(cgpu.chain_exist[i]==0 || testDone[i])
			continue;

		sprintf(logstr,"wait recv nonce on chain[%d]\n",i);
		writeLogFile(logstr);

		last_recv_num=0;
		wait_count=0;
		while(wait_count < RECV_WAIT_TIMEOUT && valid_nonce_num[i]<chain_ValidNonce[i])
		{
			if(last_recv_num!=valid_nonce_num[i])
			{
				wait_count=0;
				last_recv_num=valid_nonce_num[i];
			}
			else wait_count++;

			usleep(100000);
		}
	}

#ifdef CHECK_TICKETMASK_AFTER_TEST
	sprintf(logstr,"DEBUG after test: check TICKET_MASK:\n");
	writeLogFile(logstr);
	check_asic_reg(TICKET_MASK);
	check_asic_reg(HASH_COUNTING_NUMBER);
//	check_asic_reg(PLL_PARAMETER);
#endif

#ifdef DEBUG_READ_TEMP
	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
	{
		if(cgpu.chain_exist[i]==0 || testDone[i])
			continue;

#ifdef T9_18
		if(fpga_version>=0xE)
		{
			if(i!=8 && i!=10 && i!=12)	// only 8,10,12 chain has temp chips
				continue;
		}
		else
		{
			if(i%3!=1)	// only 1,4,7 chain has temp chips
				continue;
		}
#endif
    	board_temp[i]=read_asic_temperature(i);
	}
#endif

#ifdef ENABLE_TEMP_PROCESS
	gIsReadTemp = false;
	pthread_mutex_lock(&read_temp_mutex);
	sprintf(logstr,"read temp thread stop!\n");
	writeLogFile(logstr);
	pthread_mutex_unlock(&read_temp_mutex);

	set_fan_by_temp();
#endif

	gBegin_get_nonce=false;
	start_receive=false;

	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
	{
		if(cgpu.chain_exist[i]==0 || testDone[i])
			continue;

		sprintf(logstr,"get nonces on chain[%d]\n",i);
		writeLogFile(logstr);

    	result = get_result(i, chain_PassCount[i], chain_ValidNonce[i]);
	}

	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
	{
		if(cgpu.chain_exist[i]==0 || testDone[i])
			continue;
	
		switch(searchFreqMode[i])
    	{
#if USE_SEARCH_BASEFREQ_MODE == 2
		case SEARCH_BASE_FREQ_V89_200:
			//save to last_asic_core_enabled_flag
			SaveAsicCoreEnabledFlagByResultToLastRecord(i);

			sprintf(logstr,"SEARCH_BASE_FREQ_V89_200 mode 200M badcore num=%d on chain[%d], goto SEARCH_BASE_FREQ_V89_300\n", getChainLastBadCoreNum(i), i);
			writeLogFile(logstr);
			
			searchFreqMode[i]=ALLCHIP_FREQ_UP;
			break;
			
		case SEARCH_BASE_FREQ_V89_300:
			//get current bad core flag into temp_asic_core_enabled_flag
			SaveAsicCoreEnabledFlagByResultToTempRecord(i);
			sprintf(logstr,"SEARCH_BASE_FREQ_V89_300 mode 300M badcore num=%d on chain[%d]\n", getChainTempBadCoreNum(i), i);
			writeLogFile(logstr);

			if(isSameLastAsicCoreEnabledFlagWithTemp(i))
			{
				sprintf(logstr,"SEARCH_BASE_FREQ_V89_300 mode 300M badcore is same as 200M on chain[%d], goto SEARCH_BASE_FREQ_V89_400\n", i);
				writeLogFile(logstr);
				
				searchFreqMode[i]=ALLCHIP_FREQ_UP;
			}
			else
			{
				if(isChainAsicTempBadCoreCanAccepted(i))
				{
					SaveAsicCoreEnabledFlagByResultToLastRecord(i);

					sprintf(logstr,"SEARCH_BASE_FREQ_V89_300 mode 300M badcore can be accepted on chain[%d], force goto SEARCH_BASE_FREQ_V89_400_2\n", i);
					writeLogFile(logstr);
				
					searchFreqMode[i]=ALLCHIP_FREQ_UP;
				}
				else
				{
					if(isChainAsicLastBadCoreCanAccepted(i))
					{
						// copy 200M badcore flag into asic core enabled flag
						copyAsicCoreEnabledFlagFromLast(i);
						
						sprintf(logstr,"SEARCH_BASE_FREQ_V89_300 200M can accept badcore on chain[%d], goto ALLCHIP_FREQ_UP\n", i);
						writeLogFile(logstr);

						Fmax[i]=4;	//200M
						
						searchFreqMode[i]=ALLCHIP_FREQ_UP;
						base_freq_index[i]=Fmax[i];
						for(j = 0; j < ASIC_NUM; j++)
			            	last_freq[i][j]=Fmax[i];
						
						set_result_all_pass(i);
					}
					else
					{
						sprintf(logstr,"Chain[%d] has too much bad cores, search failed!\n", i);
						writeLogFile(logstr);

						PrintAsicCoreEnabledFlag(i);
						
						search_over[i]=true;
						testDone[i]=true;
						searchFreqMode[i]=SEARCH_OVER;

						search_freq_result[i]=false;

						sprintf(search_failed_info,"J%d:1",i+1);
						saveSearchFailedFlagInfo();
						searchStatus=SEARCH_FAILED;
						while(1)
						{
							processTEST();
							sleep(1);
						}
					}
				}
			}
			break;

		case SEARCH_BASE_FREQ_V89_400_2:
			//get current bad core flag into temp_asic_core_enabled_flag
			SaveAsicCoreEnabledFlagByResultToTempRecord(i);
			sprintf(logstr,"SEARCH_BASE_FREQ_V89_400_2 mode badcore num=%d on chain[%d]\n", getChainTempBadCoreNum(i), i);
			writeLogFile(logstr);

			if(isChainAsicTempBadCoreCanAccepted(i))
			{
				SaveAsicCoreEnabledFlagByResultToLastRecord(i);

				sprintf(logstr,"SEARCH_BASE_FREQ_V89_400_2 mode 400M badcore can be accepted on chain[%d], force goto SEARCH_BASE_FREQ_V89_500\n", i);
				writeLogFile(logstr);
			
				searchFreqMode[i]=ALLCHIP_FREQ_UP;
			}
			else
			{
				// copy 300M badcore flag into asic core enabled flag
				copyAsicCoreEnabledFlagFromLast(i);
				
				sprintf(logstr,"SEARCH_BASE_FREQ_V89_400 use last badcore flag on chain[%d], goto ALLCHIP_FREQ_UP on 300M\n", i);
				writeLogFile(logstr);

				Fmax[i]=8;	//300M
				
				searchFreqMode[i]=ALLCHIP_FREQ_UP;
				base_freq_index[i]=Fmax[i];
				for(j = 0; j < ASIC_NUM; j++)
	            	last_freq[i][j]=Fmax[i];
				
				set_result_all_pass(i);
			}
			break;
			
		case SEARCH_BASE_FREQ_V89_400:
			//get current bad core flag into temp_asic_core_enabled_flag
			SaveAsicCoreEnabledFlagByResultToTempRecord(i);
			sprintf(logstr,"SEARCH_BASE_FREQ_V89_400 mode badcore num=%d on chain[%d]\n", getChainTempBadCoreNum(i), i);
			writeLogFile(logstr);

			if(isSameLastAsicCoreEnabledFlagWithTemp(i))
			{
				sprintf(logstr,"SEARCH_BASE_FREQ_V89_400 mode badcore is same as last on chain[%d], goto SEARCH_BASE_FREQ_V86_400\n", i);
				writeLogFile(logstr);
				
				searchFreqMode[i]=ALLCHIP_FREQ_UP;
			}
			else
			{
				if(isChainAsicTempBadCoreCanAccepted(i))
				{
					SaveAsicCoreEnabledFlagByResultToLastRecord(i);

					sprintf(logstr,"SEARCH_BASE_FREQ_V89_400 mode 400M badcore can be accepted on chain[%d], force goto SEARCH_BASE_FREQ_V89_500\n", i);
					writeLogFile(logstr);
				
					searchFreqMode[i]=ALLCHIP_FREQ_UP;
				}
				else
				{
					// copy 300M badcore flag into asic core enabled flag
					copyAsicCoreEnabledFlagFromLast(i);
					
					sprintf(logstr,"SEARCH_BASE_FREQ_V89_400 use last badcore flag on chain[%d], goto ALLCHIP_FREQ_UP on 300M\n", i);
					writeLogFile(logstr);

					Fmax[i]=8;	//300M
					
					searchFreqMode[i]=ALLCHIP_FREQ_UP;
					base_freq_index[i]=Fmax[i];
					for(j = 0; j < ASIC_NUM; j++)
		            	last_freq[i][j]=Fmax[i];
					
					set_result_all_pass(i);
				}
			}
			break;

		case SEARCH_BASE_FREQ_V86_400:
			//get current bad core flag into temp_asic_core_enabled_flag
			SaveAsicCoreEnabledFlagByResultToTempRecord(i);
			sprintf(logstr,"SEARCH_BASE_FREQ_V86_400 mode badcore num=%d on chain[%d]\n", getChainTempBadCoreNum(i), i);
			writeLogFile(logstr);

			if(isChainAsicTempBadCoreCanAccepted(i))
			{
				SaveAsicCoreEnabledFlagByResultToLastRecord(i);
				
				sprintf(logstr,"SEARCH_BASE_FREQ_V86_400 mode badcore can be accepted on chain[%d], goto SEARCH_BASE_FREQ_V86_500\n", i);
				writeLogFile(logstr);
				
				searchFreqMode[i]=ALLCHIP_FREQ_UP;
			}
			else
			{
				// to faster on searching freq, so we jump to 8.9V 500M
				// do not accept bad cores here, because it is used to switch to high voltage, give miner chance to get higher voltage and higher freq
				sprintf(logstr,"SEARCH_BASE_FREQ_V86_400 mode 400M has too many badcores on chain[%d], force goto SEARCH_BASE_FREQ_V89_500\n", i);
				writeLogFile(logstr);

				searchFreqMode[i]=ALLCHIP_FREQ_UP;
			}
			break;

		case SEARCH_BASE_FREQ_V86_500:
			//get current bad core flag into temp_asic_core_enabled_flag
			SaveAsicCoreEnabledFlagByResultToTempRecord(i);
			sprintf(logstr,"SEARCH_BASE_FREQ_V86_500 mode badcore num=%d on chain[%d]\n", getChainTempBadCoreNum(i), i);
			writeLogFile(logstr);

			if(isChainAsicTempBadCoreCanAccepted(i))
			{
				SaveAsicCoreEnabledFlagByResultToLastRecord(i);

				sprintf(logstr,"SEARCH_BASE_FREQ_V86_500 mode 500M badcore can be accepted on chain[%d], force goto SEARCH_BASE_FREQ_V86_600\n", i);
				writeLogFile(logstr);
			
				searchFreqMode[i]=ALLCHIP_FREQ_UP;
			}
			else
			{
				// copy 400M badcore flag into asic core enabled flag
				copyAsicCoreEnabledFlagFromLast(i);

				Fmax[i]=12;	//400M

				sprintf(logstr,"SEARCH_BASE_FREQ_V86_500 mode use freq=%s voltage=%d on chain[%d], goto ALLCHIP_FREQ_UP\n",freq_pll_1385[Fmax[i]].freq, chain_vol_value[i], i);
				writeLogFile(logstr);
				
				searchFreqMode[i]=ALLCHIP_FREQ_UP;
				base_freq_index[i]=Fmax[i];
				for(j = 0; j < ASIC_NUM; j++)
	            	last_freq[i][j]=Fmax[i];
				
				set_result_all_pass(i);
			}
			break;

		case SEARCH_BASE_FREQ_V86_600:
			//get current bad core flag into temp_asic_core_enabled_flag
			SaveAsicCoreEnabledFlagByResultToTempRecord(i);
			sprintf(logstr,"SEARCH_BASE_FREQ_V86_600 mode badcore num=%d on chain[%d]\n", getChainTempBadCoreNum(i), i);
			writeLogFile(logstr);

			if(isChainAsicTempBadCoreCanAccepted(i))
			{
				SaveAsicCoreEnabledFlagByResultToLastRecord(i);

				sprintf(logstr,"SEARCH_BASE_FREQ_V86_600 mode 600M badcore can be accepted on chain[%d], force goto SEARCH_BASE_FREQ_V86_650\n", i);
				writeLogFile(logstr);
			
				searchFreqMode[i]=ALLCHIP_FREQ_UP;
			}
			else
			{
				// copy 500M badcore flag into asic core enabled flag
				copyAsicCoreEnabledFlagFromLast(i);

				Fmax[i]=44;	//500M

				sprintf(logstr,"SEARCH_BASE_FREQ_V86_600 mode failed. use freq=%s voltage=%d on chain[%d], goto ALLCHIP_FREQ_UP\n",freq_pll_1385[Fmax[i]].freq, chain_vol_value[i], i);
				writeLogFile(logstr);
				
				searchFreqMode[i]=ALLCHIP_FREQ_UP;
				base_freq_index[i]=Fmax[i];
				for(j = 0; j < ASIC_NUM; j++)
	            	last_freq[i][j]=Fmax[i];
				
				set_result_all_pass(i);
			}
			break;

		case SEARCH_BASE_FREQ_V86_650:
			//get current bad core flag into temp_asic_core_enabled_flag
			SaveAsicCoreEnabledFlagByResultToTempRecord(i);
			sprintf(logstr,"SEARCH_BASE_FREQ_V86_650 mode badcore num=%d on chain[%d]\n", getChainTempBadCoreNum(i), i);
			writeLogFile(logstr);

			if(isChainAsicTempBadCoreCanAccepted(i))
			{
				copyAsicCoreEnabledFlagFromTemp(i);

				sprintf(logstr,"SEARCH_BASE_FREQ_V86_650 mode 650M badcore can be accepted on chain[%d], force goto ALLCHIP_FREQ_UP\n", i);
				writeLogFile(logstr);
			
				searchFreqMode[i]=ALLCHIP_FREQ_UP;
				base_freq_index[i]=Fmax[i];
				for(j = 0; j < ASIC_NUM; j++)
	            	last_freq[i][j]=Fmax[i];
				
				set_result_all_pass(i);
			}
			else
			{
				// copy 500M badcore flag into asic core enabled flag
				copyAsicCoreEnabledFlagFromLast(i);

				Fmax[i]=66;	//600M

				sprintf(logstr,"SEARCH_BASE_FREQ_V86_650 mode failed. use freq=%s voltage=%d on chain[%d], goto ALLCHIP_FREQ_UP\n",freq_pll_1385[Fmax[i]].freq, chain_vol_value[i], i);
				writeLogFile(logstr);
				
				searchFreqMode[i]=ALLCHIP_FREQ_UP;
				base_freq_index[i]=Fmax[i];
				for(j = 0; j < ASIC_NUM; j++)
	            	last_freq[i][j]=Fmax[i];
				
				set_result_all_pass(i);
			}
			break;
			
		case SEARCH_BASE_FREQ_V89_500:
			//get current bad core flag into temp_asic_core_enabled_flag
			SaveAsicCoreEnabledFlagByResultToTempRecord(i);
			sprintf(logstr,"SEARCH_BASE_FREQ_V89_500 mode badcore num=%d on chain[%d]\n", getChainTempBadCoreNum(i), i);
			writeLogFile(logstr);

			if(isChainAsicTempBadCoreCanAccepted(i))
			{
				SaveAsicCoreEnabledFlagByResultToLastRecord(i);

				sprintf(logstr,"SEARCH_BASE_FREQ_V89_500 mode 500M badcore can be accepted on chain[%d], force goto SEARCH_BASE_FREQ_V89_600\n", i);
				writeLogFile(logstr);
			
				searchFreqMode[i]=ALLCHIP_FREQ_UP;
			}
			else
			{
				// copy 8.9V 400M badcore flag into asic core enabled flag
				copyAsicCoreEnabledFlagFromLast(i);

				Fmax[i]=12;	//400M

				sprintf(logstr,"SEARCH_BASE_FREQ_V89_500 mode use freq=%s voltage=%d on chain[%d], goto ALLCHIP_FREQ_UP\n",freq_pll_1385[Fmax[i]].freq, chain_vol_value[i], i);
				writeLogFile(logstr);
				
				searchFreqMode[i]=ALLCHIP_FREQ_UP;
				base_freq_index[i]=Fmax[i];
				for(j = 0; j < ASIC_NUM; j++)
	            	last_freq[i][j]=Fmax[i];
				
				set_result_all_pass(i);
			}
			break;

		case SEARCH_BASE_FREQ_V89_600:
			//get current bad core flag into temp_asic_core_enabled_flag
			SaveAsicCoreEnabledFlagByResultToTempRecord(i);
			sprintf(logstr,"SEARCH_BASE_FREQ_V89_600 mode badcore num=%d on chain[%d]\n", getChainTempBadCoreNum(i), i);
			writeLogFile(logstr);

			if(isChainAsicTempBadCoreCanAccepted(i))
			{
				SaveAsicCoreEnabledFlagByResultToLastRecord(i);

				sprintf(logstr,"SEARCH_BASE_FREQ_V89_600 mode 600M badcore can be accepted on chain[%d], force goto SEARCH_BASE_FREQ_V89_650\n", i);
				writeLogFile(logstr);
			
				searchFreqMode[i]=ALLCHIP_FREQ_UP;
			}
			else
			{
				// copy 8.9V 400M badcore flag into asic core enabled flag
				copyAsicCoreEnabledFlagFromLast(i);

				Fmax[i]=44;	//500M

				sprintf(logstr,"SEARCH_BASE_FREQ_V89_600 mode failed. use freq=%s voltage=%d on chain[%d], goto ALLCHIP_FREQ_UP\n",freq_pll_1385[Fmax[i]].freq, chain_vol_value[i], i);
				writeLogFile(logstr);
				
				searchFreqMode[i]=ALLCHIP_FREQ_UP;
				base_freq_index[i]=Fmax[i];
				for(j = 0; j < ASIC_NUM; j++)
	            	last_freq[i][j]=Fmax[i];
				
				set_result_all_pass(i);
			}
			break;

		case SEARCH_BASE_FREQ_V89_650:
			//get current bad core flag into temp_asic_core_enabled_flag
			SaveAsicCoreEnabledFlagByResultToTempRecord(i);
			sprintf(logstr,"SEARCH_BASE_FREQ_V89_650 mode badcore num=%d on chain[%d]\n", getChainTempBadCoreNum(i), i);
			writeLogFile(logstr);

			if(isChainAsicTempBadCoreCanAccepted(i))
			{
				copyAsicCoreEnabledFlagFromTemp(i);

				sprintf(logstr,"SEARCH_BASE_FREQ_V89_650 mode 650M badcore can be accepted on chain[%d], force goto ALLCHIP_FREQ_UP\n", i);
				writeLogFile(logstr);
			
				searchFreqMode[i]=ALLCHIP_FREQ_UP;
				base_freq_index[i]=Fmax[i];
				for(j = 0; j < ASIC_NUM; j++)
	            	last_freq[i][j]=Fmax[i];
				
				set_result_all_pass(i);
			}
			else
			{
				// copy 500M badcore flag into asic core enabled flag
				copyAsicCoreEnabledFlagFromLast(i);

				Fmax[i]=66;	//600M

				sprintf(logstr,"SEARCH_BASE_FREQ_V89_650 mode use freq=%s voltage=%d on chain[%d], goto ALLCHIP_FREQ_UP\n",freq_pll_1385[Fmax[i]].freq, chain_vol_value[i], i);
				writeLogFile(logstr);
				
				searchFreqMode[i]=ALLCHIP_FREQ_UP;
				base_freq_index[i]=Fmax[i];
				for(j = 0; j < ASIC_NUM; j++)
	            	last_freq[i][j]=Fmax[i];
				
				set_result_all_pass(i);
			}
			break;
#elif USE_SEARCH_BASEFREQ_MODE == 3
		case SEARCH_BASE_FREQ_V89_200:
			//save to last_asic_core_enabled_flag
			SaveAsicCoreEnabledFlagByResultToLastRecord(i);

			sprintf(logstr,"SEARCH_BASE_FREQ_V89_200 mode 200M badcore num=%d on chain[%d], goto SEARCH_BASE_FREQ_V89_300\n", getChainLastBadCoreNum(i), i);
			writeLogFile(logstr);
			
			searchFreqMode[i]=ALLCHIP_FREQ_UP;
			break;
			
		case SEARCH_BASE_FREQ_V89_300:
			if(last_all_pass(i))
			{
				// must reset all asic core enabled flags , because maybe 200M has badcore, but 300M has no bad core!
				ResetAsicCoreEnabledFlag(i);
				
				sprintf(logstr,"SEARCH_BASE_FREQ_V89_300 mode testpatten OK on chain[%d], goto SEARCH_BASE_FREQ_V89_400\n", i);
				writeLogFile(logstr);
			
				searchFreqMode[i]=ALLCHIP_FREQ_UP;
			}
			else
			{
				//get current bad core flag into temp_asic_core_enabled_flag
				SaveAsicCoreEnabledFlagByResultToTempRecord(i);
				sprintf(logstr,"SEARCH_BASE_FREQ_V89_300 mode 300M badcore num=%d on chain[%d]\n", getChainTempBadCoreNum(i), i);
				writeLogFile(logstr);

				if(isLastAsicGoodCoreNumLessThanTempAsicGoodCoreNum(i))
				{
					sprintf(logstr,"SEARCH_BASE_FREQ_V89_300 mode 300M badcore is same as 200M on chain[%d], goto SEARCH_BASE_FREQ_V89_400\n", i);
					writeLogFile(logstr);
					
					searchFreqMode[i]=ALLCHIP_FREQ_UP;
				}
				else
				{
					if(isChainAsicTempBadCoreCanAccepted(i))
					{
						SaveAsicCoreEnabledFlagByResultToLastRecord(i);

						sprintf(logstr,"SEARCH_BASE_FREQ_V89_300 mode 300M badcore can be accepted on chain[%d], force goto SEARCH_BASE_FREQ_V89_400\n", i);
						writeLogFile(logstr);
					
						searchFreqMode[i]=ALLCHIP_FREQ_UP;
					}
					else
					{
						if(isChainAsicLastBadCoreCanAccepted(i))
						{
							// copy 200M badcore flag into asic core enabled flag
							copyAsicCoreEnabledFlagFromLast(i);
							
							sprintf(logstr,"SEARCH_BASE_FREQ_V89_300 200M can accept badcore on chain[%d], goto ALLCHIP_FREQ_UP\n", i);
							writeLogFile(logstr);

							Fmax[i]=4;	//200M
							
							searchFreqMode[i]=ALLCHIP_FREQ_UP;
							base_freq_index[i]=Fmax[i];
							for(j = 0; j < ASIC_NUM; j++)
				            	last_freq[i][j]=Fmax[i];
							
							set_result_all_pass(i);
						}
						else
						{
							sprintf(logstr,"Chain[%d] has too much bad cores, search failed!\n", i);
							writeLogFile(logstr);

							PrintAsicCoreEnabledFlag(i);
							
							search_over[i]=true;
							testDone[i]=true;
							searchFreqMode[i]=SEARCH_OVER;

							search_freq_result[i]=false;

							sprintf(search_failed_info,"J%d:1",i+1);
							saveSearchFailedFlagInfo();
							searchStatus=SEARCH_FAILED;
							while(1)
							{
								processTEST();
								sleep(1);
							}
						}
					}
				}
			}
			break;

		case SEARCH_BASE_FREQ_V89_400:
			//get current bad core flag into temp_asic_core_enabled_flag
			SaveAsicCoreEnabledFlagByResultToTempRecord(i);
			sprintf(logstr,"SEARCH_BASE_FREQ_V89_400 mode badcore num=%d on chain[%d]\n", getChainTempBadCoreNum(i), i);
			writeLogFile(logstr);

			if(isLastAsicGoodCoreNumLessThanTempAsicGoodCoreNum(i))
			{
				sprintf(logstr,"SEARCH_BASE_FREQ_V89_400 mode badcore is same as last on chain[%d], goto SEARCH_BASE_FREQ_V89_500\n", i);
				writeLogFile(logstr);
				
				searchFreqMode[i]=ALLCHIP_FREQ_UP;
			}
			else
			{
				if(isChainAsicTempBadCoreCanAccepted(i))
				{
					SaveAsicCoreEnabledFlagByResultToLastRecord(i);

					sprintf(logstr,"SEARCH_BASE_FREQ_V89_400 mode 400M badcore can be accepted on chain[%d], force goto SEARCH_BASE_FREQ_V89_500\n", i);
					writeLogFile(logstr);
				
					searchFreqMode[i]=ALLCHIP_FREQ_UP;
				}
				else
				{
					// copy 300M badcore flag into asic core enabled flag
					copyAsicCoreEnabledFlagFromLast(i);
					
					sprintf(logstr,"SEARCH_BASE_FREQ_V89_400 use last badcore flag on chain[%d], goto ALLCHIP_FREQ_UP on 300M\n", i);
					writeLogFile(logstr);

					Fmax[i]=8;	//300M
					
					searchFreqMode[i]=ALLCHIP_FREQ_UP;
					base_freq_index[i]=Fmax[i];
					for(j = 0; j < ASIC_NUM; j++)
		            	last_freq[i][j]=Fmax[i];
					
					set_result_all_pass(i);
				}
			}
			break;

		case SEARCH_BASE_FREQ_V89_500:
			//get current bad core flag into temp_asic_core_enabled_flag
			SaveAsicCoreEnabledFlagByResultToTempRecord(i);
			sprintf(logstr,"SEARCH_BASE_FREQ_V89_500 mode badcore num=%d on chain[%d]\n", getChainTempBadCoreNum(i), i);
			writeLogFile(logstr);

			if(isLastAsicGoodCoreNumLessThanTempAsicGoodCoreNum(i))
			{
				sprintf(logstr,"SEARCH_BASE_FREQ_V89_500 mode badcore is same as last on chain[%d], force goto SEARCH_BASE_FREQ_V86_500\n", i);
				writeLogFile(logstr);
				
				searchFreqMode[i]=ALLCHIP_FREQ_UP;
			}
			else
			{
				if(isChainAsicTempBadCoreCanAccepted(i))
				{
					SaveAsicCoreEnabledFlagByResultToLastRecord(i);

					sprintf(logstr,"SEARCH_BASE_FREQ_V89_500 mode 500M badcore can be accepted on chain[%d], force goto SEARCH_BASE_FREQ_V86_500\n", i);
					writeLogFile(logstr);
				
					searchFreqMode[i]=ALLCHIP_FREQ_UP;
				}
				else
				{
					// copy 8.9V 400M badcore flag into asic core enabled flag
					copyAsicCoreEnabledFlagFromLast(i);

					Fmax[i]=12;	//400M

					sprintf(logstr,"SEARCH_BASE_FREQ_V89_500 mode use freq=%s voltage=%d on chain[%d], goto ALLCHIP_FREQ_UP\n",freq_pll_1385[Fmax[i]].freq, chain_vol_value[i], i);
					writeLogFile(logstr);
					
					searchFreqMode[i]=ALLCHIP_FREQ_UP;
					base_freq_index[i]=Fmax[i];
					for(j = 0; j < ASIC_NUM; j++)
		            	last_freq[i][j]=Fmax[i];
					
					set_result_all_pass(i);
				}
			}
			break;

		case SEARCH_BASE_FREQ_V89_600:
			//get current bad core flag into temp_asic_core_enabled_flag
			SaveAsicCoreEnabledFlagByResultToTempRecord(i);
			sprintf(logstr,"SEARCH_BASE_FREQ_V89_600 mode badcore num=%d on chain[%d]\n", getChainTempBadCoreNum(i), i);
			writeLogFile(logstr);

			if(isLastAsicGoodCoreNumLessThanTempAsicGoodCoreNum(i))
			{
				copyAsicCoreEnabledFlagFromTemp(i);
				
				sprintf(logstr,"SEARCH_BASE_FREQ_V89_600 mode badcore is same as last on chain[%d], goto ALLCHIP_FREQ_UP\n", i);
				writeLogFile(logstr);
				
				searchFreqMode[i]=ALLCHIP_FREQ_UP;
				base_freq_index[i]=Fmax[i];
				for(j = 0; j < ASIC_NUM; j++)
	            	last_freq[i][j]=Fmax[i];
				
				set_result_all_pass(i);
			}
			else
			{
				if(isChainAsicTempBadCoreCanAccepted(i))
				{
					copyAsicCoreEnabledFlagFromTemp(i);
				
					sprintf(logstr,"SEARCH_BASE_FREQ_V89_600 mode badcore is same as last on chain[%d], goto ALLCHIP_FREQ_UP\n", i);
					writeLogFile(logstr);
					
					searchFreqMode[i]=ALLCHIP_FREQ_UP;
					base_freq_index[i]=Fmax[i];
					for(j = 0; j < ASIC_NUM; j++)
		            	last_freq[i][j]=Fmax[i];
					
					set_result_all_pass(i);
				}
				else
				{
					// copy 500M badcore flag into asic core enabled flag
					copyAsicCoreEnabledFlagFromLast(i);

					Fmax[i]=44;	//500M

					sprintf(logstr,"SEARCH_BASE_FREQ_V89_600 mode use freq=%s voltage=%d on chain[%d], goto ALLCHIP_FREQ_UP\n",freq_pll_1385[Fmax[i]].freq, chain_vol_value[i], i);
					writeLogFile(logstr);
					
					searchFreqMode[i]=ALLCHIP_FREQ_UP;
					base_freq_index[i]=Fmax[i];
					for(j = 0; j < ASIC_NUM; j++)
		            	last_freq[i][j]=Fmax[i];
					
					set_result_all_pass(i);
				}
			}
			break;

		case SEARCH_BASE_FREQ_V86_600:
			//get current bad core flag into temp_asic_core_enabled_flag
			SaveAsicCoreEnabledFlagByResultToTempRecord(i);
			sprintf(logstr,"SEARCH_BASE_FREQ_V86_600 mode badcore num=%d on chain[%d]\n", getChainTempBadCoreNum(i), i);
			writeLogFile(logstr);

			if(isLastAsicGoodCoreNumLessThanTempAsicGoodCoreNum(i))
			{
				copyAsicCoreEnabledFlagFromTemp(i);
				
				sprintf(logstr,"SEARCH_BASE_FREQ_V86_600 mode badcore is same as last on chain[%d], goto ALLCHIP_FREQ_UP\n", i);
				writeLogFile(logstr);
				
				searchFreqMode[i]=ALLCHIP_FREQ_UP;
				base_freq_index[i]=Fmax[i];
				for(j = 0; j < ASIC_NUM; j++)
	            	last_freq[i][j]=Fmax[i];
				
				set_result_all_pass(i);
			}
			else
			{
				if(isChainAsicTempBadCoreCanAccepted(i))
				{
					copyAsicCoreEnabledFlagFromTemp(i);

					sprintf(logstr,"SEARCH_BASE_FREQ_V86_600 mode 600M badcore can be accepted on chain[%d], force goto ALLCHIP_FREQ_UP\n", i);
					writeLogFile(logstr);
				
					searchFreqMode[i]=ALLCHIP_FREQ_UP;
					base_freq_index[i]=Fmax[i];
					for(j = 0; j < ASIC_NUM; j++)
		            	last_freq[i][j]=Fmax[i];
					
					set_result_all_pass(i);
				}
				else
				{
					// copy 8.9V 600M badcore flag into asic core enabled flag
					copyAsicCoreEnabledFlagFromLast(i);

					Fmax[i]=44;	//500M   just use same voltage and back to 500M

					sprintf(logstr,"SEARCH_BASE_FREQ_V86_600 mode failed, will use freq=%s voltage=%d on chain[%d], goto ALLCHIP_FREQ_UP\n",freq_pll_1385[Fmax[i]].freq, chain_vol_value[i], i);
					writeLogFile(logstr);
					
					searchFreqMode[i]=ALLCHIP_FREQ_UP;
					base_freq_index[i]=Fmax[i];
					for(j = 0; j < ASIC_NUM; j++)
		            	last_freq[i][j]=Fmax[i];
					
					set_result_all_pass(i);
				}
			}
			break;

		case SEARCH_BASE_FREQ_V86_500:
			//get current bad core flag into temp_asic_core_enabled_flag
			SaveAsicCoreEnabledFlagByResultToTempRecord(i);
			sprintf(logstr,"SEARCH_BASE_FREQ_V86_500 mode badcore num=%d on chain[%d]\n", getChainTempBadCoreNum(i), i);
			writeLogFile(logstr);

			if(isLastAsicGoodCoreNumLessThanTempAsicGoodCoreNum(i))
			{
				sprintf(logstr,"SEARCH_BASE_FREQ_V86_500 mode badcore is same as last on chain[%d], goto SEARCH_BASE_FREQ_V86_600\n", i);
				writeLogFile(logstr);
				
				searchFreqMode[i]=ALLCHIP_FREQ_UP;
			}
			else
			{
				if(isChainAsicTempBadCoreCanAccepted(i))
				{
					SaveAsicCoreEnabledFlagByResultToLastRecord(i);

					sprintf(logstr,"SEARCH_BASE_FREQ_V86_500 mode badcore can be accepted on chain[%d], force goto SEARCH_BASE_FREQ_V86_600\n", i);
					writeLogFile(logstr);
				
					searchFreqMode[i]=ALLCHIP_FREQ_UP;
				}
				else
				{
					sprintf(logstr,"SEARCH_BASE_FREQ_V86_500 mode has too many badcores on chain[%d], force goto SEARCH_BASE_FREQ_V89_600\n", i);
					writeLogFile(logstr);

					searchFreqMode[i]=ALLCHIP_FREQ_UP;
				}
			}
			break;

#elif USE_SEARCH_BASEFREQ_MODE == 4
		case SEARCH_BASE_FREQ_V86_650:
			//get current bad core flag into temp_asic_core_enabled_flag
			SaveAsicCoreEnabledFlagByResultToTempRecord(i);
			sprintf(logstr,"SEARCH_BASE_FREQ_V86_650 mode badcore num=%d on chain[%d]\n", getChainTempBadCoreNum(i), i);
			writeLogFile(logstr);

			if(last_all_pass(i))
			{
				// copy badcore flag into asic core enabled flag
				copyAsicCoreEnabledFlagFromTemp(i);
				
				sprintf(logstr,"SEARCH_BASE_FREQ_V86_650 use current badcore flag on chain[%d], goto ALLCHIP_FREQ_UP on 650M\n", i);
				writeLogFile(logstr);
				
				searchFreqMode[i]=ALLCHIP_FREQ_UP;
				base_freq_index[i]=Fmax[i];
				for(j = 0; j < ASIC_NUM; j++)
	            	last_freq[i][j]=Fmax[i];
			}
			else
			{
				if(isChainAsicTempBadCoreCanAccepted(i))
				{
					// copy badcore flag into asic core enabled flag
					copyAsicCoreEnabledFlagFromTemp(i);
					
					sprintf(logstr,"SEARCH_BASE_FREQ_V86_650 use current badcore flag on chain[%d], goto ALLCHIP_FREQ_UP on 650M\n", i);
					writeLogFile(logstr);
					
					searchFreqMode[i]=ALLCHIP_FREQ_UP;
					base_freq_index[i]=Fmax[i];
					for(j = 0; j < ASIC_NUM; j++)
		            	last_freq[i][j]=Fmax[i];
					
					set_result_all_pass(i);
				}
				else
				{
					sprintf(logstr,"SEARCH_BASE_FREQ_V86_650 mode has too many badcore on chain[%d], goto SEARCH_BASE_FREQ_V86_600\n", i);
					writeLogFile(logstr);
				
					searchFreqMode[i]=ALLCHIP_FREQ_UP;
				}
			}
			break;

		case SEARCH_BASE_FREQ_V86_600:
			//get current bad core flag into temp_asic_core_enabled_flag
			SaveAsicCoreEnabledFlagByResultToTempRecord(i);
			sprintf(logstr,"SEARCH_BASE_FREQ_V86_600 mode badcore num=%d on chain[%d]\n", getChainTempBadCoreNum(i), i);
			writeLogFile(logstr);

			if(last_all_pass(i))
			{
				// copy badcore flag into asic core enabled flag
				copyAsicCoreEnabledFlagFromTemp(i);
				
				sprintf(logstr,"SEARCH_BASE_FREQ_V86_600 use current badcore flag on chain[%d], goto ALLCHIP_FREQ_UP on 600M\n", i);
				writeLogFile(logstr);
				
				searchFreqMode[i]=ALLCHIP_FREQ_UP;
				base_freq_index[i]=Fmax[i];
				for(j = 0; j < ASIC_NUM; j++)
	            	last_freq[i][j]=Fmax[i];
			}
			else
			{
				if(isChainAsicTempBadCoreCanAccepted(i))
				{
					// copy badcore flag into asic core enabled flag
					copyAsicCoreEnabledFlagFromTemp(i);
					
					sprintf(logstr,"SEARCH_BASE_FREQ_V86_600 use current badcore flag on chain[%d], goto ALLCHIP_FREQ_UP on 600M\n", i);
					writeLogFile(logstr);
					
					searchFreqMode[i]=ALLCHIP_FREQ_UP;
					base_freq_index[i]=Fmax[i];
					for(j = 0; j < ASIC_NUM; j++)
		            	last_freq[i][j]=Fmax[i];
					
					set_result_all_pass(i);
				}
				else
				{
					sprintf(logstr,"SEARCH_BASE_FREQ_V86_600 mode has too many badcore on chain[%d], goto SEARCH_BASE_FREQ_V86_500\n", i);
					writeLogFile(logstr);
				
					searchFreqMode[i]=ALLCHIP_FREQ_UP;
				}
			}
			break;

		case SEARCH_BASE_FREQ_V86_500:
			//get current bad core flag into temp_asic_core_enabled_flag
			SaveAsicCoreEnabledFlagByResultToTempRecord(i);
			sprintf(logstr,"SEARCH_BASE_FREQ_V86_500 mode badcore num=%d on chain[%d]\n", getChainTempBadCoreNum(i), i);
			writeLogFile(logstr);

			if(last_all_pass(i))
			{
				// copy badcore flag into asic core enabled flag
				copyAsicCoreEnabledFlagFromTemp(i);
				
				sprintf(logstr,"SEARCH_BASE_FREQ_V86_500 use current badcore flag on chain[%d], goto ALLCHIP_FREQ_UP on 500M\n", i);
				writeLogFile(logstr);
				
				searchFreqMode[i]=ALLCHIP_FREQ_UP;
				base_freq_index[i]=Fmax[i];
				for(j = 0; j < ASIC_NUM; j++)
	            	last_freq[i][j]=Fmax[i];
			}
			else
			{
				if(isChainAsicTempBadCoreCanAccepted(i))
				{
					// copy badcore flag into asic core enabled flag
					copyAsicCoreEnabledFlagFromTemp(i);
					
					sprintf(logstr,"SEARCH_BASE_FREQ_V86_500 use current badcore flag on chain[%d], goto ALLCHIP_FREQ_UP on 500M\n", i);
					writeLogFile(logstr);
					
					searchFreqMode[i]=ALLCHIP_FREQ_UP;
					base_freq_index[i]=Fmax[i];
					for(j = 0; j < ASIC_NUM; j++)
		            	last_freq[i][j]=Fmax[i];
					
					set_result_all_pass(i);
				}
				else
				{
					sprintf(logstr,"SEARCH_BASE_FREQ_V86_500 mode has too many badcore on chain[%d], goto SEARCH_BASE_FREQ_V89_500\n", i);
					writeLogFile(logstr);
				
					searchFreqMode[i]=ALLCHIP_FREQ_UP;
				}
			}
			break;

		case SEARCH_BASE_FREQ_V89_500:
			//get current bad core flag into temp_asic_core_enabled_flag
			SaveAsicCoreEnabledFlagByResultToTempRecord(i);
			sprintf(logstr,"SEARCH_BASE_FREQ_V89_500 mode badcore num=%d on chain[%d]\n", getChainTempBadCoreNum(i), i);
			writeLogFile(logstr);

			if(last_all_pass(i))
			{
				// copy badcore flag into last asic core enabled flag , can be used when we need back to 8.9V 500M mode 
				SaveAsicCoreEnabledFlagByResultToLastRecord(i);
				
				sprintf(logstr,"SEARCH_BASE_FREQ_V89_500 test OK on chain[%d], goto SEARCH_BASE_FREQ_V89_600\n", i);
				writeLogFile(logstr);
				
				searchFreqMode[i]=ALLCHIP_FREQ_UP;
			}
			else
			{
				if(isChainAsicTempBadCoreCanAccepted(i))
				{
					// copy badcore flag into last asic core enabled flag , can be used when we need back to 8.9V 500M mode 
					SaveAsicCoreEnabledFlagByResultToLastRecord(i);
					
					sprintf(logstr,"SEARCH_BASE_FREQ_V89_500 test OK on chain[%d], goto SEARCH_BASE_FREQ_V89_600\n", i);
					writeLogFile(logstr);
					
					searchFreqMode[i]=ALLCHIP_FREQ_UP;
				}
				else
				{
					sprintf(logstr,"SEARCH_BASE_FREQ_V89_500 mode has too many badcore on chain[%d], goto SEARCH_BASE_FREQ_V89_400\n", i);
					writeLogFile(logstr);
				
					searchFreqMode[i]=ALLCHIP_FREQ_UP;
				}
			}
			break;

		case SEARCH_BASE_FREQ_V89_600:
			//get current bad core flag into temp_asic_core_enabled_flag
			SaveAsicCoreEnabledFlagByResultToTempRecord(i);
			sprintf(logstr,"SEARCH_BASE_FREQ_V89_600 mode badcore num=%d on chain[%d]\n", getChainTempBadCoreNum(i), i);
			writeLogFile(logstr);

			if(last_all_pass(i))
			{
				// copy badcore flag into last asic core enabled flag , can be used when we need back to 8.9V 500M mode 
				SaveAsicCoreEnabledFlagByResultToLastRecord(i);
				
				sprintf(logstr,"SEARCH_BASE_FREQ_V89_600 test OK on chain[%d], goto SEARCH_BASE_FREQ_V89_650\n", i);
				writeLogFile(logstr);
				
				searchFreqMode[i]=ALLCHIP_FREQ_UP;
			}
			else
			{
				if(isChainAsicTempBadCoreCanAccepted(i))
				{
					// copy badcore flag into last asic core enabled flag , can be used when we need back to 8.9V 500M mode 
					SaveAsicCoreEnabledFlagByResultToLastRecord(i);
					
					sprintf(logstr,"SEARCH_BASE_FREQ_V89_600 test OK on chain[%d], goto SEARCH_BASE_FREQ_V89_650\n", i);
					writeLogFile(logstr);
					
					searchFreqMode[i]=ALLCHIP_FREQ_UP;
				}
				else
				{
					// copy last time's badcore flag into asic core enabled flag
					copyAsicCoreEnabledFlagFromLast(i);

					Fmax[i]=44;	//500M

					sprintf(logstr,"SEARCH_BASE_FREQ_V89_600 mode has too many badcores,will use freq=%s voltage=%d on chain[%d], goto ALLCHIP_FREQ_UP\n",freq_pll_1385[Fmax[i]].freq, chain_vol_value[i], i);
					writeLogFile(logstr);
					
					searchFreqMode[i]=ALLCHIP_FREQ_UP;
					base_freq_index[i]=Fmax[i];
					for(j = 0; j < ASIC_NUM; j++)
		            	last_freq[i][j]=Fmax[i];
					
					set_result_all_pass(i);
				}
			}
			break;

		case SEARCH_BASE_FREQ_V89_650:
			//get current bad core flag into temp_asic_core_enabled_flag
			SaveAsicCoreEnabledFlagByResultToTempRecord(i);
			sprintf(logstr,"SEARCH_BASE_FREQ_V89_650 mode badcore num=%d on chain[%d]\n", getChainTempBadCoreNum(i), i);
			writeLogFile(logstr);

			if(last_all_pass(i))
			{
				copyAsicCoreEnabledFlagFromTemp(i);

				sprintf(logstr,"SEARCH_BASE_FREQ_V89_650 mode test OK, use freq=%s voltage=%d on chain[%d], goto ALLCHIP_FREQ_UP\n",freq_pll_1385[Fmax[i]].freq, chain_vol_value[i], i);
				writeLogFile(logstr);
				
				searchFreqMode[i]=ALLCHIP_FREQ_UP;
				base_freq_index[i]=Fmax[i];
				for(j = 0; j < ASIC_NUM; j++)
	            	last_freq[i][j]=Fmax[i];
			}
			else
			{
				if(isChainAsicTempBadCoreCanAccepted(i))
				{
					copyAsicCoreEnabledFlagFromTemp(i);

					sprintf(logstr,"SEARCH_BASE_FREQ_V89_650 mode test OK, use freq=%s voltage=%d on chain[%d], goto ALLCHIP_FREQ_UP\n",freq_pll_1385[Fmax[i]].freq, chain_vol_value[i], i);
					writeLogFile(logstr);
					
					searchFreqMode[i]=ALLCHIP_FREQ_UP;
					base_freq_index[i]=Fmax[i];
					for(j = 0; j < ASIC_NUM; j++)
		            	last_freq[i][j]=Fmax[i];
					
					set_result_all_pass(i);
				}
				else
				{
					// copy last time's badcore flag into asic core enabled flag
					copyAsicCoreEnabledFlagFromLast(i);

					Fmax[i]=66;	//600M

					sprintf(logstr,"SEARCH_BASE_FREQ_V89_650 mode has too many badcores,will use freq=%s voltage=%d on chain[%d], goto ALLCHIP_FREQ_UP\n",freq_pll_1385[Fmax[i]].freq, chain_vol_value[i], i);
					writeLogFile(logstr);
					
					searchFreqMode[i]=ALLCHIP_FREQ_UP;
					base_freq_index[i]=Fmax[i];
					for(j = 0; j < ASIC_NUM; j++)
		            	last_freq[i][j]=Fmax[i];
					
					set_result_all_pass(i);
				}
			}
			break;

		case SEARCH_BASE_FREQ_V89_400:
			//get current bad core flag into temp_asic_core_enabled_flag
			SaveAsicCoreEnabledFlagByResultToTempRecord(i);
			sprintf(logstr,"SEARCH_BASE_FREQ_V89_400 mode badcore num=%d on chain[%d]\n", getChainTempBadCoreNum(i), i);
			writeLogFile(logstr);

			if(last_all_pass(i))
			{
				// copy badcore flag into asic core enabled flag
				copyAsicCoreEnabledFlagFromTemp(i);
				
				sprintf(logstr,"SEARCH_BASE_FREQ_V89_400 use current badcore flag on chain[%d], goto ALLCHIP_FREQ_UP on 400M\n", i);
				writeLogFile(logstr);
				
				searchFreqMode[i]=ALLCHIP_FREQ_UP;
				base_freq_index[i]=Fmax[i];
				for(j = 0; j < ASIC_NUM; j++)
	            	last_freq[i][j]=Fmax[i];
			}
			else
			{
				if(isChainAsicTempBadCoreCanAccepted(i))
				{
					// copy badcore flag into asic core enabled flag
					copyAsicCoreEnabledFlagFromTemp(i);
					
					sprintf(logstr,"SEARCH_BASE_FREQ_V89_400 use current badcore flag on chain[%d], goto ALLCHIP_FREQ_UP on 400M\n", i);
					writeLogFile(logstr);
					
					searchFreqMode[i]=ALLCHIP_FREQ_UP;
					base_freq_index[i]=Fmax[i];
					for(j = 0; j < ASIC_NUM; j++)
		            	last_freq[i][j]=Fmax[i];
					
					set_result_all_pass(i);
				}
				else
				{
					sprintf(logstr,"SEARCH_BASE_FREQ_V89_400 mode has too many badcore on chain[%d], goto SEARCH_BASE_FREQ_V89_300\n", i);
					writeLogFile(logstr);
				
					searchFreqMode[i]=ALLCHIP_FREQ_UP;
				}
			}
			break;

		case SEARCH_BASE_FREQ_V89_300:
			//get current bad core flag into temp_asic_core_enabled_flag
			SaveAsicCoreEnabledFlagByResultToTempRecord(i);
			sprintf(logstr,"SEARCH_BASE_FREQ_V89_300 mode badcore num=%d on chain[%d]\n", getChainTempBadCoreNum(i), i);
			writeLogFile(logstr);

			if(last_all_pass(i))
			{
				// copy badcore flag into asic core enabled flag
				copyAsicCoreEnabledFlagFromTemp(i);
				
				sprintf(logstr,"SEARCH_BASE_FREQ_V89_300 use current badcore flag on chain[%d], goto ALLCHIP_FREQ_UP on 300M\n", i);
				writeLogFile(logstr);
				
				searchFreqMode[i]=ALLCHIP_FREQ_UP;
				base_freq_index[i]=Fmax[i];
				for(j = 0; j < ASIC_NUM; j++)
	            	last_freq[i][j]=Fmax[i];
			}
			else
			{
				if(isChainAsicTempBadCoreCanAccepted(i))
				{
					// copy badcore flag into asic core enabled flag
					copyAsicCoreEnabledFlagFromTemp(i);
					
					sprintf(logstr,"SEARCH_BASE_FREQ_V89_300 use current badcore flag on chain[%d], goto ALLCHIP_FREQ_UP on 300M\n", i);
					writeLogFile(logstr);
					
					searchFreqMode[i]=ALLCHIP_FREQ_UP;
					base_freq_index[i]=Fmax[i];
					for(j = 0; j < ASIC_NUM; j++)
		            	last_freq[i][j]=Fmax[i];
					
					set_result_all_pass(i);
				}
				else
				{
					sprintf(logstr,"SEARCH_BASE_FREQ_V89_300 mode has too many badcore on chain[%d], goto SEARCH_BASE_FREQ_V89_200\n", i);
					writeLogFile(logstr);
				
					searchFreqMode[i]=ALLCHIP_FREQ_UP;
				}
			}
			break;

		case SEARCH_BASE_FREQ_V89_200:
			//get current bad core flag into temp_asic_core_enabled_flag
			SaveAsicCoreEnabledFlagByResultToTempRecord(i);
			sprintf(logstr,"SEARCH_BASE_FREQ_V89_200 mode badcore num=%d on chain[%d]\n", getChainTempBadCoreNum(i), i);
			writeLogFile(logstr);

			if(last_all_pass(i))
			{
				// copy badcore flag into asic core enabled flag
				copyAsicCoreEnabledFlagFromTemp(i);
				
				sprintf(logstr,"SEARCH_BASE_FREQ_V89_200 use current badcore flag on chain[%d], goto ALLCHIP_FREQ_UP on 200M\n", i);
				writeLogFile(logstr);
				
				searchFreqMode[i]=ALLCHIP_FREQ_UP;
				base_freq_index[i]=Fmax[i];
				for(j = 0; j < ASIC_NUM; j++)
	            	last_freq[i][j]=Fmax[i];
			}
			else
			{
				if(isChainAsicTempBadCoreCanAccepted(i))
				{
					// copy badcore flag into asic core enabled flag
					copyAsicCoreEnabledFlagFromTemp(i);
					
					sprintf(logstr,"SEARCH_BASE_FREQ_V89_200 use current badcore flag on chain[%d], goto ALLCHIP_FREQ_UP on 200M\n", i);
					writeLogFile(logstr);
					
					searchFreqMode[i]=ALLCHIP_FREQ_UP;
					base_freq_index[i]=Fmax[i];
					for(j = 0; j < ASIC_NUM; j++)
		            	last_freq[i][j]=Fmax[i];
					
					set_result_all_pass(i);
				}
				else
				{
					sprintf(logstr,"SEARCH_BASE_FREQ_V89_200 mode has too many badcore on chain[%d], FAILED!\n", i);
					writeLogFile(logstr);
				
					search_over[i]=true;
					testDone[i]=true;
					searchFreqMode[i]=SEARCH_OVER;

					search_freq_result[i]=false;

					sprintf(search_failed_info,"J%d:1",i+1);
					saveSearchFailedFlagInfo();
					searchStatus=SEARCH_FAILED;
					while(1)
					{
						processTEST();
						sleep(1);
					}
				}
			}
			break;
#else
    	case SEARCH_BASE_FREQ:
			sprintf(logstr,"SEARCH_BASE_FREQ mode set freq=%s on chain[%d]\n",freq_pll_1385[Fmax[i]].freq, i);
			writeLogFile(logstr);

			if(last_all_pass(i))
			{
				sprintf(logstr,"SEARCH_BASE_FREQ freq=%s on chain[%d]: OK! will Enter ALLCHIP_FREQ_UP and up one step\n",freq_pll_1385[Fmax[i]].freq, i);
				writeLogFile(logstr);

				searchFreqMode[i]=ALLCHIP_FREQ_UP;
				base_freq_index[i]=Fmax[i];
			}
			else
			{
				sprintf(logstr,"SEARCH_BASE_FREQ freq=%s on chain[%d]: FAILED! will down one step\n",freq_pll_1385[Fmax[i]].freq, i);
				writeLogFile(logstr);

				Fmax[i]=getNextSearchBaseFreq(Fmax[i]);

				if(Fmax[i]<LOWEST_FREQ_INDEX)	// less than min freq failed and exit
				{
					if(conf.UseConfigVol)// only use config voltage can have more chances
					{
						// search freq failed!
						search_freq_chances[i]--;
						
						sprintf(logstr,"Search Freq Failed on Chain[%d], base Freq < %sM, has %d chance to search again.\n", i,freq_pll_1385[LOWEST_FREQ_INDEX].freq,search_freq_chances[i]);
						writeLogFile(logstr);

						if(search_freq_chances[i]>0) // only use config voltage can have more chances
						{
							Fmax[i]=44;	// set second chance to 500M
							for(j=0; j<256; j++)
					    	{
					    	    last_result[i][j] = 0 ;
					    	    last_freq[i][j] = Fmax[i];
								last_success_freq[i][j]=0;
					    	}

							search_over[i]=false;
							search_freq_result[i]=true;

							searchFreqMode[i]=SEARCH_BASE_FREQ;
							
							base_freq_index[i]=0;
							testDone[i]=false;
						}
						else
						{
							search_over[i]=true;
							testDone[i]=true;
							searchFreqMode[i]=SEARCH_OVER;

							search_freq_result[i]=false;
						}
					}
					else
					{
						UpdateAsicCoreEnabledFlagByResult(i);	// [chainindex][chipindex][coreindex]=0  disabled, 1:enabled
						
						if(isChainAsicBadCoreCanAccepted(i))
						{
							UpdateTestResultFlag(i,SEARCH_BASEFREQ_PATTEN_NUM); // for at last, need check this result flag to save the freq into PIC
							
							sprintf(logstr,"Record Chain[%d] bad cores and enter ALLCHIP_FREQ_UP.\n", i);
							writeLogFile(logstr);

							PrintAsicCoreEnabledFlag(i);

							Fmax[i]++;	// we need add one, because Fmax[i]--; called once before here.
							
							searchFreqMode[i]=ALLCHIP_FREQ_UP;
							base_freq_index[i]=Fmax[i];
						}
						else
						{
							sprintf(logstr,"Chain[%d] has too much bad cores, search failed!\n", i);
							writeLogFile(logstr);

							PrintAsicCoreEnabledFlag(i);
							
							search_over[i]=true;
							testDone[i]=true;
							searchFreqMode[i]=SEARCH_OVER;

							search_freq_result[i]=false;

							sprintf(search_failed_info,"J%d:1",i+1);
							saveSearchFailedFlagInfo();
							searchStatus=SEARCH_FAILED;
							while(1)
							{
								processTEST();
								sleep(1);
							}
						}
					}
				}

				if(Fmax[i]<RETRY_FREQ_INDEX)
				{
					if(chain_vol_value[i]==RETRY_VOLTAGE)
					{
						if(tryFixAsicCoreEnabledFlagByResult_testMode(i))
						{
							UpdateTestResultFlag(i,SEARCH_BASEFREQ_PATTEN_NUM);

							PrintAsicCoreEnabledFlag(i);
							Fmax[i]++;	// we need add one, because Fmax[i]--; called once before here.
							
							sprintf(logstr,"Record Chain[%d] bad cores and enter ALLCHIP_FREQ_UP freq=%s.\n", i, freq_pll_1385[Fmax[i]].freq);
							writeLogFile(logstr);

							searchFreqMode[i]=ALLCHIP_FREQ_UP;
							base_freq_index[i]=Fmax[i];
						}
					}
					else
					{
						chain_vol_value[i]=RETRY_VOLTAGE;
						vol_pic=getPICvoltageFromValue(chain_vol_value[i]);

						sprintf(logstr,"set voltage=%d [%d] on chain[%d]\n",chain_vol_value[i],vol_pic,i);
						writeLogFile(logstr);

						//set voltage
						set_pic_voltage(i, vol_pic);
						usleep(5000000);

						searchFreqMode[i]=SEARCH_BASE_FREQ;
						base_freq_index[i]=0;
						Fmax[i]=get_plldata_i(1387,conf.freq);
#ifdef DEBUG_MODE
						Fmax[i]==7;
#endif

						sprintf(logstr,"Search Freq < %sM on Chain[%d], will use voltage=%d to search again.\n", freq_pll_1385[RETRY_FREQ_INDEX].freq, i, RETRY_VOLTAGE);
						writeLogFile(logstr);
					}
				}
			}
			break;
#endif

#ifdef USE_DOWN_VOLTAGE_LIMIT_FREQ
		case DOWN_VOLTAGE_TEST:
			if(last_all_pass(i))
			{
				sprintf(logstr,"DOWN_VOLTAGE_TEST freq=%s on chain[%d]: OK! voltage=%d, continue DOWN_VOLTAGE_TEST mode\n",freq_pll_1385[Fmax[i]+1].freq, i, chain_vol_value[i]);
				writeLogFile(logstr);

				searchFreqMode[i]=DOWN_VOLTAGE_TEST;
				chain_vol_value[i]-=10;	// down 0.1V

				vol_pic=getPICvoltageFromValue(chain_vol_value[i]);
				set_pic_voltage(i, vol_pic);
			}
			else
			{
				// will stop search freq, and record last voltage and freq
				chain_vol_value[i]+=10;	// restore last time voltage, add 0.1V

				if(last_success_freq[i][0]>0)
				{
					sprintf(logstr,"DOWN_VOLTAGE_TEST save last_success_freq with last voltage=%d on chain[%d] and exit!\n",chain_vol_value[i],i);
					writeLogFile(logstr);
					save_freq_toPIC(i,last_success_freq[i]);
					search_over[i]=true;
					searchFreqMode[i]=SEARCH_OVER;
					testDone[i]=true;
				}
				else
				{	// never come here
					sprintf(logstr,"DOWN_VOLTAGE_TEST Fatal Error: there is no last_success_freq on chain[%d] and exit!\n",i);
					writeLogFile(logstr);
					
					search_freq_result[i]=false;

					searchFreqMode[i]=SEARCH_OVER;
					search_over[i]=true;
					testDone[i]=true;
				}
			}
			break;
#endif

		case ALLCHIP_FREQ_UP:
			if(last_all_pass(i))
			{
				sprintf(logstr,"ALLCHIP_FREQ_UP freq=%s on chain[%d]: OK! will up one step\n",freq_pll_1385[Fmax[i]+1].freq, i);
				writeLogFile(logstr);

				Fmax[i]++;	// up one step
				base_freq_index[i]=Fmax[i];
			}
			else
			{
				if(Fmax[i]<=ACCEPT_BADCORE_FREQ_INDEX || chain_vol_value[i]>=RETRY_VOLTAGE)
				{
					if(tryFixAsicCoreEnabledFlagByResult_searchMode(i,Fmax[i]))
					{
						UpdateTestResultFlag(i,SEARCH_BASEFREQ_PATTEN_NUM); // for at last, need check this result flag to save the freq into PIC
						
						sprintf(logstr,"ALLCHIP_FREQ_UP freq=%s on chain[%d]: FAILED! will fix bad core num and continue ALLCHIP_FREQ_UP\n",freq_pll_1385[Fmax[i]+1].freq, i);
						writeLogFile(logstr);

						PrintAsicCoreEnabledFlag(i);

						Fmax[i]++;	// up one step
						base_freq_index[i]=Fmax[i];
					}
					else
					{
						// base freq = Fmax, must record into PIC
						base_freq_index[i]=Fmax[i];
						sprintf(logstr,"ALLCHIP_FREQ_UP freq=%s on chain[%d]: FAILED! will Enter FAILED_CHIP_DOWN\n",freq_pll_1385[Fmax[i]+1].freq, i);
						writeLogFile(logstr);
						
						searchFreqMode[i]=ALLCHIP_FREQ_UP;
					}
				}
				else
				{
					// base freq = Fmax, must record into PIC
					base_freq_index[i]=Fmax[i];
					sprintf(logstr,"ALLCHIP_FREQ_UP freq=%s on chain[%d]: FAILED! will Enter FAILED_CHIP_DOWN\n",freq_pll_1385[Fmax[i]+1].freq, i);
					writeLogFile(logstr);
					
					searchFreqMode[i]=ALLCHIP_FREQ_UP;
				}
			}
			break;
		case FAILED_CHIP_DOWN:
			if(last_all_pass(i))
			{
				sprintf(logstr,"FAILED_CHIP_DOWN Fmax=%s on chain[%d]: OK! will Enter SUCCESS_CHIP_UP!\n",freq_pll_1385[Fmax[i]].freq, i);
				writeLogFile(logstr);

				searchFreqMode[i]=FAILED_CHIP_DOWN;
				Fmax[i]++;	// up one step
			}
			else
			{
				sprintf(logstr,"FAILED_CHIP_DOWN Fmax=%s on chain[%d]: FAILED! will Enter DOWN_CHIP_ONEBYONE\n",freq_pll_1385[Fmax[i]].freq, i);
				writeLogFile(logstr);
				
				searchFreqMode[i]=DOWN_CHIP_ONEBYONE;
			}
			break;
		case DOWN_CHIP_ONEBYONE:
			if(last_all_pass(i))
			{
				sprintf(logstr,"DOWN_CHIP_ONEBYONE Fmax=%s on chain[%d]: OK! will Enter SUCCESS_CHIP_UP!\n",freq_pll_1385[Fmax[i]].freq, i);
				writeLogFile(logstr);

				searchFreqMode[i]=DOWN_CHIP_ONEBYONE;
				Fmax[i]++;	// up one step
			}
			else
			{
				sprintf(logstr,"DOWN_CHIP_ONEBYONE Fmax=%s on chain[%d]: FAILED! will continue down\n",freq_pll_1385[Fmax[i]].freq, i);
				writeLogFile(logstr);
			}
			break;
		case SUCCESS_CHIP_UP:
			if(last_all_pass(i))
			{
				sprintf(logstr,"SUCCESS_CHIP_UP freq=%s on chain[%d]: OK! will continue up!\n",freq_pll_1385[Fmax[i]+1].freq, i);
				writeLogFile(logstr);
				
				Fmax[i]++;	// up one step
			}
			else
			{
				sprintf(logstr,"SUCCESS_CHIP_UP Fmax=%s on chain[%d]: FAILED! will Enter FAILED_CHIP_DOWN\n",freq_pll_1385[Fmax[i]+1].freq, i);
				writeLogFile(logstr);
				
				searchFreqMode[i]=SUCCESS_CHIP_UP;
			}
			break;
		default:
			searchFreqMode[i]=SEARCH_OVER;
			search_over[i]=true;
			testDone[i]=true;
			sprintf(logstr,"Fatal Error: unkown search mode=%d on chain[%d]\n",searchFreqMode[i],i);
			writeLogFile(logstr);
			break;
    	}

		if(last_all_pass(i) && (!testDone[i]))
		{
			// record this success freq, for next restore useage.
			for(j=0;j<256;j++)
			{
				last_success_freq[i][j]=last_freq[i][j];
			}
			sprintf(logstr,"Record: success freq records on Chain[%d]\n",i);
			writeLogFile(logstr);

#ifdef USE_DOWN_VOLTAGE_LIMIT_FREQ
			if(searchFreqMode[i]!=DOWN_VOLTAGE_TEST && checkFreqVoltageTooHigh(i))
			{
				sprintf(logstr,"Detect chain[%d]: voltage=%d, enter DOWN_VOLTAGE_TEST mode\n", i, chain_vol_value[i]);
				writeLogFile(logstr);

				searchFreqMode[i]=DOWN_VOLTAGE_TEST;
				chain_vol_value[i]-=10;	// down 0.1V

				vol_pic=getPICvoltageFromValue(chain_vol_value[i]);
				set_pic_voltage(i, vol_pic);
			}
			else
#endif
			if(Fmax[i]>=HIGHEST_FREQ_INDEX)	// larger than HIGHEST_FREQ_INDEX , we just record last success freq
			{
				// search freq failed! will exit and print error
				sprintf(logstr,"Search Freq > %sM on Chain[%d], stop and record freq\n", freq_pll_1385[Fmax[i]].freq, i);
				writeLogFile(logstr);

				save_freq_toPIC(i,last_success_freq[i]);

				search_over[i]=true;
				searchFreqMode[i]=SEARCH_OVER;
				testDone[i]=true;
			}
		}
	}

	sprintf(logstr,"After TEST CRC error counter=%d\n",get_crc_count());
	writeLogFile(logstr);

#ifndef DEBUG_NOT_CHECK_FAN_NUM
	if(conf.force_freq)
	{
		if(!check_fan())
		{
			sprintf(search_failed_info,"F:1");
			
			saveSearchFailedFlagInfo();
			searchStatus=SEARCH_FAILED;
			while(1)
			{
				processTEST();
				sleep(1);
			}
		}
	}
#endif

#ifdef CLOSE_OPEN_DC
#ifdef USE_NEW_RESET_FPGA
	set_reset_allhashboard(1);
#endif
	for(i=0; i < BITMAIN_MAX_CHAIN_NUM; i++)
	{
		if(cgpu.chain_exist[i]==0)
			continue;

		disable_pic_dac(i);
	}

	sleep(1);

	for(i=0; i < BITMAIN_MAX_CHAIN_NUM; i++)
	{
		if(cgpu.chain_exist[i]==0)
			continue;

		enable_pic_dac(i);
	}
	sleep(1);
#ifdef USE_NEW_RESET_FPGA
	set_reset_allhashboard(0);
#endif
#endif
}

void writeLogFile(char *logstr)
{
#ifdef ENABLE_SEARCH_LOGFILE
	FILE *fd;

	pthread_mutex_lock(&log_mutex);
	fd=fopen("/tmp/search","a+");
	if(fd)
	{
		fwrite(logstr,1,strlen(logstr),fd);
		fclose(fd);
	}
	pthread_mutex_unlock(&log_mutex);
#endif
	printf(logstr);
}

static void getAsicNumTest(bool isReset)
{
    int i, j, freq_index = 0;
	unsigned char vol_pic;
    unsigned char buf[128]= {0};
	char logstr[256];

	int wait_count=0;
	int min_Freq;
	int max_Freq;
	int last_send_num;
	int last_recv_num;
	int vol_value;
	unsigned char pic_version;
		
	// flag for each chain test is done or not...
	for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
	{
		testDone[i]=false;
	}
    // init fpga
    printf("clement2 init_fpga\n");
	if(isReset)
	{
		start_pic_heart=false;
		pthread_mutex_lock(&iic_mutex);
		
		reset_fpga();

		pthread_mutex_unlock(&iic_mutex);
		send_pic_heart_once();
		start_pic_heart=true;
	}

    // reset global arg
    reset_global_arg();
    start_receive = true;

	gStartTest=false;

    // need prepare PIC and voltage
	
    // init fpga
    printf("clement reset_hashboard\n");
	if(isReset)
	{
#ifdef USE_NEW_RESET_FPGA
		set_reset_allhashboard(1);
		sleep(RESET_KEEP_TIME);
		set_reset_allhashboard(0);
		sleep(1);
#else
	    reset_hashboard();
#endif
	}
    //set CommandMode
    printf("\n--- set command mode\n");
    if(conf.CommandMode)    // fil mode
    {
        // after reset fpga, it should be fil mode as default
        cgpu.CommandMode = 1;
        printf("set command mode to FIL\n");
    }
    else                    // vil mode
    {
    	printf("clement2 set_dhash_acc_control vil\n");
        set_dhash_acc_control((get_dhash_acc_control() & (~OPERATION_MODE)) | VIL_MODE | (VIL_MIDSTATE_NUMBER(1) & (~NEW_BLOCK) & (~RUN_BIT)));
        cgpu.CommandMode = 0;
        sprintf(logstr,"set command mode to VIL\n");
		writeLogFile(logstr);
    }

    //check asic number
    if(Conf.CheckChain)
    {
        sprintf(logstr,"\n--- check asic number\n");
		writeLogFile(logstr);

	//	check_asic_reg(CHIP_ADDRESS);
		
        for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
        {
            if(cgpu.chain_exist[i] == 1 && (!testDone[i]))
            {
            	check_asic_reg_oneChain(i,CHIP_ADDRESS);
                sprintf(logstr,"check chain[%d]: asicNum = %d\n",i, cgpu.chain_asic_num[i]);
				writeLogFile(logstr);
            }
        }
    }
}

void saveTestID(int testID)
{
	FILE *fd;
	char testnumStr[32];

	fd=fopen("/etc/config/testID","wb");
	if(fd)
	{
		memset(testnumStr,'\0',sizeof(testnumStr));
		sprintf(testnumStr,"%d",testID);
		fwrite(testnumStr,1,sizeof(testnumStr),fd);
		fclose(fd);
	}
}
int readTestID()
{
	FILE *fd;
	char testnumStr[32];

	fd=fopen("/etc/config/testID","rb");
	if(fd)
	{
		memset(testnumStr,'\0',sizeof(testnumStr));
		fread(testnumStr,1,sizeof(testnumStr),fd);
		fclose(fd);

		return atoi(testnumStr);
	}
	return 0;
}


void saveRebootTestNum(int num)
{
	FILE *fd;
	char testnumStr[32];

	fd=fopen("/etc/config/rebootTest","wb");
	if(fd)
	{
		memset(testnumStr,'\0',sizeof(testnumStr));
		sprintf(testnumStr,"%d",num);
		fwrite(testnumStr,1,sizeof(testnumStr),fd);
		fclose(fd);
	}
}

int readRebootTestNum()
{
	FILE *fd;
	char testnumStr[32];

	fd=fopen("/etc/config/rebootTest","rb");
	if(fd)
	{
		memset(testnumStr,'\0',sizeof(testnumStr));
		fread(testnumStr,1,sizeof(testnumStr),fd);
		fclose(fd);

		return atoi(testnumStr);
	}
	return 0;
}

void saveRestartNum(int num)
{
	FILE *fd;
	char testnumStr[32];

	fd=fopen("/etc/config/restartTest","wb");
	if(fd)
	{
		memset(testnumStr,'\0',sizeof(testnumStr));
		sprintf(testnumStr,"%d",num);
		fwrite(testnumStr,1,sizeof(testnumStr),fd);
		fclose(fd);
	}
}

int readRestartNum()
{
	FILE *fd;
	char testnumStr[32];

	fd=fopen("/etc/config/restartTest","rb");
	if(fd)
	{
		memset(testnumStr,'\0',sizeof(testnumStr));
		fread(testnumStr,1,sizeof(testnumStr),fd);
		fclose(fd);

		return atoi(testnumStr);
	}
	return 0;
}

int GetTotalRate()
{
	int i,j;
	int totalrate=0;
	for(i=0;i<BITMAIN_MAX_CHAIN_NUM;i++)
	{
		if(last_success_freq[i][0]>0)
		{
			for(j=0;j<ASIC_NUM;j++)
			{
				totalrate+=(atoi(freq_pll_1385[last_success_freq[i][j]].freq)*getChainAsicGoodCoreNum(i,j));
			}
		}
	}

	return (totalrate/1000);
}

int GetBoardRate(int chainIndex)
{
	int j;
	int totalrate=0;
	if(last_success_freq[chainIndex][0]>0)
	{
		for(j=0;j<ASIC_NUM;j++)
		{
			totalrate+=(atoi(freq_pll_1385[last_success_freq[chainIndex][j]].freq)*getChainAsicGoodCoreNum(chainIndex,j));
		}
	}

	return (totalrate/1000);
}

#ifdef R4
int ConvirtTotalRate(int totalRate)
{
	int lowPart;
	if(totalRate>=8000 && totalRate<8700)
	{
		return 8000;
	}
	else if(totalRate>=8700 && totalRate<9500)
	{
		return 8700;
	}
	else
	{
		lowPart=totalRate%1000;	// get the low part rate, GH/s
		if(lowPart>500)
			lowPart=500;
		else lowPart=0;	//if lower than 500G, just set zero

		return (((totalRate/1000)*1000)+lowPart);
	}
}
#else
int ConvirtTotalRate(int totalRate)
{
	int lowPart=totalRate%1000;	// get the low part rate, GH/s
	if(lowPart>500)
		lowPart=500;
	else lowPart=0;	//if lower than 500G, just set zero

	return (((totalRate/1000)*1000)+lowPart);
}
#endif

bool isSomeOneSearchOver()
{
	int i;
	for(i=0; i < BITMAIN_MAX_CHAIN_NUM; i++)
	{
		if(cgpu.chain_exist[i]==0)
			continue;
		
		if(search_over[i])
			return true;
	}
	return false;
}

bool isAllSearchOver()
{
	int i;
	for(i=0; i < BITMAIN_MAX_CHAIN_NUM; i++)
	{
		if(cgpu.chain_exist[i]==0)
			continue;
		
		if(!search_over[i])
			return false;
	}
	return true;
}

int numSearchCompleted()
{
	int i;
	int num=0;
	for(i=0; i < BITMAIN_MAX_CHAIN_NUM; i++)
	{
		if(cgpu.chain_exist[i]==0)
			continue;
		if(search_over[i])
			num++;
	}
	return num;
}

bool isAllSearchSuccess()
{
	int i;
	for(i=0; i < BITMAIN_MAX_CHAIN_NUM; i++)
	{
		if(cgpu.chain_exist[i]==0)
			continue;
		if(!search_freq_result[i])
			return false;
	}
	return true;
}

bool isAllBoardTestModeDone()
{
	int i;
	for(i=0; i < BITMAIN_MAX_CHAIN_NUM; i++)
	{
		if(cgpu.chain_exist[i]==0)
			continue;
		
		if(testModeOKCounter[i]<TEST_MODE_OK_NUM)
			return false;
	}
	return true;
}

int listen_sockfd = -1;
bool ExitServer=false;

void processTEST()
{
	char logstr[256];
	int testID=readTestID();
	switch(testID)
	{
	case 1:
		// test : get asic num
		saveTestID(0);
		
		sprintf(logstr,"get TEST ID=%d getAsicNumTest \n",testID);
		writeLogFile(logstr);

		getAsicNumTest(true);
		
		break;
	case 2:
		// test: 
		saveTestID(0);
		
		sprintf(logstr,"get TEST ID=%d getAsicNumTest Without Reset \n",testID);
		writeLogFile(logstr);

		getAsicNumTest(false);
		break;
	default:
		break;
	}
}

void getFileSysComplieTime(char *versionTimeStr)
{
	int i;
	FILE *fd;
	fd=fopen("/usr/bin/compile_time","r");
	if(fd)
	{
		memset(versionTimeStr,'\0',256);
		fgets(versionTimeStr,256,fd);

		for(i=0;i<strlen(versionTimeStr);i++)
		{
			if(versionTimeStr[i]=='\r' || versionTimeStr[i]=='\n')
				versionTimeStr[i]='\0';
		}
	}
	else
	{
		sprintf(versionTimeStr,"unkown version");
	}
}

void getMinerInfo(char *infoStr)
{
	int retlen=0;
	int i;
	int chainNO;	// chain NO. from 0 to 15
	int vol_value;
	char verTime[256];
	int freq_index;
	
	if(IsSomeBoardHasNoFreq)
	{// if some board has no freq, then will not post miner info! and set chain num = 0
		sprintf(infoStr,"chainnum=0;");
		return;
	}

	getFileSysComplieTime(verTime);
	
    for(i=0; i < BITMAIN_MAX_CHAIN_NUM; i++)	// MUST use i from 0 to loop here !!!!
    {
        if(cgpu.chain_exist[i])
        {
        	chainNO=get_chain_number(i);

#ifdef T9_18
			if(fpga_version>=0xE)
			{
				int new_T9_PLUS_chainIndex,new_T9_PLUS_chainOffset;
				getPICChainIndexOffset(i,&new_T9_PLUS_chainIndex,&new_T9_PLUS_chainOffset);
				
				if(chain_pic_buf[new_T9_PLUS_chainIndex][0] == FREQ_MAGIC)
				{
					vol_value=chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+1]*10;
					retlen+=sprintf(infoStr+retlen,"chain%d_voltage=%d;",chainNO+1,vol_value);
					retlen+=sprintf(infoStr+retlen,"chain%d_voladded=%d;",chainNO+1,(chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31+2]));

					freq_index=chain_pic_buf[new_T9_PLUS_chainIndex][7+new_T9_PLUS_chainOffset*31];
					retlen+=sprintf(infoStr+retlen,"chain%d_basefreq=%s;",chainNO+1,freq_pll_1385[freq_index].freq);

					retlen+=sprintf(infoStr+retlen,"chain%d_badcore=%d;",chainNO+1,getChainBadCoreNumUserMode(i));
				}
			}
			else
			{
				if(chain_pic_buf[((i/3)*3)][0] == FREQ_MAGIC)
				{
					vol_value=chain_pic_buf[((i/3)*3)][7+(i%3)*31+1]*10;
					retlen+=sprintf(infoStr+retlen,"chain%d_voltage=%d;",chainNO+1,vol_value);
					retlen+=sprintf(infoStr+retlen,"chain%d_voladded=%d;",chainNO+1,(chain_pic_buf[((i/3)*3)][7+(i%3)*31+2]));

					freq_index=chain_pic_buf[((i/3)*3)][7+(i%3)*31];
					retlen+=sprintf(infoStr+retlen,"chain%d_basefreq=%s;",chainNO+1,freq_pll_1385[freq_index].freq);

					retlen+=sprintf(infoStr+retlen,"chain%d_badcore=%d;",chainNO+1,getChainBadCoreNumUserMode(i));
				}
			}
#else
			if(chain_pic_buf[i][1] == FREQ_MAGIC && chain_pic_buf[i][40] == 0x23)	//0x23 is backup voltage magic number
			{
				vol_value=(((chain_pic_buf[i][36]&0x0f)<<4)+(chain_pic_buf[i][38]&0x0f))*10;
				retlen+=sprintf(infoStr+retlen,"chain%d_voltage=%d;",chainNO+1,vol_value);
				retlen+=sprintf(infoStr+retlen,"chain%d_voladded=%d;",chainNO+1,(chain_pic_buf[i][10]&0x3f));

				freq_index=((chain_pic_buf[i][6]&0x0f)<<4)+(chain_pic_buf[i][8]&0x0f);
				retlen+=sprintf(infoStr+retlen,"chain%d_basefreq=%s;",chainNO+1,freq_pll_1385[freq_index].freq);

				retlen+=sprintf(infoStr+retlen,"chain%d_badcore=%d;",chainNO+1,getChainBadCoreNumUserMode(i));
			}
#endif
        }
    }

	retlen+=sprintf(infoStr+retlen,"chainnum=%d;",chainNO+1);
	retlen+=sprintf(infoStr+retlen,"version=%s;",verTime);
}

#define BUFSIZE  10240
void * statusServiceThread(void *param)
{
	int s1 = (int)param;	// accept socket
	struct timeval timeout={3,0};//3s
	char recvbuf[BUFSIZE]={0}, *precvbuf=recvbuf;
	int ret =-1;
	struct sockaddr_in from;
	unsigned char buf[4096];
	int len = 0;
	int bError=0;
	int rebootTestNum;

	char strSendbuf[BUFSIZE];
	int buflen = 0;
	int recvlen = 0;
	int sentlen=0;
	char sendbuf[3] = {0};
	int testID=-1;
	int res_mode=0;
	sendbuf[0] = 0x0d;
	sendbuf[1] = 0x0a;
	
	char endofrequest[5] = {0};
	endofrequest[0] = 0x0d;
	endofrequest[1] = 0x0a;
	endofrequest[2] = 0x0d;
	endofrequest[3] = 0x0a;

	memset(recvbuf, 0, BUFSIZE);

	while(!ExitServer)
	{
		ret = recvfrom(s1, precvbuf, BUFSIZE-buflen-1, 0,(struct sockaddr*)&from,&recvlen);
		if(ret <= 0)
		{
			close(s1);	
			printf(":statusServiceThread recvfrom<=0\n");
			return 0;
		}

		buflen = ret;
		if(buflen == BUFSIZE-1)
		{
			close(s1);
			printf("BUFSIZE is too small!\n");
			return 0;
		}
		
		precvbuf = recvbuf+ buflen;
		if(strstr(recvbuf, endofrequest) != NULL)
		{
			printf("find http request end flag!\n");
			break;
		}		
	}

	printf("get http=%s\n",recvbuf);
	if(recvbuf[0] != 'G' 
		|| recvbuf[1] != 'E' 
		|| recvbuf[2] != 'T' 
		|| recvbuf[3] != ' ')
	{
		close(s1);
		printf("statusServiceThread not support http command\n");
		return 0;
	}

	if((strstr(recvbuf+4, "rate"))!=NULL)
	{
		printf("cmd : get rate\n");
		res_mode=0;
	}
	else if((strstr(recvbuf+4, "test"))!=NULL)
	{
		sscanf(recvbuf+4, "/test.%d",&testID);
		printf("cmd : get test = %d\n",testID);	

		if(testID!=523)	// magic nunber for get info of miner!
			saveTestID(testID);
		res_mode=1;
	}
	else
	{
		close(s1);
		printf("statusServiceThread exit for Error cmd!\n");
		return 0;
	}

	ret=setsockopt(s1,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout));
	if(ret!=0)
	{
		close(s1);
		printf("setsockopt SO_SNDTIMEO failed\n");
		return 0;
	}
	
    ret=setsockopt(s1,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
	if(ret!=0)
	{
		close(s1);
		printf("setsockopt SO_RCVTIMEO failed\n");
		return 0;
	}

	if(res_mode==1)
	{
		if(testID==523)
			getMinerInfo((char *)buf);
		else sprintf((char *)buf,"OK get test=%d",testID);
	}
	else
	{
		rebootTestNum=readRebootTestNum();
		if(rebootTestNum==3333)
		{
			searchStatus=SEARCH_FAILED;	// set by bmminer, bmminer will not reboot system, so we update it here
		}
		
		switch(searchStatus)
		{
		case SEARCHING_FREQ:
			sprintf((char *)buf,"searching");
			break;
		case SEARCH_SUCCESS:
			sprintf((char *)buf,"%d",ConvirtTotalRate(GetTotalRate()));
			break;
		case SEARCH_FAILED:
			sprintf((char *)buf,"searchfailed:%s",search_failed_info);
			break;
		case TEST_PATTEN_MODE:
			sprintf((char *)buf,"%d",ConvirtTotalRate(GetTotalRate()));
			break;
		default:
			sprintf((char *)buf,"searching");
			break;
		}
	}
	len=strlen((char *)buf);
	
	ret=0;
	ret+=sprintf(strSendbuf+ret,"HTTP/1.0  200  OK%s",sendbuf);
	ret+=sprintf(strSendbuf+ret,"Server: SearchFreqServer%s",sendbuf);
	ret+=sprintf(strSendbuf+ret,"Cache-Control: no-cache%s",sendbuf);
	ret+=sprintf(strSendbuf+ret,"Pragma: no-cache%s",sendbuf);
	ret+=sprintf(strSendbuf+ret,"Content-Type: text/plain%s",sendbuf);
	ret+=sprintf(strSendbuf+ret,"Content-Length: %d%s",len,sendbuf);
	ret+=sprintf(strSendbuf+ret,"Connection: Keep-Alive%s",endofrequest);

	printf("send http response...\n");
	while(!ExitServer)
	{
		sentlen=0;
		do{
			ret = send(s1, strSendbuf+sentlen, strlen(strSendbuf)-sentlen, 0);
			if(ret==-1 && errno==EAGAIN)
			{
				printf("statusServiceThread send http timeout, try again...\n");
				usleep(100000);
				continue;
			}
			else if(ret<=0)
			{
				close(s1);
				printf("statusServiceThread send http response error\n");
				return 0;
			}
			else
			{
				sentlen+=ret;
			}
		}while(sentlen<strlen(strSendbuf) && !ExitServer);
		
		if(sentlen>=strlen(strSendbuf) || ExitServer)
			break;
	}

	printf("send http data...\n");

	sentlen=0;
	do{
		ret=send(s1, (const char *)buf+sentlen, len-sentlen, 0);
		printf("send http data ret=%d\n",ret);
		if(ret <= 0)
		{
			bError = 1;
			printf("statusServiceThread send http data error\n");
			break;
		}
		else
		{
			sentlen+=ret;
		}
	}while(sentlen<len && !bError && !ExitServer);

	printf("one client disconnected!\n");
	close(s1);

	return (void *)sentlen;
}

void * httpListenThread(void* param)
{
    struct sockaddr_in  service;
	pthread_t tid_ctrl;
    pthread_attr_t tattr_ctrl;
    int stacksize_ctrl;
    int ret;
    
	//----------------------
	// Accept the connection.
	int AcceptSocket = -1;
	struct sockaddr_in client_addr;
	int addr_len;

	// to wait for network ready!
	sleep(30);
	
    listen_sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_sockfd < 0)
    {
        printf("socket creating err in udptalk\n");
        return 0;
    }
    
    memset(&service, 0, sizeof(service));	service.sin_family = AF_INET;
	service.sin_addr.s_addr = htonl(INADDR_ANY);
	service.sin_port = htons(6060);

	if(bind(listen_sockfd, (struct sockaddr *) &service, sizeof(service)) < 0)
	{
		printf("http port bind failed!\n");
		close(listen_sockfd);
		return 0;
	}

	if (listen( listen_sockfd, 100 ) <0 )
	{
		printf("listen failed\n");

		close(listen_sockfd);
		return 0;
	}
	
	printf("start listen on 6060 ...\n");
	while(!ExitServer) 
	{
		AcceptSocket = -1;
		while( AcceptSocket == -1 && !ExitServer) 
		{
			usleep(10000);

			addr_len = sizeof(struct sockaddr);
			AcceptSocket = accept( listen_sockfd, (struct sockaddr *)&client_addr, &addr_len );
		}

		if(ExitServer)
		{
			if(AcceptSocket != -1)
			{
				close(AcceptSocket);
				AcceptSocket = -1;
			}
			break;
		}

		printf("one client connected sock=%d\n",AcceptSocket);
		statusServiceThread((void *)AcceptSocket);
		
	//	ret = pthread_attr_init(&tattr_ctrl);
    //	stacksize_ctrl = (0x200000);
    //	ret = pthread_attr_setstacksize(&tattr_ctrl, stacksize_ctrl);
    //	ret = pthread_create(&tid_ctrl, &tattr_ctrl, statusServiceThread, (void *)AcceptSocket);
	//	printf("statusServiceThread start ret=%d\n",ret);
	}
	close(listen_sockfd);
	listen_sockfd = -1;

	return (void *)ret;
}

void StartHttpThread()
{
	pthread_t tid_ctrl;
    pthread_attr_t tattr_ctrl;
    int stacksize_ctrl;
    int ret;

	ret = pthread_attr_init(&tattr_ctrl);
    stacksize_ctrl = (0x200000);
    ret = pthread_attr_setstacksize(&tattr_ctrl, stacksize_ctrl);
    ret = pthread_create(&tid_ctrl, &tattr_ctrl, httpListenThread, 0);

	printf("httpListenThread start ret=%d\n",ret);
}

void saveSearchFailedFlagInfo()
{
	FILE *fd;
	fd=fopen("/tmp/searcherror","wb");
	if(fd)
	{
		fwrite(search_failed_info,1,strlen(search_failed_info)+1,fd);
		fclose(fd);
	}

	system("cp /tmp/search /tmp/err1.log -f");
	system("cp /tmp/freq /tmp/err2.log -f");
	system("cp /tmp/lasttemp /tmp/err3.log -f");

#ifdef DEBUG_STOP_AFTER_SEARCHFREQ_FOR_REPAIRE_TEST
	{
	char logstr[256];
	sprintf(logstr,"TEST FAILED: %s\n",search_failed_info);
	writeLogFile(logstr);
	}
#endif
}

void checkSearchFailedFlagInfo()
{
	char logstr[256];
	FILE *fd;
	int rebootTestNum;
	fd=fopen("/tmp/searcherror","rb");
	if(fd)
	{
		fread(search_failed_info,1,sizeof(search_failed_info),fd);
		fclose(fd);

		search_failed_info[sizeof(search_failed_info)-1]='\0';

		sprintf(logstr,"Find /tmp/searcherror: %s  The last log is below:\n",search_failed_info);
		writeLogFile(logstr);

		system("cat /tmp/err1.log >> /tmp/search");
		system("cat /tmp/err2.log >> /tmp/freq");
		system("cat /tmp/err3.log >> /tmp/lasttemp");
		
		searchStatus=SEARCH_FAILED;
		while(1)
		{
			processTEST();
			sleep(1);
		}
	}

	rebootTestNum=readRebootTestNum();
	if(rebootTestNum==3333)	// check failed magic number
	{
		sprintf(logstr,"\n Hashrate too Low : < 98% ideal rate, BIMMINER TEST Result: FAILED! The last log is below:\n");
		writeLogFile(logstr);

		system("cat /tmp/err1.log >> /tmp/search");
		system("cat /tmp/err2.log >> /tmp/freq");
		system("cat /tmp/err3.log >> /tmp/lasttemp");
		
		sprintf(search_failed_info,"R:1");
		saveSearchFailedFlagInfo();
		searchStatus=SEARCH_FAILED;
		while(1)
		{
			processTEST();
			sleep(1);
		}
	}
}

int getHighestVoltage()
{
	int i;
	int vol_value=0;
	for(i=0; i < BITMAIN_MAX_CHAIN_NUM; i++)
	{
		if(cgpu.chain_exist[i]==0)
			continue;

		if(chain_vol_value[i]>vol_value)
			vol_value=chain_vol_value[i];
	}

	return vol_value;
}

void PostRateVoltageToServer()
{
	int i;
	char url[1024];
	char *ret_str=NULL;
	char logstr[1024];
	int hashrate[BITMAIN_MAX_CHAIN_NUM];
	int voltage[BITMAIN_MAX_CHAIN_NUM];
	int num=0;
#ifdef T9_18
	int hashrate_T9_PLUS[BITMAIN_MAX_CHAIN_NUM];
	int voltage_T9_PLUS[BITMAIN_MAX_CHAIN_NUM];
	int num_T9_PLUS=0;
#endif

	for(i=0; i < BITMAIN_MAX_CHAIN_NUM; i++)
	{
		hashrate[i]=0;
		voltage[i]=0;
#ifdef T9_18
		hashrate_T9_PLUS[i]=0;
		voltage_T9_PLUS[i]=0;
#endif
	}
	for(i=0; i < BITMAIN_MAX_CHAIN_NUM; i++)
	{
		if(cgpu.chain_exist[i]==0)
			continue;

		hashrate[num]=GetBoardRate(i);
		voltage[num]=chain_vol_value[i];
		num++;
	}

#ifdef S9_PLUS
	sprintf(url,"http://192.168.60.30/postratevol.asp?minertype=S9-PLUS&hashrate1=%d&hashrate2=%d&hashrate3=%d&voltage1=%d&voltage2=%d&voltage3=%d",hashrate[0],hashrate[1],hashrate[2],voltage[0],voltage[1],voltage[2]);
#else
#ifdef T9_18
	for(i=0; i < num/3; i++)
	{
		hashrate_T9_PLUS[num_T9_PLUS]=hashrate[i*3]+hashrate[i*3+1]+hashrate[i*3+2];
		voltage_T9_PLUS[num_T9_PLUS]=voltage[i*3];
		num_T9_PLUS++;
	}
	sprintf(url,"http://192.168.60.30/postratevol.asp?minertype=T9-PLUS&hashrate1=%d&hashrate2=%d&hashrate3=%d&voltage1=%d&voltage2=%d&voltage3=%d",hashrate_T9_PLUS[0],hashrate_T9_PLUS[1],hashrate_T9_PLUS[2],voltage_T9_PLUS[0],voltage_T9_PLUS[1],voltage_T9_PLUS[2]);
#else
	sprintf(url,"http://192.168.60.30/postratevol.asp?minertype=S9&hashrate1=%d&hashrate2=%d&hashrate3=%d&voltage1=%d&voltage2=%d&voltage3=%d",hashrate[0],hashrate[1],hashrate[2],voltage[0],voltage[1],voltage[2]);
#endif
#endif

	sprintf(logstr,"post: %s\n",url);
	writeLogFile(logstr);
	
	if(http_get(url,&ret_str)>0)
	{
		sprintf(logstr,"post result: ");
		writeLogFile(logstr);
		writeLogFile(ret_str);
		sprintf(logstr,"\n");
		writeLogFile(logstr);
	}

	if(ret_str)
		free(ret_str);
}

void DownFreqAccordingToHashrate()
{
#ifndef R4
	int i;
	char logstr[256];
	int hashrate_limited=getLimitedHashrateByVoltage(getHighestVoltage());

	sprintf(logstr,"Check: Highest voltage=%d hashrate limited=%d cur hashrate=%d\n",getHighestVoltage(),hashrate_limited,GetTotalRate());
	writeLogFile(logstr);
	// check hashrate and voltage and set failed flag if with high voltage and hashrate
	if(GetTotalRate()>=hashrate_limited)
	{
		PostRateVoltageToServer();
		
#ifdef ENABLE_RATE_LIMIT_BY_VOLTAGE
		ProcessFixFreqForChips(hashrate_limited);
		sprintf(logstr,"fixed hashrate is %d\n",GetTotalRate());
		writeLogFile(logstr);
#else
		sprintf(logstr,"Failed!: hashrate=%d is too larger when voltage highest is %d\n",GetTotalRate(),getHighestVoltage());
		writeLogFile(logstr);

		sprintf(search_failed_info,"V:1");
		saveSearchFailedFlagInfo();

		searchStatus=SEARCH_FAILED;
		while(1)
		{
			processTEST();
			sleep(1);
		}
#endif
	}
#endif
}

void termination_handler(int signum)
{
	ExitServer=true;
	close(listen_sockfd);
	signal (signum, termination_handler);
}

void ClearLogInfo()
{
	FILE *fd;
	fd=fopen("/tmp/search","w");
	if(fd)
		fclose(fd);
	fd=fopen("/tmp/freq","w");
	if(fd)
		fclose(fd);
	fd=fopen("/tmp/lasttemp","w");
	if(fd)
		fclose(fd);
}

#ifdef NEED_AUTH_SEARCHFREQ
void DoAuthOnce()
{
	char logstr[256];
	bool ret;
	int count=0;
	do
	{
		sprintf(logstr,"start get auth from network, please make sure the miner can access internet!\n");
		writeLogFile(logstr);
		
		ret=isAuthToRun();
		if(!ret)
		{
			sprintf(logstr,"Failed on get auth from network, please make sure the miner can access internet!\n");
			writeLogFile(logstr);

			sprintf(logstr,"Wait 1 min to try...\n");
			writeLogFile(logstr);

			sleep(60);
		}

		count++;

		if(count>60)
			ClearLogInfo();
		
	}while(!ret);
}

void DoPostRate(int total_rate)
{
	char logstr[256];
	bool ret;
	int count=0;
	do
	{
		sprintf(logstr,"need post hashrate=%d to server, please make sure the miner can access internet!\n",total_rate);
		writeLogFile(logstr);
		
		ret=PostRate(total_rate);
		if(!ret)
		{
			sprintf(logstr,"Failed on post hashrate to server, please make sure the miner can access internet!\n");
			writeLogFile(logstr);

			sprintf(logstr,"Wait 1 min to try...\n");
			writeLogFile(logstr);

			sleep(60);
		}

		count++;

		if(count>60)
			ClearLogInfo();
	}while(!ret);
}
#endif

int main(int argc, char* argv[])
{
	int run_count = 0;
    int ret, init_freq, i,j,k;
	char logstr[256];
	FILE *fd;
	int wait_count;
	int rebootTestNum;	// for searching process, to reboot 3times to check hashrate.
	int restartMinerNum;	// the number of chances to reboot miner, for sometime hashrate is low when first startup.
	struct sysinfo si;
	
	CURLcode retcode=curl_global_init(CURL_GLOBAL_ALL);
	
	fd=fopen("/tmp/search","w");
	if(fd)
		fclose(fd);
	fd=fopen("/tmp/freq","w");
	if(fd)
		fclose(fd);
	fd=fopen("/tmp/lasttemp","w");
	if(fd)
		fclose(fd);

	if(isC5_Board())
	{
#ifdef R4
		R4_MAX_VOLTAGE=R4_MAX_VOLTAGE_C5;
		START_VOLTAGE=START_VOLTAGE_C5;
		RETRY_VOLTAGE=RETRY_VOLTAGE_C5;
		HIGHEST_FREQ_INDEX=HIGHEST_FREQ_INDEX_C5;
#endif

		PHY_MEM_NONCE2_JOBID_ADDRESS=PHY_MEM_NONCE2_JOBID_ADDRESS_C5;

		sprintf(logstr,"This is C5 board.\n");
		writeLogFile(logstr);
	}
	else
	{
#ifdef R4
		R4_MAX_VOLTAGE=R4_MAX_VOLTAGE_XILINX;
		START_VOLTAGE=START_VOLTAGE_XILINX;
		RETRY_VOLTAGE=RETRY_VOLTAGE_XILINX;
		HIGHEST_FREQ_INDEX=HIGHEST_FREQ_INDEX_XILINX;
#endif

        sysinfo(&si);
        sprintf(logstr, "This is XILINX board. Totalram:       %ld\n", si.totalram);
		writeLogFile(logstr);

		if(si.totalram > 1000000000)
		{
			PHY_MEM_NONCE2_JOBID_ADDRESS=PHY_MEM_NONCE2_JOBID_ADDRESS_XILINX_1GB;

			sprintf(logstr, "Detect 1GB control board of XILINX\n");
			writeLogFile(logstr);
		}
		else if(si.totalram > 500000000)
		{
			PHY_MEM_NONCE2_JOBID_ADDRESS=PHY_MEM_NONCE2_JOBID_ADDRESS_XILINX_512MB;

			sprintf(logstr, "Detect 512MB control board of XILINX\n");
			writeLogFile(logstr);
		}
		else
		{
			PHY_MEM_NONCE2_JOBID_ADDRESS=PHY_MEM_NONCE2_JOBID_ADDRESS_XILINX_256MB;

			sprintf(logstr, "Detect 256MB control board of XILINX\n");
			writeLogFile(logstr);
		}
	}

    ret = cgpu_init();
    if(ret < 0)
     {
    	printf("cgpu_init Error!\n");
        return ret;
    }
	
    ret = configMiner();
    if(ret < 0)
    {
    	printf("configMiner Error!\n");
        return ret;
    }

	detectFPGAversion();	// we need use fpga version to detect how to process T9+ chain index

	if(conf.force_freq)
	{
#ifdef NEED_AUTH_SEARCHFREQ
		DoAuthOnce();
	//	DoPostRate(12300);	// clement debug for test 
#endif
		system("cp /www/pages/cgi-bin/minerConfiguration2.cgi /www/pages/cgi-bin/minerConfiguration.cgi -f");
		sprintf(logstr,"config file found, will disable freq setting.\n");
		writeLogFile(logstr);
	}
	
	printf("single board test start\n");

#ifdef R4		// if defined , for R4  63 chips
	sprintf(logstr,"Miner Type = R4\n");
	writeLogFile(logstr);
#endif

#ifdef S9_PLUS	// if defined , for T9  57 chips
	sprintf(logstr,"Miner Type = T9\n");
	writeLogFile(logstr);
#endif

#ifdef S9_63	// if defined , for S9  63 chips
	sprintf(logstr,"Miner Type = S9\n");
	writeLogFile(logstr);
#endif

#ifdef T9_18 	// if defined , for T9+  18 chips
	sprintf(logstr,"Miner Type = T9+\n");
	writeLogFile(logstr);
#endif

	//set Asic Type
    sprintf(logstr,"AsicType = %d\n", ASIC_TYPE);
	writeLogFile(logstr);

    // set real asic num
    sprintf(logstr,"real AsicNum = %d\n", ASIC_NUM);
	writeLogFile(logstr);
	
    init_freq = get_plldata_i(1387,conf.freq);
	
#ifdef DEBUG_MODE
	init_freq=7;
#endif

	Conf.DataCount=conf.dataCount=114;	// fixed to 114
	Conf.PassCount1=conf.passCount1=114;
	Conf.PassCount2=conf.passCount2=114;
	Conf.PassCount3=conf.passCount3=114;
	Conf.ValidNonce1=conf.ValidNonce1=7182;
	Conf.ValidNonce2=conf.ValidNonce2=7182;
	Conf.ValidNonce3=conf.ValidNonce3=7182;
	
	for(i=0;i<BITMAIN_MAX_CHAIN_NUM;i++)
	{
    	for(j=0; j<256; j++)
    	{
    	    last_result[i][j] = 0 ;
    	    last_freq[i][j] = init_freq;
			last_success_freq[i][j]=0;
    	}

		search_freq_chances[i]=SEARCH_FREQ_CHANCE_NUM;
			
		search_over[i]=false;
		search_freq_result[i]=true;

#if ((USE_SEARCH_BASEFREQ_MODE == 2) || (USE_SEARCH_BASEFREQ_MODE == 3))
		searchFreqMode[i]=SEARCH_BASE_FREQ_V89_200;
#elif USE_SEARCH_BASEFREQ_MODE == 4
		searchFreqMode[i]=SEARCH_BASE_FREQ_V86_650;
#else
		searchFreqMode[i]=SEARCH_BASE_FREQ;
#endif

#ifdef R4
		Fmax[i]=44;
#else
		Fmax[i]=init_freq;
#endif
		base_freq_index[i]=0;
		testDone[i]=false;

		chain_vol_value[i]=0;
		chain_vol_added[i]=0;

		testModeOKCounter[i]=0;

		chain_DataCount[i]=SEARCH_BASEFREQ_PATTEN_NUM;	// when seaching base freq, we use 8*144 patten on chip
		chain_ValidNonce[i]=SEARCH_BASEFREQ_NONCE_NUM;
		chain_PassCount[i]=SEARCH_BASEFREQ_PATTEN_NUM;

		chip_temp_offset[i]=DEFAULT_TEMP_OFFSET;
	}

	cgpu.chain_num = 0;

	first_freq=true;

	searchStatus=SEARCHING_FREQ;

	ExitServer=false;
	StartHttpThread();

	checkSearchFailedFlagInfo();	// if search failed, we will stop and show the failed info last time.

#ifdef ENABLE_DOWNLOAD_32XPATTEN
	if(conf.force_freq)
		pthread_create(&cgpu.down_id, NULL, download_testpatten_func, &cgpu);
#endif
	
	Conf.TestMode=0;

	ExitFlag=false;
	
	receiveExit=false;
	pthread_create(&cgpu.receive_id, NULL, receive_func, &cgpu);

#ifdef ENABLE_TEMP_PROCESS
	showExit=false;
	gIsReadTemp = false;
	pthread_create(&cgpu.show_id, NULL, show_status_func, &cgpu);
#endif

#ifdef LAST_TESTPATTEN_CRITICAL
	sprintf(logstr,"use critical mode to search freq...\n");
	writeLogFile(logstr);
#endif

	for(i=0;i<BITMAIN_MAX_CHAIN_NUM;i++)
    {
    	StartSendFlag[i]=false;
		
#ifndef USE_SINGLE_SEND_THREAD
		sprintf(logstr,"prepare send works thread on chain[%d]\n",i);
		writeLogFile(logstr);
	    sendExit[i]=false;
	    pthread_create(&cgpu.send_id[i], NULL, send_func, (void *)i);
#endif
    }

	picheartExit=false;
    pthread_create(&cgpu.pic_heart_beat_id, NULL, pic_heart_beat_func, &cgpu);

	InitAsicCoreEnabledFlag();
	PreparePICandVoltage();

#ifndef FORCE_8xPATTENT_TEST
    do
    {
		run_count++;
        singleBoardTest();

		sprintf(logstr,"search freq for %d times, completed chain = %d, total chain num = %d \n",run_count,numSearchCompleted(),cgpu.chain_num);
		writeLogFile(logstr);
    }while ( !isAllSearchOver());

	if(!isAllSearchSuccess())
	{
		sprintf(logstr,"\nMode B Process Over! Result: FAILED!\n");
		writeLogFile(logstr);

		searchStatus=SEARCH_FAILED;
		while(1)
		{
			processTEST();
			sleep(1);
		}	// we stop here, then bmminer will not start , so system can detect this machine is not good!
	}
#endif
	
	if(conf.force_freq)
	{
		//force to up voltage with 0.1V
		forceAddVoltageforBoard();

#ifdef CLOSE_DC_FOR_COOL_BEFORE_TESTPATTEN
		for(i=0; i < BITMAIN_MAX_CHAIN_NUM; i++)
		{
			if(cgpu.chain_exist[i]==0)
				continue;
			
			disable_pic_dac(i);
		}

		sprintf(logstr,"Wait for 5 mins to cool down, then start test patten...\n");
		writeLogFile(logstr);
		
		set_PWM(100);
		sleep(300);

#ifdef USE_NEW_RESET_FPGA
		set_reset_allhashboard(1);
		sleep(1);
#endif

		for(i=0; i < BITMAIN_MAX_CHAIN_NUM; i++)
		{
			if(cgpu.chain_exist[i]==0)
				continue;
			
			enable_pic_dac(i);
		}
		sleep(1);
		
#ifdef USE_NEW_RESET_FPGA
		set_reset_allhashboard(0);
#endif
#endif
		if(is32xPattenReady)
		{
			sprintf(logstr,"Find 32xPatten ready, need re-load test patten...\n");
			writeLogFile(logstr);

			freeWorks();
			get_works_32X();

			TEST_MODE_OK_NUM=TEST_MODE_OK_NUM_32X;
		}
		else TEST_MODE_OK_NUM=TEST_MODE_OK_NUM_8X;

		
#ifdef FORCE_8xPATTENT_TEST
		saveRebootTestNum(0);
		saveRestartNum(2);
#else
		saveRebootTestNum(REBOOT_TEST_NUM);	// save test num into file, bmminer will decrease this
#endif
		for(i=0;i<BITMAIN_MAX_CHAIN_NUM;i++)
		{
			testModeOKCounter[i]=0;

			testModeChainHasNoChipDownFreqCounter[i]=0;
			
			for(j=0; j<256; j++)
				testModeHasTriedAcceptBadCore[i][j]=false;
		}

		// only do once testmode to fix voltage or freq
		k=0;
		do{
			k++;
			sprintf(logstr,"do 8xPatten test for %d times\n",k);
			writeLogFile(logstr);
		
			for(i=0;i<BITMAIN_MAX_CHAIN_NUM;i++)
			{
		    	for(j=0; j<256; j++)
		    	{
		    	    last_result[i][j] = 0 ;
					last_opencore_result[i][j] = 0;
					last_freq[i][j] = last_success_freq[i][j];	// use the success freq to test
		    	}

				// force to test all boards at any time
				search_over[i]=false;
				testDone[i]=false;
				search_freq_result[i]=true;

				if(is32xPattenReady)
				{
					chain_DataCount[i]=TESTMODE_PATTEN_NUM_32X;	// when seaching base freq, we use 8*144 patten on chip
					chain_ValidNonce[i]=TESTMODE_NONCE_NUM_32X;
					chain_PassCount[i]=TESTMODE_PATTEN_NUM_32X;
				}
				else
				{
					chain_DataCount[i]=TESTMODE_PATTEN_NUM_8X;	// when seaching base freq, we use 8*144 patten on chip
					chain_ValidNonce[i]=TESTMODE_NONCE_NUM_8X;
					chain_PassCount[i]=TESTMODE_PATTEN_NUM_8X;
				}
			}

#ifdef PATTEN_TEST_ONE_BY_ONE
			for(i=0;i<BITMAIN_MAX_CHAIN_NUM;i++)
			{
				if(cgpu.chain_exist[i]==0 || testModeOKCounter[i]>=TEST_MODE_OK_NUM)
					continue;

				doHeatBoard(true,i);

				if(search_freq_result[i])
				{
					testModeOKCounter[i]++;
				}
#ifdef LAST_TESTPATTEN_CRITICAL
				else
				{
					if(testModeOKCounter[i]<TEST_MODE_OK_NUM)
						testModeOKCounter[i]=0;
				}
#endif
			}
#else
			doHeatBoard(true);

			for(i=0; i < BITMAIN_MAX_CHAIN_NUM; i++)
			{
				if(cgpu.chain_exist[i]==0)
					continue;
				
				if(search_freq_result[i])
				{
					testModeOKCounter[i]++;
				}
#ifdef LAST_TESTPATTEN_CRITICAL
				else
				{
					if(testModeOKCounter[i]<TEST_MODE_OK_NUM)
						testModeOKCounter[i]=0;
				}
#endif

			}
#endif

#ifdef CLOSE_OPEN_DC_TESTPATTEN
#ifdef USE_NEW_RESET_FPGA
			set_reset_allhashboard(1);
#endif

			for(i=0; i < BITMAIN_MAX_CHAIN_NUM; i++)
			{
				if(cgpu.chain_exist[i]==0)
					continue;
				disable_pic_dac(i);
			}

			sleep(1);

			for(i=0; i < BITMAIN_MAX_CHAIN_NUM; i++)
			{
				if(cgpu.chain_exist[i]==0)
					continue;
				enable_pic_dac(i);
			}
			sleep(1);
#ifdef USE_NEW_RESET_FPGA
			set_reset_allhashboard(0);
#endif
#endif
			if(isFailedOnTestPatten)
			{
				//failed on test patten
				sprintf(logstr,"Failed on TEST PATTEN \n");
				writeLogFile(logstr);

				sprintf(search_failed_info,"T:1");
				saveSearchFailedFlagInfo();
				
				searchStatus=SEARCH_FAILED;
				while(1)
				{
					processTEST();
					sleep(1);
				}
			}
		}while(!isAllBoardTestModeDone());

		sprintf(logstr,"Test Mode Done, will check hashboard voltage, try to add more 0.1V.\n");
		writeLogFile(logstr);

		DownFreqAccordingToHashrate();

#ifndef S9_PLUS
		//if there is space to raise voltage, we need up 0.1V again!!! because sometime, hashrate becames lower, so we need another chance to up voltage again!
		editVoltageforBoard();
#endif

		sprintf(logstr,"Test Mode Done, save fixed freq and voltage into PIC\n");
		writeLogFile(logstr);

#ifdef FORCE_8xPATTENT_TEST
		sprintf(logstr,"\n Done Hashrate = %d\n",GetTotalRate());
		writeLogFile(logstr);
	//	while(1)sleep(1);
#endif

#ifdef NEED_AUTH_SEARCHFREQ
		DoPostRate(ConvirtTotalRate(GetTotalRate()));
#endif

		save_fixed_FreqAndCoreNum_toPIC();
		ClearForceFreq();	//clear force to search freq flag
	}

#ifdef DEBUG_STOP_AFTER_SEARCHFREQ_FOR_REPAIRE_TEST
	sprintf(logstr,"TEST SUCCESS: SEARCH FREQ PASS\n");
	writeLogFile(logstr);
	while(1)sleep(1);
#endif

#ifdef NEED_AUTH_SEARCHFREQ
	saveRebootTestNum(0);
	saveRestartNum(2);
#endif

	rebootTestNum=readRebootTestNum();
	if(rebootTestNum==3333)	// check failed magic number
	{
		sprintf(logstr,"\n Hashrate too Low : < 98% ideal rate, BIMMINER TEST Result: FAILED!\n");
		writeLogFile(logstr);

		sprintf(search_failed_info,"R:1");
		saveSearchFailedFlagInfo();
		searchStatus=SEARCH_FAILED;
		while(1)
		{
			processTEST();
			sleep(1);
		}
	}

	if(rebootTestNum>0)
	{
#ifdef USE_NEW_RESET_FPGA
		set_reset_allhashboard(1);
		sleep(RESET_KEEP_TIME);
		set_reset_allhashboard(0);
		sleep(1);
#endif

		set_PWM(100);

#ifdef DEBUG_MODE
		sprintf(logstr,"Debug Mode: rebootTestNum=%d, start bmminer.\n",rebootTestNum);
		writeLogFile(logstr);
#else
		if(conf.force_freq)
		{
			sprintf(logstr,"rebootTestNum=%d, reboot system.\n",rebootTestNum);
			writeLogFile(logstr);
		//	system("reboot");
		}
		else
		{
			sprintf(logstr,"rebootTestNum=%d, start bmminer.\n",rebootTestNum);
			writeLogFile(logstr);
		}
#endif
	}
	else
	{
#ifdef USE_NEW_RESET_FPGA
		set_reset_allhashboard(1);
		sleep(RESET_KEEP_TIME);
		set_reset_allhashboard(0);
		sleep(1);
#endif
		restartMinerNum=readRestartNum();
		sprintf(logstr,"restart Miner chance num=%d\n",restartMinerNum);
		writeLogFile(logstr);

		isFailedOnTestPatten=false;	// only heat , do not test 
	}

#ifdef DEBUG_REBOOT
	set_PWM(100);
	sprintf(logstr,"fan speed 100%, Wait for 5 mins to start bmminer.\n");
	writeLogFile(logstr);
	sleep(300);	// wait 5min to cool down
#else
	set_PWM(100);
#endif

    close(cgpu.device_fd);
    cgpu.device_fd = -1;

	ExitFlag=true;
	
	for(i=0;i<BITMAIN_MAX_CHAIN_NUM;i++)
    {
		StartSendFlag[i]=false;
		
#ifndef USE_SINGLE_SEND_THREAD
		wait_count=0;
		sprintf(logstr,"waiting for send_func to exit of chain[%d]\n",i);
		writeLogFile(logstr);
		
		while(!sendExit[i])
		{
			wait_count++;
			usleep(1000000);

			if(wait_count>3)
			{
				pthread_cancel(cgpu.send_id[i]);
				break;
			}
		}
#endif
    }
	
	sprintf(logstr,"waiting for receive_func to exit!\n");
	writeLogFile(logstr);
	wait_count=0;
	while(!receiveExit)
	{
		wait_count++;
		usleep(1000000);

		if(wait_count>3)
		{
			pthread_cancel(cgpu.receive_id);
			break;
		}
	}

	sprintf(logstr,"waiting for pic heart to exit!\n");
	writeLogFile(logstr);
	wait_count=0;
	while(!picheartExit)
	{
		wait_count++;
		usleep(1000000);

		if(wait_count>3)
		{
			pthread_cancel(cgpu.pic_heart_beat_id);
			break;
		}
	}

#ifdef ENABLE_TEMP_PROCESS
	sprintf(logstr,"waiting for show_temp_func to exit!\n");
	writeLogFile(logstr);
	wait_count=0;
	while(!showExit)
	{
		wait_count++;
		usleep(1000000);

		if(wait_count>3)
		{
			pthread_cancel(cgpu.show_id);
			break;
		}
	}
#endif

	if(!isFailedOnTestPatten)
	{
		if(IsSomeBoardHasNoFreq)
			searchStatus=SEARCH_FAILED;	// if there is no freq in PIC, we can use default freq to mining, but we must set the status to search failed!
		else searchStatus=SEARCH_SUCCESS;
		
		sprintf(logstr,"Start bmminer ...\n");
		writeLogFile(logstr);

	//	while(1)usleep(1000);	// clement for debug

		freeWorks();

    	system("/usr/bin/bmminer --version-file /usr/bin/compile_time --api-listen --default-config /config/bmminer.conf");
		
		ExitServer=true;
		close(listen_sockfd);
		usleep(1000000);
	}
	else
	{
		sprintf(logstr,"\nMode B : After searchfreq, TestPatten Result: FAILED!\n");
		writeLogFile(logstr);

		searchStatus=SEARCH_FAILED;
		while(1)
		{
			processTEST();
			sleep(1);
		}
	}

    return 0;
}
