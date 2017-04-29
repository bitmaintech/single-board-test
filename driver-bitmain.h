/*
 * Copyright 2016-2017 Fazio Bai <yang.bai@bitmain.com>
 * Copyright 2016-2017 Clement Duan <kai.duan@bitmain.com>
 *
 */

#ifndef _DRIVER_BITMAIN_H__
#define _DRIVER_BITMAIN_H__
#include <stdbool.h>

#define Swap32(l) (((l) >> 24) | (((l) & 0x00ff0000) >> 8) | (((l) & 0x0000ff00) << 8) | ((l) << 24))

#ifndef htobe32
# if __BYTE_ORDER == __LITTLE_ENDIAN
#  define htole8(x) (x)
#  define htole16(x) (x)
#  define htole32(x) (x)
#  define htole64(x) (x)
#  define le32toh(x) (x)
#  define le64toh(x) (x)
#  define be32toh(x) bswap_32(x)
#  define be64toh(x) bswap_64(x)
#  define htobe32(x) bswap_32(x)
#  define htobe64(x) bswap_64(x)
# elif __BYTE_ORDER == __BIG_ENDIAN
#  define htole8(x) bswap_8(x)
#  define htole16(x) bswap_16(x)
#  define htole32(x) bswap_32(x)
#  define le32toh(x) bswap_32(x)
#  define le64toh(x) bswap_64(x)
#  define htole64(x) bswap_64(x)
#  define be32toh(x) (x)
#  define be64toh(x) (x)
#  define htobe32(x) (x)
#  define htobe64(x) (x)
#else
#error UNKNOWN BYTE ORDER
#endif

#else

# if __BYTE_ORDER == __LITTLE_ENDIAN
#  define htole8(x) (x)
# elif __BYTE_ORDER == __BIG_ENDIAN
#  define htole8(x) bswap_8(x)
#else
#error UNKNOWN BYTE ORDER
#endif

#endif

#define FIL 0x1
#define VIL 0x0

#define BITMAIN_RESET_PITCH (300*1000*1000)

#define BITMAIN_TOKEN_TYPE_TXCONFIG 0x51
#define BITMAIN_TOKEN_TYPE_TXTASK   0x52
#define BITMAIN_TOKEN_TYPE_RXSTATUS 0x53

#define BITMAIN_DATA_TYPE_RXSTATUS  0xa1
#define BITMAIN_DATA_TYPE_RXNONCE   0xa2

#define BTM_SEND_ERROR -1
#define BTM_SEND_OK 0

#define BITMAIN_SENDBUF_SIZE 4096
#define BITMAIN_READBUF_SIZE 8192
#define BITMAIN_RESET_TIMEOUT 100
#define BITMAIN_READ_TIMEOUT 18 /* Enough to only half fill the buffer */
#define BITMAIN_LATENCY 1

//#ifdef BITMAIN_TYPE_S4
#define BITMAIN_MAX_ASIC_NUM       64
#define BITMAIN_MAX_WORK_NUM       64
#define BITMAIN_MAX_WORK_QUEUE_NUM 4096
#define BITMAIN_MAX_DEAL_QUEUE_NUM 32
#define BITMAIN_MAX_NONCE_NUM      128
#define BITMAIN_MAX_CHAIN_NUM      16
#define BITMAIN_MAX_TEMP_NUM       32

#define BITMAIN_SEND_STATUS_TIME   15 //s
#define BITMAIN_SEND_FULL_SPACE    512
//#endif


//FPGA rgister Address Map
#define HARDWARE_VERSION                (0x00000000/sizeof(int))
#define FAN_SPEED                       (0x00000004/sizeof(int))
#define HASH_ON_PLUG                    (0x00000008/sizeof(int))
#define BUFFER_SPACE                    (0x0000000c/sizeof(int))
#define RETURN_NONCE                    (0x00000010/sizeof(int))
#define NONCE_NUMBER_IN_FIFO            (0x00000018/sizeof(int))
#define NONCE_FIFO_INTERRUPT            (0x0000001c/sizeof(int))
#define TEMPERATURE_0_3                 (0x00000020/sizeof(int))
#define TEMPERATURE_4_7                 (0x00000024/sizeof(int))
#define TEMPERATURE_8_11                (0x00000028/sizeof(int))
#define TEMPERATURE_12_15               (0x0000002c/sizeof(int))
#define IIC_COMMAND                     (0x00000030/sizeof(int))
#define RESET_HASHBOARD_COMMAND         (0x00000034/sizeof(int))
#define TW_WRITE_COMMAND                (0x00000040/sizeof(int))
#define QN_WRITE_DATA_COMMAND           (0x00000080/sizeof(int))
#define FAN_CONTROL                     (0x00000084/sizeof(int))
#define TIME_OUT_CONTROL                (0x00000088/sizeof(int))
#define TICKET_MASK_FPGA                (0x0000008c/sizeof(int))
#define HASH_COUNTING_NUMBER_FPGA       (0x00000090/sizeof(int))
#define SNO                             (0x00000094/sizeof(int))
#define BC_WRITE_COMMAND                (0x000000c0/sizeof(int))
#define BC_COMMAND_BUFFER               (0x000000c4/sizeof(int))
#define FPGA_CHIP_ID_ADDR               (0x000000f0/sizeof(int))
#define CRC_ERROR_CNT_ADDR              (0x000000f8/sizeof(int))
#define DHASH_ACC_CONTROL               (0x00000100/sizeof(int))
#define COINBASE_AND_NONCE2_LENGTH      (0x00000104/sizeof(int))
#define WORK_NONCE_2                    (0x00000108/sizeof(int))
#define NONCE2_AND_JOBID_STORE_ADDRESS  (0x00000110/sizeof(int))
#define MERKLE_BIN_NUMBER               (0x00000114/sizeof(int))
#define JOB_START_ADDRESS               (0x00000118/sizeof(int))
#define JOB_LENGTH                      (0x0000011c/sizeof(int))
#define JOB_DATA_READY                  (0x00000120/sizeof(int))
#define JOB_ID                          (0x00000124/sizeof(int))
#define BLOCK_HEADER_VERSION            (0x00000130/sizeof(int))
#define TIME_STAMP                      (0x00000134/sizeof(int))
#define TARGET_BITS                     (0x00000138/sizeof(int))
#define PRE_HEADER_HASH                 (0x00000140/sizeof(int))

//FPGA registers bit map
//QN_WRITE_DATA_COMMAND
#define RESET_HASH_BOARD                (1 << 31)
#define RESET_ALL                       (1 << 23)
#define CHAIN_ID(id)                    (id << 16)
#define RESET_FPGA                      (1 << 15)
#define RESET_TIME(time)                (time << 0)
#define TIME_OUT_VALID                  (1 << 31)
//RETURN_NONCE
#define WORK_ID_OR_CRC                  (1 << 31)
#define WORK_ID_OR_CRC_VALUE(value)     ((value >> 16) & 0x7fff)
#define NONCE_INDICATOR                 (1 << 7)
#define CHAIN_NUMBER(value)             (value & 0xf)
#define REGISTER_DATA_CRC(value)        ((value >> 24) & 0x7f)
//BC_WRITE_COMMAND
#define BC_COMMAND_BUFFER_READY         (1 << 31)
#define BC_COMMAND_EN_CHAIN_ID          (1 << 23)
#define BC_COMMAND_EN_NULL_WORK         (1 << 22)
//NONCE2_AND_JOBID_STORE_ADDRESS
#define JOB_ID_OFFSET                   (0x0/sizeof(int))
#define HEADER_VERSION_OFFSET           (0x4/sizeof(int))
#define NONCE2_L_OFFSET                 (0x8/sizeof(int))
#define NONCE2_H_OFFSET                 (0xc/sizeof(int))
#define MIDSTATE_OFFSET                 0x20
//DHASH_ACC_CONTROL
#define VIL_MODE                        (1 << 15)
#define VIL_MIDSTATE_NUMBER(value)      ((value &0x0f) << 8)
#define NEW_BLOCK                       (1 << 7)
#define RUN_BIT                         (1 << 6)
#define OPERATION_MODE                  (1 << 5)
//NONCE_FIFO_INTERRUPT
#define FLUSH_NONCE3_FIFO               (1 << 16)


//ASIC macro define
//ASIC command
#define SET_ADDRESS             0x1
#define SET_PLL_DIVIDER2        0x2
#define PATTERN_CONTROL         0x3
#define GET_STATUS              0x4
#define CHAIN_INACTIVE          0x5
#define SET_BAUD_OPS            0x6
#define SET_PLL_DIVIDER1        0x7
#define SET_CONFIG              0x8
#define COMMAND_FOR_ALL         0x80
//other ASIC macro define
#define MAX_BAUD_DIVIDER        26
#define DEFAULT_BAUD_DIVIDER    26
#define BM1385_CORE_NUM         50
#define VIL_COMMAND_TYPE        (0x02 << 5)
#define VIL_ALL                 (0x01 << 4)
#define PAT                     (0x01 << 7)
#define GRAY                    (0x01 << 6)
#define INV_CLKO                (0x01 << 5)
#define LPD                     (0x01 << 4)
#define GATEBCLK                (0x01 << 7)
#define RFS                     (0x01 << 6)
#define MMEN                    (0x01 << 7)
#define TFS(x)                  ((x & 0x03) << 5)


//other FPGA macro define
#define TOTAL_LEN                       0x160
#define FPGA_MEM_TOTAL_LEN              (16*1024*1024)  // 16M bytes
#define HARDWARE_VERSION_VALUE          0xc501
#define NONCE2_AND_JOBID_STORE_SPACE    (2*1024*1024)   // 2M bytes
#define NONCE2_AND_JOBID_STORE_SPACE_ORDER  9           // for 2M bytes space
#define JOB_STORE_SPACE                 (1 << 16)       // for 64K bytes space
#define JOB_START_SPACE                 (1024*8)        // 8K bytes
#define JOB_START_ADDRESS_ALIGN         32              // JOB_START_ADDRESS need 32 bytes aligned
#define NONCE2_AND_JOBID_ALIGN          64              // NONCE2_AND_JOBID_STORE_SPACE need 64 bytes aligned
#define MAX_TIMEOUT_VALUE               0x1ffff         // defined in TIME_OUT_CONTROL
#define MAX_NONCE_NUMBER_IN_FIFO        0x1ff           // 511 nonce
#define NONCE_DATA_LENGTH               4               // 4 bytes
#define REGISTER_DATA_LENGTH            4               // 4 bytes
#define TW_WRITE_COMMAND_LEN            52
#define TW_WRITE_COMMAND_LEN_VIL        52
#define NEW_BLOCK_MARKER                0x11
#define NORMAL_BLOCK_MARKER             0x01

// ATTENTION: if MEM size is changed, must change this micro definition too!!!   use MAX size (BYTE) - 16 MB as FPGA start memory address
#define PHY_MEM_NONCE2_JOBID_ADDRESS_XILINX_1GB    		((1024-16)*1024*1024)
#define PHY_MEM_NONCE2_JOBID_ADDRESS_XILINX_512MB    	((512-16)*1024*1024)		// XILINX use 512MB memory
#define PHY_MEM_NONCE2_JOBID_ADDRESS_XILINX_256MB    	((256-16)*1024*1024)		// XILINX use 512MB memory

#define PHY_MEM_NONCE2_JOBID_ADDRESS_C5    				((1024-16)*1024*1024)
extern unsigned int PHY_MEM_NONCE2_JOBID_ADDRESS;

#define PHY_MEM_JOB_START_ADDRESS_1     (PHY_MEM_NONCE2_JOBID_ADDRESS + NONCE2_AND_JOBID_STORE_SPACE)
#define PHY_MEM_JOB_START_ADDRESS_2     (PHY_MEM_JOB_START_ADDRESS_1 + JOB_STORE_SPACE)

// macro define about miner
#define BITMAIN_MAX_CHAIN_NUM           16
#define BITMAIN_MAX_FAN_NUM             8               // FPGA just can supports 8 fan
#define BITMAIN_DEFAULT_ASIC_NUM        64              // max support 64 ASIC on 1 HASH board
#define MIDSTATE_LEN                    32
#define DATA2_LEN                       12
#define MAX_RETURNED_NONCE_NUM          10
#define PREV_HASH_LEN                   32
#define MERKLE_BIN_LEN                  32
#define INIT_CONFIG_TYPE                0x51
#define STATUS_DATA_TYPE                0xa1
#define SEND_JOB_TYPE                   0x52
#define READ_JOB_TYPE                   0xa2
#define CHECK_SYSTEM_TIME_GAP           10000           // 10s
//fan
#define MIN_PWM_PERCENT                 20
#define MAX_PWM_PERCENT                 100
#define TEMP_INTERVAL                   2   // ??????,??PWM 
#define PWM_ADJUST_FACTOR               ((100 - MIN_PWM_PERCENT)/(60-35))   // 60???100,35???0
#define PWM_SCALE                       50  //50:   1M=1us,      20KHz??
											//25:   40KHz
#define PWM_ADJ_SCALE                   9/10
//use for hash test
#define TEST_DHASH 0
#define DEVICE_DIFF 5
#define MAX_ASIC_NUM 128
#define MAX_WORK 5000

// Pic
#define PIC_FLASH_POINTER_START_ADDRESS_H   0x03
#define PIC_FLASH_POINTER_START_ADDRESS_L   0x00
#define PIC_FLASH_POINTER_END_ADDRESS_H     0x0f
#define PIC_FLASH_POINTER_END_ADDRESS_L     0x7f
//DIFF_FREQ
#define PIC_FLASH_POINTER_FREQ_START_ADDRESS_H   0x0F
#define PIC_FLASH_POINTER_FREQ_START_ADDRESS_L   0xA0
#define PIC_FLASH_POINTER_FREQ_END_ADDRESS_H     0x0f
#define PIC_FLASH_POINTER_FREQ_END_ADDRESS_L     0xDF
#define FREQ_MAGIC                               0x7D

// BAD CORE NUM
#define PIC_FLASH_POINTER_BADCORE_START_ADDRESS_H   0x0F
#define PIC_FLASH_POINTER_BADCORE_START_ADDRESS_L   0x80
#define PIC_FLASH_POINTER_BADCORE_END_ADDRESS_H     0x0f
#define PIC_FLASH_POINTER_BADCORE_END_ADDRESS_L     0x9F
#define BADCORE_MAGIC                            0x23	// magic number for bad core num

#define PIC_FLASH_LENGTH                    (((unsigned int)PIC_FLASH_POINTER_END_ADDRESS_H<<8 + PIC_FLASH_POINTER_END_ADDRESS_L) - ((unsigned int)PIC_FLASH_POINTER_START_ADDRESS_H<<8 + PIC_FLASH_POINTER_START_ADDRESS_L) + 1)
#define PIC_FLASH_SECTOR_LENGTH             32
#define PIC_SOFTWARE_VERSION_LENGTH         1
#define PIC_VOLTAGE_TIME_LENGTH             6
#define PIC_COMMAND_1                       0x55
#define PIC_COMMAND_2                       0xaa
#define SET_PIC_FLASH_POINTER               0x01
#define SEND_DATA_TO_IIC                    0x02    // just send data into pic's cache
#define READ_DATA_FROM_IIC                  0x03
#define ERASE_IIC_FLASH                     0x04    // erase 32 bytes one time
#define WRITE_DATA_INTO_PIC                 0x05    // tell pic write data into flash from cache
#define JUMP_FROM_LOADER_TO_APP             0x06
#define RESET_PIC                           0x07
#define GET_PIC_FLASH_POINTER               0x08
#define ERASE_PIC_APP_PROGRAM				0x09
#define SET_VOLTAGE                         0x10
#define SET_VOLTAGE_TIME                    0x11
#define SET_HASH_BOARD_ID                   0x12
#define GET_HASH_BOARD_ID                   0x13
#define SET_HOST_MAC_ADDRESS                0x14
#define ENABLE_VOLTAGE                      0x15
#define SEND_HEART_BEAT                     0x16
#define GET_PIC_SOFTWARE_VERSION            0x17
#define GET_VOLTAGE                         0x18
#define GET_DATE                            0x19
#define GET_WHICH_MAC                       0x20
#define GET_MAC                             0x21
#define WR_TEMP_OFFSET_VALUE                0x22
#define RD_TEMP_OFFSET_VALUE                0x23

#define HEART_BEAT_TIME_GAP                 10      // 10s
#define IIC_READ                            (1 << 25)
#define IIC_WRITE                           (~IIC_READ)
#define IIC_REG_ADDR_VALID                  (1 << 24)
// #define IIC_ADDR_HIGH_4_BIT                 (0x0A << 20)
#define IIC_CHAIN_NUMBER(x)                 ((x & 0x0f) << 16)
#define IIC_REG_ADDR(x)                     ((x & 0xff) << 8)

// general i2c command
#define REGADDRVALID    (1 << 24)
#define DEVICEADDR      (0x98 << 16)
#define RW              (1 << 16)   // write
#define REGADDR(addr)   (addr << 8)
#define DATA(data)      (data << 0)


// ECT218 register
#define LOCAL_TEMPERATURE_VALUE                 0x0
#define EXTERNAL_TEMPERATURE_VALUE_HIGH_BYTE    0x1
#define STATUS                                  0x2
#define CONFIGURATION                           0x3
#define EXTERNAL_TEMPERATURE_VALUE_LOW_BYTE     0x10
#define EXTERNAL_TEMPERATURE_OFFSET_HIGH_BYTE   0x11
#define EXTERNAL_TEMPERATURE_OFFSET_LOW_BYTE    0x12

// AT24C02
#define AT24C02_ADDRESS		0x50
#define EEPROM_LENGTH		256
#define HASH_ID_ADDR		0x80
#define VOLTAGE_ADDR		0x90
#define SENSOR_OFFSET_ADDR	0x98
#define VOLTAGE_SET_TIME	0xA0
#define VOLTAGE_SET_TIME	0xA0
#define FREQ_BADCORE_ADDR	0x00	// 128 bytes   0 - 0x7F

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;
typedef long long int int64_t;
typedef unsigned long long int uint64_t;

enum
{
    LOG_ERR,
    LOG_WARNING,
    LOG_NOTICE,
    LOG_INFO,
    LOG_DEBUG,
};

#define applog(prio, fmt, arg...) \
    if(prio <= LOG_DEBUG) \
        printf(fmt, ##arg)


struct configuration
{
    char name[64];
    bool AutoStart;
    bool Gray;
    int NonceMask; //???
    int DataCount;
    int PassCount1;
    int PassCount2;
    int PassCount3;
    int Freq;
    int Timeout;
    bool Regulate;
    int Value;
    int ReadIntervalTimeout;
    int AddrInterval;
    int AsicNum;
	int UseFreqPIC;
    int TestMode;
    int CheckChain;
    int CommandMode;
    int ValidNonce1;
    int ValidNonce2;
    int ValidNonce3;
    unsigned int Pic;
    unsigned int Voltage1;
    unsigned int Voltage2;
    unsigned int Voltage3;
	unsigned int final_voltage1;
	unsigned int final_voltage2;
	unsigned int final_voltage3;
	unsigned int freq_gap;
    int OpenCoreGap;
    int checktemp;
    unsigned int IICPic;
    unsigned int OpenCoreNum1;
    unsigned int OpenCoreNum2;
    unsigned int OpenCoreNum3;
    unsigned int OpenCoreNum4;
    unsigned int dac;
    unsigned int GetTempFrom;
    unsigned int TempSel;
    unsigned int TempSensor1;
    unsigned int TempSensor2;
    unsigned int TempSensor3;
    unsigned int TempSensor4;
    signed char DefaultTempOffset;
    int freq_e;
    int freq_m;
    int freq_a;
    int freq_t;
    int force_freq;
	int UseConfigVol;
    int StartTemp;
    int year;
    int month;
    int date;
    int hour;
    int minute;
    int second;
};

struct _CONFIG
{
    int dataCount;
    int passCount1;
    int passCount2;
    int passCount3;
    int freq;
    int timeout;
    int baud;
    bool regulate;
    int value;
    int testMode;
    int CommandMode;
    int ValidNonce1;
    int ValidNonce2;
    int ValidNonce3;
    unsigned int Pic;
    unsigned int Voltage1;
    unsigned int Voltage2;
    unsigned int Voltage3;
    int OpenCoreGap;
    int checktemp;
    unsigned int IICPic;
	int UseFreqPIC;
	unsigned int freq_gap;
    unsigned int OpenCoreNum1;
    unsigned int OpenCoreNum2;
    unsigned int OpenCoreNum3;
    unsigned int OpenCoreNum4;
    unsigned int dac;
    unsigned int GetTempFrom;
    unsigned int TempSel;
    unsigned char TempSensor1;
    unsigned char TempSensor2;
    unsigned char TempSensor3;
    unsigned char TempSensor4;
    signed char DefaultTempOffset;
    int freq_e;
    int freq_m;
    int freq_a;
    int freq_t;
    int force_freq;
	int UseConfigVol;
    signed char StartTemp;
    int year;
    int month;
    int date;
    int hour;
    int minute;
    int second;
};

struct cgpu_info
{
    FILE * fps[MAX_ASIC_NUM];

    pthread_t receive_id, show_id, pic_heart_beat_id, read_temp,freq_id, down_id;
	pthread_t send_id[BITMAIN_MAX_CHAIN_NUM];
    int device_fd;
    int lcd_fd;

    char workdataPathPrefix[64];
    struct work *works[MAX_ASIC_NUM];
    //int work_array[MAX_ASIC_NUM]; //work number of every asic
    uint32_t results[MAX_ASIC_NUM][MAX_WORK];
    int result_array[MAX_ASIC_NUM]; //return nonce number of every asic
    int subid[MAX_ASIC_NUM];
    int min_work_subid;
    int index;
    int valid_nonce;
    int err_nonce;
    int repeated_nonce;

    int start_key_fd;
    int red_led_fd;
    int green_led_fd;
    int beep_fd;

    int freq_e;
    int freq_m;
    int freq_a;
    int freq_t;
    unsigned int    chain_num;
    unsigned short int  frequency;
    unsigned int    CommandMode;    // 1:fil  0:vil
    unsigned int    chain_exist[BITMAIN_MAX_CHAIN_NUM];
    unsigned int    timeout;
    unsigned char   chain_asic_num[BITMAIN_MAX_CHAIN_NUM];
    unsigned char   baud;
    unsigned short int  freq[BITMAIN_MAX_CHAIN_NUM];
    unsigned int max_asic_num_in_one_chain;
    unsigned char temp_sel;
    unsigned char rfs;
    unsigned char tfs;
    signed char T1_offset_value;
    signed char T2_offset_value;
    signed char T3_offset_value;
    signed char T4_offset_value;
};

struct work
{
    int     id;
    uint32_t nonce; /* For devices that hash sole work */
    unsigned char data[12];
    unsigned char   midstate[32];
};

struct reg_content
{
    unsigned int reg_value;
    unsigned char crc;
    unsigned char chain_number;
} __attribute__((packed, aligned(4)));

struct reg_buf
{
    unsigned int p_wr;
    unsigned int p_rd;
    unsigned int reg_value_num;
    unsigned int loop_back;
    pthread_mutex_t spinlock;
    struct reg_content reg_buffer[MAX_NONCE_NUMBER_IN_FIFO];
};

struct freq_pll
{
    const char *freq;
    unsigned int fildiv1;
    unsigned int fildiv2;
    unsigned int vilpll;
};
static struct freq_pll freq_pll_1385[] =
{
    {"100",0x020040, 0x0420, 0x200241},
    {"125",0x028040, 0x0420, 0x280241},
    {"150",0x030040, 0x0420, 0x300241},
    {"175",0x038040, 0x0420, 0x380241},
    {"200",0x040040, 0x0420, 0x400241},
    {"225",0x048040, 0x0420, 0x480241},
    {"250",0x050040, 0x0420, 0x500241},
    {"275",0x058040, 0x0420, 0x580241},
    {"300",0x060040, 0x0420, 0x600241},
    {"325",0x068040, 0x0420, 0x680241},
    {"350",0x070040, 0x0420, 0x700241},
    {"375",0x078040, 0x0420, 0x780241},
    {"400",0x080040, 0x0420, 0x800241},
    {"404",0x061040, 0x0320, 0x610231},
    {"406",0x041040, 0x0220, 0x410221},
    {"408",0x062040, 0x0320, 0x620231},
    {"412",0x042040, 0x0220, 0x420221},
    {"416",0x064040, 0x0320, 0x640231},
    {"418",0x043040, 0x0220, 0x430221},
    {"420",0x065040, 0x0320, 0x650231},
    {"425",0x044040, 0x0220, 0x440221},
    {"429",0x067040, 0x0320, 0x670231},
    {"431",0x045040, 0x0220, 0x450221},
    {"433",0x068040, 0x0320, 0x680231},
    {"437",0x046040, 0x0220, 0x460221},
    {"441",0x06a040, 0x0320, 0x6a0231},
    {"443",0x047040, 0x0220, 0x470221},
    {"445",0x06b040, 0x0320, 0x6b0231},
    {"450",0x048040, 0x0220, 0x480221},
    {"454",0x06d040, 0x0320, 0x6d0231},
    {"456",0x049040, 0x0220, 0x490221},
    {"458",0x06e040, 0x0320, 0x6e0231},
    {"462",0x04a040, 0x0220, 0x4a0221},
    {"466",0x070040, 0x0320, 0x700231},
    {"468",0x04b040, 0x0220, 0x4b0221},
    {"470",0x071040, 0x0320, 0x710231},
    {"475",0x04c040, 0x0220, 0x4c0221},
    {"479",0x073040, 0x0320, 0x730231},
    {"481",0x04d040, 0x0220, 0x4d0221},
    {"483",0x074040, 0x0320, 0x740231},
    {"487",0x04e040, 0x0220, 0x4e0221},
    {"491",0x076040, 0x0320, 0x760231},
    {"493",0x04f040, 0x0220, 0x4f0221},
    {"495",0x077040, 0x0320, 0x770231},
    {"500",0x050040, 0x0220, 0x500221},
    {"504",0x079040, 0x0320, 0x790231},
    {"506",0x051040, 0x0220, 0x510221},
    {"508",0x07a040, 0x0320, 0x7a0231},
    {"512",0x052040, 0x0220, 0x520221},
    {"516",0x07c040, 0x0320, 0x7c0231},
    {"518",0x053040, 0x0220, 0x530221},
    {"520",0x07d040, 0x0320, 0x7d0231},
    {"525",0x054040, 0x0220, 0x540221},
    {"529",0x07f040, 0x0320, 0x7f0231},
    {"531",0x055040, 0x0220, 0x550221},
    {"533",0x080040, 0x0320, 0x800231},
    {"537",0x056040, 0x0220, 0x560221},
    {"543",0x057040, 0x0220, 0x570221},
    {"550",0x058040, 0x0220, 0x580221},
    {"556",0x059040, 0x0220, 0x590221},
    {"562",0x05a040, 0x0220, 0x5a0221},
    {"568",0x05b040, 0x0220, 0x5b0221},
    {"575",0x05c040, 0x0220, 0x5c0221},
    {"581",0x05d040, 0x0220, 0x5d0221},
    {"587",0x05e040, 0x0220, 0x5e0221},
    {"593",0x05f040, 0x0220, 0x5f0221},
    {"600",0x060040, 0x0220, 0x600221},
    {"606",0x061040, 0x0220, 0x610221},
    {"612",0x062040, 0x0220, 0x620221},
    {"618",0x063040, 0x0220, 0x630221},
    {"625",0x064040, 0x0220, 0x640221},
    {"631",0x065040, 0x0220, 0x650221},
    {"637",0x066040, 0x0220, 0x660221},
    {"643",0x067040, 0x0220, 0x670221},
    {"650",0x068040, 0x0220, 0x680221},
    {"656",0x069040, 0x0220, 0x690221},
    {"662",0x06a040, 0x0220, 0x6a0221},
    {"668",0x06b040, 0x0220, 0x6b0221},
    {"675",0x06c040, 0x0220, 0x6c0221},
    {"681",0x06d040, 0x0220, 0x6d0221},
    {"687",0x06e040, 0x0220, 0x6e0221},
    {"693",0x06f040, 0x0220, 0x6f0221},
    {"700",0x070040, 0x0220, 0x700221},
    {"706",0x071040, 0x0220, 0x710221},
    {"712",0x072040, 0x0220, 0x720221},
    {"718",0x073040, 0x0220, 0x730221},
    {"725",0x074040, 0x0220, 0x740221},
    {"731",0x075040, 0x0220, 0x750221},
    {"737",0x076040, 0x0220, 0x760221},
    {"743",0x077040, 0x0220, 0x770221},
    {"750",0x078040, 0x0220, 0x780221},
    {"756",0x079040, 0x0220, 0x790221},
    {"762",0x07a040, 0x0220, 0x7a0221},
    {"768",0x07b040, 0x0220, 0x7b0221},
    {"775",0x07c040, 0x0220, 0x7c0221},
    {"781",0x07d040, 0x0220, 0x7d0221},
    {"787",0x07e040, 0x0220, 0x7e0221},
    {"793",0x07f040, 0x0220, 0x7f0221},
    {"800",0x080040, 0x0220, 0x800221},
    {"825",0x042040, 0x0120, 0x420211},
    {"850",0x044040, 0x0120, 0x440211},
    {"875",0x046040, 0x0120, 0x460211},
    {"900",0x048040, 0x0120, 0x480211},
    {"925",0x04a040, 0x0120, 0x4a0211},
    {"950",0x04c040, 0x0120, 0x4c0211},
    {"975",0x04e040, 0x0120, 0x4e0211},
};

struct vil_work
{
    uint8_t type;       // Bit[7:5]: Type,fixed 0x01.   Bit[4:0]:Reserved
    uint8_t length;     // data length, from Byte0 to the end.
    uint8_t wc_base;    // Bit[7]: Reserved.    Bit[6:0]: Work count base, muti-Midstate, each Midstate corresponding work count increase one by one.
    uint8_t mid_num;    // Bit[7:3]: Reserved   Bit[2:0]: MSN, midstate num,now support 1,2,4.
    //uint32_t sno;       // SPAT mode??Start Nonce Number    Normal mode??Reserved.
    uint8_t midstate[32];
    uint8_t data2[12];
};

struct vil_work_1387
{
    uint8_t work_type;
    uint8_t chain_id;
    uint8_t reserved1[2];
    uint32_t work_count;
    uint8_t data[12];
    uint8_t midstate[32];
};

int bitmain_axi_init();
signed char read_asic_temperature(int chainIndex);
void pic_heart_beat(unsigned char chain);
int get_buffer_space(void);
void set_TW_write_command(unsigned int *value);
void set_TW_write_command_vil(unsigned int *value);
int get_nonce_number_in_fifo(void);
int get_return_nonce(unsigned int *buf);
int get_chain_number(int chainIndex);
void set_pic_voltage(unsigned char chain, unsigned char voltage);
void disable_pic_dac(unsigned char chain);
unsigned char reset_iic_pic(unsigned char chain);
unsigned char erase_pic_freq(unsigned char chain);
unsigned char flash_pic_freq(unsigned char chain,unsigned char *buf1);
unsigned char read_pic_freq(unsigned char chain,unsigned char *buf1);
void reset_fpga();
void reset_hashboard();
void reset_global_arg();
void check_chain();
unsigned char jump_to_app_from_loader(unsigned char chain);
unsigned char get_pic_voltage(unsigned char chain);
void enable_pic_dac(unsigned char chain);
void set_dhash_acc_control(unsigned int value);
int get_dhash_acc_control(void);
void check_asic_reg(unsigned char reg);
void set_frequency_with_addr_plldatai(int pllindex,unsigned char mode,unsigned char addr, unsigned char chain);
void set_frequency(int chainIndex, unsigned short int frequency);
void software_set_address();
void set_baud(unsigned char bauddiv);
void set_time_out_control(unsigned int value);
void open_core(bool nullwork_enable);
int calculate_core_number(unsigned int actual_core_number);
void set_nonce_fifo_interrupt(unsigned int value);
int get_nonce_fifo_interrupt(void);
int get_result(int chainIndex, int passCount, int validnonce);
bool last_all_pass(int chainIndex);
int configMiner();
int calculate_asic_number(unsigned int actual_asic_number);

void set_hcnt(unsigned int hcnt);
void set_tickmask(unsigned int tickmask);

void set_PWM(unsigned char pwm_percent);
void UpdateTestResultFlag(int chainIndex, int passCount);

void check_asic_reg_oneChain(int chainIndex, unsigned char reg);

int getVolValueFromPICvoltage(unsigned char vol_pic);
unsigned char getPICvoltageFromValue(int vol_value);
void software_set_address_onChain(int chainIndex);
void open_core_onChain(int chainIndex, int coreNum, int opencore_num, bool nullwork_enable);
void set_baud_onChain(int chainIndex, unsigned char bauddiv);
bool check_fan();
unsigned int get_crc_count();
void reset_crc_count();
unsigned int get_Hardware_version();

#define TESTPATTEN_DOWNLOAD_URL_PREFIX	"http://192.168.60.30/download/minertest/minertest64_"
#define TESTPATTEN_DIR	"/etc/config/32xPatten"
int get_works_32X();
bool isC5_Board();
extern int fpga_version;

#include "miner_type.h"		// use setminertype to define miner type in this file instead of belows!!!
//#define R4		// if defined , for R4  63 chips
//#define S9_PLUS	// if defined , for T9  57 chips
//#define S9_63	// if defined , for S9  63 chips
//#define T9_18 	// if defined , for T9+  18 chips

#undef NEED_AUTH_SEARCHFREQ	// if defined, this program need get auth from network before start test different freq on hashbard! Only use in factory testing farm！

#undef SPEICAL_FIXED_2TEMP_TO_PIC	//if defined, will set 2 temp address into PIC , 32 and 62, temp offset = -4
									// normally, must be undefined!!!

#define RESET_KEEP_TIME		3	//keep reset signal for 1 seconds
#define CLOSE_DC_FOR_COOL_BEFORE_TESTPATTEN
#define USE_SINGLE_SEND_THREAD		// if defined, we will send works in one thread, from chain[0] to chain[1] ....
#define ENABLE_RATE_LIMIT_BY_VOLTAGE	// limit hashrate by voltage
#define ENABLE_SEARCH_LOGFILE	//enable log info into kernel info web page.
#undef LOG_CHIPS_CORE_DETAIL	// if enabled , will show details nonce number info for chips and cores, open for debug
#define ENABLE_CHECK_PIC_FLASH_ADDR		// if enabled, will check PIC FLASH ADDR value , set and read back to compare from PIC
#define ENABLE_RESTORE_PIC_APP		// if enabled, will restore PIC APP when the version is not correct!!!

// ALL below micro has been tested, they are not useful, we just keep them
/////////////////////////////////////////////////////////////////////////
#undef ENABLE_DOWNLOAD_32XPATTEN
#undef USE_OPENCORE_ONEBYONE
#undef USE_OPENCORE_TWICE
#undef USE_LOWFREQ_OPENCORE	// use PRE_OPENCORE_FREQ to open core before set frequency
#undef CHECK_TICKETMASK_AFTER_TEST
#undef ENABLE_REGISTER_CRC_CHECK	//if defined, will drop the register buffer with crc error!  s9+ has problem with this flag!!!
#undef PATTEN_TEST_ONE_BY_ONE	// if defined, will test patten on one chain per time at last step!
#undef NEED_PRE_HEAT
#undef ENABLE_TEMP_PROCESS
////////////////////// above are not used micro ///////////////////////////

//////////////////////// BELOW is for DEBUG!! ///////////////////////////////
#undef DEBUG_STOP_AFTER_SEARCHFREQ_FOR_REPAIRE_TEST	// if defined, it is used for the third party repairing team to test machine! will stop after search freq, only print result in kernel log, will not mining!!!
#undef DEBUG_READ_TEMP			// if defined, we read temp at init and after test patten
#undef DEBUG_PIC_UPGRADE		// if defined, we will force to write PIC program data once!
#undef DEBUG_TEST_ONE_BOARD_ONLY	// if defined, we will fix a fake value on check_chain!!!, only test one board for T9+
#undef DEBUG_NOT_CHECK_FAN_NUM		// if defined, we will ignore fan number checking, will keep run even without any fan!!!
#undef DEBUG_IGNORE_T9_18_PIC_CMD	// if defined, will ignore all process of PIC on T9_18, HARDWARE will keep voltage for hashboard!!!
#undef DEBUG_TEST_PIC_FLASH_RW		// if defined, will call test_PIC_flash_rw func to test, while(1) loop, will not exit!!!
#undef CLEMENT_DEBUG_T9_18_IIC
#undef FORCE_8xPATTENT_TEST	// if defined, and force freq =1 , we will read freq and voltage from PIC, and then jump over search freq, only do 8xPatten test and down freq or up voltage
#undef DEBUG_STOP_WHEN_ASICNUM_NOTENOUGH    // if defined, will stop when asicnum is not enough!!!
#undef DEBUG_MODE
#ifdef DEBUG_MODE
#define DEBUG_BOARD_NUM		2
#endif
#undef DEBUG_DC_OPEN_CLOSE		// if defined, will debug in loop, open dc and close dc ...
#undef DEBUG_REBOOT	// will reboot every 30mins, need cool miner, only for test
/////////////////////////// ABOVE is for DEBUG !! ////////////////////////////

#ifdef R4
// below are used on R4 for using one app to support C5 and XILINX board
extern int R4_MAX_VOLTAGE;
extern int START_VOLTAGE;
extern int RETRY_VOLTAGE;
extern int HIGHEST_FREQ_INDEX;

#undef USE_DOWN_VOLTAGE_LIMIT_FREQ		//if enabled, we will use voltage and avg freq to limit highest freq!
#define ENABLE_HIGH_VOLTAGE_OPENCORE

#define TIMEOUT_PERCENT		7 	//7 is min

#define ASIC_NUM			63
#define DC_AREA_NUM			21	// 21*3=63 chips, 3 chips has one DC

#define TEMP_ASIC_INDEX		3 //   62:S9    3:R4

#define R4_MAX_VOLTAGE_C5			890
#define START_VOLTAGE_C5			860
#define RETRY_VOLTAGE_C5			870
#define HIGHEST_FREQ_INDEX_C5		44	// 500M

#define R4_MAX_VOLTAGE_XILINX		910
#define START_VOLTAGE_XILINX		870
#define RETRY_VOLTAGE_XILINX		890
#define HIGHEST_FREQ_INDEX_XILINX	66	// 70:618M

#define PIC_VERSION			0x03

#define MINER_FAN_NUM		1	// fan number, will check when searching freq

#define RETRY_FREQ_INDEX	12	// 12:400M, if search base freq < 400M, will switch to use RETRY_VOLTAGE to search again.
#define LOWEST_FREQ_INDEX	4	// 8:300M       6:250M		4:200M		8:300M for debug
#define ACCEPT_BADCORE_FREQ_INDEX	HIGHEST_FREQ_INDEX	// we just accept bad cores for all freq when test in 8xPatten, so we set highest freq here

#define BAUD_LEVEL			0

#define FAN_SPEED_PWM		100

#define HIGHEST_VOLTAGE_LIMITED_HW		940	//measn the largest voltage, hw can support
#define USE_NEW_RESET_FPGA

#define USE_SEARCH_BASEFREQ_MODE		1	// normal mode

#define FINAL_TESTPATTEN_MODE			0

#undef LAST_TESTPATTEN_CRITICAL
#undef CLOSE_OPEN_DC_TESTPATTEN
#undef CLOSE_OPEN_DC	// if defined, will close dc , and enable dc when search and test patten, it can let some chip with too many bad cores to be success.
#undef USE_PREINIT_OPENCORE	// if defined, we will open core at first ,then get asicnum and do other init process
#endif

#ifdef S9_PLUS
#define S9_PLUS_VOLTAGE2	//if defined, then it support S9+ new board with new voltage controller
#undef USE_DOWN_VOLTAGE_LIMIT_FREQ		//if enabled, we will use voltage and avg freq to limit highest freq!
#define ENABLE_HIGH_VOLTAGE_OPENCORE

#define TIMEOUT_PERCENT		7 	//7 is min

#define ASIC_NUM			57
#define DC_AREA_NUM			19	// 19*3=57 chips, 3 chips has one DC

#define TEMP_ASIC_INDEX		2

// this is for normal miners.
#define START_VOLTAGE		830	//830
#define RETRY_VOLTAGE		860	//860

#define BAUD_LEVEL			0

#define PIC_VERSION			0x03

#define MINER_FAN_NUM		2	// fan number, will check when searching freq

#define RETRY_FREQ_INDEX	12	// 12:400M, if search base freq < 400M, will switch to use RETRY_VOLTAGE to search again.
#define LOWEST_FREQ_INDEX	4	// 8:300M       6:250M		4:200M		8:300M for debug
#define HIGHEST_FREQ_INDEX	83	// 850M:100 700M:82  668M:77	 83:706M   	12:400M for debug
#define ACCEPT_BADCORE_FREQ_INDEX	HIGHEST_FREQ_INDEX	// we just accept bad cores for all freq when test in 8xPatten, so we set highest freq here

#define FAN_SPEED_PWM					100

#define HIGHEST_VOLTAGE_LIMITED_HW		970	//measn the largest voltage, hw can support
#define USE_NEW_RESET_FPGA

#define USE_SEARCH_BASEFREQ_MODE		2	// 3: search freq from 200M to 600M , and swtich voltage at 500M
											// 2 : search freq from 200M to 600M and switch voltage at 400M
											// 4 : search freq from 650M to 200M , and switch voltage at 500M

#define LAST_TESTPATTEN_CRITICAL	// if defined, the last test patten must pass 3 times continuously!!!!   and will not accept badcore when freq < 400M
#define FINAL_TESTPATTEN_MODE			3

#define CLOSE_OPEN_DC_TESTPATTEN
#undef CLOSE_OPEN_DC	// if defined, will close dc , and enable dc when search and test patten, it can let some chip with too many bad cores to be success.
#undef USE_PREINIT_OPENCORE	// if defined, we will open core at first ,then get asicnum and do other init process
#endif

#ifdef S9_63
#undef DEBUG_S9_INFAN_TEMP
#undef USE_DOWN_VOLTAGE_LIMIT_FREQ		//if enabled, we will use voltage and avg freq to limit highest freq!
#define ENABLE_HIGH_VOLTAGE_OPENCORE

#define TIMEOUT_PERCENT		7 	//7 is min

#define ASIC_NUM			63
#define DC_AREA_NUM			21	// 21*3=63 chips, 3 chips has one DC

#ifdef DEBUG_S9_INFAN_TEMP
#define TEMP_ASIC_INDEX		25
#else
#define TEMP_ASIC_INDEX		62 //   62:S9    3:R4
#endif

// this is for normal miners.
#define START_VOLTAGE		860
#define RETRY_VOLTAGE		890

#define PIC_VERSION			0x03

#define MINER_FAN_NUM		2	// fan number, will check when searching freq

#define RETRY_FREQ_INDEX	12	// 12:400M, if search base freq < 400M, will switch to use RETRY_VOLTAGE to search again.
#define LOWEST_FREQ_INDEX	4	// 8:300M       6:250M		4:200M		8:300M for debug
#define HIGHEST_FREQ_INDEX	85	// 850M:100 700M:82  668M:77	85:718M	12:400M for debug
#define ACCEPT_BADCORE_FREQ_INDEX	HIGHEST_FREQ_INDEX //44:500M  66:600M 	// we just accept bad cores for all freq when test in 8xPatten, so we set highest freq here

#define BAUD_LEVEL			0

#define FAN_SPEED_PWM		100

#define HIGHEST_VOLTAGE_LIMITED_HW		940	//measn the largest voltage, hw can support
#define USE_NEW_RESET_FPGA

#define USE_SEARCH_BASEFREQ_MODE		2	// 3 : search freq from 200M to 600M , and swtich voltage at 500M
											// 2 : search freq from 200M to 600M and switch voltage at 400M
											// 4 : search freq from 650M to 200M , and switch voltage at 500M

#define LAST_TESTPATTEN_CRITICAL	// if defined, the last test patten must pass 3 times continuously!!!!   and will not accept badcore when freq < 400M
#define FINAL_TESTPATTEN_MODE			3

#define CLOSE_OPEN_DC_TESTPATTEN
#undef CLOSE_OPEN_DC	// if defined, will close dc , and enable dc when search and test patten, it can let some chip with too many bad cores to be success.
#undef USE_PREINIT_OPENCORE	// if defined, we will open core at first ,then get asicnum and do other init process
#endif

#ifdef T9_18		// T9+  18 chips, each board has 3 chains with one PIC DC controller
#undef ALL_CHAIN_INTEST	// if defined, even testDone=true, still open core on all the chains!
#undef USE_DOWN_VOLTAGE_LIMIT_FREQ		//if enabled, we will use voltage and avg freq to limit highest freq!
#define ENABLE_HIGH_VOLTAGE_OPENCORE		// for T9_18 , it work bad, if switch voltage in working state.

#define TIMEOUT_PERCENT		7 	//7 is min

#define ASIC_NUM			18
#define DC_AREA_NUM			18	// 18*1=18 chips, 1 chip has one DC

#define TEMP_ASIC_INDEX		1

// this is for normal miners.
#define START_VOLTAGE		810		//810
#define RETRY_VOLTAGE		840		//840

#define PIC_VERSION			0x03

#define MINER_FAN_NUM		2	// fan number, will check when searching freq

#define RETRY_FREQ_INDEX	12	// 12:400M, if search base freq < 400M, will switch to use RETRY_VOLTAGE to search again.
#define LOWEST_FREQ_INDEX	4	// 8:300M       6:250M		4:200M		8:300M for debug
#define HIGHEST_FREQ_INDEX	85	// 850M:100 700M:82  668M:77		12:400M for debug
#define ACCEPT_BADCORE_FREQ_INDEX	HIGHEST_FREQ_INDEX	// we just accept bad cores for all freq when test in 8xPatten, so we set highest freq here

#define BAUD_LEVEL			0

#define FAN_SPEED_PWM		100

#define HIGHEST_VOLTAGE_LIMITED_HW		930	//measn the largest voltage, hw can support
#define USE_NEW_RESET_FPGA

#define USE_SEARCH_BASEFREQ_MODE		4	// 3 : search freq from 200M to 600M , and swtich voltage at 500M
											// 2 : search freq from 200M to 600M and switch voltage at 400M
											// 4 : search freq from 650M to 200M , and switch voltage at 500M

#define LAST_TESTPATTEN_CRITICAL	// if defined, the last test patten must pass 3 times continuously!!!!   and will not accept badcore when freq < 400M
#define FINAL_TESTPATTEN_MODE			1

// T9_18 has 3 chains on one PIC DC controller, so we can not close DC for each chain, then we open DC, and keep it open when searching freq!
// when change the voltage, we can use one function to get the highest voltage of 3 chains, then only set highest voltage on PIC!
#undef CLOSE_OPEN_DC_TESTPATTEN
#undef CLOSE_OPEN_DC	// if defined, will close dc , and enable dc when search and test patten, it can let some chip with too many bad cores to be success.
#undef USE_PREINIT_OPENCORE	// if defined, we will open core at first ,then get asicnum and do other init process
#endif

#ifdef USE_PREINIT_OPENCORE
#define PRE_OPENCORE_FREQ	8	//4:200M   8:300M    12:400M
#define SET_TICKETMASK_BEFORE_TEST		// if enabled, we will set ticket mask=0 and hcn=0 before test patten
#else
#undef SET_TICKETMASK_BEFORE_TEST		// if enabled, we will set ticket mask=0 and hcn=0 before test patten
#endif

#define ASIC_TYPE			1387	// 1385 or  1387
#define CHIP_ADDR_INTERVAL	4	// fix chip address interval = 4 
#define DEFAULT_BAUD_VALUE	26
#define ASIC_CORE_NUM		114

#ifdef DEBUG_MODE
#undef RETRY_FREQ_INDEX
#undef LOWEST_FREQ_INDEX
#undef HIGHEST_FREQ_INDEX

#define RETRY_FREQ_INDEX	6	// 12:400M, if search base freq < 400M, will switch to use RETRY_VOLTAGE to search again.
#define LOWEST_FREQ_INDEX	4	// 8:300M       6:250M		4:200M		8:300M for debug
#define HIGHEST_FREQ_INDEX	8	// 850M:100 700M:82  668M:77		12:400M for debug
#endif

#define REBOOT_TEST_NUM		2	// save into file

extern int TEST_MODE_OK_NUM;
#define TEST_MODE_OK_NUM_8X		3	// if 8xPatten test mode test OK counter >= TEST_MODE_OK_NUM, then this board is OK,  4 
#define TEST_MODE_OK_NUM_32X	2	// if 32xPatten test mode test OK counter >= TEST_MODE_OK_NUM, then this board is OK,  3 

#define SEARCH_FREQ_CHANCE_NUM	2	// give each board 2 chances to search freq, the first failed, we can add voltage  SEARCH_VOLTAGE_ADD_STEP to search for next chance
#define SEARCH_VOLTAGE_ADD_STEP	30	// means, each chance will add 0.3V to search freq again.

#define SEARCH_BASEFREQ_PATTEN_NUM		912
#define SEARCH_BASEFREQ_NONCE_NUM		(SEARCH_BASEFREQ_PATTEN_NUM*ASIC_NUM)

#define SEARCH_FREQ_PATTEN_NUM			114
#define SEARCH_FREQ_NONCE_NUM			(SEARCH_FREQ_PATTEN_NUM*ASIC_NUM)

#define TESTMODE_PATTEN_NUM_8X			912
#define TESTMODE_NONCE_NUM_8X			(TESTMODE_PATTEN_NUM_8X*ASIC_NUM)

#define TESTMODE_PATTEN_NUM_32X			3648
#define TESTMODE_NONCE_NUM_32X			(TESTMODE_PATTEN_NUM_32X*ASIC_NUM)

#define DEFAULT_TEMP_OFFSET			-70

#define MAX_BAD_CORE_NUM	14	// we can accept one ASIC has 14 bad cores.

typedef enum{
	SEARCHING_FREQ,
	SEARCH_SUCCESS,
	SEARCH_FAILED,
	TEST_PATTEN_MODE,
}SEARCH_STATUS_VALUE;

#define SEND_WAIT_TIMEOUT	120 // unit is 100ms
#define RECV_WAIT_TIMEOUT	20 // unit is 100ms

#ifdef DEBUG_STOP_WHEN_ASICNUM_NOTENOUGH
#define NOBOARD_RETRY_COUNT	6000
#else
#ifdef T9_18
#define NOBOARD_RETRY_COUNT	0	// T9 can not retry close DC and open DC when asic num is not enought!!! because one DC controll 3 chains!!!
#else
#define NOBOARD_RETRY_COUNT	6
#endif
#endif

/*
T9+ E2PROM:  (max size is 128 bytes, currently we used 7+31*3=100 bytes)
header: 7 bytes
1: magic number  (0x23 , for flag of having freq)
6: MAC  (recorde controller board MAC)

each chain: 31 bytes
1: base freq (the highest freq, can test 8xPatten OK with same freq for all asics)
1: backup voltage (eg. 8.9V  we use 890,  save 890/10=89 into this byte. this voltage is working voltage)
1: added voltage (eg. 5, means we add 0.5V based on the voltage searching freq.   use backup voltage decreae added voltaged = search freq voltage)
1: testpatten temp
18: freq of asic  
9: bad core num  (4 bits for one asic, store bad core num)

real index=((chainIndex/3)*3)
sub chain offset = 7+(chainIndex%3)*31 
chain_pic_buf[((chainIndex/3)*3)][7+(chainIndex%3)*31] 

we defined chain_pic_buf[16], but only chain_pic_buf[0], chain_pic_buf[3], chain_pic_buf[6] ... has data, others are not used.
*/

#ifdef T9_18
#define IIC_ADDR_HIGH_4_BIT					(0x04 << 20)
#define EEPROM_ADDR_HIGH_4_BIT				(0x0A << 20)
#define IIC_SELECT(x)						((x & 0x03) << 26)

unsigned int get_iic();
unsigned char set_iic(unsigned int data);
unsigned char T9_plus_write_pic_iic(bool read, bool reg_addr_valid, unsigned char reg_addr, unsigned char which_iic, unsigned char data);
int dsPIC33EP16GS202_jump_to_app_from_loader(unsigned char which_iic);

int dsPIC33EP16GS202_enable_pic_dc_dc(unsigned char which_iic, unsigned char enable);
void getPICChainIndexOffset(int chainIndex, int *pChain, int *pOffset);

#else
#define IIC_ADDR_HIGH_4_BIT					(0x0A << 20)
#endif

#endif
