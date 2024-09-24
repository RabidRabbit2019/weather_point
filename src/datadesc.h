// data structures description

#ifndef _datadesc_h_
#define _datadesc_h_

#include <stdint.h>

// классы входов
#define INPUT_CLASS_NOT_EXISTS			 0 // вход не фунциклирует или его просто нет
#define INPUT_CLASS_ALARM				     1 // охранный вход
#define INPUT_CLASS_BUTTON				   2 // кнопка
#define INPUT_CLASS_TSENSOR				   3 // датчик температуры (десятые доли градуса)
#define INPUT_CLASS_PSENSOR				   4 // датчик давления (мм ртутного столба)
#define INPUT_CLASS_HSENSOR				   5 // датчик влажности (относительной, проценты)
#define INPUT_CLASS_IDREADER			   6 // считыватель идентификатора (прочитанный идентификатор)
#define INPUT_CLASS_FIRE_SMOKE       7 // пожарный датчик задымления 
#define INPUT_CLASS_FIRE_TEMP        8 // пожарный датчик температуры
#define INPUT_CLASS_LUMINOSITY       9 // датчик освещённости (?)
#define INPUT_CLASS_ACCURRENT       10 // амперметр переменного тока (миллиамперы)
#define INPUT_CLASS_DCCURRENT       11 // амперметр постоянного тока (миллиамперы)
#define INPUT_CLASS_ACVOLTMETER	    12 // вольтметр переменного тока (милливольты)
#define INPUT_CLASS_DCVOLTMETER     13 // вольтметр постоянного тока (милливольты)
#define INPUT_CLASS_WINDDIRECTION   14 // датчик направления ветра (градусы, север = 0, запад = 270)
#define INPUT_CLASS_WINDFORCE       15 // датчик скорости ветра (анемометр) см/с
#define INPUT_CLASS_WATERPRESSURE   16 // датчик давления воды (десятые доли атмосферы)
#define INPUT_CLASS_END_MARKER		  17 // это неиспользуемое граничное значение

// классы выходов
#define OUTPUT_CLASS_NOT_EXISTS			 0 // выхода нет ;)
#define OUTPUT_CLASS_RELAY				   1 // реле
#define OUTPUT_CLASS_LED				     2 // светодиод
#define OUTPUT_CLASS_OC              3 // открытый коллектор
#define OUTPUS_CLASS_DAC             4 // ЦАП (настоящий или ШИМ)
#define OUTPUT_CLASS_END_MARKER			 5 // это неиспользуемое граничное значение


#pragma pack(push,1)
struct rs485Header{
  // одноразовый ключ, передаётся в открытом виде, little endian
  // зачем - чтобы одинаковые пакеты выглядели по-разному
  // шифруется 
  uint32_t msgOnce;
	uint8_t destinationAddr;
	uint8_t sourceAddr;
  uint8_t msgCode;
  //
  uint8_t get_actual_packet_length() const noexcept;
};

// здесь кроме crc16 ничего не дожно быть
struct rs485Footer{
	uint16_t crc; // little endian
};


#define MIN_ADDRESS_VALUE			    1
#define MAX_ADDRESS_VALUE			    20

#define MAX_MODULES_COUNT			    16

#define MAX_OUTPUTS_PER_MODULE		8
#define MAX_INPUTS_PER_MODULE     8

#define MODULE_FLAG_ONLY_OUTPUTS	0
#define MODULE_FLAG_ONLY_INPUTS		1


struct extModuleDesc
{
	uint8_t address;
	uint8_t flags;
	uint8_t reserved1;
	uint8_t reserved2;
	uint8_t outputClasses[MAX_OUTPUTS_PER_MODULE];
	uint8_t inputClasses[MAX_INPUTS_PER_MODULE];
};

struct currentConfigDesc
{
	char phoneNumber1[16];
	char phoneNumber2[16];
	extModuleDesc modulesDescription[MAX_MODULES_COUNT];
	uint32_t serialNum;
	uint32_t crc;
};

extern currentConfigDesc currentConfig;


#define MODULE_STATE_OFFLINE		0
#define MODULE_STATE_ONLINE			1

#define MODULE_STATE_FLAG_ARMED		0 // поставлен на охрану
#define MODULE_STATE_FLAG_AC_DOWN	1 // нет сетевого питания
#define MODULE_STATE_FLAG_ACC_DOWN	2 // аккумулятор разряжен
#define MODULE_STATE_TAMPER_ALARM   3 // флаг вскрытия корпуса

#define OUTPUT_STATE_UNKNOWN        0
#define OUTPUT_STATE_OFF            1
#define OUTPUT_STATE_ON             2
#define OUTPUT_STATE_END_MARKER		3

#define INPUT_STATE_UNKNOWN         0
#define INPUT_STATE_OK              1
#define INPUT_STATE_BREAK           2
#define INPUT_STATE_SHORT           3
#define INPUT_STATE_ALARM           4
#define INPUT_STATE_FAILURE         5
#define INPUT_STATE_END_MARKER		  6


struct extModuleInputStates
{
	uint8_t  mStates[MAX_INPUTS_PER_MODULE];
	int16_t  mValues[MAX_INPUTS_PER_MODULE];
};


struct extModuleState{
	uint8_t  onlineState;
	uint8_t  flags;
	uint8_t  reserved1;
	uint8_t  reserved2;
	uint8_t  outputStates[MAX_OUTPUTS_PER_MODULE];
  extModuleInputStates mInputStates;
};

struct extModulesStatesDesc{
	extModuleState modulesState[MAX_MODULES_COUNT];
	uint32_t serialNum;
	uint32_t crc;
};

extern extModulesStatesDesc currentExtStates;

enum triStateBool { tsbFalse = 0, tsbTrue, tsbUnknown };


#define MSG_START_MARK              '#'
#define MSG_STOP_MARK               '$'

#define CMD_GET_INFO                        0x01
#define CMD_GET_EVENT                       0x02
#define CMD_GET_STATES                      0x03
#define CMD_SET_OUTPUT_STATE                0x04
#define CMD_GET_INPUT_VALUE                 0x05
#define CMD_SET_DATETIME                    0x06
#define CMD_MODULE_ARM                      0x07 // модуль "на охрану", т.е. он начинает транслировать события
                                                 // от входов типа INPUT_CLASS_ALARM
#define CMD_MODULE_DISARM                   0x08 // "снять с охраны" модуль, т.е. прекратить трансляцию событий
                                                 // от входов типа INPUT_CLASS_ALARM


#define ANSWER_ACK                          0x81
#define ANSWER_MODULE_INFO                  0x82
#define ANSWER_MODULE_EVENT                 0x83
#define ANSWER_MODULE_STATES                0x84
#define ANSWER_INPUT_VALUE                  0x85
#define ANSWER_NACK                         0x86

struct localDateTime
{
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    //
    localDateTime() { year=0; month=1; day=1; hours=0; minutes=0; seconds=0; };
};


struct cmdSimple
{
    rs485Header header;
    rs485Footer footer;
};


struct cmdSetDateTime
{
    rs485Header header;
    localDateTime dt;
    rs485Footer footer;
};


struct cmdSetOutputState
{
    rs485Header header;
    int8_t outputIndex;
    uint16_t newState;
    rs485Footer footer;
};


struct cmdGetInputState
{
    rs485Header header;
    int8_t inputIndex;
    rs485Footer footer;
};


struct aswModuleAck
{
    rs485Header header;
    rs485Footer footer;
};


struct aswModuleInfo
{
    rs485Header header;
    extModuleDesc moduleDesc;
    rs485Footer footer;
};


struct aswModuleState
{
    rs485Header header;
    extModuleState moduleState;
    rs485Footer footer;
};


#define EVENT_CLASS_I_AM_ALIVE              0
#define EVENT_INPUT_STATE_CHANGED           1
#define EVENT_OUTPUT_STATE_CHANGED          2
#define EVENT_TAMPER_OK                     3
#define EVENT_TAMPER_ALARM                  4
#define EVENT_AC_POWER_OK                   5
#define EVENT_AC_POWER_FAILURE              6
#define EVENT_ACCUMULATOR_OK                7
#define EVENT_ACCUMULATOR_FAILURE           8
#define EVENT_TEXT_DESC                     9
#define EVENT_MODULE_ARMED                  10
#define EVENT_MODULE_DISARMED               11


struct myDateTime
{
    uint32_t year:6; // 0-63 (2017-2080)
    uint32_t month:4; // 1-12
    uint32_t day:5; // 1-31
    uint32_t hour:5; // 0-23
    uint32_t minute:6; // 0-59
    uint32_t second:6; // 0-59
};

struct moduleEventDesc
{
    //
    uint8_t eventClass;
    uint8_t eventSubject;
    union
    {
        uint32_t eventData32[2];
        uint16_t eventData16[4];
        uint8_t  eventData8 [8];
    } eventData;
    myDateTime timeStamp;
};


struct aswModuleEvent
{
    rs485Header header;
    moduleEventDesc event;
    rs485Footer footer;
};

#pragma pack(pop)


#define SIMPLE_CMD_NONE                 0
#define SIMPLE_CMD_OUTPUT_ON            1
#define SIMPLE_CMD_OUTPUT_OFF           2
#define SIMPLE_CMD_MODULE_ARM           3
#define SIMPLE_CMD_MODULE_DISARM        4


struct SModuleSimpleCmd
{
    int m_cmd;
    int m_addr;
    int m_param;
    //
    SModuleSimpleCmd()
        : m_cmd(SIMPLE_CMD_NONE)
    {}
};


#endif
