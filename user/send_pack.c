#include "flower-bwr.png.h"
#include "png_75.h"
#include "sen_pack.h"
/*
0               3               5               7         8              x        x+1     x+1     x+1
+---------------+---------------+---------------+---------+-------------+---------+--------+--------+
|   bytes(3)    |  uShort16BE   |  uShort16BE   | byte    |  Length-1   |  byte   |  byte  |  byte  |
+---------------+---------------+---------------+---------+-------------+---------+--------+--------+
|  0xAA0x550x7E |    SN         |  Length       | Command |  Data       |  SUM    |  XOR   |  0x0D  |
+---------------+---------------+---------------+---------+-------------+---------+--------+--------+
Length === 1 + Data.Length
----------------------------------------
SUM and XOR Specification:
sum value = SUM({Length, SN, Command, Data})
xor value = XOR({Length, SN, Command, Data})
*/
#define BUFFSIZE 1024
#define PACKETSIZE 2048
#define FILE_NAME_MAX_SIZE 512
#define block 100
#define size 32
#define PACKET_HEAD          "\x57\x41\x56"
#define PACKET_NUM              "1024"
#define PACKET_TAIL         "\x56\0x41\0x57"
#define msg_t      0x69  // 'i'
#define line       "\x0D"

int a=0;
int count=0;
uint16_t leng=0;

uint8_t buff_x[size];
typedef struct tcp_TEST {
  uint16_t sn;
  uint16_t length;
  uint8_t msg_type;
  char* message;
//	uint8_t msg_sum;
//	uint8_t msg_xor;
//	uint8_t msg_line;
} tcp_TEST;
static tcp_TEST tcp_message;
tcp_TEST *replyMessage = &tcp_message;
uint8_t msg_type;
uint8_t buff_s[block];
uint8_t* message=buff_s;
const unsigned char *image=bw_image;



uint32_t ret;

uint32_t test(void)
{
//		while(1)
//	{
	 memcpy(message, image, block);
		if(count<=388)
		{
	 image += block;
		leng=100;	
		}
		else{
		image += 80;
		leng=80;
		}
		count++;
	
 ++replyMessage->sn;
  replyMessage->msg_type=msg_t;
  replyMessage->length = leng+1;
  replyMessage->message = (char *)message;
	 
static uint8_t buff_1[111];
 uint8_t *p = buff_1;
  strcpy((char *)p, PACKET_HEAD);
  p += strlen(PACKET_HEAD);  
#if defined(ITP_USE_LITTE_ENDIAN)  
  *(p++) = (replyMessage->sn & 0xff);
  *(p++) = (replyMessage->sn >> 8);    
  *(p++) = (replyMessage->length & 0xff);
  *(p++) = (replyMessage->length >> 8);
  *(p++) = (uint8_t)'R';      
 
#else
  *(p++) = (replyMessage->sn >> 8);
  *(p++) = (replyMessage->sn & 0xff);
  *(p++) = (replyMessage->length >> 8);
  *(p++) = (replyMessage->length & 0xff);
 //*(p++) = (uint8_t)'i';  

#endif 
  *(p++) = (uint8_t)msg_t;
  memcpy(p, message, leng);
  p += leng;

  uint8_t sum = 0, v_xor = 0;
  uint8_t* i = buff_1; 
  i += 7;
  while(i<p) {
    sum += *i;
    v_xor ^= *i;
    i++;
  }
  *(p++) = sum;
  *(p++) = v_xor;
  strcpy((char *)p, line);
  p += strlen(line); 
	
 //nrf_delay_ms(500);
	ret= ble_nus_c_string_send(&m_ble_nus_c, buff_1, leng+11);
		for(a=0;a<leng+11;a++)
		{
			printf("%02x",buff_1[a]);
		}
			printf("\n++++++++++++++++++\r\n");

		memset(buff_1,0,111);
		if(leng==80)
		{
//		break;
		}
//		}
	return ret;
}

const unsigned char *image1=gImage_7in5_V2_b;
uint32_t test1(void)
{
	
//		while(1)
//	{
	 memcpy(message, image1, block);
		if(count<=480)
		{
	 image1 += block;
		leng=100;	
		}

		count++;
	ret= ble_nus_c_string_send(&m_ble_nus_c, message, leng);
		
		for(a=0;a<leng;a++)
		{
			printf("%02x",message[a]);
		}
		
		memset(message,0,100);
		if(count>480)
		{
//		break;
			return 0;
		}
//	}
	return ret;
}




