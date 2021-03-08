#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "gy_common.h"

#include <stdlib.h>
#include <string.h>
#define B_ADDR_LEN    6

/*********************************************************************
 * @fn      Util_convertBdAddr2Str
 *
 * @brief   Convert Bluetooth address to string. Only needed when
 *          LCD display is used.
 *
 * @param   pAddr - BD address
 *
 * @return  BD address as a string
 */
char *Util_convertBdAddr2Str(uint8_t *pAddr)
{
  uint8_t     charCnt;
  char        hex[] = "0123456789ABCDEF";
  static char str[(2*B_ADDR_LEN)+3];
  char        *pStr = str;

  *pStr++ = '0';
  *pStr++ = 'x';

  // Start from end of addr
  pAddr += B_ADDR_LEN;

  for (charCnt = B_ADDR_LEN; charCnt > 0; charCnt--)
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }
  pStr = NULL;

  return str;
}

/*********************************************************************
 * @fn      Util_convertHex2Str
 *
 * @brief   将HEX转成String，用于转化广播数据和扫描回调数据
 *
 * @param   Hex - Hex
 *          len - Hex len
 *
 * @return  Hex as a string
 */
char *Util_convertHex2Str(uint8_t *Hex, uint16_t Len)
{
  uint8_t     charCnt;
  char        hex[] = "0123456789ABCDEF";
  static char str[(2*31)+3];    // 广播数据最大31字节
  char        *pStr = str;

  *pStr++ = '0';
  *pStr++ = 'x';

  for (charCnt = Len; charCnt > 0; charCnt--)
  {
    *pStr++ = hex[*Hex >> 4];
    *pStr++ = hex[*Hex++ & 0x0F];
  }
  pStr = NULL;

  str[2*Len+2] = '\0';
  return str;
}




 int convert_string2hex(unsigned char *in_data, int in_data_len, unsigned char *out_data, int *out_data_len)
{
    int i;
    int loop_count;
    int convert_point = 0;
    int mem_point = 0;
    unsigned char convert_result;
    unsigned char temp[3] = {0}; /* 为啥是3呢？ */
	
	/* 检查参数有效性 */
    if (in_data == NULL || in_data_len <= 0 || out_data == NULL || out_data_len == NULL || (in_data_len % 2) != 0) {
        printf("invalid parameters\n");
        return -1;
    }

	/* 判断是否超过16进制范围 0 ~ F */
    for (i = 0; i < in_data_len; i++) {
        if ((in_data[i] < '0') || (in_data[i] > 'f') || ((in_data[i] > '9') &&(in_data[i] < 'A'))) {
            printf("out of range\n");
            return -1;
        }
    }

    loop_count = in_data_len / 2;
    memset(out_data, 0x00, *out_data_len);
    *out_data_len = 0;

    for (i = 0; i < loop_count; i++) {
        memset(temp, 0x00, sizeof(temp));
        memcpy(temp, in_data + convert_point, 2);
        convert_point += 2;

        convert_result = strtoul(temp, NULL, 16);

        memcpy(out_data + mem_point, &convert_result, sizeof(unsigned char));
        mem_point += sizeof(unsigned char);
        *out_data_len += sizeof(unsigned char);
    }

    return 0;
}

