#ifndef _LCD_H
#define _LCD_H

void LCD_Init(void);
void LCD_WriteString(char * str,unsigned char line,unsigned char column);
void LCD_WriteInt(int32_t sendValue,unsigned char rate,unsigned char line,unsigned char column);

#endif
