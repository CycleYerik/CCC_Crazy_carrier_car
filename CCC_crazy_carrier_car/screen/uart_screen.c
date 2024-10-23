#include "uart_screen.h"




void itoa(int num,char str[] )
{
	int sign = num,i = 0,j = 0;
	char temp[11];
	if(sign<0)
	{
		num = -num;
	}
	do
	{
		temp[i] = num%10+'0';       
		num/=10;
		i++;
	}while(num>0);
	if(sign<0)
	{
		temp[i++] = '-';
	}
	temp[i] = '\0';
	i--;
	while(i>=0)
	{
	str[j] = temp[i];
	j++;
	i--;
	} 
	str[j] = '\0';
}

void send_data_origin(int a,int ch)
{
	char str[10];           //定义一个存放字符串的数组
	itoa(a,str);            //内嵌一个整形转字符的函数
	printf("add 1,%d,",ch);  
	printf("%s",str);        //字符串形势传输指令
	printf("\xff\xff\xff");  //结束符，告诉串口屏一次指令传输完毕
	HAL_Delay(10);           //必要的延时函数
}

void send_screen_data(char *data)
{
    printf("%s",data);
    printf("\xff\xff\xff");
    HAL_Delay(10);
}



 
