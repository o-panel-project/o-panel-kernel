
#include <linux/delay.h>
#include <linux/platform_device.h>


#include "dsihw.h"
#include "dsi.h"

#define LONG_CMD_MODE 0x39
#define SHORT_CMD_MODE 0x05

#define POWER_MODE  0x01

void send_long_cmd(struct platform_device *pdev, char *pcmd, int cnt)
{
	dsihw_send_long_packet(pdev, LONG_CMD_MODE,cnt,(unsigned int *)pcmd,POWER_MODE);	
}

void send_short_cmd(struct platform_device *pdev, int cmd)
{
	dsihw_send_short_packet(pdev, SHORT_CMD_MODE,cmd,POWER_MODE);  
}

void send_short_cmd_1(struct platform_device *pdev, int cmd, int singnal)
{
	dsihw_send_short_packet(pdev, singnal,cmd,POWER_MODE);  
}


/*
1. 如需发长包命令
	首先调用start_long_cmd
	在发完最后一个长命令后调用end_long_cmd

2. 长包格式
	pakg[0] cmd
	pakg[1] data1
	pakg[2] data2
	.       .
	.       .
	send_long_cmd(pakg, 4); 第一个参数是指针，第二个参数是传输个数
	
3. 短包格式
	send_short_cmd(cmd);
	exp： send_short_cmd(0x11);
	
4. 需要用到毫秒延时
	mdelay(100); //100毫秒的延时
*/
void send_cmd(struct platform_device *pdev)
{
/*
	char pakg[100];
	
	mdelay(100);
	//start_long_cmd();
	pakg[0] = 0xF0; 
	pakg[1] = 0x55; 
	pakg[2] = 0xAA; 
	pakg[3] = 0x52; 
	pakg[4] = 0x08; 
	pakg[5] = 0x00; 
	send_long_cmd(pakg, 6);
	pakg[0] = 0xB2; 
	pakg[1] = 0x25; 
	send_long_cmd(pakg, 2);	
	pakg[0] = 0xBB; 
	pakg[1] = 0x63; 
	pakg[2] = 0x63; 
	send_long_cmd(pakg, 3);	
	pakg[0] = 0xBC; 
	pakg[1] = 0x0F; 
	pakg[2] = 0x00; 
	send_long_cmd(pakg, 3);	
	pakg[0] = 0xBD; 
	pakg[1] = 0x01; 
	pakg[2] = 0x40; 
	pakg[3] = 0x0D 
	pakg[4] = 0x06; 
	send_long_cmd(pakg, 5);	
	pakg[0] = 0xC8; 
	pakg[1] = 0x83; 
	send_long_cmd(pakg, 2);	
	pakg[0] = 0xF0; 
	pakg[1] = 0x55; 
	pakg[2] = 0xAA; 
	pakg[3] = 0x52; 
	pakg[4] = 0x08; 
	pakg[5] = 0x01; 
	send_long_cmd(pakg, 6);	
	pakg[0] = 0xB3; 
	pakg[1] = 0x28; 
	send_long_cmd(pakg, 2);	
	pakg[0] = 0xB4; 
	pakg[1] = 0x14; 
	send_long_cmd(pakg, 2);	
	pakg[0] = 0xB9; 
	pakg[1] = 0x44; 
	send_long_cmd(pakg, 2);	
	pakg[0] = 0xBC; 
	pakg[1] = 0x78; 
	pakg[2] = 0x00; 
	send_long_cmd(pakg, 3);	
	pakg[0] = 0xBD; 
	pakg[1] = 0x78; 
	pakg[2] = 0x00; 
	send_long_cmd(pakg, 3);	
	pakg[0] = 0xC7; 
	pakg[1] = 0x00; 
	pakg[2] = 0x80; 
	pakg[3] = 0x00; 
	send_long_cmd(pakg, 4);	
	pakg[0] = 0xCA; 
	pakg[1] = 0x00; 
	send_long_cmd(pakg, 2);	
	pakg[0] = 0xCE; 
	pakg[1] = 0x04; 
	send_long_cmd(pakg, 2);	
	pakg[0] = 0xF0; 
	pakg[1] = 0x55; 
	pakg[2] = 0xAA; 
	pakg[3] = 0x52; 
	pakg[4] = 0x08; 
	pakg[5] = 0x02; 
	send_long_cmd(pakg, 6);	
	pakg[0] = 0xB0; 
	pakg[1] = 0x42; 
	send_long_cmd(pakg, 2);	
	pakg[0] = 0xD1; 
	pakg[1] = 0x00; 
	pakg[2] = 0x00; 
	pakg[3] = 0x00; 
	pakg[4] = 0x18; 
	pakg[5] = 0x00; 
	pakg[6] = 0x3E; 
	pakg[7] = 0x00; 
	pakg[8] = 0x73; 
	pakg[9] = 0x00; 
	pakg[10] = 0x9A; 
	pakg[11] = 0x00; 
	pakg[12] = 0xB9; 
	pakg[13] = 0x00; 
	pakg[14] = 0xEB; 
	send_long_cmd(pakg, 15);	
	pakg[0] = 0xD2; 
	pakg[1] = 0x01; 
	pakg[2] = 0x13; 
	pakg[3] = 0x01; 
	pakg[4] = 0x53; 
	pakg[5] = 0x01; 
	pakg[6] = 0x85; 
	pakg[7] = 0x01; 
	pakg[8] = 0xCF;
	pakg[9] = 0x02; 
	pakg[10] = 0x0B; 
	pakg[11] = 0x02; 
	pakg[12] = 0x0C; 
	pakg[13] = 0x02; 
	pakg[14] = 0x43; 
	send_long_cmd(pakg, 15);	
	//end_long_cmd();
	
	send_short_cmd(pdev, 0x11);
	mdelay(200);
	send_short_cmd(pdev, 0x29);
	mdelay(200);
	send_short_cmd_1(pdev, 0, 0x32);
	mdelay(200);



    char a[] = {0xF0,0x55,0xAA,0x52,0x08,0x00 };
	char b[] = {0xB2,0x25};
	char c[] = {0xBB,0x63,0x63};
		
	char d[] = {0xBC,0x0F,0x00};
	char e[] = {0xBD,0x01,0x40, 0x0D,0x06};
	char f[] = {0xC8,0x83};
	char g[] = {0xF0,0x55,0xAA,0x52,0x08,0x01};
	char h[] = {0xB3,0x28};
	char i[] = {0xB4,0x14};
	char j[] = {0xB9,0x44};
	char k[] = {0xBC,0x78,0x00};
	char l[] = {0xBD,0x78,0x00};
	char n3[] = {0xC7,0x00,0x80,0x00};
	char m[] = {0xCA,0x00};
	char o[] = {0xCE,0x04};
	char p[] = {0xF0,0x55,0xAA,0x52,0x08,0x02};
	char q[] = {0xB0,0x42};
	char r[] = {0xD1,0x00,0x00,0x00,0x18,0x00,0x3E,0x00,0x5A,0x00,0x73,0x00,0x9A,0x00,0xB9,0x00,0xEB};
	char s[] = {0xD2,0x01,0x13,0x01,0x53,0x01,0x85,0x01,0xCF,0x02,0x0B,0x02,0x0C,0x02,0x43,0x02,0x7D};
	char t[] = {0xD3,0x02,0xA1,0x02,0xD4,0x02,0xF6,0x03,0x26,0x03,0x43,0x03,0x6C,0x03,0x86,0x03,0xA1};
	char u[] = {0xD4,0x03,0xBA,0x03,0xFF};
	char v[] = {0xF0,0x55,0xAA,0x52,0x08,0x03};
	char w[] = {0xB0,0x00,0x00,0x00,0x00};
	char x[] = {0xB1,0x00,0x00,0x00,0x00};
	char y[] = {0xB2,0x01,0x00,0x09,0x03,0x00,0x34,0x48};
	char z[] = {0xB3,0x01,0x00,0x08,0x03,0x00,0x34,0x48};
	char a1[] = {0xB6,0xF0,0x05,0x04,0x05,0x00,0x00,0x00,0x00,0x34,0x48};
	char b1[] = {0xB7,0xF0,0x05,0x04,0x05,0x00,0x00,0x00,0x00,0x34,0x48};
	char c1[] = {0xBA,0x84,0x04,0x00,0x07,0x01,0x34,0x48};
	char d1[] = {0xBB,0x84,0x04,0x00,0x06,0x01,0x34,0x48};
	char e1[] = {0xC4,0x10,0x00};
	char f1[] = {0xC5,0x00,0x00};
	char g1[] = {0xF0,0x55,0xAA,0x52,0x08,0x05};
	char h1[] = {0xB0,0x33,0x03,0x00,0x03};
	char i1[] = {0xB1,0x30,0x00};
	char j1[] = {0xB2,0x03,0x01,0x00};
	char k1[] = {0xB3,0x82,0x00,0x81,0x38};
	char l1[] = {0xB4,0xD5,0x75,0x07,0x57};
	char n1[] = {0xB6,0x01,0x00,0xD5,0x71,0x07,0x57};
	char m1[] = {0xB9,0x09,0x00,0x00,0x05,0x00};
	char o1[] = {0xC0,0x75,0x07,0x00,0x57,0x05};
	char p1[] = {0xC6,0x20};
	char r1[] = {0xD0,0x00,0x25,0x01,0x00,0x00};
	char s1[] = {0xD1,0x00,0x25,0x02,0x00,0x00};
	char t1[] = {0xF0,0x55,0xAA,0x52,0x08,0x06};
	char u1[] = {0xB0,0x32,0x32,0x02,0x02,0x03};
	char v1[] = {0xB1,0x03,0x33,0x33,0x08,0x3D};
	char w1[] = {0xB2,0x0A,0x0A,0x3D,0x17,0x17};
	char x1[] = {0xB3,0x16,0x16,0x19,0x19,0x18};
	char y1[] = {0xB4,0x18,0x33};
	char z1[] = {0xB5,0x32,0x32,0x00,0x00,0x01};
	char a2[] = {0xB6,0x01,0x33,0x33,0x08,0x3D};
	char b2[] = {0xB7,0x0A,0x0A,0x3D,0x11,0x11};
	char c2[] = {0xB8,0x10,0x10,0x13,0x13,0x12};
	char d2[] = {0xB9,0x12,0x33};
	char e2[] = {0xFF,0xAA,0x55,0xA5,0x80};
	char f2[] = {0x6F,0x08};
	char g2[] = {0xFC,0x00};
	char h2[] = {0x6F,0x09};
	char i2[] = {0xF7,0x82};
	char j2[] = {0x6F,0x0B};
	char k2[] = {0xF7,0xE0};
	mdelay(100);
	send_long_cmd(pdev,a, 6);	
	send_long_cmd(pdev,b, 2);	
	send_long_cmd(pdev,c, 3);	
	send_long_cmd(pdev,d, 3);	
	send_long_cmd(pdev,e, 5);	
	send_long_cmd(pdev,f, 2);	
	send_long_cmd(pdev,g, 6);	
	send_long_cmd(pdev,h, 2);	
	send_long_cmd(pdev,i, 2);	
	send_long_cmd(pdev,j, 2);	
	send_long_cmd(pdev,k, 3);	
	send_long_cmd(pdev,l, 3);	
	send_long_cmd(pdev,n3, 4);	
	send_long_cmd(pdev,m, 2);	
	send_long_cmd(pdev,o, 2);	
	send_long_cmd(pdev,p, 6);	
	send_long_cmd(pdev,q, 2);	
	send_long_cmd(pdev,r, 17);	
	send_long_cmd(pdev,s, 17);	
	send_long_cmd(pdev,t, 17);	
	send_long_cmd(pdev,u, 5);	
	send_long_cmd(pdev,v, 6);	
	send_long_cmd(pdev,w, 5);	
	send_long_cmd(pdev,x, 5);	
	send_long_cmd(pdev,y, 8);	
	send_long_cmd(pdev,z, 8);	
	send_long_cmd(pdev,a1, 11);	
	send_long_cmd(pdev,b1, 11);	
	send_long_cmd(pdev,c1, 8);	
	send_long_cmd(pdev,d1, 8);	
	send_long_cmd(pdev,e1, 3);	
	send_long_cmd(pdev,f1, 3);	
	send_long_cmd(pdev,g1, 6);	
	send_long_cmd(pdev,h1, 5);	
	send_long_cmd(pdev,i1, 3);	
	send_long_cmd(pdev,j1, 4);	
	send_long_cmd(pdev,k1, 5);	
	send_long_cmd(pdev,l1, 5);	
	send_long_cmd(pdev,n1, 7);	
	send_long_cmd(pdev,m1, 6);	
	send_long_cmd(pdev,o1, 6);	
	send_long_cmd(pdev,p1, 2);	
	send_long_cmd(pdev,r1, 6);	
	send_long_cmd(pdev,s1, 6);	
	send_long_cmd(pdev,t1, 6);	
	send_long_cmd(pdev,u1, 6);	
	send_long_cmd(pdev,v1, 6);	
	send_long_cmd(pdev,w1, 6);	
	send_long_cmd(pdev,x1, 6);	
	send_long_cmd(pdev,y1, 3);	
	send_long_cmd(pdev,z1, 6);	
	send_long_cmd(pdev,a2, 6);	
	send_long_cmd(pdev,b2, 6);	
	send_long_cmd(pdev,c2, 6);	
	send_long_cmd(pdev,d2, 3);	
	send_long_cmd(pdev,e2, 5);	
	send_long_cmd(pdev,f2, 2);	
	send_long_cmd(pdev,g2, 2);	
	send_long_cmd(pdev,h2, 2);	
	send_long_cmd(pdev,i2, 2);	
	send_long_cmd(pdev,j2, 2);	
	send_long_cmd(pdev,k2, 2);	
	send_short_cmd(pdev,0x11);
	mdelay(120);
	send_short_cmd(pdev,0x29);
	//send_short_cmd(0x29);



*/

}


void send_cmd_test(struct platform_device *pdev)
{
	char pakg[100];
	
	pakg[0] = 0xbf; 
	pakg[1] = 0x93; 
	pakg[2] = 0x61; 
	pakg[3] = 0xf4; 
	send_long_cmd(pdev, pakg, 4);
	pakg[0] = 0xbf; 
	pakg[1] = 0x93; 
	send_long_cmd(pdev, pakg, 2);	
}
