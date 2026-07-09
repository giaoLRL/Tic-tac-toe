#include "oneline.h"
#include "usart.h"
#include <string.h>
#include "flash.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "common.h"
extern char All_down_num;
extern uint16 UartRec[MOTOR_NUM]; 			  //��λ���ַ����������������������
extern uint16 pos[7][MOTOR_NUM];
extern uint16 CPWM[MOTOR_NUM];
extern char redata[257];
extern uint8 ps2_buf[120];
	uint8 flag_mp3=0;
extern char point_in;
extern uint8 flag_stop;
extern uint8 line;
extern uint8 flag_in;
extern uint8 flag_out;
extern uint8 flag_download;
extern uint8 flag_All_download;
extern uint8 flag_All_Stop_Down;//��־λ1
extern uint8 flag_read;
extern uint8 flag_RecFul;
extern uint8 flag_stop_download;
extern uint8 flag_online_run;
uint8 pwm_num;	
extern uint16 tuoji_count;
extern unsigned long send_mode;
extern MotorData motor_data;
extern MotorOneCmd motor_one_cmd;
extern CurrentItem cur_item;
extern uint8 error;
/***************************************************************************************************************
�� �� ����RecStr_to_pos()  
�����������ѽ���֮�����λ���������� ���뻺�� 
����������ޣ���ȫ��������� UartRec[33]���� 
��    ע�����������º������÷���������ѭ������ж�ֱ�ӵ���; 
****************************************************************************************************************/	
void RecStr_to_pos(void)	 
{
	uint8 i = 0;
	if (!flag_download)
	{
		if(line<7) 					//���滹�п��У�û����
		{   
			line++;					//��������������һ������
			point_in++;
			if(point_in==7)
				point_in=0;
			for(i=0;i<MOTOR_NUM;i++)
			{
				pos[point_in][i]= UartRec[i];
				
			}
		}
		else
		{
			flag_in=0;				//��ʾû�пռ���
		}

		if(line>0)
		{
		   flag_out=1;					//��ʾ������������
		   //flag_stop = 1;
		}
	}
	else 
	{
		if (motor_one_cmd.end < CMD_FLASH_ADDR)
		{
			if (((motor_one_cmd.end)% 16) == 0)
			{
				SpiFlashEraseSector((motor_one_cmd.end) >> 4);  //ҳ������16�õ�������ַ  ÿ������ 16ҳ 4K
			}
			SpiFlashWrite((unsigned char*)redata,(((unsigned long int)motor_one_cmd.end)<<WRITE_BIT_DEPTH),WRITE_SIZE); //ÿ��д��256���ֽ�
			motor_one_cmd.end += 1;		
			send_mode |= SEND_A;
			error &= ~ERROR_FLASH_FULL;
		}
		else
		{
			error |= ERROR_FLASH_FULL;
		}
	}
}

/***************************************************************************************************************
�� �� ����RecStr_to_pwm()  
�����������ѽ���֮�����λ��ָ��-������ʵʱ�Ƕȱ仯ֵ ���뻺�� 
����������ޣ���ȫ��������� UartRec[17]���� 
��    ע����ΪҪ��ʵʱ�Ժã���Ҫ��岹��ֱ�ӷ���pwm[] 
****************************************************************************************************************/	
void RrcStr_to_pwm(void)
{
	CPWM[pwm_num]= UartRec[pwm_num];
}
/***************************************************************************************************************
�� �� ����Parse_String_Cmd()  
�����������������ڽ��յ��������ַ���
������������ݽ��ܵ��������� 
��    ע��
****************************************************************************************************************/

int Parse_String_Cmd( char *str)
{
	char *p = NULL;

	p = strchr(str,'#');

	
	if (p == NULL )
		return 1;
	p++;
	if ((*(p) >= '0' && *(p) <= '9' && *(p+1) == 'P')|| ((*(p)>='0' && *(p) <= '9')&& (*(p+1) == '0'&&*(p+1) <= '9') &&*(p+2) == 'P'))//�����ǰ�ַ�����#����P��ʼ�ģ�˵���Ƕ��ת������ֱ�ӷ���
		return 0;
	if(*p=='M')
	{
		//UART_PutStr(USART3,str);// ����2���ͳ�ȥ�������
		return 0;
	}
	if (strstr(str,"#255PRAD"))
	{
		send_mode |= SEND_SET_READ_UART_MOTOR_ANGLE;
		memset(ps2_buf,0,sizeof(ps2_buf));
		cur_item.read_num = 0;
		UART_PutStr(USART2,"#PWMR");
		return 1;
	}
	if (strstr(str,"#255PULK"))
	{
		//UART_PutStr(USART2,"#255PULK\r\n");
		send_mode |= SEND_SET_SET_UART_MOTOR_PULK;
		cur_item.pulk_num = 0;
		return 1;
	}
	if ((p = strstr(str,"#255PMOD"))!=NULL)
	{
		//UART_PutStr(USART2,"#255PULK\r\n");
		send_mode |= SEND_SET_SET_UART_MOTOR_ANGLE;
		cur_item.pulk_num = 0;
		cur_item.angle_mode = atoi(p + strlen("#255PMOD"));
		return 1;
	}
	if (strstr(str,"#Stop"))//�յ�����ʱ����λ������Stop˵�����سɹ���
	{
		if (flag_download)
		{
			flag_stop_download = 1;
		}
		else
		{
			flag_stop = 1;
			flag_out = 0;
		}
		return 1;
	}

	if (strstr(str,"#Down"))//��������
	{
		ReadMoterInfor();
		memset(&motor_one_cmd,0,sizeof(motor_one_cmd));
		motor_one_cmd.start = motor_data.sum ;
		motor_one_cmd.end = motor_data.sum ;
		flag_download = 1;//��־λ1
		send_mode |= SEND_A;
		return 1;
	}

	
	if (strstr(str,"#STOP"))//���ܵ�ֹͣ����
	{
		tuoji_count = 0;
		flag_download = 0;
		flag_stop = 1;
		flag_out = 0;
		return 1;
	}
	if (strstr(str,"#Veri+"))//���յ���������
	{
		flag_stop_download = 0;
		flag_download = 0;
		flag_read = 0;
		send_mode |= SEND_START_OK;
		return 1;
	}

	if (strstr(str,"#Flist"))//���յ�ˢ������
	{
		flag_download = 0;
		flag_All_download=0;
		flag_stop = 1;
		flag_out = 0;
		ReadMoterInfor();
		send_mode |= SEND_READ_FILE;
		return 1;
	}
	if ((p = strstr(str,"#FRead-"))!=NULL)//���յ���ȡ����
	{
		flag_download = 0;
		flag_read = 1;
		flag_stop = 1;
		flag_out = 0;
		ReadMoterInfor();
		cur_item.read_num = atoi(p+strlen("#FRead-"));
		ReadOneCmdInfor(cur_item.read_num);
		send_mode |= SEND_SEND_FILE;
		return 1;
	}
	if ( strstr(str,"#PS2"))//���յ���PS2�ֱ�������Ϣ
	{
		SpiFlashEraseSector((PS2_FLASH_ADDR) >> 4);//������ǰ�洢����Ϣ
		SpiFlashWrite((unsigned char*)redata,(PS2_FLASH_ADDR)<<WRITE_BIT_DEPTH,WRITE_SIZE); 
		send_mode |= SEND_SET_PS2_OK;
		return 1;
	}	
	if (((p = strstr(str,"#Enable#")))!=NULL)//���յ��ѻ����м�������
	{
		flag_download = 0;
		cur_item.file_num = atoi(p+strlen("#Enable#"));//�����ǰ�ļ�
		ReadOneCmdInfor(cur_item.file_num);
		p = strstr(str,"GC");
		motor_one_cmd.tuoji_count = atoi(p+strlen("GC"));//����ѻ�ִ�д���
		send_mode |= SEND_SET_OFFLINE_OK;//
		flag_stop = 1;
		flag_out = 0;
		return 1;
	}
	if (strstr(str,"#Disable"))//���յ���ֹ�ѻ����е�����
	{
		flag_download = 0;
		ReadMoterInfor();
		motor_data.stop_tuoji_flag = 1;
		send_mode |= SEND_SET_DISABLEOFFLINE_OK;
		flag_stop = 1;
		flag_out = 0;
		return 1;
	}
	if ((p = strstr(str,"GC"))!=NULL)//���յ����������м��ε�����
	{
		flag_download = 0;
		ReadMoterInfor();
		p = strstr(str,"#");
		cur_item.file_num = atoi(p+strlen("#"));
		ReadOneCmdInfor(cur_item.file_num);
		p = strstr(str,"GC");
		cur_item.tuoji_count = atoi(p+strlen("GC"));	
		send_mode |= SEND_SET_ONLINE_OK;
		flag_online_run = 1;
		flag_stop = 1;
		flag_out = 0;
		return 1;
	}
	
	if ((p = strstr(str,"#FDel-"))!=NULL)//���յ�ɾ������
	{
		flag_download = 0;
		ReadMoterInfor();
		cur_item.delete_num = atoi(p+strlen("#FDel-"));
		send_mode |= SEND_SET_DELETE_ONE_FILE_OK;
		flag_stop = 1;
		flag_out = 0;
		return 1;
	}
	if ( strstr(str,"#Format"))//���յ��˸�ʽ��������
	{
		send_mode |= SEND_SET_DELETE_ALL_FILE_OK;
		flag_stop = 1;
		flag_out = 0;
		return 1;
	}
	if ( strstr(str,"#FMQENABLE"))//���յ��˸�ʽ��������
	{
		send_mode |= SEND_SET_BEEP_ON;
		return 1;
	}
	if ( strstr(str,"#FMQDISABLE"))//���յ��˸�ʽ��������
	{
		send_mode |= SEND_SET_BEEP_OFF;
		return 1;
	}
	return 0;
}
/***************************************************************************************************************
�� �� ����OneLine()  
�����������Ѵ��ڽ��յ�����λ���༭���������ַ��������������ŵ�UartRec[33] 
���������*str.���ڽ����ַ��Ĵ�ŵ�ַ 
�� �� ֵ����  
��    ע��ԭ�������������OneMotor()�����ġ����ںϲ�Ϊһ��������������T<100ʱֱ�ӷ���pwm[],�����в岹
****************************************************************************************************************/	
void OneLine(char *str)
{
	uint8 motor_num=0;		   //�����
	uint16 motor_jidu=0;	   //�������ֵ
	uint16 motor_time=0;	   //ִ��ʱ��
	uint8 num_now=0;		   //��Ž����м����
	uint8 jidu_now=0;		   //���������м����
	uint8 time_now=0;		   //ִ��ʱ������м����
	uint8 flag_num=0;		   //��ǳ��ֹ�#
	uint8 flag_jidu=0;		   //��ǳ��ֹ�P
	uint8 flag_time=0;		   //��ǳ��ֹ�T

	uint16 i=0;
//	uint8 model_flag = 0;				   //�����ƶ��ַ���
	//uint8 buf[257] = {0};
  if (Parse_String_Cmd(str))
		return ;
	UART_PutStr(USART3,str);// ����2���ͳ�ȥ�������
	if(strncmp(str,"#MP3",4)==0)
	{
	
		//UART_PutStr(USART2,"test\r\n");
		if(flag_download)
			{
					RecStr_to_pos();
			}
		else
		{
				send_mode |= SEND_CC;
		}
			
  }
	else 
	{
		while( str[i]!='\n'  && i < 255)
		{
			if(flag_num==1)	 				//���ֹ�#
			{
				if(str[i]!='P' && str[i]!='M')				//�����ǰ�ַ�����P
				{
					num_now=ASC_To_Valu(str[i]);//�ѵ�ǰ�����ַ�ת�������ֵ�ֵ
					motor_num=motor_num*10+num_now;
				}
				else  						//��ǰ�ַ���P
					flag_num=0;
			}
			

			if(flag_jidu==1)				//���ֹ�P
			{
				if((str[i]!='T')&(str[i]!='#')& (str[i-1]!='M'))	
				{							//��ǰ�ַ��ǳ���p֮��ķ�#��T���ַ�
					jidu_now=ASC_To_Valu(str[i]);//�ѵ�ǰ�����ַ�ת�������ֵ�ֵ
					motor_jidu=motor_jidu*10+jidu_now;
				}
				else  						//��ǰ�ַ���#����T���Ƕ����ݽ���
				{
					flag_jidu=0;
					if(motor_jidu>2500)
						motor_jidu=2500;
					UartRec[motor_num]=motor_jidu;
					pwm_num=motor_num;
					motor_jidu=0;
					motor_num=0;
				}
			}

			if(flag_time==1)				//������T
			{
				time_now=ASC_To_Valu(str[i]);//�ѵ�ǰ�����ַ�ת�������ֵ�ֵ
				motor_time=motor_time*10+time_now;
				UartRec[0]=motor_time;	   	//ִ��ʱ����ڡ�0��λ��
				if(str[i+1]=='\r')
				{	
					if(motor_time<=100)		 //���������ʱ��С��100us�������������ٶȿ��ƣ�ʵʱ�ĸı����Ƕ�
						RrcStr_to_pwm();
					else					 //���ʱ�����100us�����ٶȿ���
					{
						RecStr_to_pos(); 	//��һ���ַ��ǣ�����ʾ����ָ����� �����ٶȿ��Ƶ�ʱ��ֵ����������0λ
					}
				}
			}
		
			if(str[i]=='#')
				flag_num=1;
			if(str[i]=='P')
				flag_jidu=1;
			if(str[i]=='T')
				flag_time=1;
			//if(str[i]=='M')
			//	flag_mp3=1;
			i++;
		}	 
 }
}
