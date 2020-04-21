 
#include <hidef.h>      /* common defines and macros */
#include <MC9S12XS128.h>     /* derivative information */
#include <math.h>
#include "derivative.h"
#include "LCD.h"
#define uchar unsigned char
#define uint unsigned int

#define  servo_max        3459+450//460        // 200           4460+875
#define  servo_mid         3459
           
#define  servo_min         3459-450//480        // 200           4460-875
#define d_len        20//60 //20  //60   // 70  // 当前位置偏差与第d_len次偏差平均作运算
#define d_amp       0.1//0.1//0.1 //4     //  5/2 03D item coef
#define SPEED_DANG_NUM 100


#define VV_MAX             250 	//1000 	速度PID，调节最大值
#define VV_MIN 	         -250     //-1000 	速度PID，调节最小值
#define VV_DEADLINE  1	  //速度PID，设置死区范围

#define mkp   8.5//8.5// 6.46   //p项临界值，小于等于9.23   60%~70%          
#define mki   0.183//0.183 //0.168//0.45//i项值  小于等于0.33   150%~180%	          
#define mkd   0.002//0.002//0.003/ 0.024  //d项值    小于0.25            30%

#define mkp0   8.5//8.5// 6.46   //p项临界值，小于等于9.23   60%~70%          
#define mki0   0.183//0.183//0.168//0.45//i项值  小于等于0.33   150%~180%	          
#define mkd0   0.002//0.002//0.003// 0.024  //d项值    小于0.25            30%

float pp=0;
float g='z';  
#define cs         40             // 40差速  小转角时没有差速
#define cs1         70             // 80差速  小转角时没有差速
#define cc         2.5             // 1.5差速系数调整
#define cc1         0.8             //0.8差速系数调整

uint now_speed0; 
uint now_speed;
int speed_ept0;
int speed_ept;
int a_m[7]=0;  //液晶屏变量
int b_m[7]=0;
int s_m[7]=0;
int e_m[7]=0;
int a_d[7]=0;
int x;
int f;
int w;
int Pulse_count=0;//码盘返回值
int Pulse_count0=0;
int ideal_speed;
int ideal_speed0;
int pre_error=0;
int pre_error0=0;
int ppre_error=0;
int ppre_error0=0;  
int speed;
int speed0;
//int speed_buff[5]; 
uint speed_tab[SPEED_DANG_NUM]={0};
int sadd=0,sadd_cnt=0;
uint LOW_SPEED=0;
uint HIGH_SPEED=0;
int car_driver=0,car_driver0=0;
float ad_div=0;
float pre_div;
int wxstop;
int  position;
uint Servo_Dir,preServo_Dir; 
int stop_flag=0,stop_cnt=0,slope_flag=0,START_LINE=0;
int slope_flag1=0,slope_flag2=0,slope_flag3=0;
int pcnt1=0,pcnt2=0;
int stop_finish;
uint ad_result[7]={0} ,ad_result10[6]={0};
/***********************duojiPxiang*******************************/ 
//int P_seg1=90,P_seg2=210,P_seg3=290,P_seg4=350,P_max=450;
int P_seg1=90,P_seg2=180,P_seg3=280,P_seg4=350,P_max=500;

void AD_ctrl(void);                 
void Steer(void);
int Cal_P_Item(void);
int Cal_D_Item(int length);            
void road_Judgement(void);
void Speed_Control(void);                   
int abs(int x);
void PWM_init_motor0(void);
void PWM_init_motor1(void);
void PWM_rudder_init(void); 
void steer_control(void);
void Dly_us(int jj);
void RD_TSL(void);
void dongtaiyuzhi(void);
void zhangai(void);
uint ADV0[128]={0},ADV_0[128]={0};     //声明数组，用于存放采集的线性数值
uint CCD1=0,CCD2=0,pre_CCD1=0,pre_CCD2=0,total=0;
uint slope_cnt=0,zhang_cnt=0,zai_cnt=0,ceju=0,ceju0=0;
volatile int s2w_flag=0,sza_flag=0,szal_flag=0,za_flag,zai_flag=0,lsza_flag=0,straight_flag=0,long_straight_flag=0,ls2w_flag=0,lls2w_flag=0,
     angleflag=1,st_flag=0,lst_flag=0,po_flag=0,zhang_flag,pozhi_flag=0,poqi_flag=0,zj_flag=0,lzj_flag=0;  
uint start=0,start_cnt=0;
volatile int po_cnt=0;
uint k_cnt;
int k=0;
int l2=0,r2=0,lock=0; 
int pit_cnt=0;
int pre_PTS_PTS4;
int pre_PTS_PTS2;
int tingche=0;
uint time;
int kaiguan=0,start_flag=0;
int r2,l2;

/**************************
////////锁相环初始化///////
****************************/
void SetBusCLK_40M()             
{   
    CLKSEL=0x00;             //时钟选择寄存器 设置时钟模式
    PLLCTL_PLLON=1;          //锁相环电路允许位
    SYNR= 0x04;        //SYNR=4
    REFDV=0x01;        //REFDV=1   PLLCLK=2*OSCCLK*(SYNR+1)/(REFDV+1)  OSCCLK=16M BusCLK=PLLCLK/2
    POSTDIV=0x00;                                        
    _asm(nop);                  //延时程序
    _asm(nop);                  //延时程序
    while(!(CRGFLG_LOCK==1));    //锁相环时钟稳定
    CLKSEL_PLLSEL =1;            //锁相环使能
}
/**************************
////////舵机初始化///////
****************************/
void PWM_rudder_init(void) 
{
   PWME_PWME2=0;
   PWME_PWME3=0;
   PWMPRCLK_PCKB=2;     //对总线时钟进行预分频，总线时钟为40M，0000 0010 4分频  Clock A=10Mhz           
   PWMCLK_PCLK3=1;    //设SA为其时钟源    (0,1,4,5可选时钟A或SA)
   PWMSCLB=2;         //SA分频   Clock SA=Clock A/(2*PWMSCLA)
   PWMCTL_CON23=1;
   PWMPOL_PPOL3=1;    //上升沿翻转  开始为高电平
   PWMCAE_CAE3=0;     //左对齐输出                       
   PWMPER23=50000;     //输出为频率为 Clock SA/PWMPER01 的波   
   PWMCNT23=0X00;      //通道计数器清0 
   PWME_PWME3=1;      //通道使能，1通道为输出通道 
   
   
  }  
/**************************
////////电机初始化///////
****************************/
void PWM_init_motor0(void)
{ 
   PWME_PWME0=0;
   PWMPRCLK_PCKA=2;     //对总线时钟进行预分频，总线时钟为40M，0000 0010 4分频  Clock A=10Mhz           
   PWMCLK_PCLK0=1;    //设SB为其时钟源    (0,1,4,5可选时钟A或SA)
   PWMSCLA=2;         //SB分频   Clock SB=Clock B/(2*PWMSCLA)
   PWMPOL_PPOL0=1;    //上升沿翻转  开始为高电平
   PWMCAE_CAE0=0;     //左对齐输出                       
   PWMPER0=250;     //输出为频率为 Clock SB/PWMPER01 的波   
   PWMCNT0=0X00;      //通道计数器清0 
   PWME_PWME0=1;      //通道使能，1通道为输出通道
   
   PWME_PWME1=0;
   PWMPRCLK_PCKA=2;     //对总线时钟进行预分频，总线时钟为40M，0000 0010 4分频  Clock A=10Mhz           
   PWMCLK_PCLK1=1;    //设SA为其时钟源    (0,1,4,5可选时钟A或SA)
   PWMSCLA=2;         //SA分频   Clock SA=Clock A/(2*PWMSCLA)
   PWMPOL_PPOL1=1;    //上升沿翻转  开始为高电平
   PWMCAE_CAE1=0;     //左对齐输出                        
   PWMPER1=250;     //输出为频率为 Clock SA/PWMPER01 的波   
   PWMCNT1=0X00;      //通道计数器清0 
   PWME_PWME1=1;      //通道使能，1通道为输出通道 
   
}
void PWM_init_motor1(void)
{ 
   PWME_PWME4=0;
   PWMPRCLK_PCKA=2;     //对总线时钟进行预分频，总线时钟为40M，0000 0010 4分频  Clock A=10Mhz           
   PWMCLK_PCLK4=1;    //设SB为其时钟源    (0,1,4,5可选时钟A或SA)
   PWMSCLA=2;         //SB分频   Clock SB=Clock B/(2*PWMSCLA)
   PWMPOL_PPOL4=1;    //上升沿翻转  开始为高电平
   PWMCAE_CAE4=0;     //左对齐输出                       
   PWMPER4=250;     //输出为频率为 Clock SB/PWMPER01 的波   
   PWMCNT4=0X00;      //通道计数器清0 
   PWME_PWME4=1;      //通道使能，1通道为输出通道
   
   PWME_PWME5=0;
   PWMPRCLK_PCKA=2;     //对总线时钟进行预分频，总线时钟为40M，0000 0010 4分频  Clock A=10Mhz           
   PWMCLK_PCLK5=1;    //设SA为其时钟源    (0,1,4,5可选时钟A或SA)
   PWMSCLA=2;         //SA分频   Clock SA=Clock A/(2*PWMSCLA)
   PWMPOL_PPOL5=1;    //上升沿翻转  开始为高电平
   PWMCAE_CAE5=0;     //左对齐输出                        
   PWMPER5=250;     //输出为频率为 Clock SA/PWMPER01 的波   
   PWMCNT5=0X00;      //通道计数器清0 
   PWME_PWME5=1;      //通道使能，1通道为输出通道 
 
 
}  
 void AD_Init(void)     
{
    ATD0DIEN=0x00;   //禁止数字输入
    ATD0CTL1=0x20;   //选择AD通道为外部触发,12位精度,采样前不放电  
    ATD0CTL2=0x40;   //标志位自动清零，禁止外部触发, 禁止中断     
    ATD0CTL3=0x80;   //右对齐无符号,每次转换4个序列, No FIFO, Freeze模式下继续转     
    ATD0CTL4=0x00;  //采样时间为4个AD时钟周期,PRS=9,ATDClock=40/(2*(9+1))2MHz  
    ATD0CTL5=0x30;   //特殊通道禁止,连续转换2个通道 ,多通道转换，起始通道为0转换 
    
}



/**************************
////////拨码开关///////
****************************/
void boman()
{
 if(PTM_PTM1==1&&PTM_PTM2==1) {         //1
  HIGH_SPEED=55;

 }
  if(PTM_PTM1==1&&PTM_PTM2==0) {          // 2
    HIGH_SPEED=50;
      
  }
if(PTM_PTM1==0&&PTM_PTM2==1) {          // 2
     HIGH_SPEED=45;
      
  }
      
if(PTM_PTM1==0&&PTM_PTM2==0) {          // 2
    HIGH_SPEED=40;
      
  }   
  
  
 if(PTM_PTM3==1&&PTM_PTM4==1) {
       
     LOW_SPEED=35;
 
  }
  
   if(PTM_PTM3==1&&PTM_PTM4==0) {
                   
       LOW_SPEED=30;
   }
   
   if(PTM_PTM3==0&&PTM_PTM4==1){
    
       LOW_SPEED=25; 
   }
   if(PTM_PTM3==0&&PTM_PTM4==0){
    LOW_SPEED=20;
   } 
 
}


/*int  Speed_Filter(int get_speed )
{
	static int j=0;
	int count;
	int  sum=0;
	speed_buff[j++]=get_speed;
	if(j==5)	j=0;
	for(count=0;count<5;count++)
		sum=sum+speed_buff[count];
	return (sum/5);
  
} 
 */
//*************************************************************************/
//**Function name:		  AD_Process1
//**Description:          方案一新电源AD数据处理(),确定值最大,次大传感器,并计算差值           
//**Input parameters:	  NONE
//**Output parameters:	  NONE
//**Returned value:		  NONE
//*************************************************************************/
void steer_control(void)
{  
  
  ////////****************光电算法处理*************////////////////
  int i=0,j=0,l1=-1,r1=-1;
  
  int shizi=0;
     for(i=63;i>0;i--)
      {
        l1=l1+1;
        l2=l1;
        if(ADV_0[i-1]==0&&ADV_0[i-2]==0&&ADV_0[i-3]==0&&ADV_0[i-4]==0) break;
      }
   
    for(j=64;j<127;j++)
      {
        r1=r1+1;
        r2=r1;
        if(ADV_0[j+1]==0&&ADV_0[j+2]==0&&ADV_0[j+3]==0&&ADV_0[j+4]==0) break;
      }
     
    ad_div=((float)r1-(float)l1)/50;
     ad_div=ad_div*500;
    ad_div=ad_div+(ad_div-pre_div)*1/30;
    ad_div=ad_div*4/5; 

  if(r1>55||l1>55||lock==1)  // ||shizi==1
      {
         if((r1-l1)<-48) 
             ad_div=-450; 
               
         if((r1-l1)>48)
         
             ad_div=450;
         lock=1;       
      }
                  
    
    if(r1>45&&l1>45)
      { 
          shizi=1;
          
      }
      
    if(shizi) 
      {
         ad_div=pre_div; 
       } 
     if(r1+l1<25)
         ad_div=pre_div;
      
    if(r1<40&&r1>25&&l1>25&&l1<40)
      {
        lock=0;
               
      } 
          
} 
void AD_ctrl(void)
{   
  uint angle,pre_angle; 
      
      steer_control(); 
      pre_div =ad_div;
      if(lock==1)
         ad_div=pre_div;
       if((ad_div-pre_div)>300||(ad_div-pre_div)<-300)
          ad_div=pre_div;    
      
      position=(int)ad_div;

      Steer();

     // angle=servo_mid+ad_div;              //  先测试用
       angle=Servo_Dir;
       
      if(angle>=servo_max)
      {  
         angle=servo_max;     
      }
      if(angle<servo_min)
      { 
         angle=servo_min;
        }
      pre_angle=angle;
      
     
      PWMDTY23=angle; 
      
     
}

  void Steer(void)              /* 前轮舵机转向 */
{
       uint Dir_Tmp;
       int P,D;
       uint pre_dir; 
                                                                 // ppre_dir=pre_dir; 
      
        pre_dir =Servo_Dir;                // (int16)
         
        P = Cal_P_Item();
        
        D = Cal_D_Item(d_len);
       
        Dir_Tmp = servo_mid+P+D*d_amp ;         //d项 ,d_amp可调P++D*d_amp 
          
      
        if(Dir_Tmp<servo_min)
        {
            Dir_Tmp = servo_min ;
        }
        else if(Dir_Tmp>servo_max)
        {
            Dir_Tmp = servo_max ;
        }
     
         Servo_Dir = (int)Dir_Tmp ; 
         
}

int Cal_P_Item(void)
   {
   int P_part ; 
    
    P_part = abs(position);
    
 if(position>0)               //左侧
    {  
         if(P_part<20)
        {
            return 0;
        }
         else if(P_part<P_seg1)
        {
            return -(P_part*2/3);
         }
        else if(P_part<P_seg2)
        {  
            return -( P_seg1*2/3+ (P_part-P_seg1)*2/5 );
        }
        else if(P_part<P_seg3)    
        {
            return -(P_seg1*2/3 + (P_seg2-P_seg1)*2/5  + (P_part-P_seg2)*3/7) ;
        }
           else if(P_part<P_seg4)
        {
            return -(P_seg1*2/3 + (P_seg2-P_seg1)*2/5 + (P_seg3-P_seg2)*3/7+ (P_part-P_seg3)*3/2) ;
        }
        else                     //超过这个范围就P 最大
        {
            if((P_seg1*2/3+ (P_seg2-P_seg1)*2/5  + (P_seg3-P_seg2)*3/7+ (P_part-P_seg3)*3/2)<P_max)
            {
                return -(P_seg1*2/3+ (P_seg2-P_seg1)*2/5 + (P_seg3-P_seg2)*3/7+ (P_part-P_seg3)*3/2);
            }
            else
            {
                return -P_max;
            }
            
        }
    }
    else // 右侧
    {   
    if(P_part<20)
         {
             return 0;
         }
        if(P_part<P_seg1)
        {
            return P_part ;
        }
        else if(P_part<P_seg2*2/3)
        {
            return ( P_seg1*2/3+ (P_part-P_seg1)*2/5  );
        }
        else if(P_part<P_seg3)
        {
            return (P_seg1*2/3 + (P_seg2-P_seg1)*2/5  + (P_part-P_seg2)*3/7);
        }
         else if(P_part<P_seg4)
        {
            return (P_seg1*2/3 + (P_seg2-P_seg1)*2/5  + (P_seg3-P_seg2)*3/7 + (P_part-P_seg3)*3/2) ;
        }
        else //超过这个范围就P 最大
        {
            if((P_seg1*2/3+ (P_seg2-P_seg1)*2/5 + (P_seg3-P_seg2)*3/7+ (P_part-P_seg3)*3/2)<P_max)
            {
                return (P_seg1*2/3 + (P_seg2-P_seg1)*2/5  + (P_seg3-P_seg2)*3/7+ (P_part-P_seg3)*3/2);
            }
            else
            {
                return P_max;
            }
        }
    }
  }
int Cal_D_Item(int length)       /* 前轮舵机D项计算 */
{
  
  
    int Dcal_D, err_1, err_2, q_l_1, q_l_2, q_l_3;
    static int Z1[102],queue_index=0;                     /* 历史数据 */
                                                                           //循环队列
    Z1[queue_index] = position*4/5;                           //Z1[queue_index] = position*3/4;
    
    queue_index = (queue_index+1) % 102 ;
    
    q_l_1 = queue_index-1;
    
    if(q_l_1<0)
    {
        q_l_1 = 102+q_l_1;
    }
    
    q_l_2 = queue_index-2;
    if(q_l_2<0)
    {
        q_l_2 = 102+q_l_2;
    }
    
    q_l_3 = queue_index-3;
    if(q_l_3<0)
    {
        q_l_3 = 102+q_l_3;
    }
    err_1 = (Z1[q_l_1]+Z1[q_l_2]+Z1[q_l_3])/3 ;
    
    q_l_1 = queue_index-1-length;
    if(q_l_1<0)
    {
        q_l_1 = 102+q_l_1 ;
    }
    
    q_l_2 = queue_index-2-length;
    if(q_l_2<0)
    {
        q_l_2 = 102+q_l_2 ;
    }
    
    q_l_3 = queue_index-3-length;
    if(q_l_3<0)
    {
        q_l_3 = 102+q_l_3 ;
    }
    
    err_2 = (Z1[q_l_1]+Z1[q_l_2]+Z1[q_l_3])/3 ;
    Dcal_D = err_1 - err_2; 
    
    return Dcal_D;
}       
 
int abs(int x)
{
if(x<0)
return -x;
else
return x;
}

void Init_speed_tab(void)
{
    int i;
    for(i=0; i<SPEED_DANG_NUM; i++)
    { /********上弧**************/
       //     speed_tab[i] =HIGH_SPEED-(int)((long)(i-SPEED_DANG_NUM+1)*(long)(i-SPEED_DANG_NUM+1)*(long)(HIGH_SPEED-LOW_SPEED)/
             //               (long)(SPEED_DANG_NUM-1)/(long)(SPEED_DANG_NUM-1));
     speed_tab[i] =HIGH_SPEED-(int)((long)i*(long)i*(long)(HIGH_SPEED-LOW_SPEED)/
                      (long)(SPEED_DANG_NUM-1)/(long)(SPEED_DANG_NUM-1));               
     /********下弧**************/  
   //speed_tab[i] =LOW_SPEED+(int)((long)(i-SPEED_DANG_NUM+1)*(long)(i-SPEED_DANG_NUM+1)*(long)(HIGH_SPEED-LOW_SPEED)/
      //(long)(SPEED_DANG_NUM-1)/(long)(SPEED_DANG_NUM-1));
    } 
} 

 void MOTOR_forward(int mt2,int mt3)
{
int s;
s=mt2-mt3;

if(s>-2&&s<2)
{  
    if(mt2>=0) 
   {    
   PWMDTY0=0;
   PWMDTY1=mt2;                          //  MCF_PWM_PWMDTY0=(uint8)(mt2);                                                                               //  MCF_PWM_PWMDTY1=0;
  
   PWMDTY5=0;
   PWMDTY4=mt2;      
   } 
   else
   {
     mt2=-mt2;
     PWMDTY0=mt2; 
     PWMDTY1=0;        
     PWMDTY5=mt2;
     PWMDTY4=0;                             // MCF_PWM_PWMDTY1=(uint8)(mt2);                                                              // MCF_PWM_PWMDTY0=0;       
   }                     
}
else
{
   if(mt2>=0) 
   {    
   PWMDTY0=0;
   PWMDTY1=mt2;                          //  MCF_PWM_PWMDTY0=(uint8)(mt2);                                                                               //  MCF_PWM_PWMDTY1=0;
   } 
   else
   {
     mt2=-mt2;
     PWMDTY0=mt2;         
     PWMDTY1=0;                       // MCF_PWM_PWMDTY1=(uint8)(mt2);                                                              // MCF_PWM_PWMDTY0=0;       
   }
   
   if(mt3>=0) 
   {   
   PWMDTY5=0;
   PWMDTY4=mt3;                             //  MCF_PWM_PWMDTY0=(uint8)(mt2);                                                                            //  MCF_PWM_PWMDTY1=0;
   } 
   else 
   {
     mt3=-mt3;        
     PWMDTY5=mt3;
     PWMDTY4=0;                        // MCF_PWM_PWMDTY1=(uint8)(mt2);                                                             // MCF_PWM_PWMDTY0=0;       
   }
   
}	
}


void MOTORforward_Control(void) 
{  
         
 if((ideal_speed-Pulse_count)>=15||( Pulse_count-ideal_speed )>=15)          //bang_bang
      	{
          	if(ideal_speed >Pulse_count)
 	         	  {
  	         	    car_driver=250;
 	         	   }
        	  else
         	    {
                  car_driver=-250; 
       	       }
  	     }
 else
        {    
            car_driver= motor_ctrl2(ideal_speed,Pulse_count);                                                                //   V_PIDCalc(&sPID)  ; 
         }
 if((ideal_speed0-Pulse_count0)>=15||( Pulse_count0-ideal_speed0 )>=15)          //bang_bang
      	{
          	if(ideal_speed0 >Pulse_count0)
 	           	{
  	         	    car_driver0=250;
 	         	   }
        	  else
         	    {
                   car_driver0=-250; 
       	        }
  	    }
 else
        {    
             car_driver0= motor_ctrl20(ideal_speed0,Pulse_count0);                                                                //   V_PIDCalc(&sPID)  ; 
        }   
                
    MOTOR_forward(car_driver,car_driver0);   	
    	
}   
   


/*************************速度控制*****************/
  
 int motor_ctrl2(int ideal_speed,uint Pulse_count)
 {                     
    int error;
    int d_error;
    int dd_error;
   
    error=ideal_speed-Pulse_count;	          //偏差计算(积分)   速度的偏差		
    d_error = error - pre_error;               //偏差计算(比例)  速度的偏差率
    dd_error = d_error -ppre_error;           //偏差计算(微分)   
        
	   pre_error = error;		                      //存储当前偏差
	   ppre_error=d_error; 

    if(Pulse_count<=5)                        //PID调整程序    ：避免车在急刹车时出现倒转的关键程序
    {
       speed =250;
       pre_error=0;
       ppre_error=0;     
    }              
   
    if(( error < VV_DEADLINE ) && ( error > -VV_DEADLINE ))
     {
       ;            //do nothing
      }
    
    else
     {
      speed+=(int)(mkp*d_error + mki*error + mkd*dd_error); //速度PID控制算式 
  
      }
    
    if( speed >= VV_MAX ) 		  //速度PID，防止调节最高溢出
	      { speed = VV_MAX;
	      //	 car_driver=VV_MAX;
	      } 
    else if( speed <= VV_MIN )	//速度PID，防止调节最低溢出 
	       {speed = VV_MIN; 
	     //  	  car_driver= VV_MIN; 
	       } 	
                                                   
 	  return (speed );	          

   
 }                  


 int motor_ctrl20(int ideal_speed0,uint Pulse_count0)
 {                     
    int error;
    int d_error;
    int dd_error;
   
    error=ideal_speed0-Pulse_count0;	          //偏差计算(积分)   速度的偏差		
    d_error = error - pre_error0;               //偏差计算(比例)  速度的偏差率
    dd_error = d_error -ppre_error0;           //偏差计算(微分)   
        
	   pre_error0= error;		                      //存储当前偏差
	   ppre_error0=d_error; 

    if(Pulse_count0<=5)                        //PID调整程序    ：避免车在急刹车时出现倒转的关键程序
    {
       speed0 =250;
       pre_error0=0;
       ppre_error0=0;     
    }              
   
    if(( error < VV_DEADLINE ) && ( error > -VV_DEADLINE ))
     {
       ;            //do nothing
      }
    
    else
     {
      speed0+=(int)(mkp0*d_error + mki0*error + mkd0*dd_error); //速度PID控制算式 
  
      }
    
    if( speed0 >= VV_MAX ) 		  //速度PID，防止调节最高溢出
	      { speed0 = VV_MAX;
	      //	 car_driver=VV_MAX;
	      } 
    else if( speed0 <= VV_MIN )	//速度PID，防止调节最低溢出 
	       {speed0 = VV_MIN; 
	     //  	  car_driver= VV_MIN; 
	       } 	
                                                   
 	  return (speed0 );	          
  
 }                  


void road_Judgement(void)     
{ 
    static int sza_cnt=0,lsza_cnt=0,straight_cnt=0,s2w_cnt=0,st_cnt=0,lst_cnt=0,ls2w_cnt=0,lsb_cnt=0,zj_cnt0=0,lzj_cnt=0,po_cnt1=0,slope_cnt,sz_cnt=0;  
    static int long_straight_cnt=0,zj_cnt=0,pozhi_cnt=0,za_cnt=0,lls2w_cnt=0;
       
    /***************障碍判断，必须的直道判断，在有一段直道时才判断障碍**************/
    //if(abs(position)>=80)StopFlag = 1;
     if(abs(position)<=200) 
    { 
      if(zai_cnt<20)      //800
          zai_cnt++ ; 
      if(zai_cnt>=20) 
      { 
         zai_flag = 1;
         zai_cnt = 0; 
      } 
    } 
    else 
    {
       zai_flag=0;
       zai_cnt = 0; 
    }  
   /***************坡道判断，同上**************/ 
     if(abs(position)<=170) 
    { 
      if(za_cnt<20)      //800
          za_cnt++ ; 
      if(za_cnt>=20) 
      { 
         za_flag = 1;
         za_cnt = 0; 
      } 
    } 
    else 
    {
       za_flag=0;
       za_cnt = 0; 
    }  
    
    
    
   /***************短直道 **************/
    
   /***************短直道 **************/
    
    if(abs(position)<=100) 
    { 
      if(straight_cnt<10)      //800
          straight_cnt++ ; 
      if(straight_cnt>=10) 
      { 
         straight_flag = 1;
         straight_cnt = 0; 
       
      } 
    } 
    else 
    {
       straight_flag = 0;
       straight_cnt = 0; 
       
    } 
     
    
    if(straight_flag && abs(position)>120)
    {
       straight_flag = 0;
       s2w_flag = 1;
                            //StopFlag = 1;
    }
    

    if(s2w_flag)
    {
      s2w_cnt++;
      if(s2w_cnt>=20)              //2000
      {
        s2w_cnt = 0;
        s2w_flag=0;
       
      }
    }
 
    /***************长直道**************/
   
    
    if(abs(position)<=200) 
    { 
      if(long_straight_cnt<30) 
          long_straight_cnt++ ; 
      
      if(long_straight_cnt>=30)                  // 2000
      { 
         long_straight_flag = 1;
         long_straight_cnt =0; 
        
      } 
    }
    else 
    {
       long_straight_flag = 0;
       long_straight_cnt =0; 
    }
    
    
    if(long_straight_flag && abs(position)>180)                                      //180
    {
       long_straight_flag = 0;
       ls2w_flag = 1;                                 
                                                                                                    // wxstop=1; 
    }
    
    
    if(ls2w_flag)
    {
      ls2w_cnt++;
      if(ls2w_cnt>=20)    //     180  180
      {
        ls2w_cnt =0;
        ls2w_flag=0;
        
         }
     }
/***************检测到坡道，屏蔽障碍**************/     
if(PTM_PTM6==1)
{

 if(l2>28&&l2<40&&r2>28&&r2>40&&za_flag)
      
  {
  	slope_flag=1;
  	PORTB_PB0=0;                                   //wxstop=1;
  }


 if(slope_flag)
    { 
    
      PWMDTY23=servo_mid;
      pozhi_flag=1;
      poqi_flag=1;
     wxstop==0;
      slope_cnt++;
      if(slope_cnt>=30)    //     180  180
      {
        slope_cnt =0;
        slope_flag=0; 
         }
     }
 if(pozhi_flag)  //pozhi_flag
    {
     PWMDTY23=servo_mid;
     zai_cnt=0;
     pozhi_cnt++;
     wxstop==0;
        if(pozhi_cnt>40)
             pozhi_cnt=40;
        if(pozhi_cnt>40&&l2<38&&r2<38) 
         {
             pozhi_cnt=0;
             pozhi_flag=0;
             PORTB_PB0=1;	
         }
    }
    
  /******坡道标志，减速*******/   
  if(l2>28&&l2<40&&r2>28&&r2>40&&za_flag)
      
  {
  	slope_flag1=1;
  }
 
   else  
     slope_flag1=0;
  }

}   
 


      
void Speed_Control(void)
{

    int sp_dang; 
    
   // dir =  absabs(position)/4;
    
    sp_dang = abs(position)/5;        //12 系数需调整     
    
    if(sp_dang>=SPEED_DANG_NUM)
    {
        sp_dang=SPEED_DANG_NUM-1;
    }

    speed_ept= speed_tab[sp_dang];
    speed_ept0= speed_ept;//speed_tab[sp_dang];	
   
   
    if(sp_dang>=cs1) //cs,cs1,差速分段，根据转向调整差速
    {
    if(Servo_Dir>servo_mid)
     {
       speed_ept0 = speed_ept0; 	  
       speed_ept =(int)(speed_ept*(500.0+(float)(cs1-cs)+(float)(sp_dang-cs1)/cc1)/500); //差速   5    
         }
      if(Servo_Dir<servo_mid)
     {
       speed_ept =speed_ept; 
        speed_ept0= (int)(speed_ept0*(500.0+(float)(cs1-cs)+(float)(sp_dang-cs1)/cc1)/500);  	    //5     
         }
    }   
    
   else  if(sp_dang>=cs) 
    {
      if(Servo_Dir>servo_mid)
     {speed_ept0 =speed_ept0;                                        
       speed_ept = (int)(speed_ept*(500.0+(float)(sp_dang-cs)/cc)/500);  	   //      3        +8  
             
         }
      if(Servo_Dir<servo_mid)
     {
         speed_ept =speed_ept;
       speed_ept0 = (int)(speed_ept0*(500.0+(float)(sp_dang-cs)/cc)/500); 	   //     3
         }
    
    }
    
 
 /* if(abs(Servo_Dir-preServo_Dir)<80) 
   {
     sadd=1;
   
   if(abs(Servo_Dir-servo_mid)>200&&abs(Servo_Dir-servo_mid)<=500&&sadd==1)
   {
     sadd_cnt++;	
   }
   else 
   {
   	sadd=0;
   	sadd_cnt=0;
   }
  
 
  if(sadd_cnt>13)      //直道入弯道减速后的加速
  {
  
     speed_ept=LOW_SPEED+((sadd_cnt-13)/0.9)*((sadd_cnt-13)/0.9) ;          //+9+((500-(Servo_Dir-servo_mid))/20) 	
     speed_ept0=speed_ept ;
     
      if(Servo_Dir>servo_mid&&sp_dang>cs)
     {
       speed_ept0 =speed_ept0;      
       speed_ept =( int)(speed_ept*(500.0+(float)(sp_dang-cs))/500+10); 
       	         
         }
      if(Servo_Dir<servo_mid&&sp_dang>cs)
     {
        speed_ept=speed_ept; 
        speed_ept0 =( int)(speed_ept0*(500.0+(float)(sp_dang-cs))/500+10);  	         
         } 
 
       if(Servo_Dir>servo_mid&&sp_dang>cs1)
     {
       speed_ept0 = speed_ept0;
       speed_ept =( int)(speed_ept*(500.0+(float)(cs1-cs)+(float)(sp_dang-cs1)/cc1)/500+8);  	         
         }
      if(Servo_Dir<servo_mid&&sp_dang>cs1)
     {
       speed_ept=speed_ept;                                                                
        speed_ept0 = ( int)(speed_ept0*(500.0+(float)(cs1-cs)+(float)(sp_dang-cs1)/cc1)/500+8);  	         
      }
    
     }
    if(sadd_cnt>16)     
   {
   	sadd_cnt=16;
     } 
     
     
     if(abs(Servo_Dir-servo_mid)>=450&&sadd_cnt>5)//死角时的差速
     {
     if(Servo_Dir>servo_mid)
     {
       speed_ept0 = speed_ept0;
       speed_ept=1.2*speed_ept0;  	         
         }
      if(Servo_Dir<servo_mid)
     {
       speed_ept=speed_ept; 
        speed_ept0 =(uint)(1.2*speed_ept);  	         
      }
    	
     }
      if(speed_ept>95)
        speed_ept = 95;
       if(speed_ept0>95) 
        speed_ept0 = 95;
     
	 }
   else
   {
     	 sadd_cnt=0;
    	 sadd=0;
   }
   
   */
   preServo_Dir=Servo_Dir;
   
    
   
  if(straight_flag)
   {
    speed_ept=HIGH_SPEED+5; 
     speed_ept0= speed_ept;  
     
   } 
   
  
  
   if(long_straight_flag )   
   {
      speed_ept=HIGH_SPEED+10;    //+HIGH_SPEED/10; 	
      speed_ept0= speed_ept; 
      
   }
      
      
  if(s2w_flag)//短直道入弯       
   {
     speed_ept=now_speed-3;//0; now_speed-5
     speed_ept0= now_speed-3; 
       if(Servo_Dir>servo_mid)
     {
       speed_ept0 = now_speed0-8;       //8
       speed_ept =now_speed-2;  	
                
         }
      if(Servo_Dir<servo_mid)
     {
       speed_ept =now_speed-8; 
       speed_ept0 = now_speed0-2;  	
         
                                         //8
       	         
         }       
      /* if(speed_ept<40)
          speed_ept = 40;
         if(speed_ept0<40) 
          speed_ept0 = 40;*/	 
      //if(speed_ept<s2w_speed)
      //speed_ept = s2w_speed ;  
   // wxstop=1; 
    } 
  

   if(ls2w_flag ) //长直道入弯
   {
       speed_ept =now_speed-8;        //3
       speed_ept0 =now_speed0-8;      //3
  
   
   if(sp_dang<=cs1) 
    {
      if(Servo_Dir>servo_mid)
     {
       speed_ept0 = speed_ept0-8;        //13
       speed_ept =( int)(speed_ept*(500.0+(float)(cs1-cs))/500-2); 	 //10        
         }
      if(Servo_Dir<servo_mid)
     {
       speed_ept =speed_ept-8;          //13
       speed_ept0 = ( int)(speed_ept0*(500.0+(float)(cs1-cs))/500-2);  //10	         
         }
    }
    else
    {
    if(Servo_Dir>servo_mid)
     {                                              //13
       speed_ept0 = speed_ept0-5;
       speed_ept =( int)(speed_ept*(500.0+(float)(cs1-cs)+(float)(sp_dang-cs1)/cc1)/500-2);  	         
         }                                          
      if(Servo_Dir<servo_mid)
     {
       speed_ept =speed_ept-5;                     //13
       speed_ept0 = ( int)(speed_ept0*(500.0+(float)(cs1-cs)+(float)(sp_dang-cs1)/cc1)/500-2);  	         
         }
    }

    /*	 if(speed_ept<35)
          speed_ept = 35;
         if(speed_ept0<35) 
          speed_ept0 = 35;	*/

    //   wxstop=1;
   }
 

 if(slope_flag1) //坡道标志
  {   

     speed_ept =now_speed-15; 
     speed_ept0 = speed_ept; 
     if(now_speed<45) 
     {
   //  if( speed_ept<45)
          speed_ept=45;
    // if( speed_ept0<45)
          speed_ept0=45;	
     }
      
     
 
  }  
  
     
     ideal_speed=speed_ept;
     ideal_speed0=speed_ept0; 
    MOTORforward_Control() ;

}
  

/**************************
////////脉冲累加器初始化///
****************************/

void PA_Init(void)
{
  
  	PACTL_PAEN=0;                 // PACTL = 0x40; 
    PACNT=0x00;
    PACTL_PAMOD=0;
    PACTL_PEDGE=0;
    PACTL_PAEN=1;          
}

/**************************
////////PIT初始化///
****************************/
void initPIT(void)//定时中断初始化函数20ms 定时中断设置
{
   
    PITCFLMT_PITE=0;                      /* 禁止 PIT                   */
    PITCE_PCE0=1;                         /* 使能通道 0                 */
    PITMUX=0X00;                          /* ch0 连接到微计数器         */
                                          //定时周期=(PITMTLD0+1)*(PITLD0+1)/40Mhz=1ms
    PITMTLD0=200-1;                       /* 时钟基于40M  该寄存器用于设置PIT模块中的8位计数器初值，以实现24位的计数。设定值为0到255范围              */
    PITLD0=200-1;                        /* 溢出计数初值的设定         */
    PITINTE_PINTE0=1;                     /* 使能通道0中断              */
    PITCFLMT_PITE=1;                      /* 使能 PIT                   */
}



/**************************
////// 延时函数（cnt*1ms）////////////
****************************/    
void delay(uint cnt) {
  uint loop_i,loop_j;
  for(loop_i=0;loop_i<cnt;loop_i++) {
    loop_j=0x1300;              
    while(loop_j--);
  }
}

void zhangai(void)			//障碍检测处理
{    uint d=0;
	 if(!poqi_flag)		
        {
           if(zai_flag&&((r2<15&&l2>20&&l2<35)||(l2<15&&r2>20&&r2<35)))//&&ad_result[7]<1400
          {
        
          if(r2<15)
          {
          for(d=2000;d>0;d--)
          	{
          
          	                //3750
              PWMDTY23=servo_mid+300;                      
              
          	}
          		for(d=2000;d>0;d--)
          	{
          
          	  
              PWMDTY23=servo_mid; 	
          	}	
          }     
           if(l2<15)
           
           {
           
          		for(d=2000;d>0;d--)
          	{
          
          	               //3750
              PWMDTY23=servo_mid-300;  
          	}
          		for(d=2000;d>0;d--)
          	{
          
          	  ;              //3750
              PWMDTY23=servo_mid; 
 	           
          	}	  
           }
          
          }
           
       }
       
   /*     if(!poqi_flag)		
        {
           if(zai_flag&&((r2<15&&l2>25&&l2<35)||(l2<15&&r2>25&&r2<35)))
          {
        
          if(r2<15)
          {
          for(d=2000;d>0;d--)
          	{
          
          	  
              PWMDTY23=servo_mid+450;  	
          	}
          		for(d=2000;d>0;d--)
          	{
          
          	  //PORTB_PB0=1;
              PWMDTY23=servo_mid; 	
          	}	
          }
           if(l2<15)
           {
           
          		for(d=2000;d>0;d--)
          	{
          
          	  
              PWMDTY23=servo_mid-450;  
          	}
          		for(d=2000;d>0;d--)
          	{
          
          	  
              PWMDTY23=servo_mid; 
 	
          	}	
           }  
          
          } 
                
       }*/
}   


/**************************
////// 主函数////////////
****************************/   

void main() 
{
    
    DisableInterrupts;
    
    SetBusCLK_40M();
    
   // PTTRR=0x30;
    PWM_rudder_init();
    PWM_init_motor0();
    PWM_init_motor1();
    AD_Init();
    
    	  	
     
    initPIT();  
    PA_Init();
 ///////////////IO口////////////////////// 
    PORTB=0xff;
    DDRB=0x01;  
    PORTA=0xff;
    DDRA=0xff;      
    PTH=0xff;
    DDRH=0x00;//H口设置为输入
    DDRM=0x00;
    DDRS=0X00;     //S口设置为按键输入
    PORTE=0xff;
    DDRE=0xff;
    
    PUCR=0X10;         //选择PWM对应口 
    
    IRQCR=0X00;
    LCD_init();  //初始化LCD模块
    PWMDTY23=servo_mid;
    delay(1000);
   
    boman();
    Init_speed_tab();

    EnableInterrupts;
              
    for(;;) {  
         
         RD_TSL();//读取线性传感器的数值
          dongtaiyuzhi();             
            
     	    //zhangai(); 
                      
          AD_ctrl();
         	
         
        
      	   //zhangai();
      	 
                
	      start_flag=1;
	      if(start_flag)   
	       { 
	             
               /*if(time>18000)
                 {
                    
                    if((PORTB_PB3==1&&PORTB_PB1==1)||(PORTB_PB3==1&&PORTB_PB2==1)||(PORTB_PB4==1&&PORTB_PB2==1)||(PORTB_PB4==1&&PORTB_PB1==1))
	                       {
	                        if(Servo_Dir>3280&&Servo_Dir<3620)
	                        wxstop=1;
                         
	                       }     
	                     	
                 }  */       
                     
            
            
	        if(wxstop==1)
	         { 
	 
	    	        road_Judgement();
	              if( now_speed>10&&stop_finish==0) 
	                 {    
	                     // PORTA=11111100;
	                      ideal_speed=0;
	                      ideal_speed0=0;
	                      MOTORforward_Control(); 
	                  }
	              else
	                 {     
	                      stop_cnt++;
	                     // PORTA=11111111;
	                      PWMDTY0=0;
   	                    PWMDTY1=0;
   	                    PWMDTY4=0;
                        PWMDTY5=0;    
	                  } 
	         
	             if(stop_cnt>10)             //        4500
	                  {
	                      stop_finish=1;	
	                      stop_cnt=0;
	                      //PORTB_PB0=0;
	                  }     
	        
	         }        
	       else
	         {   
	             road_Judgement(); 
	       
	             if(start==0)
	                 {   
	                           PWMDTY0=0;
   	                         PWMDTY1=200;
   	                         PWMDTY4=200;
                             PWMDTY5=0;    
                             start_cnt++; 
	                           if( start_cnt>30)//4000
                               {
                                  start_cnt=0;
                                  start=1;	
                               }
                            
                        
	                 
	                 }
	             else   
                   {   
                                
                      Speed_Control();
                  
                   }   
                
            
	         }
	       }  
	     
	    }
      
}
    

/*********************************光电****************************************/
/*********************************光电****************************************/
void Dly_us(int jj)
{
   int ii;    
   for(ii=0;ii<jj;ii++) asm("nop"); //300 100   170 50us   120 20us  150 30   180 60   
}
     
void RD_TSL(void) 
{ 

  uint i=0,j=0,k=0,t=0,tslp=1;

  PORTA_PA5=1;//TSL_CLK=1;//起始电平高
  PORTA_PA6=0;//TSL_SI=0; //起始电平低
 
 
  Dly_us(50); //合理的延时  //300    
  
  PORTA_PA5=0;//TSL_CLK=0;//下降沿
  PORTA_PA6=1;//TSL_SI=1; //上升沿
 

  Dly_us(50); //合理延时

  PORTA_PA5=1;//TSL_CLK=1;//起始电平高
  PORTA_PA6=0;//TSL_SI=0; //起始电平低
  
  Dly_us(50); //合理延时      
  for(i=1;i<128;i++)
    { 
        PORTA_PA5=0;//TSL_CLK=0;//下降沿
 
        Dly_us(300); //根据环境光线的亮度，合理延时
        while(!ATD0STAT2L_CCF0); 
        ADV0[tslp]=  ATD0DR0L;  //ADC0-12CH采集,AN0,12bit
   
  
        total=total+ADV0[tslp];

        ++tslp;
        PORTA_PA5=1;//TSL_CLK=1;//上升沿 

        Dly_us(50); //根据环境光线的亮度，合理延时 
   }

}

void dongtaiyuzhi(void)
{
   uint cnt=0,cnts=0,f=0;
   long int m=0,q=0,total_low=0,total_high=0;
   long int avfor=0,avback=0,g1=0,g2=0;    //u32  long int
   int max1=0,min1=0,g=0;    
   if(ad_div<350&&ad_div>-350)
      {
    
       max1=ADV0[5];
       min1=ADV0[5];
 
       for(g=5;g<120;g++)
         {
       
           if(max1<ADV0[g]&&ADV0[g]<250)
               max1=ADV0[g];
           if(min1>ADV0[g]&&ADV0[g]>3)
               min1=ADV0[g];
            
          }
      CCD1=(2*max1+min1)*2/7;// 4 5/18        2 2/7        2 1/4
   
     }
   
   else
     {
       CCD1=pre_CCD1;
   
      }
 
    for(f=1;f<127;f++)
      {
         if(ADV0[f]>CCD1) 
             ADV_0[f]=1;
         else   
             ADV_0[f]=0;
   
       }
  
    pre_CCD1=CCD1;
    CCD2=CCD1;
 
 }

/*********************************光电****************************************/  
/*********************************光电****************************************/
    

#pragma CODE_SEG __NEAR_SEG NON_BANKED
void interrupt 66 PIT0(void)           //pit中断 码盘测得速度值
{
   
   DisableInterrupts;
  
   pit_cnt++;

  if(pit_cnt==2)
  {
     PACTL_PAEN=0;
     Pulse_count=PACNT;
     Pulse_count0=PTH&0xff;
    
     PORTE|= (0x01<<7);
     Dly_us(1);
     PACNT=0x00; 
     PORTE&=~(0x01<<7);
  
     pit_cnt=0;
    
    //pcnt1 = Pulse_count;       //测试用
   
    //pcnt2+= Pulse_count;
     now_speed= Pulse_count;
     now_speed0= Pulse_count0;
    //PACNT=0;
     PACTL_PAEN=1;
   }
   time++;
   
  if(PTM_PTM0==1){
  
       if(time>16000)
     time=19000;  
  }
 if(PTM_PTM0==0){
  
       if(time>17000)
        time=19000;  
  } 
        
  
  
  if(PTM_PTM7==1)
  { 
   	 k_cnt++;
   	 if(k_cnt>25) 
  	    wxstop=1;
   	  
     if(k_cnt==200)    
      {  
     LCD_P6x8Str(1,1,"ASDF");
      //LCD_P14x16Str(0,0,"姜");
      //LCD_write_shu(35,0,g,4);
      LCD_write_shu(35,0,g,4);
               //LCD_write_shu(1,0,g,4);
              //LCD_write_shu(40,0,Servo_Dir,4);
              //LCD_write_shu(1,1,Servo_Dir,4);
              //LCD_write_shu(40,1,Servo_Dir,4);
            	//LCD_write_shu(1,2,l2,4);
            	//LCD_write_shu(40,2,r2,4);   
     LCD_write_shu(1,3,Pulse_count,4);                                                   
            ////	LCD_write_shu(40,3,Pulse_count0,4);            
            //	LCD_write_shu(1,4,LOW_SPEED,4);
              //LCD_write_shu(40,4, HIGH_SPEED ,4);
             // LCD_write_shu(1,5,time,6);
             // LCD_write_shu(40,5,ad_result[6],5); 
              k_cnt=0;  
        }
   }      
    PITTF_PTF0=1;  
   
    PITTF_PTF0=1;                     //清中断标志
    
    EnableInterrupts;        

    
}        
#pragma CODE_SEG DEFAULT