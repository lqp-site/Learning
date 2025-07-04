一、HAL库配置

<img src="C:\Users\Administrator\AppData\Roaming\Typora\typora-user-images\image-20240805141136607.png" style="zoom: 80%;" />

<img src="C:\Users\Administrator\AppData\Roaming\Typora\typora-user-images\image-20240805141602493.png" style="zoom:80%;" />

![](C:\Users\Administrator\AppData\Roaming\Typora\typora-user-images\image-20240805141850735.png)

```c
int main(void)
{
  HAL_Init();
    
  SystemClock_Config();
    
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
    
	if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK)
    {
        Error_Handler(); // 错误处理：编码器启动失败
    }

    while (1)
    {
        //计数脉冲，累加当前的值 可能为正也可能为负
        curentLenght += (short)(__HAL_TIM_GET_COUNTER(&htim2));  
		__HAL_TIM_SET_COUNTER(&htim2,0); //读取完成清空
        HAL_Delay(100);
        
    }
}
```

```

```

二、标准库配置



三、寄存器配置

```c
RCC->APB1ENR|=1<<0;         //TIM2时钟使能
RCC->APB1ENR|=1<<1;         //TIM3时钟使能
RCC->APB1ENR|=1<<3;         //TIM5时钟使能
RCC->APB2ENR|=1<<1;        //TIM8时钟使能
GPIO_AF_Set(GPIOD,12,2);    //PD12,AF2    
GPIO_AF_Set(GPIOD,13,2);    //PD13,AF2 
static void _TIM2_Configuration(void)
{
  TIM2->PSC = 0x0;//预分频器
  TIM2->ARR = 65535;//设定计数器自动重装值 
  TIM2->CR1 &=~(3<<8);// 选择时钟分频：不分频
  TIM2->CR1 &=~(3<<5);// 选择计数模式:边沿对齐模式
  
  TIM2->CCMR1 |= 1<<0; //CC1S='01' IC1FP1映射到TI1
  TIM2->CCMR1 |= 1<<8; //CC2S='01' IC2FP2映射到TI2
  TIM2->CCER &= ~(1<<1);	 //CC1P='0'	 IC1FP1不反相，IC1FP1=TI1
  TIM2->CCER &= ~(1<<5);	 //CC2P='0'	 IC2FP2不反相，IC2FP2=TI2
  TIM2->CCMR1 |= 3<<4; //	IC1F='1000' 输入捕获1滤波器
  TIM2->SMCR |= 3<<0;	 //SMS='011' 所有的输入均在上升沿和下降沿有效
  TIM2->CR1 |= 0x01;    //CEN=1，使能定时器
}
static void _TIM3_Configuration(void)
{
  TIM3->PSC = 0x0;//预分频器
  TIM3->ARR = 65535;//设定计数器自动重装值 
  TIM3->CR1 &=~(3<<8);// 选择时钟分频：不分频
  TIM3->CR1 &=~(3<<5);// 选择计数模式:边沿对齐模式
  
  TIM3->CCMR1 |= 1<<0; //CC1S='01' IC1FP1映射到TI1
  TIM3->CCMR1 |= 1<<8; //CC2S='01' IC2FP2映射到TI2
  TIM3->CCER &= ~(1<<1);	 //CC1P='0'	 IC1FP1不反相，IC1FP1=TI1
  TIM3->CCER &= ~(1<<5);	 //CC2P='0'	 IC2FP2不反相，IC2FP2=TI2
  TIM3->CCMR1 |= 3<<4; //	IC1F='1000' 输入捕获1滤波器
  TIM3->SMCR |= 3<<0;	 //SMS='011' 所有的输入均在上升沿和下降沿有效
  TIM3->CR1 |= 0x01;    //CEN=1，使能定时器
}
static void _TIM5_Configuration(void)
{
  TIM5->PSC = 0x0;//预分频器
  TIM5->ARR = 65535;//设定计数器自动重装值 
  TIM5->CR1 &=~(3<<8);// 选择时钟分频：不分频
  TIM5->CR1 &=~(3<<5);// 选择计数模式:边沿对齐模式
  
  TIM5->CCMR1 |= 1<<0; //CC1S='01' IC1FP1映射到TI1
  TIM5->CCMR1 |= 1<<8; //CC2S='01' IC2FP2映射到TI2
  TIM5->CCER &= ~(1<<1);	 //CC1P='0'	 IC1FP1不反相，IC1FP1=TI1
  TIM5->CCER &= ~(1<<5);	 //CC2P='0'	 IC2FP2不反相，IC2FP2=TI2
  TIM5->CCMR1 |= 3<<4; //	IC1F='1000' 输入捕获1滤波器
  TIM5->SMCR |= 3<<0;	 //SMS='011' 所有的输入均在上升沿和下降沿有效
  TIM5->CR1 |= 0x01;    //CEN=1，使能定时器
}
 
static void _TIM8_Configuration(void)
{
  TIM8->PSC = 0x0;//预分频器
  TIM8->ARR = 65535;//设定计数器自动重装值 
  TIM8->CR1 &=~(3<<8);// 选择时钟分频：不分频
  TIM8->CR1 &=~(3<<5);// 选择计数模式:边沿对齐模式
  
  TIM8->CCMR1 |= 1<<0; //CC1S='01' IC1FP1映射到TI1
  TIM8->CCMR1 |= 1<<8; //CC2S='01' IC2FP2映射到TI2
  TIM8->CCER &= ~(1<<1);	 //CC1P='0'	 IC1FP1不反相，IC1FP1=TI1
  TIM8->CCER &= ~(1<<5);	 //CC2P='0'	 IC2FP2不反相，IC2FP2=TI2
  TIM8->CCMR1 |= 3<<4; //	IC1F='1000' 输入捕获1滤波器
  TIM8->SMCR |= 3<<0;	 //SMS='011' 所有的输入均在上升沿和下降沿有效
  TIM8->CR1 |= 0x01;    //CEN=1，使能定时器
}
 
//读取
void TIM7_IRQHandler(void)   //100ms
{ 
  if( TIM7->SR )
  {
      _speed_encode[0] = (short)TIM8 -> CNT;
      TIM8 -> CNT=0;
      _speed_encode[1] = (short)TIM2 -> CNT;
      TIM2 -> CNT=0;
      _speed_encode[2] = (short)TIM3 -> CNT;
      TIM2 -> CNT=0;
      _speed_encode[3] = (short)TIM5 -> CNT; 
      TIM5 -> CNT=0; 
    TIM7->SR&=~(1<<0);//清除中断标志位 
  }   
}
```

