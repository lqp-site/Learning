```C
u8 FDCAN1_Mode_Init(void)
{
    FDCAN_FilterTypeDef FDCAN1_RXFilter;

    HAL_FDCAN_DeInit(&FDCAN1_Handler);                              //先清除以前的设置
    FDCAN1_Handler.Instance=FDCAN1;
    FDCAN1_Handler.Init.FrameFormat=FDCAN_FRAME_CLASSIC;            //传统模式
    FDCAN1_Handler.Init.Mode=FDCAN_MODE_NORMAL;                     //正常模式
    FDCAN1_Handler.Init.AutoRetransmission=DISABLE;                 //关闭自动重传
    FDCAN1_Handler.Init.TransmitPause=DISABLE;                      //关闭传输暂停           
    FDCAN1_Handler.Init.ProtocolException=DISABLE;                  //关闭协议异常处理
       
        //时钟为200M，baudrate=200M/(NominalTimeSeg1+NominalTimeSeg2+1)/NominalPrescaler，这里配置为1M
    FDCAN1_Handler.Init.NominalPrescaler=10;                        //分频系数
    FDCAN1_Handler.Init.NominalSyncJumpWidth=8;                     //重新同步跳跃宽度
    FDCAN1_Handler.Init.NominalTimeSeg1=11;                         //tsg1范围:2~256
    FDCAN1_Handler.Init.NominalTimeSeg2=8;                          //tsg2范围:2~128
       
    FDCAN1_Handler.Init.MessageRAMOffset=0;                         //信息RAM偏移，10KB消息RAM共有2560字，故可以偏移0~2560
        //使用了多少个滤波器就要设置为多少
    FDCAN1_Handler.Init.StdFiltersNbr=3;                            //标准帧滤波器个数，0~128
    FDCAN1_Handler.Init.ExtFiltersNbr=2;                            //扩展帧滤波器个数，0~64
       
        //接收FIFO0、FIFO1和buffer配置，此处没有使用FIFO1故个数设置为0
    FDCAN1_Handler.Init.RxFifo0ElmtsNbr=64;                         //设置接收FIFO0元素个数，0-64
    FDCAN1_Handler.Init.RxFifo0ElmtSize=FDCAN_DATA_BYTES_8;         //接收FIFO0元素的数据域大小:8字节       
    FDCAN1_Handler.Init.RxFifo1ElmtsNbr=0;                          //设置接收FIFO1元素个数，0-64
    FDCAN1_Handler.Init.RxFifo1ElmtSize=FDCAN_DATA_BYTES_8;         //接收FIFO1元素的数据域大小:8字节               
    FDCAN1_Handler.Init.RxBuffersNbr=64;                            //接收buffer元素个数，0~64
        FDCAN1_Handler.Init.RxBufferSize=FDCAN_DATA_BYTES_8;            //接收buffer元素的数据域大小:8字节       
       
        //没有使用发送事件FIFO功能，故TxEventsNbr设置为0。把发送buffer全部作为专用发送buffer使用，故TxFifoQueueElmtsNbr设为0.
    FDCAN1_Handler.Init.TxEventsNbr=0;                              //发送事件FIFO元素个数，0~32
    FDCAN1_Handler.Init.TxBuffersNbr=32;                            //发送buffer元素个数，0~32
    FDCAN1_Handler.Init.TxFifoQueueElmtsNbr=0;                      //发送Buffer被用作发送FIFO/队列的元素个数，0~32
    FDCAN1_Handler.Init.TxFifoQueueMode=FDCAN_TX_FIFO_OPERATION;    //发送FIFO模式选择，可以选择FIFO模式或队列模式
    FDCAN1_Handler.Init.TxElmtSize=FDCAN_DATA_BYTES_8;              //发送元素的数据域大小:8字节
       
    if(HAL_FDCAN_Init(&FDCAN1_Handler)!=HAL_OK) return 1;          //初始化FDCAN

    //配置RX滤波器，标准帧   
    FDCAN1_RXFilter.IdType=FDCAN_STANDARD_ID;                       //标准ID
    FDCAN1_RXFilter.FilterIndex=0;                                  //滤波器索引                  
    FDCAN1_RXFilter.FilterType=FDCAN_FILTER_MASK;                   //滤波器类型
    FDCAN1_RXFilter.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;           //过滤器0关联到FIFO0  
    FDCAN1_RXFilter.FilterID1=0x112;                                //11位ID
    FDCAN1_RXFilter.FilterID2=0x7FF;                                //11位掩码
    if(HAL_FDCAN_ConfigFilter(&FDCAN1_Handler,&FDCAN1_RXFilter)!=HAL_OK) return 2;//滤波器初始化

    FDCAN1_RXFilter.IdType=FDCAN_STANDARD_ID;                       //标准ID
    FDCAN1_RXFilter.FilterIndex=1;                                  //滤波器索引                  
    FDCAN1_RXFilter.FilterType=FDCAN_FILTER_MASK;                   //滤波器类型
    FDCAN1_RXFilter.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;           //过滤器0关联到FIFO0  
    FDCAN1_RXFilter.FilterID1=0x113;                                //11位ID
    FDCAN1_RXFilter.FilterID2=0x7FF;                                //11位掩码
    if(HAL_FDCAN_ConfigFilter(&FDCAN1_Handler,&FDCAN1_RXFilter)!=HAL_OK) return 2;//滤波器初始化
       
    FDCAN1_RXFilter.IdType=FDCAN_STANDARD_ID;                       //标准ID
    FDCAN1_RXFilter.FilterIndex=2;                                  //滤波器索引                  
    FDCAN1_RXFilter.FilterType=FDCAN_FILTER_MASK;                   //滤波器类型
    FDCAN1_RXFilter.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;           //过滤器0关联到FIFO0  
    FDCAN1_RXFilter.FilterID1=0x114;                                //11位ID
    FDCAN1_RXFilter.FilterID2=0x7FF;                                //11位掩码
    if(HAL_FDCAN_ConfigFilter(&FDCAN1_Handler,&FDCAN1_RXFilter)!=HAL_OK) return 2;//滤波器初始化
       
        //配置RX滤波器，扩展帧。标准帧和扩展帧的滤波器索引是分开的   
    FDCAN1_RXFilter.IdType=FDCAN_EXTENDED_ID;                       //扩展ID
    FDCAN1_RXFilter.FilterIndex=0;                                  //滤波器索引                  
    FDCAN1_RXFilter.FilterType=FDCAN_FILTER_MASK;                   //滤波器类型
    FDCAN1_RXFilter.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;           //过滤器0关联到FIFO0  
    FDCAN1_RXFilter.FilterID1=(1 << 20)|(2 << 12);                  //32位ID
    FDCAN1_RXFilter.FilterID2=0x1FFFF000;                           //32位掩码
    if(HAL_FDCAN_ConfigFilter(&FDCAN1_Handler,&FDCAN1_RXFilter)!=HAL_OK) return 2;//滤波器初始化

    FDCAN1_RXFilter.IdType=FDCAN_EXTENDED_ID;                       //扩展ID
    FDCAN1_RXFilter.FilterIndex=1;                                  //滤波器索引                  
    FDCAN1_RXFilter.FilterType=FDCAN_FILTER_MASK;                   //滤波器类型
    FDCAN1_RXFilter.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;           //过滤器0关联到FIFO0  
    FDCAN1_RXFilter.FilterID1=(1 << 20)|(3 << 12);                  //32位ID
    FDCAN1_RXFilter.FilterID2=0x1FFFF000;                           //32位掩码
    if(HAL_FDCAN_ConfigFilter(&FDCAN1_Handler,&FDCAN1_RXFilter)!=HAL_OK) return 2;    //滤波器初始化
       
        //滤除的消息直接丢弃
        HAL_FDCAN_ConfigGlobalFilter(&FDCAN1_Handler,FDCAN_REJECT, FDCAN_REJECT, DISABLE, DISABLE);  //设置被滤除掉的消息的处理方式
       
        HAL_FDCAN_ActivateNotification(&FDCAN1_Handler,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);   //使能新消息接收中断
        HAL_FDCAN_ActivateNotification(&FDCAN1_Handler,FDCAN_IT_TX_COMPLETE,0xffffffff);   //使能消息发送中断，0xffffffff表示所有的发送buffer都触发中断       
       
    HAL_FDCAN_Start(&FDCAN1_Handler);                               //开启FDCAN
    return 0;
}

```

```c
/**************** CAN发送示例（来自正点原子）**************************/

//can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)       
//len:数据长度(最大为8),可设置为FDCAN_DLC_BYTES_2~FDCAN_DLC_BYTES_8                                     
//msg:数据指针,最大为8个字节.
//返回值:0,成功;
//                 其他,失败;
u8 FDCAN1_Send_Msg(u8* msg,u32 len)
{       
    FDCAN1_TxHeader.Identifier=0x12;                           //32位ID
    FDCAN1_TxHeader.IdType=FDCAN_STANDARD_ID;                  //标准ID
    FDCAN1_TxHeader.TxFrameType=FDCAN_DATA_FRAME;              //数据帧
    FDCAN1_TxHeader.DataLength=len;                            //数据长度
    FDCAN1_TxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;            
    FDCAN1_TxHeader.BitRateSwitch=FDCAN_BRS_OFF;               //关闭速率切换
    FDCAN1_TxHeader.FDFormat=FDCAN_CLASSIC_CAN;                //传统的CAN模式
    FDCAN1_TxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;     //无发送事件
    FDCAN1_TxHeader.MessageMarker=0;                           

    if(HAL_FDCAN_AddMessageToTxFifoQ(&FDCAN1_Handler,&FDCAN1_TxHeader,msg)!=HAL_OK) return 1;//发送
    return 0;       
}
```

```c
/*** CAN接收示例（来自正点原子，因为滤波器被关联到FIFO0，因此消息只会放入到FIFO0）***/

//can口接收数据查询
//buf:数据缓存区;         
//返回值:0,无数据被收到;
//                 其他,接收的数据长度;
u8 FDCAN1_Receive_Msg(u8 *buf)
{       
    if(HAL_FDCAN_GetRxMessage(&FDCAN1_Handler,FDCAN_RX_FIFO0,&FDCAN1_RxHeader,buf)!=HAL_OK)return 0;//接收数据
        return FDCAN1_RxHeader.DataLength>>16;       
}
```

