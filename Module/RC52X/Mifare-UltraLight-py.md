# Mifare Ultra Light 非接触式IC卡

```python
########## 初始化配置 ###############
################## addr
RC52X_I2C_ADDR_0 = 0x52 >> 1
RC52X_I2C_ADDR_1 = 0x54 >> 1
RC52X_I2C_ADDR_2 = 0x58 >> 1

################## read len
MAXRLEN = 18


##################  M1,ISO14443A
REG_SET_1          = 1    #A卡寄存配置1
REG_SET_2          = 2    #A卡寄存配置2
REG_SET_3          = 3    #A卡寄存配置3
REG_SET_4          = 4    #A卡寄存配置4
TYPEA_REG_CONFIG   = REG_SET_1     #选择寄存器配置4

PICC_REQIDL         = 0x26               #寻天线区内未进入休眠状态
PICC_REQALL         = 0x52               #寻天线区内全部卡
PICC_ANTICOLL1      = 0x93               #防冲撞
PICC_ANTICOLL2      = 0x95               #防冲撞
PICC_ANTICOLL3      = 0x97   
PICC_AUTHENT1A      = 0x60               #验证A密钥
PICC_AUTHENT1B      = 0x61               #验证B密钥
PICC_READ           = 0x30               #读块
PICC_WRITE          = 0xA0               #写块
PICC_DECREMENT      = 0xC0               #扣款
PICC_INCREMENT      = 0xC1               #充值
PICC_RESTORE        = 0xC2               #调块数据到缓冲区
PICC_TRANSFER       = 0xB0               #保存缓冲区中数据
PICC_HALT           = 0x50               #休眠

##################  ISO14443B
PICC_ANTI           = 0x05 
PICC_ATTRIB         = 0x1D 

##################  命令字
RC52X_IDLE            = 0x00               #取消当前命令
RC52X_MEM             = 0x01               #CONFIG命令
RC52X_AUTHENT         = 0x0E               #验证密钥
RC52X_RECEIVE         = 0x08               #接收数据
RC52X_TRANSMIT        = 0x04               #发送数据
RC52X_TRANSCEIVE      = 0x0C               #发送并接收数据
RC52X_RESETPHASE      = 0x0F               #复位
RC52X_CALCCRC         = 0x03               #CRC计算
RC52X_AUTOCOLL        = 0x0D               #MIFARE anticollision, Card Operation mode only

#寄存器定义
# PAGE 0
RFU00               = 0x00  
CommandReg          = 0x01
ComIEnReg           = 0x02    
DivlEnReg           = 0x03 
ComIrqReg           = 0x04 
DivIrqReg           = 0x05
ErrorReg            = 0x06    
Status1Reg          = 0x07    
Status2Reg          = 0x08 
FIFODataReg         = 0x09
FIFOLevelReg        = 0x0A
WaterLevelReg       = 0x0B
ControlReg          = 0x0C
BitFramingReg       = 0x0D 
CollReg             = 0x0E
RFU0F               = 0x0F
# PAGE 1
RFU10               =  0x10
ModeReg             =  0x11
TxModeReg           =  0x12
RxModeReg           =  0x13
TxControlReg        =  0x14
TxAskReg            =  0x15        # rc523
TxAutoReg           =  0x15        # pn512
TxSelReg            =  0x16
RxSelReg            =  0x17
RxThresholdReg      =  0x18
DemodReg            =  0x19
RFU1A               =  0x1A
RFU1B               =  0x1B
MifareReg           =  0x1C
ManualRCVReg        =  0x1D
TypeBReg            =  0x1E
SerialSpeedReg      =  0x1F
# PAGE 2
RFU20               =  0x20  
CRCResultRegM       =  0x21
CRCResultRegL       =  0x22
GsNOffReg           =  0x23
ModWidthReg         =  0x24
TxBitPhaseReg       =  0x25
RFCfgReg            =  0x26
GsNOnReg            =  0x27
CWGsPReg            =  0x28
ModGsPReg           =  0x29
TModeReg            =  0x2A
TPrescalerReg       =  0x2B
TReloadRegH         =  0x2C
TReloadRegL         =  0x2D
TCounterValueRegH   =  0x2E
TCounterValueRegL   =  0x2F
# PAGE 3
RFU30               =  0x30
TestSel1Reg         =  0x31
TestSel2Reg         =  0x32
TestPinEnReg        =  0x33
TestPinValueReg     =  0x34
TestBusReg          =  0x35
AutoTestReg         =  0x36
VersionReg          =  0x37
AnalogTestReg       =  0x38
TestDAC1Reg         =  0x39  
TestDAC2Reg         =  0x3A   
TestADCReg          =  0x3B   
RFU3C               =  0x3C   
RFU3D               =  0x3D   
RFU3E               =  0x3E   
RFU3F               =  0x3F

MI_OK                        = 0
# error code
MI_NOTAGERR                  = -1
#MI_CHK_FAILED                = -1
MI_CRCERR                    = -2
#MI_CHK_COMPERR               = -2
#MI_EMPTY                     = -3
MI_AUTHERR                   = -4
#MI_PARITYERR                 = -5
MI_CODEERR                   = -6
#MI_SERNRERR                  = -8
#MI_KEYERR                    = -9
MI_NOTAUTHERR                = -10
MI_BITCOUNTERR               = -11
#MI_BYTECOUNTERR              = -12
#MI_IDLE                      = -13
#MI_TRANSERR                  = -14
MI_WRITEERR                  = -15
#MI_INCRERR                   = -16
#MI_DECRERR                   = -17
#MI_READERR                   = -18
#MI_OVFLERR                   = -19
#MI_POLLING                   = -20
#MI_FRAMINGERR                = -21
#MI_ACCESSERR                 = -22
#MI_UNKNOWN_COMMAND           = -23
MI_COLLERR                   = -24
#MI_RESETERR                  = -25
#MI_INITERR                   = -25
#MI_INTERFACEERR              = -26
MI_ACCESSTIMEOUT             = -27
#MI_NOBITWISEANTICOLL         = -28
#MI_QUIT                      = -30
#MI_RECBUF_OVERFLOW           = -50
#MI_SENDBYTENR                = -51
#MI_SENDBUF_OVERFLOW          = -53
#MI_BAUDRATE_NOT_SUPPORTED    = -54
#MI_SAME_BAUDRATE_REQUIRED    = -55
#MI_WRONG_PARAMETER_VALUE     = -60
#MI_BREAK                     = -99
#MI_NY_IMPLEMENTED            = -100
#MI_NO_MFRC                   = -101
#MI_MFRC_NOTAUTH              = -102
#MI_WRONG_DES_MODE            = -103
#MI_HOST_AUTH_FAILED          = -104
#MI_WRONG_LOAD_MODE           = -106
#MI_WRONG_DESKEY              = -107
#MI_MKLOAD_FAILED             = -108
#MI_FIFOERR                   = -109
#MI_WRONG_ADDR                = -110
#MI_DESKEYLOAD_FAILED         = -111
#MI_WRONG_SEL_CNT             = -114
#MI_WRONG_TEST_MODE           = -117
#MI_TEST_FAILED               = -118
#MI_TOC_ERROR                 = -119
#MI_COMM_ABORT                = -120
#MI_INVALID_BASE              = -121
#MI_MFRC_RESET                = -122
#MI_WRONG_VALUE               = -123
MI_VALERR                    = -124
MI_COM_ERR                   = -125
MI_ERR                       = -126

RC52X_MAX_BUSY_CYCLES= 8
```

```python
class NFC_RC52X:
    def __init__(self, config):
        self.printer = config.get_printer()
        ppins = self.printer.lookup_object('pins')
        self.name = config.get_name()
        self.reactor = self.printer.get_reactor()
        self.last_print_time = 0.
        self.scale = 1
        self.pi = {'MfCommand' : 0xff, 'MfLength': 0xff, 'MfData':[0xff,0xff]}
        
        self.i2c_0 = bus.MCU_I2C_from_config(config, default_addr=RC52X_I2C_ADDR_0, default_speed=100000)
        self.i2c_1 = bus.MCU_I2C_from_config(config, default_addr=RC52X_I2C_ADDR_1, default_speed=100000)
        self.i2c_2 = bus.MCU_I2C_from_config(config, default_addr=RC52X_I2C_ADDR_2, default_speed=100000)    
        self.cur_i2c = self.i2c_0 # Default selection i2c_0
        self.cur_i2c_index = 0
        self.ID_DATA = [[],[],[],[]]
        self.ID_STATUS = [0,0,0,0,0]

        self.printer.register_event_handler("klippy:ready",self.handle_connect)
        self.nrstpd_pin = config.get('nrstpd_name')
        self.gcode = self.printer.lookup_object('gcode')
        '''
        # reset pin
        self.nrstpd_pin = ppins.setup_pin('digital_out', config.get('nrstpd_pin'))

        # Determine start and shutdown values
        static_value = config.getfloat('static_value', None,
                                       minval=0., maxval=self.scale)
        if static_value is not None:
            config.deprecate('static_value')
            self.last_value = self.shutdown_value = static_value / self.scale
        else:
            self.last_value = config.getfloat(
                'value', 0., minval=0., maxval=self.scale) / self.scale
            self.shutdown_value = config.getfloat(
                'shutdown_value', 0., minval=0., maxval=self.scale) / self.scale
        self.nrstpd_pin.setup_start_value(self.last_value, self.shutdown_value) 
        '''

    def handle_connect(self):
        self._set_nrstpd_pin(0)
        self.reactor.pause(self.reactor.monotonic() + .10)
        self._set_nrstpd_pin(1)
        self.reactor.pause(self.reactor.monotonic() + .01)

        self._set_current_i2c(0)
        self.i2c_write_rc52x(CommandReg,RC52X_RESETPHASE)
        self.rc52x_fieldOff()
        self.reactor.pause(self.reactor.monotonic() + .01)
        self.rc52x_fieldOn()

        self._set_current_i2c(1)
        self.i2c_write_rc52x(CommandReg,RC52X_RESETPHASE)
        self.rc52x_fieldOff()
        self.reactor.pause(self.reactor.monotonic() + .01)
        self.rc52x_fieldOn()

        self._set_current_i2c(2)
        self.i2c_write_rc52x(CommandReg,RC52X_RESETPHASE)
        self.rc52x_fieldOff()
        self.reactor.pause(self.reactor.monotonic() + .01)
        self.rc52x_fieldOn()

        self.i2c_read_rc52x(VersionReg, 1)
        self.reactor.pause(self.reactor.monotonic() + .01)
        #self.resend_timer = self.reactor.register_timer(self._reset_rc52x, self.reactor.NOW)
    
    def _reset_rc52x(self):
        self._set_nrstpd_pin(0)
        self.reactor.pause(self.reactor.monotonic() + .01)
        self._set_nrstpd_pin(1)
        self.reactor.pause(self.reactor.monotonic() + .02)

        self.i2c_write_rc52x(CommandReg,RC52X_RESETPHASE)
        self.i2c_read_rc52x(VersionReg, 1)
        logging.info("rc52x: reset device ")

    def _set_nrstpd_pin(self, value):
        self.gcode.run_script_from_command("SET_PIN PIN=%s VALUE=%d" % ('nrstpd_pin', value))

    def _set_current_i2c(self, i2c_index):
        if i2c_index == 0:
            self.cur_i2c = self.i2c_0
            self.cur_i2c_index = 0
        elif i2c_index == 1:
            self.cur_i2c = self.i2c_1
            self.cur_i2c_index = 1
        elif i2c_index == 2:
            self.cur_i2c = self.i2c_2
            self.cur_i2c_index = 2
        
    def rc52x_setBitMask(self, reg, mask):
        temp = self.i2c_read_rc52x(reg, 1)
        if temp[0] == []:
            temp = [0]
        self.i2c_write_rc52x(reg, temp[0] | mask)

    def rc52x_cleanBitMask(self, reg, mask):
        temp = self.i2c_read_rc52x(reg, 1)
        if temp == []:
            temp = [0]
        self.i2c_write_rc52x(reg, temp[0] & (~mask))

    def rc52x_fieldOn(self):
        temp = self.i2c_read_rc52x(TxControlReg, 1) 
        if temp == []:
            self.i2c_write_rc52x(TxControlReg, 0x00 | 0x03)
        elif (not (temp[0] & 0x03)):
            self.i2c_write_rc52x(TxControlReg, temp[0] | 0x03)

    def rc52x_fieldOff(self):
        self.rc52x_cleanBitMask(TxControlReg, 0x03) 

    def i2c_write_rc52x(self, reg, data):
        try:
            # Write command for updating temperature+status bit
            writeBuf = []
            writeBuf.append(reg)
            if isinstance(data, int):
                writeBuf.append(data)
            else:
                writeBuf.extend(data)
            self.cur_i2c.i2c_write(writeBuf)
            # Wait 110ms after first read, 75ms minimum
            # self.reactor.pause(self.reactor.monotonic() + .10)
        except Exception as e:
            logging.exception("rc52x: exception encountered" +
                              " reading data: %s"%str(e))

    def i2c_read_rc52x(self, reg, r_len = 1):
        data = []        
        cycles = 0
        if r_len < 1:
            return data
        try:
            while True:
                # Check if we're constantly busy. If so, send soft-reset.
                # and issue warning.
                if cycles > RC52X_MAX_BUSY_CYCLES:
                    logging.warning("rc52x: device reported busy after " +
                        "%d cycles, resetting device"% RC52X_MAX_BUSY_CYCLES)
                    self._reset_rc52x()
                    data = []
                    break

                cycles += 1

                # Read data
                read = self.cur_i2c.i2c_read([reg], r_len)
                if read is None:
                    logging.warning("rc52x: received data from" +
                                    " i2c_read is None")
                    continue
                data = bytearray(read['response'])

                if len(data) < r_len:
                    logging.warning("rc52x: received bytes less than" +
                                    " expected 6 [%d]"%len(data))
                    continue
                return data
        except Exception as e:
            logging.exception("rc52x: exception encountered" +
                              " reading data: %s"%str(e))
            return data

    #命令发送函数，各种卡通用。pi：命令和数据指针；  
    def rc52x_pcdComTransceive(self, pi):
        status  = MI_ERR
        n = None
        irqEn = waitFor = 0
        if pi['MfCommand'] == RC52X_AUTHENT :
            irqEn   = 0x12
            waitFor = 0x10
        elif pi['MfCommand'] == RC52X_TRANSCEIVE :
            irqEn   = 0x77
            waitFor = 0x30

        # 发送命令帧前准备
        self.i2c_write_rc52x(ComIEnReg, irqEn | 0x80)   # 开中断
        self.i2c_write_rc52x(CommandReg, RC52X_IDLE)    # 取消当前命令的执行
        self.rc52x_setBitMask(FIFOLevelReg, 0x80)       # Flush FIFO  清除FIFO缓冲区及其标志位 
        self.rc52x_cleanBitMask(ComIrqReg, 0x80)        # 清除中断标志位SET1   
        self.i2c_write_rc52x(ComIrqReg, 0x7F) 

        # 发送命令帧
        self.i2c_write_rc52x(FIFODataReg, pi['MfData'])    
        # 执行命令
        self.i2c_write_rc52x(CommandReg, pi['MfCommand'])

        if pi['MfCommand'] == RC52X_TRANSCEIVE:
            self.rc52x_setBitMask(BitFramingReg, 0x80)   #Start Send 

        for i in range(5000): #根据时钟频率调整,操作M1卡最大等待时间25ms
            if (i % 5 == 0):
                n = self.i2c_read_rc52x(ComIrqReg, 1)   #查询事件中断
                if (n[0] & irqEn & 0x01) or (n[0] & waitFor): # timerirq idleirq rxirq #等待命令完成
                    #logging.info("timerirq idleirq rxirq") 
                    break; 
        self.rc52x_cleanBitMask(BitFramingReg, 0x80)   # 停止发送

        if n[0] & waitFor :  #IdleIRq
            temp = self.i2c_read_rc52x(ErrorReg, 1) #读错误标志寄存器
            if temp[0] & 0x1b : 
                status = MI_ERR #-126
            else:
                status = MI_OK
                if pi['MfCommand'] == RC52X_TRANSCEIVE:
                    n = self.i2c_read_rc52x(FIFOLevelReg, 1) #读FIFO中保存的字节数
                    lastBits = self.i2c_read_rc52x(ControlReg, 1) #最后接收到得字节的有效位数
                    lastBits[0] &= 0x07
                    #logging.info("read: n=%d, lastBits=%d" % (n[0],lastBits[0]))
                    if lastBits[0]:
                        pi['MfLength'] = (n[0]-1)*8 + lastBits[0]
                    else:
                        pi['MfLength'] = n[0]*8 #最后接收到的字节整个字节有效
                    if n[0] == 0:
                        n[0] = 1
                    if n[0] > MAXRLEN: 
                        n[0] = MAXRLEN

                    pi['MfData'] = self.i2c_read_rc52x(FIFODataReg, n[0])

        elif (n[0] & irqEn & 0x01):    #TimerIRq 发生定时器中断
            status = MI_NOTAGERR  #-1
        else:
            temp = self.i2c_read_rc52x(ErrorReg, 1)
            if (not (temp[0] & 0x1B)):
                #temp = self.i2c_read_rc52x(ErrorReg, 1)
                status = MI_ACCESSTIMEOUT #-27

        self.rc52x_setBitMask(ControlReg, 0x80)     #stop timer now 停止定时器运行
        self.i2c_write_rc52x(CommandReg, RC52X_IDLE)  

        return  status


class RC52X_TYPE(NFC_RC52X):
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        NFC_RC52X.__init__(self, config) # 初始化父类
        self.printer.add_object(self.name, self)

        self.report_time = config.getint('rc52x_report_time', 10, minval=1)
        self.sample_timer = self.reactor.register_timer(self.rc52x_getCardInfo)
        self.printer.register_event_handler("klippy:ready", self.startTimerGetCard)

    def startTimerGetCard(self):
        s_time = self.reactor.monotonic() + self.report_time
        self.reactor.update_timer(self.sample_timer, s_time)

    def testGetCard(self):
        logging.info("rc52x: start get device id")
        id_data = []
        self.reactor.pause(self.reactor.monotonic() + .5)
        for j in range(1):
            self._set_current_i2c(2) #选择dev地址
            logging.info("****** rc52x: device = %d ******" % (j))
            for i in range(3):
                id_len = self.rc52x_typeA(id_data)
                if len(id_data) > 0 :
                    self.ID_DATA[self.cur_i2c_index] = []
                    self.ID_DATA[self.cur_i2c_index] = id_data
                    out_data = "".join(f"{item:02x}" for item in id_data)
                    logging.info("rc52x: id=%s, device=%d" %(out_data, self.cur_i2c_index))
                    break
                else:
                    logging.warning("rc52x: nfc device get id failed!!!")
                    self._reset_rc52x()
                    self.reactor.pause(self.reactor.monotonic() + .50)
                
    def rc52x_getCardInfo(self, eventtime):
        s_time = 0
        for i in range(3):
            atqa = []
            self._set_current_i2c(i) #选择dev地址
            if self.ID_STATUS[i]: # 寻卡，卡是否一直存在
                self.rc52x_fieldOff()
                t_time = self.reactor.monotonic()
                self.reactor.pause(t_time + .05)
                self.rc52x_fieldOn()
                self.reactor.pause(t_time + .1)
                status = self.rc52x_pcdRequestA(PICC_REQALL,atqa)
                if(status == MI_OK): # 卡依然存在
                    s_time = self.report_time
                else: # 卡已经移除
                    self.ID_STATUS[i] = 0
            else: #获取 id
                UID = R_UID = []
                status,ulen,sak = self.rc52x_pcdActivateA(atqa, R_UID)    #寻卡，防冲撞，选卡
                if (status == MI_OK): #寻卡，防冲撞，选卡 ok
                    # out_data = "".join(f"{item:02x}" for item in R_UID)
                    # logging.info("rc52x: R_UID=%s" %(out_data))
                    if (ulen == 8):
                        r_len = ulen - 1
                        UID = R_UID[1:8]                      
                    else:
                        r_len = ulen
                        UID = R_UID[:4]

                    self.ID_STATUS[i] = 1
                    self.ID_DATA[self.cur_i2c_index] = UID
                    out_data = "".join(f"{item:02x}" for item in UID)
                    logging.info("rc52x: UID=%s, device=%d" %(out_data, self.cur_i2c_index))

                    # block = 5
                    # WD_Data = [0x01, 0x02, 0x03, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
                    # status = self.rc52x_pcdWrite(block, WD_Data)
                    # logging.info("WRITE PAGE block=%d: status=%d" % (block, status))

                    # RD_Data = [] 
                    # status = self.rc52x_pcdRead(block, RD_Data) 
                    # out_data = " ".join(f"{item:02x}" for item in RD_Data)
                    # logging.info("READ PAGE block=%d: status=%d, data=%s" % (block, status,out_data))

                else:
                    logging.info("get or select ID_CARD failed:  device = %d" % (i))
                    self._reset_rc52x()
            self.reactor.pause(self.reactor.monotonic() + .05)
        
        now_time = self.reactor.monotonic()
        next_time = now_time + self.report_time + s_time
        return next_time
            
    def rc52x_writeCardData(self, gate_sel, attribute, value):
        status = -1
        if gate_sel < 0:
            return status
        self._set_current_i2c(gate_sel) #选择dev地址
        
        block = attribute
        if block < 4:
            logging.warning("the block=%d is only read!!!" % (block))
            return status

        WD_Data = [0,0,0,0]
        idle_Data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        for i in range(4):
            temp = value >> i*8
            if temp != 0:
                WD_Data[i] = temp
        WD_Data.extend(idle_Data)
        status = self.rc52x_pcdWrite(block, WD_Data)
        logging.info("WRITE PAGE block=%d: status=%d, value=%ld" % (block, status, value))
        return status


    def rc52x_readCardValue(self, gate_sel, attribute):
        status = -1
        value = 0
        if gate_sel < 0:
            return status, value
        self._set_current_i2c(gate_sel) #选择dev地址
        
        block = attribute
        if block < 4:
            logging.warning("the block=%d is only read!!!" % (block))
            return status, value

        RD_Data = []
        status = self.rc52x_pcdRead(block, RD_Data) 
        out_data = " ".join(f"{item:02x}" for item in RD_Data)
        logging.info("READ PAGE block=%d: status=%d, data=%s" % (block, status,out_data))
        if (status != MI_OK): 
            return status, value
        for i in range(4):
            temp = RD_Data[i]
            value |= (temp << i*8)
        logging.info("READ PAGE block=%d: value=%ld" % (block,value))
        return status, value


                         
        
    #读取A卡返回UID。UID_a，len：需要返回数据的指针；
    def rc52x_JewelTransceive(self, pi):
        n = status = count = 0
        irqEn   = 0x77
        waitFor = 0x30    
        self.i2c_write_rc52x(CommandReg, RC52X_IDLE)
        self.rc52x_setBitMask(FIFOLevelReg, 0x80)
        self.i2c_write_rc52x(ComIrqReg, 0x7F)  
        self.i2c_write_rc52x(DivIrqReg, 0x7F)
        self.i2c_write_rc52x(ManualRCVReg, 0x10)

        d_len = pi['MfLength']
        self.i2c_write_rc52x(FIFODataReg, pi['MfData'][:d_len])

        self.i2c_write_rc52x(ComIEnReg, irqEn|0x80)
        self.i2c_write_rc52x(CommandReg, pi['MfCommand'])
        self.rc52x_setBitMask(BitFramingReg, 0x80)
        
        while True:
            temp = self.i2c_read_rc52x(ComIrqReg, 1)      #wait for TxIRQ
            if (temp[0] & 0x40) != 0 :
                break
            count += 1
            if count > 1000 :
                logging.info("rc52x_JewelTransceive: wait for TxIRQ timeout")
                return MI_ERR

        if pi['MfCommand'] == RC52X_TRANSMIT:
            temp = self.i2c_read_rc52x(ErrorReg, 1)
            status = MI_OK if temp[0] == 0 else -1
        
        if pi['MfCommand'] == RC52X_TRANSCEIVE:
            self.i2c_write_rc52x(ManualRCVReg, 0x00)
            
            for i in range(1000):
                n = self.i2c_read_rc52x(ComIrqReg, 1)
                if (n[0] & irqEn & 0x01) or (n[0] & waitFor):
                     break  # timerirq idleirq rxirq
        
            if n[0] & waitFor:       #IdleIRq
                temp = self.i2c_read_rc52x(ErrorReg, 1)
                if temp[0] & 0x1b:      #06h    
                    status = MI_ERR
                else:
                    status = MI_OK
                    if (pi['MfCommand'] == RC52X_TRANSCEIVE):
                        n = self.i2c_read_rc52x(FIFOLevelReg, 1)         #0ah
                        lastBits = self.i2c_read_rc52x(ControlReg, 1)    #0ch
                        lastBits[0] &= 0x07
                        pi['MfLength'] = ((n[0] - 1)*8 + lastBits[0]) if lastBits[0] else (n[0] * 8)
                        if n[0] == 0:
                            n[0] = 1
                        if n[0] > MAXRLEN:
                            n[0] = MAXRLEN 
                        pi['MfData'] = self.i2c_read_rc52x(FIFODataReg, n[0]);  #09h    
            elif (n[0] & irqEn & 0x01):    #TimerIRq
                status = MI_NOTAGERR
            else :
                temp = self.i2c_read_rc52x(ErrorReg, 1)
                temp[0] &= 0x1B
                if not temp[0]:
                    temp = self.i2c_read_rc52x(ErrorReg, 1)
                    status = MI_ACCESSTIMEOUT #-27
        return status

    def rc52x_pcdConfigISOType(self, type):
        if type == 'A':             #ISO14443_A
            self.i2c_write_rc52x(ControlReg, 0x10)
            self.i2c_write_rc52x(TxAutoReg, 0x40)
            self.i2c_write_rc52x(TxModeReg, 0x00) 
            self.i2c_write_rc52x(RxModeReg, 0x00)#   仅检测A卡的设置
            
            self.i2c_write_rc52x(ModWidthReg, 0x26) #24h  若AB卡检测都有，则必须设置
            
            self.i2c_write_rc52x(BitFramingReg, 0)    #0dh,
            self.i2c_write_rc52x(ModeReg, 0x3D)       #11h,    
            self.i2c_write_rc52x(TxControlReg, 0x84)  #14h,      
            self.i2c_write_rc52x(RxSelReg, 0x88)       #17h,        
            self.i2c_write_rc52x(DemodReg, 0x8D)       #19h,     #AddIQ     
            self.i2c_write_rc52x(TypeBReg, 0)          #1eh,
            self.i2c_write_rc52x(TModeReg, 0x8D) #2ah,
            self.i2c_write_rc52x(TPrescalerReg, 0x3e)    #2bh
            self.i2c_write_rc52x(TReloadRegL, 30)#2dh
            self.i2c_write_rc52x(TReloadRegH, 0) #2ch
            self.i2c_write_rc52x(AutoTestReg, 0)        #36h    
            
            if TYPEA_REG_CONFIG == REG_SET_1:
                self.i2c_write_rc52x(RxThresholdReg, 0x33) #20201117
                self.i2c_write_rc52x(RFCfgReg, 0x44)       #26h     
                self.i2c_write_rc52x(GsNOnReg, 0xF8)       #27h        
                self.i2c_write_rc52x(CWGsPReg, 0x3F)       #28h

            elif TYPEA_REG_CONFIG == REG_SET_2:
                self.i2c_write_rc52x(RxThresholdReg, 0x33) #20201117
                self.i2c_write_rc52x(RFCfgReg, 0x44)       #26h     
                self.i2c_write_rc52x(GsNOnReg, 0x88)       #27h        
                self.i2c_write_rc52x(CWGsPReg, 0x20)       #28h

            elif TYPEA_REG_CONFIG == REG_SET_3:
                self.i2c_write_rc52x(RxThresholdReg, 0x42) #20201117
                self.i2c_write_rc52x(RFCfgReg, 0x44)       #26h     
                self.i2c_write_rc52x(GsNOnReg, 0xF8)       #27h        
                self.i2c_write_rc52x(CWGsPReg, 0x3F)       #28h
    
            else:   
                self.i2c_write_rc52x(RxThresholdReg, 0x42); #20201117   42
                self.i2c_write_rc52x(RFCfgReg, 0x59)       #26h     44
                self.i2c_write_rc52x(GsNOnReg, 0x88)       #27h        
                self.i2c_write_rc52x(CWGsPReg, 0x20)       #28h

            # self.rc52x_fieldOff()
            # self.reactor.pause(self.reactor.monotonic() + .01)
            # self.rc52x_fieldOn() #防止某些卡复位较慢,提前打开射频场
            # self.reactor.pause(self.reactor.monotonic() + .05)      

        elif type =='K':           #CARD SIMULATION
            self.i2c_write_rc52x(TxModeReg,0x80)        #12h,
            self.i2c_write_rc52x(RxModeReg,0x80)        #13h,
            self.i2c_write_rc52x(RxThresholdReg,0x55)   #18h,
            self.i2c_write_rc52x(DemodReg,0x61)         #19h,     
            self.i2c_write_rc52x(MifareReg,0x62)        #1ch, 
            self.i2c_write_rc52x(GsNOffReg,0xf2)        #/23h       
            self.i2c_write_rc52x(TxBitPhaseReg,0x87)    #25h    
            self.i2c_write_rc52x(RFCfgReg,0x59)         #26h    
            self.i2c_write_rc52x(GsNOnReg,0x6F)         #27h
            self.i2c_write_rc52x(CWGsPReg,0x3f)         #28h      
            self.i2c_write_rc52x(ModGsPReg,0x3A)        #29h          
            self.i2c_write_rc52x(TxSelReg,0x17)         #16h 
            self.i2c_write_rc52x(FIFOLevelReg,0x80)

        elif (type == 'L'):
            self.i2c_write_rc52x(TxModeReg,0x92)        #12h 
            self.i2c_write_rc52x(RxModeReg,0x92)        #13h     
            self.i2c_write_rc52x(TxSelReg,0x17)         #16h  
            self.i2c_write_rc52x(RxThresholdReg,0x55)   #18h,          
            self.i2c_write_rc52x(DemodReg,0x61)         #19h         
            self.i2c_write_rc52x(RFU1A,0x00)            #1Bh 
            self.i2c_write_rc52x(ManualRCVReg,0x08)
            self.i2c_write_rc52x(RFU1B,0x80)            #1Bh         
            self.i2c_write_rc52x(GsNOffReg,0xf2)        #23h 
            self.i2c_write_rc52x(RFCfgReg,0x59)         #26h
            self.i2c_write_rc52x(GsNOnReg,0x6f)         #27h
            self.i2c_write_rc52x(CWGsPReg,0x3f)         #28h     
            self.i2c_write_rc52x(ModGsPReg,0x3a)        #29h
            self.i2c_write_rc52x(FIFOLevelReg,0x80)

        else:
            return MI_ERR
        return MI_OK

    def rc52x_simulate_Card(self):
        CardConfig = [0x04,0x00,0xa1,0xa2,0xa3,0x11,0x01,0xfe,0xb2,0xb3,0xb4,0xb5,
                        0xb6,0xb7,0xc0,0xc1,0xc2,0xc3,0xc4,0xc5,0xc6,0xc7,0x23,0x45,0xfa]
        
        self.rc52x_pcdConfigISOType('K')  

        self.i2c_write_rc52x(FIFODataReg, CardConfig)
        
        self.i2c_write_rc52x(CommandReg, RC52X_MEM)
        self.reactor.pause(self.reactor.monotonic() + .25)
        self.i2c_write_rc52x(CommandReg, RC52X_AUTOCOLL)

    def rc52x_simulate_CardF(self):
        CardConfig = [0x10,0xfe,0x01,0x12,0x01,0x24,0x69,0x0b,0x50,0x37,0x2E,0x3d,
                        0x24,0x69,0x0b,0x50,0x37,0x2E,0x3d,0x24,0x69,0x0b,0x50,0x37,0x0b] 
        self.rc52x_pcdConfigISOType('L')
        self.reactor.pause(self.reactor.monotonic() + .05)
        self.i2c_write_rc52x(FIFODataReg, CardConfig)                                                       
        
        self.reactor.pause(self.reactor.monotonic() + .1)
        self.i2c_write_rc52x(CommandReg, RC52X_MEM)
        self.i2c_write_rc52x(CommandReg, RC52X_AUTOCOLL)

    # 寻卡
    def rc52x_pcdRequestA(self, req_code, pTagType):
        self.rc52x_cleanBitMask(TxModeReg,0x80)      #12h,    //TxCRCEn
        self.rc52x_cleanBitMask(RxModeReg,0x80)      #13h,
        self.i2c_write_rc52x(TReloadRegL,0x59)      #2dh,
        self.i2c_write_rc52x(TReloadRegH,0x0A)      #2ch,
        self.rc52x_cleanBitMask(Status2Reg,0x08)     #08h
        self.i2c_write_rc52x(BitFramingReg,0x07)    #0dh
        self.rc52x_setBitMask(TxControlReg,0x03)     #14h     0x03

        self.pi['MfCommand']= RC52X_TRANSCEIVE
        self.pi['MfLength'] = 1
        self.pi['MfData']   = [req_code]

        status = self.rc52x_pcdComTransceive(self.pi)
        if (status == MI_OK):
            if(self.pi['MfLength'] == 0x10):
                temp = self.pi['MfData']
                pTagType.append(temp[0])
                pTagType.append(temp[1])
            else:
                status = MI_VALERR #-124
        return status

    # 选卡 level：防撞级别；pSnr：id；pSize：长度；
    def rc52x_pcdSelect(self, level, pSnr):
        self.i2c_write_rc52x(TxModeReg,0x80)          #12h        TxCRCEn
        self.i2c_write_rc52x(RxModeReg,0x80)          #13h

        self.pi['MfCommand']= RC52X_TRANSCEIVE
        self.pi['MfLength'] = 7
        self.pi['MfData']   = [level, 0x70]

        pSize = snr_check = 0
        for i in range(4):
            snr_check ^= pSnr[i]
            self.pi['MfData'].append(pSnr[i])
    
        self.pi['MfData'].append(snr_check)
        status = self.rc52x_pcdComTransceive(self.pi)
        if (status == MI_OK):
            if (self.pi['MfLength'] != 0x8):
                status = MI_BITCOUNTERR #-11
            else:
                pSize = self.pi['MfData'][0]

        return (status, pSize)

    # 密钥认证函数。auth_mode：认证模式；block：要认证的扇区号；pKey：密钥指令；pSnr：
    def rc52X_pcdAuthState(self, auth_mode, block, pKey, pSnr):
        self.pi['MfCommand']= RC52X_AUTHENT
        self.pi['MfLength'] = 12
        self.pi['MfData']   = [auth_mode, block]
        self.pi['MfData'].extend(pKey[:6])
        self.pi['MfData'].extend(pSnr[:4])

        status = self.rc52x_pcdComTransceive(self.pi)
        if (status == MI_OK): 
            reg = self.i2c_read_rc52x(Status2Reg, 1)
            if(not (reg[0] & 0x08)):
                status = MI_AUTHERR
        return status

    #读指定扇区。addr：扇区号；pWritedata：数据指针；
    def rc52x_pcdRead(self, addr, pReaddata):
        self.pi['MfCommand']= RC52X_TRANSCEIVE
        self.pi['MfLength'] = 2
        self.pi['MfData']   = [PICC_READ, addr]

        status = self.rc52x_pcdComTransceive(self.pi)
        if (status == MI_OK):
            if (self.pi['MfLength'] != 0x80):
                status = MI_BITCOUNTERR
            else:
                pReaddata.extend(self.pi['MfData'])
        return status

    #写指定扇区。addr：扇区号；pWritedata：数据指针；
    def rc52x_pcdWrite(self, addr, pWritedata):
        self.pi['MfCommand']= RC52X_TRANSCEIVE
        self.pi['MfLength'] = 2
        self.pi['MfData']   = [PICC_WRITE, addr]

        status = self.rc52x_pcdComTransceive(self.pi)
        logging.info("PICC_WRITE : status=%d, len=%d" % (status, self.pi['MfLength']))
        if (status != MI_NOTAGERR):
            if(self.pi['MfLength'] != 4):
                status = MI_BITCOUNTERR
            else:
                self.pi['MfData'][0] &= 0x0F
                if self.pi['MfData'][0] == 0x00:
                    status = MI_NOTAUTHERR
                elif self.pi['MfData'][0] == 0x0a:
                    status = MI_OK
                else:
                    status = MI_CODEERR #-6

        if (status == MI_OK):
            self.pi['MfCommand']= RC52X_TRANSCEIVE
            self.pi['MfLength'] = len(pWritedata)
            self.pi['MfData']   = pWritedata

            status = self.rc52x_pcdComTransceive(self.pi)
            logging.info("pWritedata : status=%d, pi['MfData'][0]=0x%02x" % (status, self.pi['MfData'][0]))
            if (status != MI_NOTAGERR):
                self.pi['MfData'][0] &= 0x0F
                if self.pi['MfData'][0] == 0x00:
                    status = MI_WRITEERR
                elif self.pi['MfData'][0] == 0x0a:
                    status = MI_OK
                else:
                    status = MI_CODEERR #-6
        return status

    #发送暂停命令给卡。 addr：扇区号；pWritedata：数据指针；   
    def rc52x_pcdHaltA(self):
        self.pi['MfCommand']= RC52X_TRANSCEIVE
        self.pi['MfLength'] = 2
        self.pi['MfData']   = [PICC_HALT, 0]

        status = self.rc52x_pcdComTransceive(self.pi)
        if status:
            if (status == MI_NOTAGERR) or (status == MI_ACCESSTIMEOUT):
                status = MI_OK
        self.i2c_write_rc52x(CommandReg, RC52X_IDLE)
        return status

    # 防冲撞处理。level：防撞级别；pSnr：UID指针；    
    def rc52x_pcdAnticoll(self, level, pSnr):
        ucBits = ucBytes = ucCollPosition = 0
        ucTemp = snr_check = 0
        ucSNR = [0,0,0,0,0]
        status = 0

        self.i2c_write_rc52x(BitFramingReg,0x00)       
        self.rc52x_cleanBitMask(CollReg,0x80)             
        self.rc52x_cleanBitMask(TxModeReg,0x80)
        self.rc52x_cleanBitMask(RxModeReg,0x80)

        while True:
            ucBits = ucCollPosition % 8
            t_ucCollPosition = ucCollPosition // 8
            ucBytes = t_ucCollPosition
            if (ucBits != 0):
                ucBytes = t_ucCollPosition + 1
                self.i2c_write_rc52x(BitFramingReg, (ucBits << 4) + ucBits)
            
            self.pi['MfCommand']= RC52X_TRANSCEIVE
            self.pi['MfLength'] = ucBytes + 2
            self.pi['MfData']   = [level, 0x20 + (t_ucCollPosition << 4) + (ucBits & 0x0F)]
            self.pi['MfData'].extend(ucSNR[:ucBytes])

            status = self.rc52x_pcdComTransceive(self.pi)

            ucTemp = ucSNR[t_ucCollPosition]
            if (status == MI_COLLERR):
                for i in range(5 - t_ucCollPosition):
                    ucSNR[i + t_ucCollPosition] = self.pi['MfData'][i+1]
                ucSNR[t_ucCollPosition] |= ucTemp
                ucCollPosition = self.pi['MfData'][0]
            elif (status == MI_OK):
                d_len = self.pi['MfLength']
                t_len = d_len // 8
                #logging.info("rc52x_pcdAnticoll: MfLength=%d" % (d_len))
                for i in range(t_len):
                    ucSNR[4 - i] = self.pi['MfData'][t_len - i - 1]
                ucSNR[t_ucCollPosition] |= ucTemp

            if (status != MI_COLLERR) : 
                break
            
        if (status == MI_OK):
            for i in range(4):
                pSnr.append(ucSNR[i])
                snr_check ^= ucSNR[i]

            if (snr_check != ucSNR[4]):
                status = MI_COM_ERR # -125
        
        self.rc52x_setBitMask(CollReg,0x80)
        return status

    def updateCrc_B(self, bCh, pLpwCrc):
        bCh = (bCh ^ (pLpwCrc & 0x00FF))
        bCh = (bCh ^ (bCh<<4))
        temp = (pLpwCrc >> 8) ^ (bCh << 8) ^ (bCh << 3) ^ (bCh >> 4)
        return temp

    def computeCrc_B(self, pData, dwLength, pCrcr):
        bChBlock = 0
        wCrc = 0xFFFF
        for i in range(dwLength):
            bChBlock = pData[i]
            wCrc = self.updateCrc_B(bChBlock, wCrc)
        wCrc = ~wCrc
        p = [0,0]
        p[0] = (wCrc & 0xFF)
        p[1]= ((wCrc>>8) & 0xFF)
        pCrcr = p

    def rc52X_pcdJewelCommand(self, pdata, lenth, resp):
        crc = [0, 0]
        replen = 0
        for i in range(lenth-1):
            dat = 0x07 if i == 0 else 0x00
            self.i2c_write_rc52x(BitFramingReg, dat)

            self.pi['MfCommand']= RC52X_TRANSMIT
            self.pi['MfLength'] = 1
            self.pi['MfData']   = [pdata[i]] 

            status = self.rc52x_JewelTransceive(self.pi)

            if (status != MI_OK):
                return status

        self.pi['MfCommand']= RC52X_TRANSCEIVE
        self.pi['MfLength'] = 1
        self.pi['MfData']   = [pdata[lenth-1]]    
        status = self.rc52x_JewelTransceive(self.pi)

        if (status == MI_OK):
            replen = self.pi['MfLength'] // 8
            if (replen != 0):
                resp.extend(self.pi['MfData'][:replen])
                self.ComputeCrc_B(resp, replen-2, crc)
                if (crc[0] != resp[replen-2]) or (crc[1] != resp[replen-1]):
                    status = MI_CRCERR
        return (status,replen)

    def rc52x_pcdRidA(self, pUid):
        cmd_rid = [0x78,0,0,0,0,0,0]
        resp = []
        re_rid = [0,0]
        lenth = 0
    
        self.ComputeCrc_B(cmd_rid, 7, re_rid)
        cmd_rid.extend(re_rid)
        status,lenth = self.rc52x_pcdJewelCommand(cmd_rid, len(cmd_rid), resp)
        # out_resp = "".join(f"{item:02x}" for item in resp)
        #logging.info("RID:%s len=%d" % (out_resp, lenth))
        if (status == MI_OK):
            pUid.extend(resp[2:6])
        return status

    def rc52x_pcdReadA(self, pUid):
        cmdbuf = [1,8,0]
        resp = []
        re_buf = [0,0]
        lenth = 0
        cmdbuf.extend(pUid[:4])
        self.ComputeCrc_B(cmdbuf, 7, re_buf)
        status,lenth = self.rc52x_pcdJewelCommand(cmdbuf, len(cmdbuf), resp)
        # out_resp = "".join(f"{item:02x}" for item in resp)
        #logging.info("read:%s len=%d" % (out_resp, lenth))
        return status

    def rc52x_pcdJewel(self):
        uid = []
        status = self.rc52x_pcdRidA(uid)
        if(status != MI_OK): 
            return status
        status = self.rc52x_pcdReadA(uid)
        return status

    # 读取A卡并进行防撞处理。patqa; puid：ID; plen：ID长度;  psak：类型
    def rc52x_pcdActivateA(self, patqa, puid):
        plen = psak = 0
        self.rc52x_pcdConfigISOType('A')
        self.reactor.pause(self.reactor.monotonic() + .05)
        status = self.rc52x_pcdRequestA(PICC_REQALL, patqa)  #PICC_REQALL,patqa = 0X4400; PICC_REQIDL

        #logging.info("ATQA: status=%d" % (status))
        if (status == MI_OK):
            logging.info("card type: pata = 0x%02X%02X" % (patqa[0], patqa[1]))
        else: 
            return (status,plen, psak)
        
        if ((patqa[1] == 0x0C) and (patqa[0] == 0)):    
            plen = 6
            status = self.rc52x_pcdJewel()
            if (status != MI_OK): 
                return (status,plen, psak)
        else:    
            plen = 4
            puid_dat = []
            status = self.rc52x_pcdAnticoll(PICC_ANTICOLL1, puid_dat)
            puid.extend(puid_dat[:])
            # out_puid = "".join(f"{item:02x}" for item in puid)
            # logging.info("UID1: status=%d, puid=%s, len=%d" % (status,out_puid,len(puid)))
            if (status != MI_OK):
                return (status,plen, psak)

            status,psak = self.rc52x_pcdSelect(PICC_ANTICOLL1, puid_dat)
            # logging.info("SEL1: status=%d, psak=%d" % (status,psak))
            if (status != MI_OK):
                return (status,plen, psak)
            
            if(patqa[0] & 0xC0):    #level 2 (0x44 & 0xc0)
                plen = 8
                puid_dat = []
                status = self.rc52x_pcdAnticoll(PICC_ANTICOLL2, puid_dat)
                puid.extend(puid_dat[:])
                # out_puid = "".join(f"{item:02x}" for item in puid)
                # logging.info("UID2: status=%d,puid=%s, len=%d" % (status,out_puid,len(puid)))

                if (status != MI_OK):
                    return (status,plen, psak)

                status, psak = self.rc52x_pcdSelect(PICC_ANTICOLL2, puid_dat)
                # logging.info("SEL2: status=%d, psak=%d" % (status,psak))
                if (status != MI_OK):
                    return (status,plen, psak)
            
            if (patqa[0] & 0x80):    #level 3  (0x44 & 0x80)
                plen = 12
                puid_dat = []
                status = self.rc52x_pcdAnticoll(PICC_ANTICOLL3, puid_dat)
                puid.extend(puid_dat[:])
                # out_puid = " ".join(f"{item:2x}" for item in puid)
                # logging.info("UID3: status=%d, puid=%s, len=%d" % (status,out_puid,len(puid)))
                if (status != MI_OK):
                    return (status,plen, psak)

                status,psak = self.rc52x_pcdSelect(PICC_ANTICOLL3, puid_dat)
                # logging.info("SEL3: status=%d, psak=%d" % (status,psak))
                if (status != MI_OK):
                    return (status,plen, psak)
        return (status,plen, psak)

    def rc52x_setBaudrate(self, txrate, rrate):
        tempts = self.i2c_read_rc52x(TxModeReg, 1)
        temprs = self.i2c_read_rc52x(RxModeReg, 1)
        tempt = tempts[0]
        tempr = temprs[0]

        if txrate == 1: # 212K
            self.i2c_write_rc52x(TxModeReg,tempt&0x9f) #2fh,
            self.i2c_write_rc52x(TxModeReg,tempt|0x10)
            self.i2c_write_rc52x(ModWidthReg,0x10)

        elif txrate == 2: # 424K
            self.i2c_write_rc52x(TxModeReg,tempt&0xaf) #2fh,
            self.i2c_write_rc52x(TxModeReg,tempt|0x20)
            self.i2c_write_rc52x(ModWidthReg,0x07)  

        elif txrate == 3: # 848K
            self.i2c_write_rc52x(TxModeReg,tempt&0xbf) #2fh,
            self.i2c_write_rc52x(TxModeReg,tempt|0x30)
            self.i2c_write_rc52x(ModWidthReg,0x02)  

        else: # 106K
            self.i2c_write_rc52x(TxModeReg,tempt&0x8f); #2fh,
            self.i2c_write_rc52x(ModWidthReg,0x26); 

        if rrate == 1: # 212K
            self.i2c_write_rc52x(RxModeReg,tempr&0x9f) #2fh,
            self.i2c_write_rc52x(RxModeReg,tempr|0x10)

        elif rrate == 2: # 424K
            self.i2c_write_rc52x(RxModeReg,tempr&0xaf) #2fh,
            self.i2c_write_rc52x(RxModeReg,tempr|0x20)

        elif rrate == 3: # 848K
            self.i2c_write_rc52x(RxModeReg,tempr&0xbf) #2fh,
            self.i2c_write_rc52x(RxModeReg,tempr|0x30)
        else: # 106K
            self.i2c_write_rc52x(RxModeReg,tempr&0x8f) #35h,

    def rc52x_ratsA(self, pbuf):
        plen = 0
        self.pi['MfCommand']= RC52X_TRANSCEIVE
        self.pi['MfLength'] = 2
        self.pi['MfData']   = [0xE0, 0x51] #default=0x51    Fsdi,CID

        status = self.rc52x_pcdComTransceive(self.pi) 
        if (status == MI_OK):
            plen = self.pi['MfLength'] // 8
            pbuf = self.pi['MfData'][:plen]
        return (status,plen)

    def rc52x_ppsA(self,param):
        pPpss = 0
        self.pi['MfCommand']= RC52X_TRANSCEIVE
        self.pi['MfLength'] = 3
        self.pi['MfData']   = [0xD1, 0x11, param] # 0xD0 | CID,default=0x51  bFsdi,bCid, 0x0A(424,424); 0x0F(848,848)

        status = self.rc52x_pcdComTransceive(self.pi) 
        if (status == MI_OK):
            if (self.pi['MfLength'] != 8):
                status = MI_BITCOUNTERR
            else:
                pPpss = self.pi['MfData'][0]
        return (status,pPpss)

    def rc52x_pcdMfulRead(self, addr, pReaddata):
        self.rc52x_setBitMask(TxModeReg,0x80) #12h,    //TxCRCEn
        self.rc52x_setBitMask(RxModeReg,0x80) #13h,

        self.pi['MfCommand']= RC52X_TRANSCEIVE
        self.pi['MfLength'] = 2
        self.pi['MfData']   = [PICC_READ, addr]
        status = self.rc52x_pcdComTransceive(self.pi) 
        if (status == MI_OK):
            if (self.pi['MfLength'] != 0x80):
                status = MI_BITCOUNTERR #-11
            else:
                pReaddata.extend(self.pi['MfData'])
        return status

    def rc52X_CPU_I_Block(self, psbuf, slen, prbuf):
        prlen = 0
        self.i2c_write_rc52x(TReloadRegL,0x0B)  #2dh,        //30    
        self.i2c_write_rc52x(TReloadRegH,0x41) 

        self.pi['MfCommand']= RC52X_TRANSCEIVE
        self.pi['MfLength'] = slen+2
        self.pi['MfData']   = [0x0A, 0x01]  #PCB ,Cid
        self.pi['MfData'].extend(psbuf[:slen])
        status = self.rc52x_pcdComTransceive(self.pi) 
        if (status == MI_OK):
            prlen= self.pi['MfLength'] // 8
            prbuf = self.pi['MfData'][:prlen]
        return (status,prlen)

    #读取A卡返回UID。 UID_a: ID，需传入空列表; r_len: ID长度; isRead: 是否读取扇区，addr：扇区地址
    def rc52x_typeA(self, UID_a):
        ulen = sak = r_len = 0
        ver1 = [0x60]
        atqa = []
        uid = []
        RD_Data = []
        KEY = [0xFF,0xFF,0xFF,0xFF,0xFF,0xFF]
        random = [0x00,0x84,0x00,0x00,0x08]
        self._set_nrstpd_pin(0)
        self.reactor.pause(self.reactor.monotonic() + .01)
        self._set_nrstpd_pin(1)
        self.reactor.pause(self.reactor.monotonic() + .05)

        status,ulen,sak = self.rc52x_pcdActivateA(atqa, uid)    #寻卡，防冲撞，选卡
        if (status == MI_OK):
            if(ulen == 8):
                r_len = ulen - 1
                UID_a.extend(uid[1:8])
            else:
                r_len = ulen
                UID_a.extend(uid[:4])
        else:
            return r_len
        
        if (ulen != 6):        
            if (sak & 0x04):
                logging.info("uid not complete")
                return r_len
            if (sak & 0x20):    #compliant with ISO-14443-4  
                RD_Data = [] 
                status,ulen = self.rc52x_ratsA(RD_Data)
                # out_data = "".join(f"{item:2d}" for item in RD_Data)
                # logging.info("Rats: status=%d, read=%s, ulen=%d" % (status,out_data,ulen))
                if(status != MI_OK):
                    return r_len
                
                if ((RD_Data[1] & 0x10) and ((RD_Data[2] & 0x44) == 0x44)):    #TA(1) 848k supported    //mifare desfire    
                    status,sak = self.rc52x_ppsA(0x0f)  #106:0x00   212:0x05  424:0x0a  848:0x0f
                    logging.info("Pps: status=%d, sak=%d" % (status,sak))
                    if(status != MI_OK):
                        return r_len               
                    self.rc52x_setBaudrate(3,3)        #106：0，0  212：1，1   424：2，2   848：3，3   
                    uid = [] 
                    status,ulen = self.rc52X_CPU_I_Block(ver1, 1, uid)
                    out_data = "".join(f"{item:2d}" for item in uid)
                    logging.info("ver1: status=%d, uid=%s, ulen=%d" % (status,out_data,ulen))
                    if (status != MI_OK):
                        return r_len
                    self.i2c_write_rc52x(ModWidthReg,0x26)               
                    self.rc52x_setBaudrate(0,0)                
                else:
                    uid = []
                    status,ulen = self.rc52X_CPU_I_Block(random, 5, uid)
                    out_data = "".join(f"{item:2d}" for item in uid)
                    logging.info("random: status=%d, uid=%s, ulen=%d" % (status,out_data,ulen))
                    if (status != MI_OK): 
                        return r_len

            elif (ulen == 4):    #not compliant with ISO-14443-4    
                if (sak == 0x08) and (atqa[0] == 0x04):
                    logging.info("mifare S50 has detected!")
                elif (sak == 0x18) and (atqa[0] == 0x02):
                    logging.info("mifare S70 has detected!")

                status = self.rc52X_pcdAuthState(0x60, 0x00, KEY, uid)
                logging.info("AUTH: %d " % (status))

                RD_Data = [] 
                status = self.rc52x_pcdRead(0,RD_Data)  #读 0 页扇区
                out_data = "".join(f"{item:2d}" for item in RD_Data)
                logging.info("READ: status=%d, data=%s, ulen=%d" % (status,out_data,16))

                if (status!=MI_OK):
                    return r_len
            else:  
                RD_Data = [] 
                #status = self.rc52x_pcdMfulRead(3,RD_Data)
                status = self.rc52x_pcdRead(3,RD_Data)  #读 3 页扇区
                out_data = " ".join(f"{item:02x}" for item in RD_Data)
                logging.info("READ PAGE3: status=%d, data=%s, ulen=%d" % (status,out_data,16))
                if (status != MI_OK):
                    return r_len
        return r_len

def load_config(config):
    return RC52X_TYPE(config)
```

