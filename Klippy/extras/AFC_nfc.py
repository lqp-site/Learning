import logging
import json
from . import bus
from configfile import error
from . import AFC_assist
import struct
from datetime import datetime

#  M1卡分为16个扇区，每个扇区由四个块（块0、块1、块2、块3）组成，每个块 4 个字节
#  将16个扇区的64个块按绝对地址编号为：0~63
#  第0个扇区的块0（即绝对地址0块），用于存放厂商代码，已经固化不可更改 
#  每个扇区的块0、块1、块2为数据块，可用于存放数据
#  每个扇区的块3为控制块（绝对地址为:块3、块7、块11.....）包括密码A，存取控制、密码B等
# Page4-Page15是可读写的用户数据区，出厂时其内容初始化为0，用户可以任意读写
#   每次仅写一块数据，但需要发送16字节长度数据，写入的是前4个字节
#   每次读取数据16个字节

KEY_MODE_A = 0x60
KEY_MODE_B = 0x61
KEY_A = [0xff,0xff,0xff,0xff,0xff,0xff]
KEY_B = [0xff,0xff,0xff,0xff,0xff,0xff]
KEY_SECTOR = {'NTAG213_PWD':0x2B,'NTAG215_PWD':0x85,'NTAG216_PWD':0xE5}
PWD_KEY = [0xE0,0xE1,0xE2,0xE3,0x00,0x00]
# PWD_KEY = [0,0,0,0,0,0]

################## BLOCK addr 从0 到 63 块，每个扇区 4 块
#  adr1_0=0x04;  	第1扇区0区块(第5块)
#  adr1_3=0x07;  	第1扇区3区块(第8块)
#  adr2_0=0x08;  	第2扇区0区块(第9块)
sectorAddr_map = [0, 0x04, 0x08, 0x0C, 0x10, 0x14, 0x18, 0x1C, 0x20, 0x24, 0x28, 0x2C, 0x30, 0x34, 0x38, 0x3C]
################## IIC addr
RC52X_I2C_ADDR_0 = 0x02 >> 1
RC52X_I2C_ADDR_1 = 0x06 >> 1
RC52X_I2C_ADDR_2 = 0x0A >> 1

################## SPI addr

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
PICC_ANTICOLL1      = 0x93               #防冲撞1
PICC_ANTICOLL2      = 0x95               #防冲撞2
PICC_ANTICOLL3      = 0x97               #防冲撞3   
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

RC52X_MAX_BUSY_CYCLES= 5
class NFC_RC52X:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.reactor = self.printer.get_reactor()
        self.AFC = self.printer.lookup_object('AFC')
        self.gcode = self.printer.lookup_object('gcode')
        self.checked_lane_hub = False

        self.last_print_time = 0.
        self.scale = 1
        self.pi = {'MfCommand' : 0xff, 'MfLength': 0xff, 'MfData': [0xff, 0xff]}
        
        self.bus_type = config.get('bus_type', 'i2c')
        self.card_number = config.getint('card_number', 5, minval=0)
        self.rc52x_number = config.getint('rc52x_number', 3, minval=0) 
        self.auth_pwd = config.getboolean("auth_pwd", False)
        pwd_key = config.get('pwd_key', None)
        if pwd_key != None :
            PWD_KEY = self._rc52x_convertToHexStrData(pwd_key, isToString=False)
            # hex_list = [format(num, '02x') for num in PWD_KEY]
            # hex_str = ' '.join(map(str,hex_list)) 
            # self.gcode.respond_info('PWD_KEY = {}'.format(hex_str))


        # 是否启用测试变量
        self.NFC_TEST = config.getboolean("NFC_TEST", False)
        if self.NFC_TEST == True : # 测试：定时获取卡信息
            self.report_time = config.getint('rc52x_report_time', 10, minval=5)
        
        self.encoder_count = [0,0,0,0,0]
        self.nfc_objList = []
        self.nfc_obj = None
        if self.bus_type == 'i2c':
            self._i2c_bus_config(config)
        elif self.bus_type == 'spi':
            # self._spi_bus_config(config)
            raise error("Error config: bus_type cannot be spi")
        else:
            raise error("Error config: bus_type cannot be empty")

        self.printer.register_event_handler("klippy:ready",self.handle_ready)
        self.gcode.register_command('NFC_WRITE_INFO', self.cmd_rc52x_writeSectorInitInfo)
        self.gcode.register_command('NFC_READ_INFO', self.cmd_rc52x_readInfo)
        # self.gcode.register_command('NFC_WRITE_PWD_CONFIG', self.cmd_rc52x_wirtePWDConfig)
        # self.gcode.register_command('NFC_TEST_AUTH', self.cmd_rc52x_testAuthWriteData) 
        
    def handle_ready(self):
        # 启动天线
        for i in range(self.rc52x_number):
            self._set_current_obj(i+1) #选择dev地址
            self._write_rc52x(CommandReg,RC52X_RESETPHASE)
            self._rc52x_fieldOff()
            self.reactor.pause(self.reactor.monotonic() + .05)
            self._rc52x_fieldOn()

        if self.NFC_TEST == True : # 测试：定时获取卡信息
            self.nfc_read = False
            self.sample_timer = self.reactor.register_timer(self.rc52x_testGetCard)
            s_time = self.reactor.monotonic() + self.report_time
            self.reactor.update_timer(self.sample_timer, s_time)
        self._set_current_obj(1) #选择dev地址

    def rc52x_testGetCard(self, eventtime):
        s_time = 0
        for i in range(self.rc52x_number):
            atqa = []
            self._set_current_obj(i+1) #选择dev地址
            UID = R_UID = []
            status,ulen,sak = self.rc52x_pcdActivateA(atqa, R_UID)    #寻卡，防冲撞，选卡
            if (status == MI_OK): #寻卡，防冲撞，选卡 ok
                if (ulen == 8):
                    r_len = ulen - 1
                    UID = R_UID[1:8]                      
                else:
                    r_len = ulen
                    UID = R_UID[:4]

                out_data = " ".join(f"{item:02x}" for item in UID)
                logging.info("rc52x: UID=%s, device=%d" %(out_data, i))

                sector = 0x04 #扇区1
                if self.nfc_read == False: 
                    WD_Data = [0x01, 0x02, 0x03, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
                    status = self.rc52x_pcdWrite(sector, WD_Data)
                    logging.info("WRITE PAGE sector block=%d: status=%d" % (sector, status))
                    self.nfc_read = True
                else:
                    RD_Data = [] 
                    status = self.rc52x_pcdRead(sector, RD_Data) 
                    out_data = " ".join(f"{item:02x}" for item in RD_Data)
                    logging.info("READ PAGE sector block=%d: status=%d, data=%s" % (sector, status,out_data))
                    self.nfc_read = False

            else:
                logging.info("rc52x_testRead: get or select ID_CARD failed, device = %d" % (i))
                # self._reset_rc52x()
            self.reactor.pause(self.reactor.monotonic() + .05)
        
        now_time = self.reactor.monotonic()
        next_time = now_time + self.report_time + s_time
        return next_time

    def _set_current_obj(self, lane=0):
        if lane < 1 :
            self.nfc_obj = None
            return
        # self.nfc_map = [0,0,1,1,2]
        self.nfc_obj = self.nfc_objList[(lane-1) // 2] 
    
    def _i2c_bus_config(self, config):
        i2c_addrList = [RC52X_I2C_ADDR_0, RC52X_I2C_ADDR_1, RC52X_I2C_ADDR_2]
        for i in range(self.rc52x_number):
            i2c_obj = bus.MCU_I2C_from_config(config, default_addr=i2c_addrList[i], default_speed=100000)
            self.nfc_objList.append(i2c_obj)

    def _spi_bus_config(self, config):
        for i in range(self.rc52x_number):
            spi_obj = bus.MCU_SPI_from_config(config, 0, pin_option="cs_pin",
                        default_speed=100000, share_type=None,
                        cs_active_high=False)
            self.nfc_objList.append(spi_obj)    

    def _write_rc52x(self, reg, data):
        writeBuf = []
        if self.nfc_obj is None:
            logging.warning("_write_rc52x: nfc_obj is None")
            return

        if self.bus_type == 'i2c':
            try:
                writeBuf.append(reg)
                if isinstance(data, int):
                    writeBuf.append(data)
                else:
                    writeBuf.extend(data)
                self.nfc_obj.i2c_write(writeBuf)
                # self.reactor.pause(self.reactor.monotonic() + .10)
            except Exception as e:
                logging.exception("rc52x: exception encountered" +
                                " write data: %s"%str(e))
        elif self.bus_type == 'spi':
            try:
                writeBuf.append((reg << 1) & 0x7E)
                if isinstance(data, int):
                    writeBuf.append(data)
                else:
                    writeBuf.extend(data)
                self.nfc_obj.spi_transfer(writeBuf)
            except Exception as e:
                logging.exception("rc52x: exception encountered" +
                                " write data: %s"%str(e))

    def _read_rc52x(self, reg, r_len = 1):
        read = data = []        
        cycles = 0
        if r_len < 1:
            return data
        if self.nfc_obj is None:
            logging.warning("_read_rc52x: nfc_obj is None")
            return data
        spi_reg = ((reg << 1) & 0x7E) | 0x80

        while True:
            # Check if we're constantly busy. If so, send soft-reset.
            # and issue warning.
            if cycles > RC52X_MAX_BUSY_CYCLES:
                logging.warning("rc52x: device reported busy after " +
                    "%d cycles, resetting device" % RC52X_MAX_BUSY_CYCLES)
                # self._reset_rc52x()
                break
            cycles += 1

            try:
                # Read data
                if self.bus_type == 'i2c':
                    read = self.nfc_obj.i2c_read([reg], r_len)
                    if read is None:
                        logging.warning("_read_rc52x: received data  is None")
                        continue
                    data = bytearray(read['response'])
                elif self.bus_type == 'spi':
                    self.nfc_obj.spi_transfer([spi_reg])
                    for i in range(6):
                        read[i] = self.nfc_obj.spi_transfer([0])
                    data = read

                if len(data) < r_len:
                    logging.warning("_read_rc52x: received bytes less than" +
                                    " expected 6,  actually [%d]" % len(data))
                    continue
                return data
            except Exception as e:
                logging.exception("rc52x: exception encountered" +
                              " reading data: %s"%str(e))
            
            return []

    def _reset_rc52x(self):
        self._write_rc52x(CommandReg, RC52X_RESETPHASE)
        self.reactor.pause(self.reactor.monotonic() + .01)
        # self._read_rc52x(VersionReg, 1)
        logging.info("rc52x: reset device ")
        
    def _rc52x_setBitMask(self, reg, mask):
        temp = self._read_rc52x(reg, 1)
        if temp == []:
            temp = [0]
        self._write_rc52x(reg, temp[0] | mask)

    def _rc52x_cleanBitMask(self, reg, mask):
        temp = self._read_rc52x(reg, 1)
        if temp == []:
            temp = [0]
        self._write_rc52x(reg, temp[0] & (~mask))

    def _rc52x_fieldOn(self):
        temp = self._read_rc52x(TxControlReg, 1) 
        if temp == []:
            self._write_rc52x(TxControlReg, 0x00 | 0x03)
        elif (not (temp[0] & 0x03)):
            self._write_rc52x(TxControlReg, temp[0] | 0x03)
        self.reactor.pause(self.reactor.monotonic() + .01)
        logging.info("rc52x: field On ")

    def _rc52x_fieldOff(self):
        self._rc52x_cleanBitMask(TxControlReg, 0x03) 

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
        self._write_rc52x(ComIEnReg, irqEn | 0x80)   # 开中断
        self._write_rc52x(CommandReg, RC52X_IDLE)    # 取消当前命令的执行
        self._rc52x_setBitMask(FIFOLevelReg, 0x80)       # Flush FIFO  清除FIFO缓冲区及其标志位 
        self._rc52x_cleanBitMask(ComIrqReg, 0x80)        # 清除中断标志位SET1   
        self._write_rc52x(ComIrqReg, 0x7F) 

        # 发送命令帧
        self._write_rc52x(FIFODataReg, pi['MfData'])    
        # 执行命令
        self._write_rc52x(CommandReg, pi['MfCommand'])

        if pi['MfCommand'] == RC52X_TRANSCEIVE:
            self._rc52x_setBitMask(BitFramingReg, 0x80)   #Start Send 

        for i in range(2000): #根据时钟频率调整,操作M1卡最大等待时间25ms
            if (i % 20 == 0):
                n = self._read_rc52x(ComIrqReg, 1)   #查询事件中断
                if n == []:
                    continue
                if (n[0] & irqEn & 0x01) or (n[0] & waitFor): # timerirq idleirq rxirq #等待命令完成
                    # logging.info("timerirq idleirq rxirq") 
                    break

        self._rc52x_cleanBitMask(BitFramingReg, 0x80)   # 停止发送
        if n == []:
            return -1

        if n[0] & waitFor :  #IdleIRq
            temp = self._read_rc52x(ErrorReg, 1) #读错误标志寄存器
            if temp[0] & 0x1b : 
                status = MI_ERR #-126
            else:
                status = MI_OK
                if pi['MfCommand'] == RC52X_TRANSCEIVE:
                    n = self._read_rc52x(FIFOLevelReg, 1) #读FIFO中保存的字节数
                    lastBits = self._read_rc52x(ControlReg, 1) #最后接收到得字节的有效位数
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

                    pi['MfData'] = self._read_rc52x(FIFODataReg, n[0])

        elif (n[0] & irqEn & 0x01):    #TimerIRq 发生定时器中断
            status = MI_NOTAGERR  #-1
        else:
            temp = self._read_rc52x(ErrorReg, 1)
            if (not (temp[0] & 0x1B)):
                #temp = self._read_rc52x(ErrorReg, 1)
                status = MI_ACCESSTIMEOUT #-27

        self._rc52x_setBitMask(ControlReg, 0x80)     #stop timer now 停止定时器运行
        self._write_rc52x(CommandReg, RC52X_IDLE)  

        return  status
    
    def rc52x_resetLoadedPosition(self, lane_num):
        CUR_LANE = self.AFC.lanes["lane{}".format(lane_num)] 
        cycle_num = 10
        while CUR_LANE.load_state == True and cycle_num > 0:
            CUR_LANE.move( -100 , CUR_LANE.long_moves_speed, CUR_LANE.long_moves_accel, True)
            cycle_num -= 1
        
        cycle_num = 8
        while CUR_LANE.load_state == False and cycle_num > 0:
            CUR_LANE.move( 15 , CUR_LANE.short_moves_speed, CUR_LANE.short_moves_accel, False)
            cycle_num -= 1
        

    def _rc52x_findCardAction(self, lane_num, CUR_LANE):
        status = -1

        # move_dis = CUR_LANE.short_move_dis 
        move_speed = CUR_LANE.short_moves_speed 
        move_accel = CUR_LANE.short_moves_accel 

        CUR_LANE.unsync_to_extruder(False)
        CUR_LANE.do_enable(True)

        # 启动挤出电机，并寻卡
        move_dis = 56
        cycle_num = 400 // move_dis
        while status != MI_OK and cycle_num > 0:
            CUR_LANE.move(move_dis, move_speed, move_accel, False)
            atqa = []
            self.rc52x_pcdConfigISOType('A')
            status = self.rc52x_pcdRequestA(PICC_REQALL, atqa)
            cycle_num -= 1
        
        if status != MI_OK:
            move_dis = 36
            cycle_num = 336 // move_dis
            while status != MI_OK and cycle_num > 0:
                CUR_LANE.move( move_dis * -1, move_speed, move_accel, True)
                atqa = []
                self.rc52x_pcdConfigISOType('A')
                status = self.rc52x_pcdRequestA(PICC_REQALL,atqa)
                cycle_num -= 1

        self.reactor.pause(self.reactor.monotonic() + .01)
        atqa = []
        self.rc52x_pcdConfigISOType('A')
        status = self.rc52x_pcdRequestA(PICC_REQALL, atqa)
        # 二次检查
        if(status != MI_OK):
            # 启动挤出电机，缓慢前进到hub限位开关前并寻卡
            move_dis = 40
            cycle_num = 400 // move_dis
            while  status != MI_OK and cycle_num > 0:
                CUR_LANE.move(move_dis, move_speed, move_accel, False)
                atqa = []
                self.rc52x_pcdConfigISOType('A')
                status = self.rc52x_pcdRequestA(PICC_REQALL, atqa)
                cycle_num -= 1

            # if status != MI_OK:
            #     move_dis = 30
            #     cycle_num = 300 // move_dis
            #     while status != MI_OK and cycle_num > 0:
            #         CUR_LANE.move( move_dis * -1, move_speed, move_accel, True)
            #         atqa = []
            #         self.rc52x_pcdConfigISOType('A')
            #         status = self.rc52x_pcdRequestA(PICC_REQALL,atqa)
            #         cycle_num -= 1
        
        CUR_LANE.do_enable(False)
        return status

    def _rc52x_checkCurrentLine(self, lane_num, CUR_LANE):
        atqa = []
        
        # 移动关联通道，查看是否对应卡
        if lane_num == self.card_number:
            otherIndex = self.card_number - 1
        else:
            otherIndex = (lane_num - 1) if lane_num % 2 == 0 else (lane_num + 1)
        OTHER_LANE = self.AFC.lanes["lane{}".format(otherIndex)]
        OTHER_LANE.unsync_to_extruder(False)
        OTHER_LANE.do_enable(True)

        if not OTHER_LANE.prep_state:
            OTHER_LANE.move( -60, CUR_LANE.short_moves_speed, CUR_LANE.short_moves_accel, True)
            self.rc52x_pcdConfigISOType('A')
            status = self.rc52x_pcdRequestA(PICC_REQALL, atqa)
            OTHER_LANE.do_enable(False)
            return status

        cycle_num = 8 
        while OTHER_LANE.load_state != True and cycle_num > 0:
            OTHER_LANE.move(30 , CUR_LANE.short_moves_speed, CUR_LANE.short_moves_accel, False)
            cycle_num -= 1
            atqa = []
            self.rc52x_pcdConfigISOType('A')
            status = self.rc52x_pcdRequestA(PICC_REQALL, atqa)
            if status != MI_OK:
                OTHER_LANE.do_enable(False)
                return status

        if cycle_num > 5:
            OTHER_LANE.move(CUR_LANE.short_move_dis * -1, CUR_LANE.short_moves_speed, CUR_LANE.short_moves_accel, True)
            cycle_num -= 1
            atqa = []
            self.rc52x_pcdConfigISOType('A')
            status = self.rc52x_pcdRequestA(PICC_REQALL, atqa)

        OTHER_LANE.do_enable(False)
        return status

    def rc52x_findCard(self, lane_num):
        atqa = []

        #选择dev地址
        self._set_current_obj(lane_num)

        # 校验通道 
        CUR_LANE = self.AFC.lanes["lane{}".format(lane_num)] 
        if not CUR_LANE.prep_state:
            self.gcode.respond_info("filament not inserted, prep load sensor has not Triggered!!!")
            return -1
        if CUR_LANE.get_toolhead_sensor_state(): #toolhead has filament
            self.gcode.respond_info("toolhead sensor has Triggered!!!")
            return -2

        CUR_LANE.unsync_to_extruder(False)
        CUR_LANE.do_enable(True)
        
        # 快速移动5个通道检查是否在hub, 如果存在则复位
        if (CUR_LANE._afc_prep_done == True) or (CUR_LANE._afc_prep_done == False and self.checked_lane_hub == False):
            self.checked_lane_hub = True
            encoder = self.printer.lookup_object('AFC_hallEncoder') 
            pre_count = encoder.get_counts()
            for i in range(5):
                CUR_LANE_t = self.AFC.lanes["lane{}".format(i+1)] 
                if not CUR_LANE_t.prep_state:
                    continue
                CUR_LANE_t.move( 12 , CUR_LANE.short_moves_speed, CUR_LANE.short_moves_accel, False)
                CUR_LANE_t.move( -12 , CUR_LANE.short_moves_speed, CUR_LANE.short_moves_accel, False)
                cur_count = encoder.get_counts()
                if cur_count != pre_count: # 通道耗材复位
                    cur_count == pre_count
                    cycle_num = 16
                    while CUR_LANE_t.load_state == True and cycle_num > 0:
                        CUR_LANE_t.move( -100 , CUR_LANE.long_moves_speed, CUR_LANE.long_moves_accel, True)
                        cycle_num -= 1
                        if cycle_num < 1:
                            self.gcode.respond_info("lane{} Reset the filament loading position failed".format(i+1))
                            return -3
                    break

        self.rc52x_pcdConfigISOType('A')
        status = self.rc52x_pcdRequestA(PICC_REQALL,atqa)                 #寻卡
        if status == MI_OK :
            status = -1
            if bool(CUR_LANE.spool_id) == True:
                h_id = self.rc52x_readCardHeaderIDInfo()
                if str(h_id) == CUR_LANE.spool_id:
                    status = MI_OK
            if status != MI_OK:
                status = self._rc52x_checkCurrentLine(lane_num, CUR_LANE)      # 校验通道与卡对应 

        if status != MI_OK:
            status = self._rc52x_findCardAction(lane_num, CUR_LANE)        #电机运行寻卡
            return (1 if status == MI_OK else status)
        
        return status

    def rc52x_writeCardInfo(self, lane_num, sector, data):
        if sector == 4:
            return self.rc52x_writeCardWeightInfo(data)
        else:
            self.gcode.respond_info('Unable to write sector{}'.format(sector))
        return -1

    def rc52x_writeCardWeightInfo(self, data=None):
        status = -1
        # 先读取数据 sector-3 block-0
        send_data = []
        if data is None:
            self.gcode.respond_info(' data == None ')
            return -1

        w_data_list = self._rc52x_convertIntData(data, isToInt=False)
        send_data = w_data_list + [0,0,0,0,0,0,0,0,0,0,0,0]

        status = self.rc52x_writeSectorData(sectorAddr_map[4], send_data, PWD_KEY)
        return status

    def rc52x_readCardHeaderIDInfo(self):
        read_data = self.rc52x_readSectorData(sectorAddr_map[1])
        if read_data == []:
            return -1
        h_id = self._rc52x_convertIntData(read_data[:4], isToInt=True)
        self.gcode.respond_info("id={}\n".format(h_id))
        return h_id

    def rc52x_readCardSectorInfo(self, lane_num, sector):
        status = -1
        
        read_data = self.rc52x_readSectorData(sectorAddr_map[sector])
        if read_data == []:
            return -1
        str_message = "--------- read LANE{} filament sector{} data ---------".format(lane_num,sector)
        self.gcode.respond_info(str_message)

        CUR_LANE = self.AFC.lanes["lane{}".format(lane_num)] 
        if sector == 1:
            header = self._rc52x_convertIntData(read_data[:4], isToInt=True)
            CUR_LANE.spool_id = str(header)

            manufacturer_code = self._rc52x_convertToHexStrData(read_data[4:8], isToString=True)
            CUR_LANE.manufacturer_code = manufacturer_code

            filament_code = self._rc52x_convertToHexStrData(read_data[8:10], isToString=True)
            CUR_LANE.filament_code = filament_code

            _date_year = read_data[12] << 8 | read_data[13]
            _date_month = read_data[14] << 8 | read_data[15]
            if _date_year > 0:
                production_date = str(_date_year) + '-' + str(_date_month)
            else:
                production_date = ''
            CUR_LANE.production_date = production_date

            str_message = "header_id={}\n manufacturer code={}\n filament code={}\n".format(header,manufacturer_code,filament_code)
            str_message += "production date={}\n".format(production_date)


        elif sector == 2:
            material_name = self._rc52x_convertCharStrData(read_data[:4], isToString=True, del_tail0=True)
            material_sup = self._rc52x_convertCharStrData(read_data[4:8], isToString=True, del_tail0=True)
            if material_sup != '':
                material = material_name + ' ' + material_sup
            else:
                material = material_name

            CUR_LANE.material = material

            color_code = '#' + self._rc52x_convertToHexStrData(read_data[8:11], isToString=True)
            CUR_LANE.color = color_code

            filament_diameter = (read_data[12] << 8 | read_data[13]) / 100   #"{:.2f}".format(result)
            CUR_LANE.diameter = round(filament_diameter, 2)

            density = (read_data[14] << 8 | read_data[15]) / 100 
            density_dict = {'PLA':1.24, 'PETG':1.27, 'ABS':1.05, 'TPU':0.92, 'PA':1.14, 'PEEK':1.32, 'CPE':1.22,'PC':1.24, 'PVA':1.23, 'ASA':1.07} 
            if density == 0 and material_name in density_dict:
                density = density_dict[material_name]
            CUR_LANE.density = round(density, 2)

            str_message = "material={}\n color code={}\n diameter={:.2f}\n density={:.2f}".format(material,color_code,filament_diameter,density)
        elif sector == 3:
            min_temp = (read_data[0] << 8 | read_data[1])
            max_temp = (read_data[2] << 8 | read_data[3])
            CUR_LANE.extruder_temp = (min_temp + max_temp) // 2
            str_message = "min_temp={}\n max_temp={}\n".format(min_temp,max_temp)
        elif sector == 4:
            filament_weight = self._rc52x_convertIntData(read_data[:4], isToInt=True)
            CUR_LANE.weight = filament_weight
            str_message = "filament weight={}\n".format(filament_weight)
        else:
            str_message = "There is no data written to the current sector"
        
        self.gcode.respond_info(str_message)

        return MI_OK

    # 读取所有信息
    def rc52x_readCardAllInfo(self, lane_num):
        status = -1

        # 第 1 扇区 0块：数据头, 1块：制造商编号, 2块：灯丝编号, 3块：生产日期
        read_data = self.rc52x_readSectorData(sectorAddr_map[1])
        if read_data == []:
            return -1
        str_message = "----------- read LANE{} filament all data -----------".format(lane_num)
        self.gcode.respond_info(str_message)
        header = self._rc52x_convertIntData(read_data[:4], isToInt=True)
        manufacturer_code = self._rc52x_convertToHexStrData(read_data[4:8], isToString=True)
        filament_code = self._rc52x_convertToHexStrData(read_data[8:10], isToString=True)

        _date_year = read_data[12] << 8 | read_data[13]
        _date_month = read_data[14] << 8 | read_data[15]
        if _date_year > 0:
            production_date = str(_date_year) + '-' + str(_date_month)
        else:
            production_date = ''

        # 第 2 扇区 0块：材质, 1块：材质补充, 2块：颜色, 3块：直径，密度
        read_data = self.rc52x_readSectorData(sectorAddr_map[2])
        if read_data == []:
            return -2
        material_name = self._rc52x_convertCharStrData(read_data[:4], isToString=True, del_tail0=True)
        material_sup = self._rc52x_convertCharStrData(read_data[4:8], isToString=True, del_tail0=True)
        if material_sup != '':
            material = material_name + ' ' + material_sup
        else:
            material = material_name
        color_code = '#' + self._rc52x_convertToHexStrData(read_data[8:11], isToString=True)
        filament_diameter = (read_data[12] << 8 | read_data[13]) / 100   #"{:.2f}".format(result)
        density = (read_data[14] << 8 | read_data[15]) / 100 
        density_dict = {'PLA':1.24, 'PETG':1.27, 'ABS':1.05, 'TPU':0.92, 'PA':1.14, 'PEEK':1.32, 'CPE':1.22,'PC':1.24, 'PVA':1.23, 'ASA':1.07} 
        if density == 0 and material_name in density_dict:
            density = density_dict[material_name]
        
        # 第 3 扇区 0块：温度范围, 1块：密度 * 100
        read_data = self.rc52x_readSectorData(sectorAddr_map[3])
        if read_data == []:
            return -3
        min_temp = (read_data[0] << 8 | read_data[1])
        max_temp = (read_data[2] << 8 | read_data[3])

        # 第 4 扇区 0块：剩余重量(g)
        read_data = self.rc52x_readSectorData(sectorAddr_map[4])
        if read_data == []:
            return -4
        filament_weight = self._rc52x_convertIntData(read_data[:4], isToInt=True)

        CUR_LANE = self.AFC.lanes["lane{}".format(lane_num)]
        if CUR_LANE.weight is not None and CUR_LANE.weight < filament_weight:
            status = self.rc52x_writeCardWeightInfo(CUR_LANE.weight)
            self.gcode.respond_info('write weight Info is {}'.format('OK' if status == MI_OK else 'failed'))
            filament_weight = CUR_LANE.weight

        CUR_LANE = self.AFC.lanes["lane{}".format(lane_num)]
        CUR_LANE.spool_id           = str(header)
        CUR_LANE.manufacturer_code  = manufacturer_code 
        CUR_LANE.filament_code      = filament_code
        CUR_LANE.production_date    = production_date
        CUR_LANE.material           = material
        CUR_LANE.color              = color_code
        CUR_LANE.diameter           = round(filament_diameter, 2)
        CUR_LANE.extruder_temp      = (min_temp + max_temp) // 2
        CUR_LANE.density            = round(density, 2)
        CUR_LANE.weight             = filament_weight
        
        self.AFC.save_vars()
        str_message = "header_id={}\n manufacturer code={}\n filament code={}\n".format(header,manufacturer_code,filament_code)
        str_message += "production date={}\n".format(production_date)
        str_message += "material={}\n color code={}\n diameter={:.2f}\n density={:.2f}\n".format(material,color_code,filament_diameter,density)
        str_message += "min_temp={}\n max_temp={}\n".format(min_temp,max_temp)
        str_message += "filament weight={}\n".format(filament_weight)
        
        self.gcode.respond_info(str_message)

        return MI_OK

    def rc52x_writeSectorData(self, sector_addr, w_data, pwd_data=[]):
        atqa = R_UID = []
        status = -1
        for i in range(1):
            status,ulen,sak = self.rc52x_pcdActivateA(atqa, R_UID)  #寻卡，防冲撞，选卡
            if (status == MI_OK): 
                if self.auth_pwd:
                    status = self.rc52X_pcdAuthState_ntag(pwd_data)
                if (status == MI_OK): 
                    status = self.rc52x_pcdWrite(sector_addr, w_data)
                    if (status == MI_OK): 
                        return status
            if i < 2:
                self.reactor.pause(self.reactor.monotonic() + .01)

        self._reset_rc52x()
        return status

    def rc52x_readSectorData(self, sector_addr):
        atqa = R_UID = read_data = []
        status = -1
        for i in range(1):
            read_data = []
            status,ulen,sak = self.rc52x_pcdActivateA(atqa, R_UID)    #寻卡，防冲撞，选卡
            if (status == MI_OK):
                status = self.rc52x_pcdRead(sector_addr, read_data) 
                if (status == MI_OK): 
                    # out_data = " 0x".join(f"{item:02x}" for item in read_data)
                    # logging.info("rc52x_readSector: sector block=%d, status=%d, data=%s ,len=%d" % (sector, status,out_data,len(read_data)))
                    return read_data
            if i < 2:
                self.reactor.pause(self.reactor.monotonic() + .01)
        
        self._reset_rc52x()
        return [] 
    
    def _rc52x_convertToHexStrData(self, data, isToString=False): # "#00cc00"
        if isToString:
            hex_list = [format(num, '02x') for num in data]
            hex_str = ''.join(map(str,hex_list)) 
            return hex_str
        else:
            len_t = len(data)
            if len_t < 1:
                return []
            if len_t % 2 != 0:
                len_t -= 1
                tmp_t = int(data[-1], 16)
                result = [int(data[i:i+2], 16) for i in range(0, len_t, 2)]
                result += [tmp_t]
            else:
                result = [int(data[i:i+2], 16) for i in range(0, len_t, 2)]

            lenght = len(result)
            for i in range(lenght,4):
                result += [0]
            return result
        

    def _rc52x_convertCharStrData(self, data, isToString=False, del_tail0=False):
        if isToString:
            str_data = ''.join(chr(c) for c in data) #使用 map 将所有元素转换成字符串
            if str_data == '':
                return ''
            str_data = str_data.strip() #移除空字符
            str_data = str_data.strip('\u0000') #移除空字符
            str_data = str_data.lstrip('0') #使用 rstrip() 移除左侧字符'0'
            if del_tail0 :
                str_data = str_data.rstrip('0') #使用 rstrip() 移除右侧字符'0'
            return str_data
        else:
            list_data = list(data.encode('utf-8'))
            lenght = len(list_data)
            for i in range(lenght,4):
                list_data += [0]
            return list_data
     
    def _rc52x_convertIntData(self, data, isToInt=True): 
        if isToInt:
            int_data = struct.unpack('>I', bytearray(data))[0] #大端： 无符号 '<I', 有符号 '<i'
            # self.gcode.respond_info("Card int_data: {}".format(int_data))
            return int_data
        else:
            list_data = bytearray(struct.pack('>I', data))  
            return list(list_data)

    def rc52x_pcdConfigISOType(self, type):
        if type == 'A':             #ISO14443_A
            self._write_rc52x(ControlReg, 0x10)
            self._write_rc52x(TxAutoReg, 0x40)
            self._write_rc52x(TxModeReg, 0x00) 
            self._write_rc52x(RxModeReg, 0x00)#   仅检测A卡的设置
            
            self._write_rc52x(ModWidthReg, 0x26) #24h  若AB卡检测都有，则必须设置
            
            self._write_rc52x(BitFramingReg, 0)    #0dh,
            self._write_rc52x(ModeReg, 0x3D)       #11h,    
            self._write_rc52x(TxControlReg, 0x84)  #14h,      
            self._write_rc52x(RxSelReg, 0x88)       #17h,        
            self._write_rc52x(DemodReg, 0x8D)       #19h,     #AddIQ     
            self._write_rc52x(TypeBReg, 0)          #1eh,
            self._write_rc52x(TModeReg, 0x8D) #2ah,
            self._write_rc52x(TPrescalerReg, 0x3e)    #2bh
            self._write_rc52x(TReloadRegL, 30)#2dh
            self._write_rc52x(TReloadRegH, 0) #2ch
            self._write_rc52x(AutoTestReg, 0)        #36h    
            
            if TYPEA_REG_CONFIG == REG_SET_1:
                self._write_rc52x(RxThresholdReg, 0x33) #20201117
                self._write_rc52x(RFCfgReg, 0x44)       #26h     
                self._write_rc52x(GsNOnReg, 0xF8)       #27h        
                self._write_rc52x(CWGsPReg, 0x3F)       #28h

            elif TYPEA_REG_CONFIG == REG_SET_2:
                self._write_rc52x(RxThresholdReg, 0x33) #20201117
                self._write_rc52x(RFCfgReg, 0x44)       #26h     
                self._write_rc52x(GsNOnReg, 0x88)       #27h        
                self._write_rc52x(CWGsPReg, 0x20)       #28h

            elif TYPEA_REG_CONFIG == REG_SET_3:
                self._write_rc52x(RxThresholdReg, 0x42) #20201117
                self._write_rc52x(RFCfgReg, 0x44)       #26h     
                self._write_rc52x(GsNOnReg, 0xF8)       #27h        
                self._write_rc52x(CWGsPReg, 0x3F)       #28h
    
            else:   
                self._write_rc52x(RxThresholdReg, 0x42); #20201117   42
                self._write_rc52x(RFCfgReg, 0x59)       #26h     44
                self._write_rc52x(GsNOnReg, 0x88)       #27h        
                self._write_rc52x(CWGsPReg, 0x20)       #28h

            # self._rc52x_fieldOff()
            # self.reactor.pause(self.reactor.monotonic() + .01)
            # self._rc52x_fieldOn() #防止某些卡复位较慢,提前打开射频场
            # self.reactor.pause(self.reactor.monotonic() + .05)      

        elif type =='K':           #CARD SIMULATION
            self._write_rc52x(TxModeReg,0x80)        #12h,
            self._write_rc52x(RxModeReg,0x80)        #13h,
            self._write_rc52x(RxThresholdReg,0x55)   #18h,
            self._write_rc52x(DemodReg,0x61)         #19h,     
            self._write_rc52x(MifareReg,0x62)        #1ch, 
            self._write_rc52x(GsNOffReg,0xf2)        #/23h       
            self._write_rc52x(TxBitPhaseReg,0x87)    #25h    
            self._write_rc52x(RFCfgReg,0x59)         #26h    
            self._write_rc52x(GsNOnReg,0x6F)         #27h
            self._write_rc52x(CWGsPReg,0x3f)         #28h      
            self._write_rc52x(ModGsPReg,0x3A)        #29h          
            self._write_rc52x(TxSelReg,0x17)         #16h 
            self._write_rc52x(FIFOLevelReg,0x80)

        elif (type == 'L'):
            self._write_rc52x(TxModeReg,0x92)        #12h 
            self._write_rc52x(RxModeReg,0x92)        #13h     
            self._write_rc52x(TxSelReg,0x17)         #16h  
            self._write_rc52x(RxThresholdReg,0x55)   #18h,          
            self._write_rc52x(DemodReg,0x61)         #19h         
            self._write_rc52x(RFU1A,0x00)            #1Bh 
            self._write_rc52x(ManualRCVReg,0x08)
            self._write_rc52x(RFU1B,0x80)            #1Bh         
            self._write_rc52x(GsNOffReg,0xf2)        #23h 
            self._write_rc52x(RFCfgReg,0x59)         #26h
            self._write_rc52x(GsNOnReg,0x6f)         #27h
            self._write_rc52x(CWGsPReg,0x3f)         #28h     
            self._write_rc52x(ModGsPReg,0x3a)        #29h
            self._write_rc52x(FIFOLevelReg,0x80)

        else:
            return MI_ERR
        return MI_OK
    
    # 寻卡
    def rc52x_pcdRequestA(self, req_code, pTagType):
        self._rc52x_cleanBitMask(TxModeReg,0x80)      #12h,    //TxCRCEn
        self._rc52x_cleanBitMask(RxModeReg,0x80)      #13h,
        self._write_rc52x(TReloadRegL,0x59)      #2dh,
        self._write_rc52x(TReloadRegH,0x0A)      #2ch,
        self._rc52x_cleanBitMask(Status2Reg,0x08)     #08h
        self._write_rc52x(BitFramingReg,0x07)    #0dh
        self._rc52x_setBitMask(TxControlReg,0x03)     #14h     0x03

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

    # 防冲撞处理。level：防撞级别；pSnr：UID指针；    
    def rc52x_pcdAnticoll(self, level, pSnr):
        ucBits = ucBytes = ucCollPosition = 0
        ucTemp = snr_check = 0
        ucSNR = [0,0,0,0,0]
        status = 0

        self._write_rc52x(BitFramingReg,0x00)       
        self._rc52x_cleanBitMask(CollReg,0x80)             
        self._rc52x_cleanBitMask(TxModeReg,0x80)
        self._rc52x_cleanBitMask(RxModeReg,0x80)

        while True:
            ucBits = ucCollPosition % 8
            t_ucCollPosition = ucCollPosition // 8
            ucBytes = t_ucCollPosition
            if (ucBits != 0):
                ucBytes = t_ucCollPosition + 1
                self._write_rc52x(BitFramingReg, (ucBits << 4) + ucBits)
            
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
        
        self._rc52x_setBitMask(CollReg,0x80)
        return status

    # 选卡 level：防撞级别；pSnr：id；pSize：长度；
    def rc52x_pcdSelect(self, level, pSnr):
        self._write_rc52x(TxModeReg,0x80)          #12h        TxCRCEn
        self._write_rc52x(RxModeReg,0x80)          #13h

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

    # 读取A卡并进行防撞处理。patqa; puid：ID; plen：ID长度;  psak：类型
    def rc52x_pcdActivateA(self, patqa, puid):
        plen = psak = 0
        self.rc52x_pcdConfigISOType('A')
        self.reactor.pause(self.reactor.monotonic() + .05)
        status = self.rc52x_pcdRequestA(PICC_REQALL, patqa)  #PICC_REQALL,patqa = 0X4400; PICC_REQIDL

        #logging.info("ATQA: status=%d" % (status))
        if (status == MI_OK):
            logging.info("rc52x: card type, pata = 0x%02X%02X" % (patqa[0], patqa[1]))
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

    # 密钥认证函数。auth_mode：认证模式；sector：要认证的扇区号；pKey：密钥指令；pSnr：uid
    def rc52X_pcdAuthState_ntag(self, pKey):
        self.pi['MfCommand']= RC52X_TRANSCEIVE 
        self.pi['MfLength'] = 5
        self.pi['MfData']   = [0x1b]
        self.pi['MfData'].extend(pKey[:4])

        status = self.rc52x_pcdComTransceive(self.pi)
        return status

    def rc52X_pcdAuthState(self, auth_mode, sector, pKey, pSnr):
        self.pi['MfCommand']= RC52X_AUTHENT
        self.pi['MfLength'] = 12
        self.pi['MfData']   = [auth_mode, sector]
        self.pi['MfData'].extend(pKey[:6])
        self.pi['MfData'].extend(pSnr[:4])

        status = self.rc52x_pcdComTransceive(self.pi)
        if (status == MI_OK): 
            reg = self._read_rc52x(Status2Reg, 1)
            if(not (reg[0] & 0x08)):
                status = MI_AUTHERR
        return status

    #读指定扇区。sector：扇区号；pWritedata：数据指针；
    def rc52x_pcdRead(self, sector, pReaddata):
        self.pi['MfCommand']= RC52X_TRANSCEIVE
        self.pi['MfLength'] = 2
        self.pi['MfData']   = [PICC_READ, sector]

        status = self.rc52x_pcdComTransceive(self.pi)
        if (status == MI_OK):
            if (self.pi['MfLength'] != 0x80):
                status = MI_BITCOUNTERR
            else:
                pReaddata.extend(self.pi['MfData'])
        return status

    #写指定扇区。sector：扇区号；pWritedata：数据指针；
    def rc52x_pcdWrite(self, sector, pWritedata):
        self.pi['MfCommand']= RC52X_TRANSCEIVE
        self.pi['MfLength'] = 2
        self.pi['MfData']   = [PICC_WRITE, sector]

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

    #读取A卡返回UID。UID_a，len：需要返回数据的指针；
    def rc52x_JewelTransceive(self, pi):
        n = status = count = 0
        irqEn   = 0x77
        waitFor = 0x30    
        self._write_rc52x(CommandReg, RC52X_IDLE)
        self._rc52x_setBitMask(FIFOLevelReg, 0x80)
        self._write_rc52x(ComIrqReg, 0x7F)  
        self._write_rc52x(DivIrqReg, 0x7F)
        self._write_rc52x(ManualRCVReg, 0x10)

        d_len = pi['MfLength']
        self._write_rc52x(FIFODataReg, pi['MfData'][:d_len])

        self._write_rc52x(ComIEnReg, irqEn|0x80)
        self._write_rc52x(CommandReg, pi['MfCommand'])
        self._rc52x_setBitMask(BitFramingReg, 0x80)
        
        while True:
            temp = self._read_rc52x(ComIrqReg, 1)      #wait for TxIRQ
            if (temp[0] & 0x40) != 0 :
                break
            count += 1
            if count > 1000 :
                logging.info("rc52x_JewelTransceive: wait for TxIRQ timeout")
                return MI_ERR

        if pi['MfCommand'] == RC52X_TRANSMIT:
            temp = self._read_rc52x(ErrorReg, 1)
            status = MI_OK if temp[0] == 0 else -1
        
        if pi['MfCommand'] == RC52X_TRANSCEIVE:
            self._write_rc52x(ManualRCVReg, 0x00)
            
            for i in range(1000):
                n = self._read_rc52x(ComIrqReg, 1)
                if (n[0] & irqEn & 0x01) or (n[0] & waitFor):
                     break  # timerirq idleirq rxirq
        
            if n[0] & waitFor:       #IdleIRq
                temp = self._read_rc52x(ErrorReg, 1)
                if temp[0] & 0x1b:      #06h    
                    status = MI_ERR
                else:
                    status = MI_OK
                    if (pi['MfCommand'] == RC52X_TRANSCEIVE):
                        n = self._read_rc52x(FIFOLevelReg, 1)         #0ah
                        lastBits = self._read_rc52x(ControlReg, 1)    #0ch
                        lastBits[0] &= 0x07
                        pi['MfLength'] = ((n[0] - 1)*8 + lastBits[0]) if lastBits[0] else (n[0] * 8)
                        if n[0] == 0:
                            n[0] = 1
                        if n[0] > MAXRLEN:
                            n[0] = MAXRLEN 
                        pi['MfData'] = self._read_rc52x(FIFODataReg, n[0]);  #09h    
            elif (n[0] & irqEn & 0x01):    #TimerIRq
                status = MI_NOTAGERR
            else :
                temp = self._read_rc52x(ErrorReg, 1)
                temp[0] &= 0x1B
                if not temp[0]:
                    temp = self._read_rc52x(ErrorReg, 1)
                    status = MI_ACCESSTIMEOUT #-27
        return status

    def rc52x_simulate_Card(self):
        CardConfig = [0x04,0x00,0xa1,0xa2,0xa3,0x11,0x01,0xfe,0xb2,0xb3,0xb4,0xb5,
                        0xb6,0xb7,0xc0,0xc1,0xc2,0xc3,0xc4,0xc5,0xc6,0xc7,0x23,0x45,0xfa]
        
        self.rc52x_pcdConfigISOType('K')  

        self._write_rc52x(FIFODataReg, CardConfig)
        
        self._write_rc52x(CommandReg, RC52X_MEM)
        self.reactor.pause(self.reactor.monotonic() + .25)
        self._write_rc52x(CommandReg, RC52X_AUTOCOLL)

    def rc52x_simulate_CardF(self):
        CardConfig = [0x10,0xfe,0x01,0x12,0x01,0x24,0x69,0x0b,0x50,0x37,0x2E,0x3d,
                        0x24,0x69,0x0b,0x50,0x37,0x2E,0x3d,0x24,0x69,0x0b,0x50,0x37,0x0b] 
        self.rc52x_pcdConfigISOType('L')
        self.reactor.pause(self.reactor.monotonic() + .05)
        self._write_rc52x(FIFODataReg, CardConfig)                                                       
        
        self.reactor.pause(self.reactor.monotonic() + .1)
        self._write_rc52x(CommandReg, RC52X_MEM)
        self._write_rc52x(CommandReg, RC52X_AUTOCOLL)

    #发送暂停命令给卡。 addr：扇区号；pWritedata：数据指针；   
    def rc52x_pcdHaltA(self):
        self.pi['MfCommand']= RC52X_TRANSCEIVE
        self.pi['MfLength'] = 2
        self.pi['MfData']   = [PICC_HALT, 0]

        status = self.rc52x_pcdComTransceive(self.pi)
        if status:
            if (status == MI_NOTAGERR) or (status == MI_ACCESSTIMEOUT):
                status = MI_OK
        self._write_rc52x(CommandReg, RC52X_IDLE)
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
            self._write_rc52x(BitFramingReg, dat)

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

    def rc52x_setBaudrate(self, txrate, rrate):
        tempts = self._read_rc52x(TxModeReg, 1)
        temprs = self._read_rc52x(RxModeReg, 1)
        tempt = tempts[0]
        tempr = temprs[0]

        if txrate == 1: # 212K
            self._write_rc52x(TxModeReg,tempt&0x9f) #2fh,
            self._write_rc52x(TxModeReg,tempt|0x10)
            self._write_rc52x(ModWidthReg,0x10)

        elif txrate == 2: # 424K
            self._write_rc52x(TxModeReg,tempt&0xaf) #2fh,
            self._write_rc52x(TxModeReg,tempt|0x20)
            self._write_rc52x(ModWidthReg,0x07)  

        elif txrate == 3: # 848K
            self._write_rc52x(TxModeReg,tempt&0xbf) #2fh,
            self._write_rc52x(TxModeReg,tempt|0x30)
            self._write_rc52x(ModWidthReg,0x02)  

        else: # 106K
            self._write_rc52x(TxModeReg,tempt&0x8f); #2fh,
            self._write_rc52x(ModWidthReg,0x26); 

        if rrate == 1: # 212K
            self._write_rc52x(RxModeReg,tempr&0x9f) #2fh,
            self._write_rc52x(RxModeReg,tempr|0x10)

        elif rrate == 2: # 424K
            self._write_rc52x(RxModeReg,tempr&0xaf) #2fh,
            self._write_rc52x(RxModeReg,tempr|0x20)

        elif rrate == 3: # 848K
            self._write_rc52x(RxModeReg,tempr&0xbf) #2fh,
            self._write_rc52x(RxModeReg,tempr|0x30)
        else: # 106K
            self._write_rc52x(RxModeReg,tempr&0x8f) #35h,

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

    def rc52x_pcdMfulRead(self, sector, pReaddata):
        self._rc52x_setBitMask(TxModeReg,0x80) #12h,    //TxCRCEn
        self._rc52x_setBitMask(RxModeReg,0x80) #13h,

        self.pi['MfCommand']= RC52X_TRANSCEIVE
        self.pi['MfLength'] = 2
        self.pi['MfData']   = [PICC_READ, sector]
        status = self.rc52x_pcdComTransceive(self.pi) 
        if (status == MI_OK):
            if (self.pi['MfLength'] != 0x80):
                status = MI_BITCOUNTERR #-11
            else:
                pReaddata.extend(self.pi['MfData'])
        return status

    def rc52X_CPU_I_Block(self, psbuf, slen, prbuf):
        prlen = 0
        self._write_rc52x(TReloadRegL,0x0B)  #2dh,        //30    
        self._write_rc52x(TReloadRegH,0x41) 

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
                logging.info("rc52x: uid not complete")
                return r_len
            if (sak & 0x20):    #compliant with ISO-14443-4  
                RD_Data = [] 
                status,ulen = self.rc52x_ratsA(RD_Data)
                # out_data = "".join(f"{item:2d}" for item in RD_Data)
                # logging.info("rc52x: Rats, status=%d, read=%s, ulen=%d" % (status,out_data,ulen))
                if(status != MI_OK):
                    return r_len
                
                if ((RD_Data[1] & 0x10) and ((RD_Data[2] & 0x44) == 0x44)):    #TA(1) 848k supported    //mifare desfire    
                    status,sak = self.rc52x_ppsA(0x0f)  #106:0x00   212:0x05  424:0x0a  848:0x0f
                    logging.info("rc52x: Pps, status=%d, sak=%d" % (status,sak))
                    if(status != MI_OK):
                        return r_len               
                    self.rc52x_setBaudrate(3,3)        #106：0，0  212：1，1   424：2，2   848：3，3   
                    uid = [] 
                    status,ulen = self.rc52X_CPU_I_Block(ver1, 1, uid)
                    out_data = "".join(f"{item:2d}" for item in uid)
                    logging.info("rc52x: ver1, status=%d, uid=%s, ulen=%d" % (status,out_data,ulen))
                    if (status != MI_OK):
                        return r_len
                    self._write_rc52x(ModWidthReg,0x26)               
                    self.rc52x_setBaudrate(0,0)                
                else:
                    uid = []
                    status,ulen = self.rc52X_CPU_I_Block(random, 5, uid)
                    out_data = "".join(f"{item:2d}" for item in uid)
                    logging.info("rc52x: random, status=%d, uid=%s, ulen=%d" % (status,out_data,ulen))
                    if (status != MI_OK): 
                        return r_len

            elif (ulen == 4):    #not compliant with ISO-14443-4    
                if (sak == 0x08) and (atqa[0] == 0x04):
                    logging.info("rc52x: mifare S50 has detected!")
                elif (sak == 0x18) and (atqa[0] == 0x02):
                    logging.info("rc52x: mifare S70 has detected!")

                status = self.rc52X_pcdAuthState(KEY_MODE_A, 0x01, KEY_A, uid)
                logging.info("rc52x: AUTH %d " % (status))

                RD_Data = [] 
                status = self.rc52x_pcdRead(0,RD_Data)  #读 0 页扇区
                out_data = "".join(f"{item:2d}" for item in RD_Data)
                logging.info("rc52x_read: status=%d, data=%s, ulen=%d" % (status,out_data,16))

                if (status!=MI_OK):
                    return r_len
            else:  
                RD_Data = [] 
                #status = self.rc52x_pcdMfulRead(3,RD_Data)
                status = self.rc52x_pcdRead(3,RD_Data)  #读 3 页扇区
                out_data = " ".join(f"{item:02x}" for item in RD_Data)
                logging.info("rc52x_read: sector=3, status=%d, data=%s, ulen=%d" % (status,out_data,16))
                if (status != MI_OK):
                    return r_len
        return r_len

    def cmd_rc52x_testAuthWriteData(self, gcmd):
        send_data = [0xe0,0xe1,0xe2,0xe3,0,0,0,0,0,0,0,0,0,0,0,0]
        pwd_data = [0xe0, 0xe1, 0xe2, 0xe3, 0, 0] # pwd*[0:4] + pack*[4:6]
        status = self.rc52x_writeSectorData(0x06, send_data, PWD_KEY)
        if status != MI_OK:
            self.gcode.respond_info('test Auth Write Data failed!!!')
            return -1
        sector_data = self.rc52x_readSectorData(0x06) # 读取 page 3 byte 2容量,确定类型
        sector_list = [format(num, '02x') for num in sector_data]
        sector_str = ' '.join(map(str,sector_list)) 
        self.gcode.respond_info('Auth Write Data:{}'.format(sector_str))
    
    def get_ntag21x_crc(self,in_buffer, in_len):
        w_crc = 0x6363
        i = 0
        while in_len > 0:
            bt = in_buffer[i]
            bt = (bt ^ (w_crc & 0x00FF))                                                     
            bt = (bt ^ (bt << 4))                                                   
            w_crc = (w_crc >> 8) ^ ( bt << 8) ^ ( bt << 3) ^ ( bt >> 4)
        output = [0,0]
        output[0] = w_crc & 0xFF                                                     
        output[1] = (w_crc >> 8) & 0xFF
        return output

    def cmd_rc52x_wirtePWDConfig(self, gcmd):
        t_data = [0,0,0,0,0,0,0,0,0,0,0,0]
        w_data_list = []
        sector_data = []

        # get write addr
        sector_data = self.rc52x_readSectorData(sectorAddr_map[0]) # 读取 page 3 byte 2容量,确定类型
        if len(sector_data) < 1:
            self.gcode.respond_info('write failed: get sector 0 failed!!!')
            return 
        mem_size = sector_data[14]
        if mem_size == 0x12:
            pwd_page_addr = KEY_SECTOR['NTAG213_PWD']
            self.gcode.respond_info('card is NTAG213!!!')
        elif mem_size == 0x3e:
            pwd_page_addr = KEY_SECTOR['NTAG215_PWD']
            self.gcode.respond_info('card is NTAG215!!!')
        elif mem_size == 0x6d:
            pwd_page_addr = KEY_SECTOR['NTAG216_PWD']
            self.gcode.respond_info('card is NTAG216!!!')
        else:
            self.gcode.respond_info('card not NTAG213/215/216, mem_size={}!!!'.format(mem_size))
            return
        access_page_addr = pwd_page_addr - 1
        auth_page_addr = pwd_page_addr - 2

        sector_data = []
        sector_data = self.rc52x_readSectorData(auth_page_addr) # 读取 CFG0 CFG1 PWD 信息
        if len(sector_data) < 1:
            self.gcode.respond_info('write failed: get sector 0 failed!!!')
            return 
        sector_list = [format(num, '02x') for num in sector_data]
        sector_str = ' '.join(map(str,sector_list)) 
        self.gcode.respond_info('cfg and pwd:{}'.format(sector_str))
        
        #  PWD attempts - unlimited attempts page2a/84/e4h index 0
        w_data_list = []
        pwd_attempts_data = gcmd.get_int('PWD_ATTEMPTS', 0)
        w_data_list = [pwd_attempts_data] + sector_data[5:8]
        # crc_data = [0x1b] + w_data_list
        # pack = self.get_ntag21x_crc(crc_data, 5)
        send_data = w_data_list + [0,0,0,0,0,0,0,0,0,0,0,0]
        pwd_data = [0,0,0,0]
        status = self.rc52x_writeSectorData(access_page_addr, send_data, pwd_data)
        if status != MI_OK:
            self.gcode.respond_info('write pwd attempts failed!!!')
            return -1

        # write pwd
        w_data_list = []
        pwd_data_i = gcmd.get_int('PWD', 0xE0E1E2E3)
        w_data_list = self._rc52x_convertIntData(pwd_data_i, isToInt=False)
        send_data = w_data_list + [0,0,0,0,0,0,0,0,0,0,0,0]
        status = self.rc52x_writeSectorData(pwd_page_addr, send_data, pwd_data)
        if status != MI_OK:
            self.gcode.respond_info('write pwd failed!!!')
            return -1
        
        pwd_data = w_data_list

        # Write the starting address that requires PWD  page29/83/e3h index 3
        pwd_start_addr_data = gcmd.get_int('PWD_START_ADDR', 0)
        # w_data_list = self._rc52x_convertIntData(pwd_start_addr_data, isToInt=False)
        w_data_list = sector_data[0:3] + [pwd_start_addr_data]
        send_data = w_data_list + [0,0,0,0,0,0,0,0,0,0,0,0]
        status = self.rc52x_writeSectorData(auth_page_addr, send_data, pwd_data)
        if status != MI_OK:
            self.gcode.respond_info('write pwd start addr failed!!!')
            return -1

        self.gcode.respond_info('card is write PWD success!!!')

    
    def cmd_rc52x_writeSectorInitInfo(self, gcmd):
        t_data = [0,0,0,0,0,0,0,0,0,0,0,0]
        # # 第 1 扇区 0块：数据头, 1块：制造商编号, 2块：灯丝编号, 3块：生产日期
        header              = gcmd.get_int('HEADER', 255)
        manufacturer_code   = gcmd.get('MFC', '00000000')
        filament_code       = gcmd.get('FC', '0000') + '0000'
        date_t              = gcmd.get_int('DATE', 202508)

        h_data = self._rc52x_convertIntData(header, isToInt=False)
        write_data = h_data + t_data
        status = self.rc52x_writeSectorData(0x04, write_data, PWD_KEY)

        mfc_data = self._rc52x_convertToHexStrData(manufacturer_code, isToString=False)
        write_data = mfc_data + t_data
        status = self.rc52x_writeSectorData(0x05, write_data, PWD_KEY)

        fc_data = self._rc52x_convertToHexStrData(filament_code, isToString=False)
        write_data = fc_data + t_data
        status = self.rc52x_writeSectorData(0x06, write_data, PWD_KEY)

        year_data = [0,0,0,0]
        year_i = (date_t // 100)
        month_i = (date_t % 100)
        year_data[0] = (year_i & 0xff00) >> 8
        year_data[1] = (year_i & 0x00ff) 
        year_data[2] = (month_i & 0xff00) >> 8
        year_data[3] = (month_i & 0x00ff) 
        write_data = year_data + t_data
        status = self.rc52x_writeSectorData(0x07, write_data, PWD_KEY)
        if status != MI_OK:
            self.gcode.respond_info('write sector1 failed!!!')
            return -1

        # 第 2 扇区 0块：材质, 1块：材质补充, 2块：颜色, 3块：直径，密度
        material_name = gcmd.get('NAME', 'PLA')
        material_sup = gcmd.get('SUP', '0000')
        color_code = gcmd.get('COLOR', 'FF3700')
        filament_diameter = gcmd.get_int('DIAMETER', 0x00AF)
        density = gcmd.get_int('DENSITY', 0x007C)
        material_name_data = self._rc52x_convertCharStrData(material_name, isToString=False, del_tail0=True)
        write_data = material_name_data + t_data
        status = self.rc52x_writeSectorData(0x08, write_data, PWD_KEY)

        material_sup_data = self._rc52x_convertCharStrData(material_sup, isToString=False, del_tail0=True)
        write_data = material_sup_data + t_data
        status = self.rc52x_writeSectorData(0x09, write_data, PWD_KEY)

        color_code_data = self._rc52x_convertToHexStrData(color_code, isToString=False)
        write_data = color_code_data + t_data
        status = self.rc52x_writeSectorData(0x0a, write_data, PWD_KEY)

        diameter_density_data = self._rc52x_convertIntData((filament_diameter << 16) | density, isToInt=False)
        write_data = diameter_density_data + t_data
        status = self.rc52x_writeSectorData(0x0b, write_data, PWD_KEY)

        if status != MI_OK:
            self.gcode.respond_info('write sector2 failed!!!')
            return -1

        # 第 3 扇区 0块：温度范围, 1块：
        min_temp = gcmd.get_int('MIN_TEMP', 0x00BE)
        max_temp = gcmd.get_int('MAX_TEMP', 0x00E6)
        temp_data = self._rc52x_convertIntData((min_temp << 16) | max_temp, isToInt=False)

        write_data = temp_data + t_data
        status = self.rc52x_writeSectorData(0x0c, write_data, PWD_KEY)
        if status != MI_OK:
            self.gcode.respond_info('write sector3 failed!!!')
            return -1

        # # 第 4 扇区 0块：剩余重量(g)
        filament_weight = gcmd.get_int('WEIGHT', 0x03E8)
        filament_weight_data = self._rc52x_convertIntData(filament_weight, isToInt=False)
        write_data = filament_weight_data + t_data
        status = self.rc52x_writeSectorData(0x10, write_data, PWD_KEY)
        if status != MI_OK:
            self.gcode.respond_info('write sector4 failed!!!')
            return -1
        self.gcode.respond_info('write successed!!!')

    
    def cmd_rc52x_writeSectorInitInfo_back(self, gcmd):
        t_data = [0,0,0,0,0,0,0,0,0,0,0,0]
        # # 第 1 扇区 0块：数据头, 1块：制造商编号, 2块：灯丝编号, 3块：生产日期
        header              = gcmd.get_int('HEADER', 255)
        manufacturer_code   = gcmd.get_int('MFC', 0xffffffff)
        filament_code       = gcmd.get_int('FC', 0xffff)
        date_t              = gcmd.get_int('DATE', 2508)

        h_data = self._rc52x_convertIntData(header, isToInt=False)
        write_data = h_data + t_data
        status = self.rc52x_writeSectorData(0x04, write_data, PWD_KEY)

        mfc_data = self._rc52x_convertIntData(manufacturer_code, isToInt=False)
        write_data = mfc_data + t_data
        status = self.rc52x_writeSectorData(0x05, write_data, PWD_KEY)

        fc_data = self._rc52x_convertIntData(filament_code, isToInt=False)
        write_data = fc_data + t_data
        status = self.rc52x_writeSectorData(0x06, write_data, PWD_KEY)

        year_data = [0,0,0,0]
        year_i = (date_t // 100)
        month_i = (date_t % 100)
        year_data[0] = (year_i & 0xff00) >> 8
        year_data[1] = (year_i & 0x00ff) 
        year_data[2] = (month_i & 0xff00) >> 8
        year_data[3] = (month_i & 0x00ff) 
        write_data = year_data + t_data
        status = self.rc52x_writeSectorData(0x07, write_data, PWD_KEY)
        if status != MI_OK:
            self.gcode.respond_info('write sector1 failed!!!')
            return -1

        # 第 2 扇区 0块：材质, 1块：材质补充, 2块：颜色, 3块：直径，密度
        material_name = gcmd.get_int('NAME', 0x00504c41)
        material_sup = gcmd.get_int('SUP', 0x00004346)
        color_code = gcmd.get_int('COLOR', 0xFF3700)
        filament_diameter = gcmd.get_int('DIAMETER', 0x00AF)
        density = gcmd.get_int('DENSITY', 0x007C)
        material_name_data = self._rc52x_convertIntData(material_name, isToInt=False)
        write_data = material_name_data + t_data
        status = self.rc52x_writeSectorData(0x08, write_data, PWD_KEY)

        material_sup_data = self._rc52x_convertIntData(material_sup, isToInt=False)
        write_data = material_sup_data + t_data
        status = self.rc52x_writeSectorData(0x09, write_data, PWD_KEY)

        color_code_data = self._rc52x_convertIntData(color_code, isToInt=False)
        write_data = color_code_data + t_data
        status = self.rc52x_writeSectorData(0x0a, write_data, PWD_KEY)

        diameter_density_data = self._rc52x_convertIntData((filament_diameter << 16) | density, isToInt=False)
        write_data = diameter_density_data + t_data
        status = self.rc52x_writeSectorData(0x0b, write_data, PWD_KEY)
        if status != MI_OK:
            self.gcode.respond_info('write sector2 failed!!!')
            return -1

        # 第 3 扇区 0块：温度范围, 1块：
        min_temp = gcmd.get_int('MIN_TEMP', 0x00BE)
        max_temp = gcmd.get_int('MAX_TEMP', 0x00E6)
        temp_data = self._rc52x_convertIntData((min_temp << 16) | max_temp, isToInt=False)

        write_data = temp_data + t_data
        status = self.rc52x_writeSectorData(0x0c, write_data, PWD_KEY)
        if status != MI_OK:
            self.gcode.respond_info('write sector3 failed!!!')
            return -1

        # # 第 4 扇区 0块：剩余重量(g)
        filament_weight = gcmd.get_int('WEIGHT', 0x03E8)
        filament_weight_data = self._rc52x_convertIntData(filament_weight, isToInt=False)
        write_data = filament_weight_data + t_data
        status = self.rc52x_writeSectorData(0x10, write_data, PWD_KEY)
        if status != MI_OK:
            self.gcode.respond_info('write sector4 failed!!!')
            return -1

        self.gcode.respond_info('write successed!!!')

    def cmd_rc52x_readInfo(self, gcmd):
        lane = gcmd.get_int('LANE', None)
        lane_name = "lane{}".format(lane)
        if lane_name not in self.AFC.lanes :
            self.gcode.respond_info('LANE={} is Unknown'.format(lane))
            return
        
        #选择dev地址
        self._set_current_obj(lane)

        status = self.rc52x_readCardAllInfo(lane)
        status_str = 'Success' if status == 0 else 'Failed'
        self.gcode.respond_info('{} to read NFC card data from LANE'.format(status_str))
        

def load_config_prefix(config):
    return NFC_RC52X(config)    
