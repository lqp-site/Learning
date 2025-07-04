1. 对 0.96 寸 oled 屏字体显示大小变更为24*24
2. 注释 aht10 温湿度初始化
3.  RC52x 驱动芯片读取 A 卡或 Ntag 卡

	```python
	# 配置
	[AFC_nfc nfc_1]
	i2c_mcu: Turtle_1        # mcu 名称
	bus_type: i2c            # 总线类型
	i2c_bus: i2c1_PB6_PB7    # i2c总线
	# auth_pwd: False        # 是否需要秘钥
	# pwd_key: 00000000      # 秘钥
	# NFC_TEST: True         # 轮询测试
	# rc52x_report_time: 10  # 间隔时间扫描
	# rc52x_number: 3
	```

	

​			
