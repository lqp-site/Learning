# Mifare Ultra Light 非接触式IC卡

概述：UltraLight卡简单地看成是一种存储介质，对它的操作也就是对扇区中每个Page的读取和写入的过程

```C
/*
1、  容量512bit，分为16个page，每个page占4byte
2、  每个page可以通过编程的方式锁定为只读功能
3、  384位(从page4往后)用户读写区域
4、  唯一7字节物理卡号（page0前3个byte加page1）
*/
```

存储结构：

| 页号/地址 | byte0 | byte0 | byte0 | byte0 | 说明                                                         |
| :-------: | ----- | ----- | ----- | ----- | ------------------------------------------------------------ |
|     0     | SN0   | SN1   | SN2   | SBCC0 | 只读，存放**卡**的序列号：Page0前3字节+整个Page1             |
|     1     | SN3   | SN4   | SN5   | SN6   |                                                              |
|     2     | BCC1  | 保留  | LOCK0 | LOCK1 | 只读，通过设置LOCK0和LOCK1可以讲16个page设为只读             |
|     3     | OTP0  | OTP1  | OTP2  | OTP3  | 可读写，一次性交易计数器，不可逆。用来记录卡片已经完成的交易次数，交易完成后计数器会自动递增。 |
|     4     | Data0 | Data1 | Data2 | Data3 | 余下为数据存放区域，可读写                                   |
|     5     | Data0 | Data1 | Data2 | Data3 |                                                              |
|     6     | Data0 | Data1 | Data2 | Data3 |                                                              |
|     7     | Data0 | Data1 | Data2 | Data3 |                                                              |
|     8     | Data0 | Data1 | Data2 | Data3 |                                                              |
|     9     | Data0 | Data1 | Data2 | Data3 |                                                              |
|    10     | Data0 | Data1 | Data2 | Data3 |                                                              |
|    11     | Data0 | Data1 | Data2 | Data3 |                                                              |
|    12     | Data0 | Data1 | Data2 | Data3 |                                                              |
|    13     | Data0 | Data1 | Data2 | Data3 |                                                              |
|    14     | Data0 | Data1 | Data2 | Data3 |                                                              |
|    15     | Data0 | Data1 | Data2 | Data3 |                                                              |

