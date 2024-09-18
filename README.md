# [Air001_BedProbe](https://github.com/aoangen/Air001_BedProbe)

[获取硬件资料](https://oshwhub.com/aoang/air001_bedprobe)

3D打印机压力热床，主控使用PY32F002AF15P(普冉) Air001(合宙)  本质是同一款，Arduino开发环境由合宙提供，可以使用HX71708、CS1237(推荐)两种AD转换芯片

电路简单易于修改，可以很容易嵌入各种其它主板或项目电路中

新增5档LED进度条显示功能，触发进度转换成百分比，从30-100%控制LED点亮



> 芯片可选数据速率
> 
> HX71708 **10、20、80、320**
> 
> CS1237 **10、40、640、1280**



![截图_2023-10-23_14-26-45.png](https://s2.loli.net/2023/10/23/9xsdfjF1c4yCkgL.png)



> **触发逻辑**
> 
> 实时读取ADC芯片的压力传感器数据，并转换成压力值，速率最高1280Hz
> 
> 滤波后的压力值大于阈值即可触发两个输出引脚的电平变化，有最长时间限制，超过后更新阈值回到未触发状态
> 
> 当前的静态压力值+预设阈值=触发阈值，超过一定时间没有压力值突变，更新一次触发阈值
> 
> *压力值长时间的缓慢增减不会导致误触发，仅在当前压力值的基础上发生突变大于预设阈值触发*



测试条件：CS1237 1280Hz，5mm/s速度，皮带Z 80:20减速比，触发重量150g
连续10次重复精度测试结果：

| Range  | Deviation |
| ------ | --------- |
| 0.0025 | 0.001146  |
| 0.0025 | 0.001146  |
| 0.0050 | 0.001581  |
| 0.0050 | 0.001581  |
| 0.0025 | 0.001000  |
| 0.0075 | 0.001871  |
| 0.0050 | 0.002000  |
| 0.0050 | 0.001658  |
| 0.0050 | 0.001581  |
| 0.0050 | 0.001346  |

[klipper官方建议调平探针](https://www.klipper3d.org/zh/Probe_Calibrate.html)的重复精度范围在0.025以内，如果使用HX711，数据速率最高80Hz的情况下，只能降低速度勉强达到要求，高采样率的HX71708和CS1237可以显著提高重复精度以获得更好的探测数据

> 如果测试结果显示范围(range)值大于25微米（0.025毫米），那么探针不满足典型的床面调平流程的精确度要求。可以尝试调整探测速度和/或探测起点高度以提高探头的重复性。

![截图_2023-10-23_15-47-28.png](https://s2.loli.net/2023/10/23/CWjFhsDMmn9zIQR.png)
对比HX71708 80和320Hz的数据间隔，还有CS1237的数据间隔

## 下载说明

需要自备串口下载器，连接预留的串口排针插座进行下载

根据合宙官方教程配置开发环境[Air001基于Arduino的用户手册](https://wiki.luatos.com/chips/air001/Air001-Arduino.html)

**` [重点]下载时将主频设置为24MHz`**

![PixPin_2024-09-18_13-58-17.png](https://s2.loli.net/2024/09/18/EpPlBKAyr1HM8QU.png)

## 程序说明

> 程序都来自ChatGPT4编写

* 修改代码中` const int THRESHOLDS[] = { 50, 150, 300 }; `自定义触发阈值重量，boot按键在运行中可以用于切换。

* 串口波特率设置为115200，其它设置和调试都可以使用串口命令进行修改。

* 串口发送` HELP`获取帮助信息

* 使用命令:` SET SPEED 索引`，重启后生效，用于修改CS1237数据速率，降低可以得到更精准的重量，0=10,1=40,2=640,3=1280。

* 使用串口发送命令:` ADC 数值`，此值是一个校准因子，用于将 ADC（模数转换器）的读数转换为实际的重量，不同传感器结果不同，需要自行测试修改得到准确的称重结果。

* 使用命令开启串口输出`SERIAL`，则串口始终实时输出当前重量读数，使用已知重量物品称重，观察读数，修改校准因子 `ADC 209` 的数值，在默认209的基础上增减，直到读数接近所称重物品的实际重量。
  **`[重点]air001的性能不足以在640Hz以上采样率开启串口稳定运行，仅在调试时降低采样率使用，高采样率下务必关闭`**

* **调试说明：**
  
  * `SERIAL`命令开启实时重量读数输出，每100ms输出一次三个数据：重量、滤波后的重量、触发阈值。
  
  * 开启`SERIAL`输出后，可以使用Arduino IDE的串口绘图仪，将数据使用可视化图表展示。
  
  ![PixPin_2024-09-18_14-22-07.png](https://s2.loli.net/2024/09/18/EPnhGWaeXbNHZpq.png)
  
  * ` RECORD`命令开始记录读取重量的最高值和最低值，可以用于观察空载下读数浮动，或者触发力度。
  
  * `EMA`用于开启、关闭指数移动平均滤波器，此选项设置后不保存，默认开启

***

` 以上特性只有CS1237版本代码可用，HX71708已弃用，下方内容仅供参考，不建议使用`

HX711库不支持HX71708修改数据速率，HX71708只有一个通道，增益固定，通过代码修改数据速率；而HX711通过硬件修改速率，代码修改增益，所以需要修改hx711.cpp的库文件(给出的程序默认已修改好，可直接编译)
![截图_2023-10-23_15-59-30.png](https://s2.loli.net/2023/10/23/RGyBl7eSj8WkuN3.png)
![RTKw9UnKRZjVjXnvv6nNOltJxF7OJTKdrTqowNbj.png](https://s2.loli.net/2023/10/23/gKIcJoqEwVU5Pxk.png)
代码中scale.set_gain(64);设置增益，再把对应增益在,cpp库文件中的脉冲数改为4，即可实现对数据速率的控制

HX711程序以绝对压力值判断，压力值超过阈值即触发，每隔一段时间读数归零

都使用Boot键切换代码中的预设阈值

输出两个相反的高低电平信号，短接跳线处选择

建议使用CS1237，相对价格更低、性能更好，代码调试功能更丰富
