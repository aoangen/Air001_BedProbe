# Air001_BedProbe

3D打印机压力热床，主控MCU使用合宙Air001，可以使用HX71708、CS1237两种AD转换芯片



> 芯片可选数据速率
> 
> HX71708 **10、20、80、320**
> 
> CS1237 **10、40、640、1280**



![截图_2023-10-23_14-26-45.png](https://s2.loli.net/2023/10/23/9xsdfjF1c4yCkgL.png)



测试条件：CS1237 1280Hz，5mm/s速度，皮带Z 80：16减速比，触发重量150g
连续10次测试结果：

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
