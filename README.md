# HLL Super Hero



## Hll 2022 Super Hero 工程日志


- [x] 模块检测任务
- [x] 串口 1 DMA 双缓冲接收
- [x] 遥控器任务完成
- [x] 增加并完成 CAN1 数据解析任务（已废弃不用，底盘数据需要实时解析）
- [x] 增加并完成 CAN2 数据解析（待删除）
- [x] 增加机器人模式
- [x] 增加遥控器进一步的数据解析
- [x] 改进 printf 使用 Mutex 打印
- [x] 云台角度控制
- [x] can 发送队列（废弃不用）
- [x] 蜂鸣器任务
- [x] 云台发射电机速度初步设置
- [x] 底盘基本运动控制
- [x] 陀螺仪数据读取四元数解析输出三轴角度
- [ ] 卡尔曼滤波数据解析
- [ ] 超级电容管理
- [ ] 底盘完整的模式切换
- [ ] 裁判系统数据读取解析
- [ ] 自瞄任务
- [ ] ui 界面
- [ ] 显示屏
- [ ] 专门配合英雄或者步兵的 QT 上位机界面




## Problem

- [ ] 注意最好不要在任务中接收 CAN 的数据，经测试会卡死，但也不一定，只是我出现了这个问题

- [ ] 出现了莫名其妙的 undefined reference to Rc_Ctrl_t ，虽然后面不报了，但仍没找出 bug

- [x] CAN 在使用 HAL 库的时候，如果不接收 CAN 的数据，会出现任务卡死

  ```c++
  HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, can2_rx_header, can2_rxd_data_buffer);
  ```
  **`该问题已解决，由于语法出错，在接收函数中传入了一个空地址导致该错误的发生`**

- [ ] 注意不要在中断中随便打印数据，否则可能会造成死锁。

## Advanced

- 基于 Mutex 的 printf 打印，避免操作系统使用 printf 时出现的死锁以及任务优先级翻转等问题。

- 模仿 JavaScript 使用的 Console.print() 、Console.error()、Console.log() 提高程序的规范化（其实就是一个函数指针结构体的使用，看起来高级了点）。

- 使用板载陀螺仪，配合四元数进行解析。

- 待增加 Kalman Filter 进行滤波融合。

- 基于回调函数（函数指针）、链表和无线串口的在线 CMD。

- 专门配合于英雄或者步兵的基于 QT 的上位机界面。

- 图传UI界面设计

  瞄准镜风格的十字横线，全车调试完成之后测试UI，以中心为界，横线表示不同距离下的射击位置。

  

  
  
  



