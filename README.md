`StartinPID`中速度环的`Actual`应该是电机报文的`data2`，且未知电机ID。

`gimbal.c`:初始将内环pitch和yaw分别放在`data1`和`data2`

`chassis.c`:接收的来源初定为CAN2

`gimbal.c`中`void StartexPIDTask(void *argument)`暂定视觉传回的欧拉角范围-180~180

`chassis.c`中chassis外环绝对角度的大小等于云台yaw轴角度

`rc.c`中根据返回的通道值映射为电流值，平动和转动分开实现。而`chassis.c`中进行底盘的姿态结算，需要提供目标速度，目标角度,目标角速度。

`HAL_CAN_RxFifo0MsgPendingCallback`里CAN1收到的数据具体存放到yaw还是pitch的Rxdata数组还不确定，因为不确定具体ID。
此前一直希望bsp层的代码和中层代码能解耦，所以bsp层的函数大多使用队列将数据发送到rtos任务中处理.

`chassis_CAN_RxTask`中获取的是转子转速(经减速箱后的最终输出轴转速)，怎么映射为底盘速度，将转子速度*sin45°？

`哨兵模式`还要优化