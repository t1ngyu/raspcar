# 树莓派小车

## 硬件环境

* 底盘

    底盘带四个直流电机；

    ![img](images/car.jpg)


* 电机驱动

    每个L298N可驱动两个电机，共需要两块；

    ![img](images/L298N.jpg)


* 码盘计数器

    光电计数器，每个电机一个，共四个；

    ![img](images/counter.jpg)


* 直流电机电源

    * 电池

        使用3节18650锂电池串联作为电机的电源，输出电压11.1v（3.7*3）；

        ![img](images/18650.jpg)

    * 电池盒

        ![img](images/batterypack.jpg)


* 控制板

    使用一块STM32板子来控制板电机和各个传感器，通过USB接口与树莓派连接，可灵活控制电机启动时序、保证稳定的控制周期、减少树莓派的IO口使用；

    ![img](images/stm32f103c8t6.jpg)

* 树莓派

    ![img](images/raspberry-3b+.jpg)

* 树莓派电源

    使用充电宝给树莓派供电；


## 整车效果图


![img](images/all0.jpg)

![img](images/all1.jpg)

![img](images/all2.jpg)

![img](images/all3.jpg)
