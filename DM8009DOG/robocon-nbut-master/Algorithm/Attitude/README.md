# 姿态解算算法
**from Zhiyuan Mao**

目前我们使用的一部分陀螺仪已经有了直接获得角度的功能，但是有的时候还是有一些特殊的需求，除了遇到老式的只能返回加速度角速度的陀螺仪(MPU6050)，有的时候其他算法中会用到四元数。因此我写了一些姿态解算算法来解决这些问题，这些算法也将会使用在机械马和飞控上。

参考资料：

[AHRS姿态解算](https://blog.csdn.net/superfly_csu/article/details/79128460?utm_medium=distribute.pc_relevant.none-task-blog-2~default~baidujs_title~default-5.no_search_link&spm=1001.2101.3001.4242)

[四元数与欧拉角（Yaw、Pitch、Roll）的转换](https://blog.csdn.net/xiaoma_bk/article/details/79082629)

[陀螺仪加速度计6轴数据融合](https://blog.csdn.net/u010097644/article/details/70881395/?utm_medium=distribute.pc_relevant.none-task-blog-title-4&spm=1001.2101.3001.4242)