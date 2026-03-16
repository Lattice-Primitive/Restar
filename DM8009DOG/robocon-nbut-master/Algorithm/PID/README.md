#  PID算法
**from Zhiyuan Mao**
## 介绍
PID算法是一种闭环控制算法，被广泛运用于工业控制领域。也是我们最常用的一种算法。

关于PID算法的原理可以看看这篇博客就够了<https://blog.csdn.net/qq_25352981/article/details/81007075?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522163111025616780366587407%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=163111025616780366587407&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~top_positive~default-2-81007075.pc_search_ecpm_flag&utm_term=PID&spm=1018.2226.3001.4187>

## 使用方法
这里目前有两套PID算法，一个是其他学校的PID算法(pid.\*)，一个是学长之前写的PID算法(pid_old.\*)，后期我打算自己写一个比较简便的PID，并且增加电机上经常使用的双环PID(速度环、位置环)

冷知识：arm_math自带PID算法