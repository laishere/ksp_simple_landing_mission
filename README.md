# krpc控制ksp火箭着陆任务

[演示和教程](https://www.bilibili.com/video/BV1Cb4y1k7J5/)


## 项目结构

```
📦ksp软着陆
 ┣ 📂control                主要的控制算法和一些测试
 ┃ ┣ 📜auto_pilot.py        使用运动模型实现的姿态控制算法
 ┃ ┣ 📜dynamics.py          运动模型实现
 ┃ ┣ 📜rocket.py            本地测试火箭实现
 ┃ ┣ 📜test.py              本地着陆测试、姿态控制测试等
 ┃ ┗ 📜vec.py               常用向量运算封装
 ┣ 📜ap_test.py             ksp姿态控制测试
 ┣ 📜ksp.py                 ksp火箭封装、地面参考系、地面位置计算
 ┣ 📜KSP初级着陆.xmind      着陆xmind思维导图  
 ┣ 📜KSP初级着陆教程.pptx   教程ppt
 ┣ 📜mission.py             任务主程序，发射助推器回收 和 发射一级火箭回收
 ┗ 📜tutorials.py           教程脚本
```