# 单关节机械臂 PID 控制仿真
## 项目简介
基于Python的单关节机械臂PID控制数值仿真。
建立了包含转动惯量、关节摩擦和重力力矩的机械臂动力学模型
（J=0.5 kg·m²，b=0.1，m=1.0 kg，L=0.5 m），
对比了有无重力前馈补偿两种控制策略的响应曲线。
## 实验内容
写了一个PID类用来简洁的调用PID的计算过程,用有无前馈的两个循环计算,并保存了PID控制的过程信息,最后用画图的库在同一张图片上展示了有无前馈的图像对比。 
结果显示主要有两个区别: 
1. 前馈PID在最初会多给关节提供一个较大的力矩,导致超调,可能引发震荡。
2. 前馈PID能让被控制量很快稳定在目标值
## 环境配置
下载conda,打开anaconda prompt 
每次一行输入以下三行命令即可配置环境 :
conda create -n pid python=3.11 
conda activate pid 
pip install numpy matplotlib  
## 运行
python pid.py
会得到图像
## 结果图
<img width="692" height="458" alt="image" src="https://github.com/user-attachments/assets/a2e3897a-2ba2-43d0-928b-d54d75b0526d" />
