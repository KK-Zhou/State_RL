# Unitree Guide Framework for Legged Robots based on DRL

**Maintainer**: KK-Zhou 

**Affiliation**: SUCRO Lab 

**Email**: kkizhou@qq.com

---
## Update
- 完整的部署代码请见实验室官方开源: [unitree_ws](https://github.com/SUCRO-Legged/unitree_ws)

---
## Q&A
- 这个版本采用的是50hz的RL决策频率，关节角度和速度的反馈也是50hz。最好是将训练用的PD参数和期望关节位置速度下发，而不是直接下发自己计算出来的力矩（感谢unitree_guide群主提供的建议）。

---
## Stress
- 一定要确保在gazebo中的仿真正常了再上实机测试，测试时用狗链子牵着同时注意电机有无抽动。发生异常要立即passive。
