# Unitree Guide Framework for Legged Robots based on DRL

---

## Introduction
- 本repo是作者入学时在unitree_guide框架基础上编写的四足机器人深度强化学习Sim2Sim/Sim2Real框架。
- 由于实验室开源协议和论文还未投稿，因此只开源作者探索之初成功部署的状态机，欢迎读者refer/debug。
- 如有侵权，请联系作者删除。

**Maintainer**: KK-Zhou 

**Affiliation**: Sucro Lab 

**Email**: kkizhou@qq.com

**WeChat**: Z17354029494

---

## Q&A
- 这个版本采用的是50hz的RL决策频率，并同时计算力矩下发，这样关节角度和速度的反馈也是50hz。最好是将训练用的PD参数和期望关节位置速度下发（感谢unitree_guide群主提供的建议）。
