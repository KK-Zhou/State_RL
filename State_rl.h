#ifndef STATE_RL_H
#define STATE_RL_H

#include "FSMState.h"
#include "reinforcement/dataTypes.h"

class State_rl : public FSMState
{
private:
    // net structure
    // torch::Device device = torch::kCPU;
    torch::jit::script::Module module;
    std::vector<torch::jit::IValue> inputs;
    torch::Tensor output;

    // Robot State
    rlType::Vec3<double> _projGravity;   
    rlType::Vec3<double> _highCmd;
    rlType::Vec12<double> _dofDePos, _dofPos, _targetDofPos, _dofDefaultPos;
    rlType::Vec12<double> _dofVel, _targetDofVel;
    rlType::Vec12<double> _actions, _preactions, _actionsScaled;
    rlType::Vec12<double> pos_err;
    rlType::Vec12<double> vel_err;
    rlType::VecX<double, 42> _obs; // 观测缓冲


    // config
    int count;
    int frequencyFactor;
    const int times = 5; // print times
    const double policyQueried = 30; // hz
    const double decimation = 6; // 6
    const  double _GainP = 80.; // 刚度值stiffness，即比例系数P
    const double _GainD = 1.;  // 阻尼值damping，即微分系数D
    const rlType::Vec3<double> _G = {0, 0, -1};
    const double actionScale = 0.25;
    const double linVelScale = 2.0;
    const double angleVelScale = 0.25;
    const double dofPosScale = 1.0;
    const double dofVelScale = 0.05;
    const double clipObs = 100.0;
    const double clipActions = 100.0;    
    const rlType::Vec3<double> torque_limits = {33.5, 33.5, 33.5};
    const rlType::Vec2<double> _vxLim = {-1.0, 2.0}, _vyLim = {-0.3, 0.3}, _wyawLim = {-1.57, 1.57};
    
    double _yaw, _dYaw; 
    rlType::RotMat<double> _B2G_RotMat, _G2B_RotMat; // 互为逆矩阵
    rlType::Vec12<double> _torques;

    // robot command
    rlType::Vec3<double> _vCmdBody;
    double _dYawCmd, _dYawCmdPast;
    rlType::Vec3<double> _command;

    void act();
    void step();
    void computeTorques();
    void limitTorques();
    void computeObservations();
    virtual void getUserCmd();
    
public:
    State_rl(CtrlComponents *ctrlComp);
    ~State_rl();
    void enter();
    void run();
    void exit();
    virtual FSMStateName checkChange();
};

#endif