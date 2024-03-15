/*
    注意legged_gym中训练的腿的次序是 FL FR RL RR
    而unitree_guide中motorCmd腿的次序是 FR FL RR RL
    所以要将腿的次序交换
*/

#include "FSM/State_rl.h"

State_rl::State_rl(CtrlComponents *ctrlComp) : FSMState(ctrlComp, FSMStateName::RL, "rl")
{
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    std::cout << "            Reinforcement Learning Warm Up and Init           " << std::endl;
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;

    std::string modulePath = "/reinforcement/model/model.jit";
    modulePath = std::string(get_current_dir_name()) + "/src/unitree_guide/unitree_guide/src" + modulePath;
    _obs .setZero();
    
    std::vector<float> floatVector(_obs .data(), _obs .data() + _obs .size());
    torch::Tensor temp = torch::from_blob(floatVector.data(), {_obs .size()}, torch::kFloat32);
    inputs.push_back(temp);

    try
    {
        module = torch::jit::load(modulePath, torch::kCPU);
        output = module.forward(inputs).toTensor();
        std::cout << output << std::endl;
        std::cout << "State_rl: " << "rl network init finished! " << '\n';
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}


void State_rl::enter()
{
    //
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    std::cout << "          Start Joint Position Control learned by RL          " << std::endl;
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    
    _dofDefaultPos<< 0., 0.9, -1.8, 
                     0., 0.9, -1.8, 
                     0., 0.9, -1.8, 
                     0., 0.9, -1.8; // legged_gym: FL FR RL RR  

    _obs .setZero();
    _targetDofPos.setZero();
    _targetDofVel.setZero();
    _highCmd.setZero();
    _vCmdBody.setZero(); 
    _dYawCmd = 0.0;
    _actions.setZero();

    frequencyFactor = int(1 / (policyQueried * _ctrlComp -> dt) + 0.5);
    count = 0;

    std::cout << "frequencyFactor = " << frequencyFactor << std::endl;
    step(); // reset
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    std::cout << "                       Start RL FSM RUN !                     " << std::endl;
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    
}

/*
    run函数的频率在主函数main.cpp第69行设定为500hz，运行等待函数在FSM.cpp第76行中实现
    在run函数中通过count对act和step进行分频，step的频率和run函数一致，act的频率为step / count
*/
void State_rl::run()
{
    count++;
    if (count >= frequencyFactor)
    {
        count = 0;  
        act();
    }
    step(); 
}
  
void State_rl::act()
{
    std::vector<float> floatVector(_obs .data(), _obs .data() + _obs .size());
    torch::Tensor temp = torch::from_blob(floatVector.data(), {_obs .size()}, torch::kFloat32);
    inputs.clear();
    inputs.push_back(temp);
    output = module.forward(inputs).toTensor();
    output = torch::clip(output, -clipActions, clipActions);
    _actions = rlType::TensorToVector<double>(output);
}

void State_rl::step()
{
    for (int i = 0; i < decimation; i++)
    {
        computeTorques();
        _lowCmd->setZeroGain();
        _torques = rlType::changeLegs<double>(_torques); // legged_gym->unitree_guide
        _lowCmd->setTau(_torques);   
        _dofPos = rlType::changeLegs<double>(rlType::vec34ToVec12<double>(_lowState->getQ())); // unitree_guide->legged_gym
        _dofVel = rlType::changeLegs<double>(rlType::vec34ToVec12<double>(_lowState->getQd())); // unitree_guide->legged_gym   
    }

    _B2G_RotMat = _lowState->getRotMat();
    _G2B_RotMat = _B2G_RotMat.transpose();
    _projGravity = _G2B_RotMat * _G;
    
    getUserCmd();
    
    computeObservations();
    _preactions = _actions;
}

void State_rl::computeTorques()
{
    _actionsScaled = _actions * actionScale;

    _targetDofPos = _actionsScaled + _dofDefaultPos;

    _dofPos = rlType::changeLegs<double>(rlType::vec34ToVec12<double>(_lowState->getQ())); // unitree_guide->legged_gym
    _dofVel = rlType::changeLegs<double>(rlType::vec34ToVec12<double>(_lowState->getQd())); // unitree_guide->legged_gym 

    pos_err = _targetDofPos - _dofPos;
    vel_err = _dofVel;
    _torques = _GainP * pos_err - _GainD * vel_err;
    limitTorques();
}

void State_rl::limitTorques()
{
    double bound;
    for (int i = 0; i < 12; i++)
    {
        bound =torque_limits(i % 3);
        _torques(i) = _torques(i) > bound ? bound : _torques(i);
        _torques(i) = _torques(i) < -bound ? -bound : _torques(i);    
    }
}

void State_rl::computeObservations()
{
    _highCmd << _command(0) * linVelScale, _command(1) * linVelScale, _command(2) * angleVelScale;
    _obs << _projGravity,                                           // 3     
            _highCmd,                                               // 3                                        
            (_dofPos - _dofDefaultPos) * dofPosScale,               // 12                
            _dofVel * dofVelScale,                                  // 12
            _preactions;                                            // 12
                                                                    // 42
}

void State_rl::getUserCmd()
{
    _userValue = _lowState->userValue;
    /* Movement */
    _vCmdBody(0) = invNormalize(_userValue.ly, _vxLim(0), _vxLim(1));
    _vCmdBody(1) = -invNormalize(_userValue.lx, _vyLim(0), _vyLim(1));
    _vCmdBody(2) = 0;

    /* Turning */
    _dYawCmd = -invNormalize(_userValue.rx, _wyawLim(0), _wyawLim(1));
    _dYawCmd = 0.9 * _dYawCmdPast + (1.0 - 0.9) * _dYawCmd; // 控制旋转速度
    _dYawCmdPast = _dYawCmd;

    // save to command
    _command << _vCmdBody(0), _vCmdBody(1), _dYawCmd;
    std::cout << "get _highCmd:" << _command << std::endl;   
}

FSMStateName State_rl::checkChange()
{
    if (_lowState->userCmd == UserCommand::L2_B)
    {
        return FSMStateName::PASSIVE;
    }
    else if (_lowState->userCmd == UserCommand::L2_A)
    {
        return FSMStateName::FIXEDSTAND;
    }
    else
    {
        return FSMStateName::RL;
    }
}

void State_rl::exit()
{
    //
}

State_rl::~State_rl()
{
    //
}
