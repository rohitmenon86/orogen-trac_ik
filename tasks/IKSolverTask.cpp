/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "IKSolverTask.hpp"
#include <base/commands/Joints.hpp>
#include <iostream>
using namespace trac_ik;

IKSolverTask::IKSolverTask(std::string const& name)
    : IKSolverTaskBase(name)
{
}

IKSolverTask::IKSolverTask(std::string const& name, RTT::ExecutionEngine* engine)
    : IKSolverTaskBase(name, engine)
{
}

IKSolverTask::~IKSolverTask()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See IKSolverTask.hpp for more detailed
// documentation about them.

bool IKSolverTask::configureHook()
{
    if (! IKSolverTaskBase::configureHook())
        return false;
    
    baseLink  = _base_link.value();
    tipLink   = _tip_link.value();
    URDFParam = _urdf_filepath.value();
    timeout   = _timeout_in_secs.value();
    error     = _error.value();
       
    ikSolver= new TRAC_IK::TRAC_IK( baseLink, tipLink, URDFParam, timeout, error);  
    return true;
}
bool IKSolverTask::startHook()
{
    if (! IKSolverTaskBase::startHook())
        return false;
    return true;
}
void IKSolverTask::updateHook()
{
  base::samples::Joints jointStatus;
  base::commands::Joints jointCommands;
  base::samples::RigidBodyState desiredRigidBodyState;
  KDL::Frame desiredKDLFrame;
  KDL::JntArray qIn, qOut;
  KDL::Twist tolerances;
  
  
  if(_in_joint_samples.readNewest(jointStatus)==RTT::NoData)
  {
    if(state()!= NO_JOINT_STATUS)
      state(NO_JOINT_STATUS);
    
    return;
  }
  else
  {
    jointCommands = jointStatus;
    kdl_conversions::baseJoints2KDLJntArray(jointStatus, qIn);
  }
  
  if(_in_desired_frame.readNewest(desiredRigidBodyState)==RTT::NewData)
  {
    if(state()!= CARTESIAN_POSE_RECEIVED)
      state(CARTESIAN_POSE_RECEIVED);
    
    kdl_conversions::RigidBodyState2KDL(desiredRigidBodyState, desiredKDLFrame);
    
    int rc = ikSolver->CartToJnt(qIn,desiredKDLFrame, qOut,tolerances);
    
    if(rc>0)
    {
      if(state()!= IK_SOLUTION_FOUND)
	state(IK_SOLUTION_FOUND);

      kdl_conversions::KDLJntArray2baseJoints(qOut, jointCommands);     
      
      _out_joint_commands.write(jointCommands);
    }
    
    else
    {
      if(state()!= NO_IK_SOLUTION)
	state(NO_IK_SOLUTION);
    }
    
    
    
  }
  
  
  
    IKSolverTaskBase::updateHook();
}
void IKSolverTask::errorHook()
{
    IKSolverTaskBase::errorHook();
}
void IKSolverTask::stopHook()
{
    delete ikSolver;
    ikSolver = NULL;
    IKSolverTaskBase::stopHook();
}
void IKSolverTask::cleanupHook()
{
    IKSolverTaskBase::cleanupHook();
}
