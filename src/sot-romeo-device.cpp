/*
 * Copyright 2011,
 *
 * Olivier Stasse
 *
 * LAAS, CNRS
 *
 * This file is part of RomeoController.
 *
 */

#include <fstream>
#include <map>

#include <sot/core/debug.hh>

#include "sot-romeo-device.hh"
#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>

using namespace std;

const double SoTRomeoDevice::TIMESTEP_DEFAULT = 0.005;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(SoTRomeoDevice,"Device");

SoTRomeoDevice::SoTRomeoDevice(std::string RobotName):
  dgsot::Device(RobotName),
  timestep_(TIMESTEP_DEFAULT),
  previousState_ (),
  robotState_ ("StackOfTasks(" + RobotName + ")::output(vector)::robotState"),
  baseff_ ()
{
  sotDEBUGIN(25) ;
  signalRegistration (robotState_);
  ml::Vector data (3); data.setZero ();
  baseff_.resize(12);

  using namespace dynamicgraph::command;
  std::string docstring;
  /* Command increment. */
  docstring =
    "\n"
    "    Integrate dynamics for time step provided as input\n"
    "\n"
    "      take one floating point number as input\n"
    "\n";
  addCommand("increment",
	     makeCommandVoid1((Device&)*this,
			      &Device::increment, docstring));
  
  sotDEBUGOUT(25);
}

SoTRomeoDevice::~SoTRomeoDevice()
{ }

void SoTRomeoDevice::setSensors(map<string,dgsot::SensorValues> &SensorsIn)
{
  sotDEBUGIN(25) ;
  map<string,dgsot::SensorValues>::iterator it;
  int t = stateSOUT.getTime () + 1;
/*
  it = SensorsIn.find("forces");
  if (it!=SensorsIn.end())
    {

      // Implements force recollection.
      const vector<double>& forcesIn = it->second.getValues();
      for(int i=0;i<4;++i)
        {
          for(int j=0;j<6;++j)
            mlforces(j) = forcesIn[i*6+j];
          forcesSOUT[i]->setConstant(mlforces);
          forcesSOUT[i]->setTime (t);
        }
    }

  it = SensorsIn.find("attitude");
  if (it!=SensorsIn.end())
    {
      const vector<double>& attitude = it->second.getValues ();
      for (unsigned int i = 0; i < 3; ++i)
        for (unsigned int j = 0; j < 3; ++j)
          pose (i, j) = attitude [i * 3 + j];
      attitudeSOUT.setConstant (pose);
      attitudeSOUT.setTime (t);
    }
*/
  it = SensorsIn.find("joints");
  if (it!=SensorsIn.end())
    {
      const vector<double>& anglesIn = it->second.getValues();
      mlRobotState.resize (anglesIn.size () + 6);
      for (unsigned i = 0; i < 6; ++i)
        mlRobotState (i) = 0.;
      updateRobotState(anglesIn);
    }
/*
  it = SensorsIn.find("accelerometer_0");
  if (it!=SensorsIn.end())
    {
      const vector<double>& accelerometer = 
        SensorsIn ["accelerometer_0"].getValues ();
      for (std::size_t i=0; i<3; ++i) 
        accelerometer_ (i) = accelerometer [i];
      accelerometerSOUT_.setConstant (accelerometer_);
    }

  it = SensorsIn.find("gyrometer_0");
  if (it!=SensorsIn.end())
    {
      const vector<double>& gyrometer = SensorsIn ["gyrometer_0"].getValues ();
      for (std::size_t i=0; i<3; ++i) 
        gyrometer_ (i) = gyrometer [i];
      gyrometerSOUT_.setConstant (gyrometer_);
    }
*/
  sotDEBUGOUT(25);
}

void SoTRomeoDevice::setupSetSensors(map<string,dgsot::SensorValues> &SensorsIn)
{
  setSensors (SensorsIn);
}


void SoTRomeoDevice::nominalSetSensors(map<string,dgsot::SensorValues> &SensorsIn)
{
  setSensors (SensorsIn);
}

void SoTRomeoDevice::cleanupSetSensors(map<string, dgsot::SensorValues> &SensorsIn)
{
  setSensors (SensorsIn);
}

void SoTRomeoDevice::getControl(map<string,dgsot::ControlValues> &controlOut)
{
  sotDEBUGIN(25) ;
  vector<double> anglesOut;
  anglesOut.resize(state_.size());

  // Integrate control
  increment(timestep_);

  sotDEBUG (25) << "state = " << state_ << std::endl;
  sotDEBUG (25) << "diff  = " << state_ - previousState_ << std::endl;
  previousState_ = state_;

  // Specify the joint values for the controller.
  if (anglesOut.size()!=state_.size()-6)
    anglesOut.resize(state_.size()-6);

  for(unsigned int i=6; i < state_.size();++i)
    anglesOut[i-6] = state_(i);
  controlOut["joints"].setValues(anglesOut);
  
  // Read zmp reference from input signal if plugged
  int time = controlSIN.getTime ();
  zmpSIN.recompute (time + 1);
  // Express ZMP in free flyer reference frame
  ml::Vector zmpGlobal (4);
  for (unsigned int i = 0; i < 3; ++i)
    zmpGlobal(i) = zmpSIN(time + 1)(i);
  zmpGlobal(3) = 1.;
  dgsot::MatrixHomogeneous inversePose;
  freeFlyerPose().inverse(inversePose);
  ml::Vector localZmp = inversePose * zmpGlobal;
  vector<double> ZMPRef(3);
  for(unsigned int i=0;i<3;++i)
    ZMPRef[i] = localZmp(i);

  controlOut["zmp"].setName("zmp");
  controlOut["zmp"].setValues(ZMPRef);
  
  // Update position of freeflyer in global frame
  for (int i = 0;i < 3; ++i)
    baseff_[i*4+3] = freeFlyerPose () (i, 3);
  for(unsigned i = 0;i < 3; ++i)
    for(unsigned j = 0; j < 3; ++j)
      baseff_[i * 4 + j] = freeFlyerPose () (i, j);

  controlOut["baseff"].setValues(baseff_);
  sotDEBUGOUT(25) ;
}

void SoTRomeoDevice::updateRobotState(const vector<double> &anglesIn)
{
  sotDEBUGIN(25) ;
  for (unsigned i = 0; i < anglesIn.size(); ++i)
    mlRobotState (i + 6) = anglesIn[i];
  robotState_.setConstant(mlRobotState);
  sotDEBUGOUT(25) ;
}
