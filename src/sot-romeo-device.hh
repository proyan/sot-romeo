/*
 * Copyright 2011,
 *
 * Olivier Stasse
 *
 * LAAS, CNRS
 *
 * This file is part of RomeoController.
 * RomeoController is not a free software, 
 * it contains information related to Romeo which involves
 * that you either purchased the proper license to havec access to
 * those informations, or that you signed the appropriate
 * Non-Disclosure agreement.
 *
 *
 */

#ifndef _SOT_RomeoDevice_H_
#define _SOT_RomeoDevice_H_

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/linear-algebra.h>
#include <sot/core/device.hh>
#include <sot/core/abstract-sot-external-interface.hh>
#include <sot/core/matrix-rotation.hh>

namespace dgsot=dynamicgraph::sot;

class SoTRomeoDevice: public 
dgsot::Device
{
 public:

  static const std::string CLASS_NAME;
  static const double TIMESTEP_DEFAULT;

  virtual const std::string& getClassName () const		
  {  
    return CLASS_NAME;							    
  }
  
  SoTRomeoDevice(std::string RobotName);
  virtual ~SoTRomeoDevice();
  
  void setSensors(std::map<std::string,dgsot::SensorValues> &sensorsIn);

  void setupSetSensors(std::map<std::string,dgsot::SensorValues> &sensorsIn);

  void nominalSetSensors(std::map<std::string,dgsot::SensorValues> &sensorsIn);

  void cleanupSetSensors(std::map<std::string, dgsot::SensorValues> &sensorsIn);

  void getControl(std::map<std::string, dgsot::ControlValues> &anglesOut);

protected:
  // Update output port with the control computed from the
  // dynamic graph.
  void updateRobotState(const std::vector<double> &anglesIn);

  /// \brief Current integration step.
  double timestep_;
  
  /// \brief Previous robot configuration.
  maal::boost::Vector previousState_;
  
  /// \brief Robot state provided by OpenHRP.
  ///
  /// This corresponds to the real encoders values and take into
  /// account the stabilization step. Therefore, this usually
  /// does *not* match the state control input signal.
  ///
  dynamicgraph::Signal<ml::Vector, int> robotState_;

  /// Intermediate variables to avoid allocation during control
  ml::Vector mlRobotState;
  std::vector<double> baseff_;
};
#endif /* _SOT_RomeoDevice_H_*/
