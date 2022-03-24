/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Feeder extends SubsystemBase {
  private final int FEED_MOTOR_CAN_ID = 11;
  
  CANSparkMax feedMotor;
  DigitalInput feedTriggerSwitch1;
  //DigitalInput feedTriggerSwitch2; //reserved for additional switch if needed

  public Feeder() {
    feedMotor = new CANSparkMax(FEED_MOTOR_CAN_ID, MotorType.kBrushless);
    feedMotor.setSmartCurrentLimit(20);
    feedTriggerSwitch1 = new DigitalInput(0);
    //feedTriggerSwitch2 = new DigitalInput(1);
  }

  public void feed(double speed) {
    feedMotor.set(speed);
  }

  public boolean ballStatus1(){
    return !feedTriggerSwitch1.get();
  }

  // public boolean ballStatus2(){
  //   return !feedTriggerSwitch2.get();

  // }

  // @Override
  // public void initDefaultCommand() {
  // }
 
}
