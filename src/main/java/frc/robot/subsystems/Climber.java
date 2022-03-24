/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private static final int CLIMBER_CAN_ID = 14;

  TalonFX climberMotor;

  public Climber() {
    climberMotor = new TalonFX(CLIMBER_CAN_ID);
    climberMotor.setNeutralMode(NeutralMode.Brake);

    //setDefaultCommand(new ClimbCommand());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climb Current", climberMotor.getSupplyCurrent());
  }

  public void climb(double speed) {
    climberMotor.set(ControlMode.PercentOutput, speed);
    SmartDashboard.putNumber("Climber", speed);
    
  }
 
}