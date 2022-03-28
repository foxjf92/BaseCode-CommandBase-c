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

public class ClimberHigh extends SubsystemBase {
  private static final int CLIMBERHIGH1_CAN_ID = 15;
  private static final int CLIMBERHIGH2_CAN_ID = 16;

  TalonFX climberHighMotor1;
  TalonFX climberHighMotor2;
  
  public ClimberHigh() {
    climberHighMotor1 = new TalonFX(CLIMBERHIGH1_CAN_ID);
    climberHighMotor1.setNeutralMode(NeutralMode.Brake);
    climberHighMotor2 = new TalonFX(CLIMBERHIGH2_CAN_ID);
    climberHighMotor2.setNeutralMode(NeutralMode.Brake);

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climb(double speed) {
    climberHighMotor1.set(ControlMode.PercentOutput, speed);
    climberHighMotor2.set(ControlMode.PercentOutput, speed);
  
  }
 
}