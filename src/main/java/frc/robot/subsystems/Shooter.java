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

public class Shooter extends SubsystemBase {

  private Double rpmSetpoint;

  private final int SHOOTER_MOTOR_CAN_ID = 13;

  private static final double DEFAULT_RPM = 0.0;
  public static final double MAX_RPM = 6500.0;
  //private static final double MAX_VEL = 22000.0;

  /**
   * Creates a new shooter.
   */

  TalonFX shooterMotor;
  //CANSparkMax feedMotor;

  public Shooter() {
    
    shooterMotor = new TalonFX(SHOOTER_MOTOR_CAN_ID);
    shooterMotor.setNeutralMode(NeutralMode.Coast);
    shooterMotor.config_kF(0, 0.048, 0);
    shooterMotor.config_kP(0, 0.010, 0);


    //Leftover from 2020 code, separated into feed subsystem
    // feedMotor = new CANSparkMax(FEED_MOTOR_CAN_ID, MotorType.kBrushless);
    // feedMotor.setIdleMode(IdleMode.kCoast);
    // feedMotor.setSmartCurrentLimit(20);

    rpmSetpoint = DEFAULT_RPM;

    //feedTriggerSwitch = new DigitalInput(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putBoolean("Ball Ready", ballReady());
    SmartDashboard.putNumber("Shooter RPM", getRpm());
    SmartDashboard.putNumber("Shooter VEL", shooterMotor.getSelectedSensorVelocity());
  }

  public Double getRpm() {
    Double countsPerHundredMs = shooterMotor.getSelectedSensorVelocity();
    Double rpm = countsPerHundredMs * 60000.0 / (100 * 2048);

    return rpm;
  }

  public void setMotorRPM(double rpm) {

    double velocity = (rpm * 100.0 * 2048.0 )/ 60000.0;

    shooterMotor.set(ControlMode.Velocity, velocity);
    //System.out.println("shooter velocity = " + velocity);
  }
  
  // public void setMotorPercent(double value) {
  //   shooterMotor.set(ControlMode.PercentOutput, value);
  // }

  // public void setMotorVelocity(double vel) {
  //   shooterMotor.set(ControlMode.Velocity, vel);
  // }

  // public void setFeedSpeed(double speed) {
  //   feedMotor.set(.5*speed);
  // }

// public boolean ballReady() {
//     return false;
//}
}