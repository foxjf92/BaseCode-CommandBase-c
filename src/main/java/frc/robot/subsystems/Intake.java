/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */
  private static final int INTAKE_POWER_CAN_ID  = 12;
  private static final int EXTENDER1_CAN_ID = 10;
  private static final int EXTENDER2_CAN_ID = 9;

  CANSparkMax intakeMotor;
  CANSparkMax extenderMotor1;
  CANSparkMax extenderMotor2;
  
  public Intake() {
    intakeMotor = new CANSparkMax(INTAKE_POWER_CAN_ID, MotorType.kBrushless);
    intakeMotor.setIdleMode(IdleMode.kCoast);
    intakeMotor.setSmartCurrentLimit(20);
    intakeMotor.setInverted(true);
    extenderMotor1 = new CANSparkMax(EXTENDER1_CAN_ID, MotorType.kBrushless);
    extenderMotor2 = new CANSparkMax(EXTENDER2_CAN_ID, MotorType.kBrushless);
  }


  public void setIntakeSpeed(double intakeSpeed) {
    intakeMotor.set(intakeSpeed);
  }

  public void setExtenderCurrentLimit(int currentLimit) {
    extenderMotor1.setSmartCurrentLimit(currentLimit);
    extenderMotor2.setSmartCurrentLimit(currentLimit);
  }

  public void extend(double speed) {
    extenderMotor1.set(-speed);
    extenderMotor2.set(speed);
  SmartDashboard.putNumber("Extender", speed);
  }

  public void retract(double speed) {
    extenderMotor1.set(speed);
    extenderMotor2.set(-speed);
  SmartDashboard.putNumber("Extender", speed);
  }
}
