/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class SetShooterSpeed extends CommandBase {
  double setpoint;
  double rpmTarget;

  public static final double tolerance = 200.0;

  public SetShooterSpeed(double motorRPM) {
    // Use requires() here to declare subsystem dependencies
    setpoint = motorRPM;
    addRequirements(RobotContainer.shooter);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    RobotContainer.shooter.setMotorRPM(setpoint);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return Math.abs(RobotContainer.shooter.getRpm() - setpoint) < tolerance;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }
}
