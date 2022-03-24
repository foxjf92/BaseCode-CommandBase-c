/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;

public class ShootBall extends ParallelCommandGroup {
  public ShootBall() {
    // Use requires() here to declare subsystem dependencies
    addRequirements(RobotContainer.shooter);
    addRequirements(RobotContainer.feeder);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    RobotContainer.feeder.feed(1.0);
    System.out.println("Shooting RPM = " + RobotContainer.shooter.getRpm());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return !RobotContainer.feeder.ballStatus1();
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    RobotContainer.feeder.feed(0.0);
  }
}
