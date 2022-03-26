/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;

public class ShootBall extends CommandBase {
private final Feeder m_feeder;

  public ShootBall(Feeder feeder) {
    m_feeder = feeder;
    // Use requires() here to declare subsystem dependencies
    //addRequirements(RobotContainer.shooter);
    addRequirements(m_feeder);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    System.out.println("Shooting RPM = " + RobotContainer.shooter.getRpm());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    RobotContainer.feeder.feed(1.0);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return !RobotContainer.feeder.ballStatus();
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    RobotContainer.feeder.feed(0.0);
  }
}
