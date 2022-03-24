/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ClimbCommand extends CommandBase {
  private static final double CLIMB_SPEED = 1.0;

  /**
   * Creates a new ClimbCommand.
   */
  public ClimbCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(RobotContainer.shooterController.getRawButton(6)) {
      SmartDashboard.putString("ClimbState", "CLIMB EXTENDING");
      RobotContainer.climber.climb(CLIMB_SPEED);
    } 
    else if(RobotContainer.shooterController.getRawButton(5)) {
      SmartDashboard.putString("ClimbState", "CLIMB RETRACTING");
      RobotContainer.climber.climb(-CLIMB_SPEED);
    } 
    else {
      SmartDashboard.putString("ClimbState", "STOPPED");
      RobotContainer.climber.climb(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
