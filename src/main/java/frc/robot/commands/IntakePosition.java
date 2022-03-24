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

public class IntakePosition extends CommandBase {
  
  private static final double SPEED = 0.15;

  /**
   * Creates a new Collect command.
   */
  
   public IntakePosition() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.intake.setExtenderCurrentLimit(10);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(RobotContainer.shooterController.getRawButton(4)) { // Y Button
      SmartDashboard.putString("ExtendState", "EXTEND");
      RobotContainer.intake.extend(SPEED);
    } else if(RobotContainer.shooterController.getRawButton(3)){ // X Button
      SmartDashboard.putString("ExtendState", "RETRACT");
      RobotContainer.intake.retract(SPEED);
    } else {
      SmartDashboard.putString("ExtendState", "STOP");
      RobotContainer.intake.extend(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intake.extend(0);
    RobotContainer.intake.retract(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(RobotContainer.shooterController.getRawButtonReleased(4) || RobotContainer.shooterController.getRawButtonReleased(3)){
        return true;
    }
    else{
        return false;
    }
  }
}
