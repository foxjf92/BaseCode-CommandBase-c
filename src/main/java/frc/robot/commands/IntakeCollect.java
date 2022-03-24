/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class IntakeCollect extends CommandBase {
  //private static final double EXTEND_SPEED = 0.5;

  /**
   * Creates a new Collect command.
   */
  public IntakeCollect() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.intake.setExtenderCurrentLimit(1);

    // RobotContainer.intake.setIntakeSpeed(.75);
    // RobotContainer.intake.extend(.05);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //double intakeTriggerCheck = -Robot.oi.shooterController.getRawAxis(2); //should be left trigger
    
    // if(intakeTriggerCheck > 0.1){
    //   Robot.intake.setIntakeSpeed(.75);
    //   Robot.intake.extend(.25);
    // }
    // else{
    //   Robot.intake.setIntakeSpeed(0);
    // }

    RobotContainer.intake.setIntakeSpeed(.75);
    RobotContainer.intake.extend(.05);

    //FIXME Work out logic to run feed while intaking, maybe here?

    // if(Robot.oi.shooterController.getRawButton(4)) {
    //   SmartDashboard.putString("ExtendState", "EXTEND");
    //   Robot.climber.extend(EXTEND_SPEED);
    // } else if(Robot.oi.shooterController.getRawButton(3)){
    //   SmartDashboard.putString("ExtendState", "RETRACT");
    //   Robot.climber.extend(-EXTEND_SPEED);
    // } else {
    //   SmartDashboard.putString("ExtendState", "STOP");
    //   Robot.climber.extend(0);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intake.setIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(RobotContainer.shooterController.getAButtonReleased()){
      return true;
    }
    else{
      return false;
    }

  }
}
