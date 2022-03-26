/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;

public class FeederCollect extends CommandBase {
  private final Feeder m_feeder;
  private final double m_feedSpeed;
  /**
   * Creates a new Collect command.
   */
  public FeederCollect(Feeder feeder, double feedSpeed) {
    m_feeder = feeder;
    m_feedSpeed = feedSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // if(RobotContainer.shooterController.getAButtonPressed()){
    //   //Remove after DI logic is sorted
    // RobotContainer.feeder.feed(.25);

    //Uncomment after figuring out DI logic
    if(RobotContainer.feeder.ballStatus()){
      RobotContainer.feeder.feed(m_feedSpeed); 
    }
    else if(!RobotContainer.feeder.ballStatus()){
      RobotContainer.feeder.feed(0.0);
    }
      
  }
  
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.feeder.feed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
    // if(RobotContainer.shooterController.getAButtonReleased()){
    //   return true;
    // }
    // else{
    //   return false;
    }
}

