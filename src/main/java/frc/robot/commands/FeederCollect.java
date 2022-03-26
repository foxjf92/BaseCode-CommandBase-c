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
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    m_feeder.feed(m_feedSpeed);

    // if(RobotContainer.shooterController.getAButtonPressed()){
    //   m_feeder.feed(m_feedSpeed);
    // }

    //System.out.print("Executing FeederCollect");
    // //Remove after DI logic is sorted
    // m_feeder.feed(m_feedSpeed);
   
  }
  
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //System.out.print("End FeederCollect");
    m_feeder.feed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(RobotContainer.shooterController.getAButtonReleased() || m_feeder.ballStatus()){
    //   return true;
    // }
    // else{
    //   return false;
    // }
    if(m_feeder.ballStatus()){
      return true;
    }

    return false;
  }

}

