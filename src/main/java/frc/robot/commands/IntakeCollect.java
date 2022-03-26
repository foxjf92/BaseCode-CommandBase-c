/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class IntakeCollect extends CommandBase {

  private final Intake m_intake;
  private final double m_intakeSpeed;

  /**
   * Creates a new Collect command.
   */
  public IntakeCollect(Intake intake, double intakeSpeed) {
    m_intake = intake;
    m_intakeSpeed = intakeSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setExtenderCurrentLimit(3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //System.out.print("Executing IntakeCollect");
    m_intake.setIntakeSpeed(m_intakeSpeed);
    m_intake.extend(.15);
  
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //System.out.print("End IntakeCollect");
    m_intake.extend(0);
    m_intake.setIntakeSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return false;

    // if(RobotContainer.shooterController.getAButtonReleased()){
    //   return true;
    // }
    // else{
    //   return false;
    // }

  }
}
