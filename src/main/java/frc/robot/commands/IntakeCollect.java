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
  //private static final double EXTEND_SPEED = 0.5;
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
    RobotContainer.intake.setExtenderCurrentLimit(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.shooterController.getAButtonPressed()){
      RobotContainer.intake.setIntakeSpeed(m_intakeSpeed);
      RobotContainer.intake.extend(.05);
    }
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intake.extend(0);
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
