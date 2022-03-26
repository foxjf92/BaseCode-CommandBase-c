/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

public class ShootAndDrive extends SequentialCommandGroup {
  /**
   * Add your docs here.
   */
  //private final DriveTrain m_driveTrain;
  private final Shooter m_shooter;
  private final Feeder m_feeder;
  
  public ShootAndDrive(DriveTrain driveTrain, Feeder feeder, Shooter shooter) {
    //m_driveTrain = driveTrain;
    m_shooter = shooter;
    m_feeder = feeder;
    
    addCommands(new SetShooterSpeed(m_shooter, ManualShooterSpeed.FAR_SETPOINT));
    //addSequential(new DriveXY(100, 0, 0, 0.5));
    //addSequential(new DriveXY(120, 0, 0, 0.2));
    //addSequential(new SetShooterSpeed(ManualShooterCommand.FAR_SETPOINT));
    //addSequential(new LoadBall());
    //addSequential(new ShootBall());
    //addSequential(new SetShooterSpeed(ManualShooterCommand.FAR_SETPOINT));
    //addSequential(new LoadBall());
    //addSequential(new ShootBall());
    //addSequential(new SetShooterSpeed(ManualShooterCommand.FAR_SETPOINT));
    //addSequential(new LoadBall());
    addCommands(new ShootBall(m_feeder));
    addCommands(new DriveXY(100, 0, 0, 0.5));
    //addSequential(new DriveXY(100, 50, 0, 0.3));
  }
}
