/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DriveTriangle extends SequentialCommandGroup {
  /**
   * Add your docs here.
   */
  public DriveTriangle() {
    addCommands(new DriveXY(100, 0, 0, 0.5));
    //addSequential(new DriveXY(100, 100, 0, 0.5));
    //addSequential(new DriveXY(0, 0, 0, 0.5));
  }
}
