/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;


public class Collect extends ParallelCommandGroup {
  /**
   * Add your docs here.
   */
  public Collect() {
    addCommands(
      new IntakeCollect()
      //new FeederCollect()
    );

  }
}
