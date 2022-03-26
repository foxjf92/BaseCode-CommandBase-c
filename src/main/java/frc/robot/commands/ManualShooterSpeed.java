/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class ManualShooterSpeed extends CommandBase {
  /**
   * Creates a new ManualShooterCommand.
   */
  //FIXME Shooter RPM setpoints below for manual and auton
  static final double INCREMENT = 25.0;
  static final double ZERO_SPEED = 0;
  static final double NEAR_SETPOINT = 3000.0;
  static final double FAR_SETPOINT = 4000.0;

  double rpmSetpoint = 0.0;

  public ManualShooterSpeed() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Checks on initilization instead of continuously?
    //int pov = RobotContainer.shooterController.getPOV();
    
    // if(pov == 0) {
    //   rpmSetpoint += INCREMENT;
    // } else if(pov == 180) {
    //   rpmSetpoint = ZERO_SPEED;
    // } else if(pov == 90) {
    //   rpmSetpoint = NEAR_SETPOINT;
    // } else if(pov == 270) {
    //   rpmSetpoint = FAR_SETPOINT;
    // }
  //  rpmSetpoint = 0.0; //I think commenting this will let shooter speed persist 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    int pov = RobotContainer.shooterController.getPOV();
    
    if(pov == 0) {
      rpmSetpoint += INCREMENT;
    } else if(pov == 180) {
      rpmSetpoint = ZERO_SPEED;
    } else if(pov == 90) {
      rpmSetpoint = NEAR_SETPOINT;
    } else if(pov == 270) {
      rpmSetpoint = FAR_SETPOINT;
    }

    rpmSetpoint = MathUtil.clamp(rpmSetpoint, 0.0, Shooter.MAX_RPM
    );

    double rpm = RobotContainer.shooter.getRpm();
    double sendValue = MathUtil.clamp(rpmSetpoint, rpm - 500, rpm + 500);
    
    RobotContainer.shooter.setMotorRPM(sendValue);
    SmartDashboard.putNumber("Shooter Setpoint", rpmSetpoint);

  }

  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
