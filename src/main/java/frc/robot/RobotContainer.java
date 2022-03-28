// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ClimbHighCommand;
import frc.robot.commands.DoNothing;
import frc.robot.commands.DriveXY;
import frc.robot.commands.FeederCollect;
import frc.robot.commands.FeederShoot;
import frc.robot.commands.IntakeCollect;
import frc.robot.commands.IntakeHoldRetracted;
import frc.robot.commands.IntakePosition;
import frc.robot.commands.ManualShooterSpeed;
import frc.robot.commands.ShootAndDrive;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ClimberHigh;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final Climber climber = new Climber();
  public static final ClimberHigh climberHigh = new ClimberHigh();
  public final static SwerveDrive swerveDrive = new SwerveDrive();
  //public static final DriveTrain driveTrain = new DriveTrain();
  public static final Feeder feeder = new Feeder();
  public static final Intake intake = new Intake();
  public static final Shooter shooter = new Shooter();
  
  //Commands here
  private final Command m_climbCommand = new ClimbCommand();
  private final Command m_climbHighCommand = new ClimbHighCommand();
  //private final Command m_driveCommand = new SwerveJoystickCmd(swerveDrive);
  //private final Command m_feederCollect = new FeederCollect(feeder, speed);
  //private final Command m_feederShoot = new FeederShoot(feeder);
  //private final Command m_intakeCollect = new IntakeCollect(intake, speed);
  private final Command m_intakeHoldRetracted = new IntakeHoldRetracted(intake);
  private final Command m_intakePosition = new IntakePosition();
  //private final Command m_manualShootCommand = new ManualShootCommand();
  private final Command m_manualShooterSpeed = new ManualShooterSpeed();
  
  //Command Groups
  //private final ParallelCommandGroup m_collect = new Collect(intake, intakeSpeed, feeder, feederSpeed);
  //private final ParallelCommandGroup m_shoot = new ShootBall();

  //Controllers
  public static XboxController driveController = new XboxController(0);
  public static XboxController shooterController = new XboxController(1);

  //Autons here
  //private final Command doNothing = new DoNothing();
  //private final Command shootAndDrive = new ShootAndDrive(driveTrain, feeder, shooter);
  //private final Command driveTriangle = new DriveTriangle();
  //private final Command driveXY = new DriveXY(x, y, angleDegrees, speed);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    //Set default commands for subsystems
    climber.setDefaultCommand(m_climbCommand);
    //RobotContainer.driveTrain.setDefaultCommand(m_driveCommand);
    swerveDrive.setDefaultCommand(new SwerveJoystickCmd(
                swerveDrive,
                () -> -driveController.getRawAxis(1), //Left Stick Y axis
                () -> driveController.getRawAxis(0), //Left Stick X Axis
                () -> driveController.getRawAxis(4), //Right Stick X axis
                () -> !driveController.getRawButton(6))); // Right Bumper Field Oriented Flag 
    //Don't want a default feeder command, tied to command groups?
    //m_feeder.setDefaultCommand(m_feederCollect); 
    intake.setDefaultCommand(m_intakeHoldRetracted);
    shooter.setDefaultCommand(m_manualShooterSpeed);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //Driver Controls
    //Only binding commands for subsytems that need to change from non-default command, i.e. Operator Controls for now

    //Operator Controls
    //Extend Intake
    new JoystickButton(shooterController, XboxController.Button.kY.value).whileHeld(m_intakePosition);
    
    //Retract Intake
    new JoystickButton(shooterController, XboxController.Button.kX.value).whileHeld(m_intakePosition);
    
    
    new JoystickButton(shooterController, XboxController.Button.kA.value)
      .whenHeld(new FeederCollect(feeder, 0.5)
      .alongWith(new IntakeCollect(intake, Constants.IntakeConstants.kIntakeCollectSpeed))
      );
     

    //Shoot - Feeder
    new JoystickButton(shooterController, XboxController.Button.kB.value)
      .whenHeld(new FeederShoot(feeder, 1.0));


    //Manual Shoot Button
    //new JoystickButton(shooterController, XboxController.Button.kB.value).whileHeld(m_manualShootCommand);
 
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public CommandBase getAutonomousCommand() {
    // An ExampleCommand will run in autonomous

    SendableChooser<CommandBase> chooser = new SendableChooser<>();
    
    chooser.addOption("Do Nothing", new DoNothing());
    chooser.addOption("Forward 10", new DriveXY(10, 0, 0, 0.2));
    chooser.addOption("Forward 25", new DriveXY(25, 0, 0, 0.2));
    chooser.addOption("Forward 50", new DriveXY(50, 0, 0, 0.2));
    chooser.addOption("Shoot and Drive", new ShootAndDrive(swerveDrive, feeder, shooter));
    chooser.setDefaultOption("Forward 100", new DriveXY(100, 0, 0, 0.2));
    chooser.addOption("Left 100", new DriveXY(0, 100, 0, 0.2));
    chooser.addOption("Turn +45", new TurnToAngle(45));
    chooser.addOption("Turn -90", new TurnToAngle(-90));
    
    SmartDashboard.putData("Auto mode", chooser);
    
    return getAutonomousCommand();
  }
}