package frc.robot.commands;

import org.frcteam2910.common.robot.Utilities;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class DriveCommand extends CommandBase {
    private final DriveTrain m_driveTrain;

    public DriveCommand(DriveTrain driveTrain) {
        m_driveTrain = driveTrain;
        
        addRequirements(m_driveTrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double forward = -RobotContainer.driveController.getRawAxis(1);
        forward = Utilities.deadband(forward);
        // Square the forward stick
        forward = Math.copySign(Math.pow(forward, 2.0), forward);

        double strafe = -RobotContainer.driveController.getRawAxis(0);
        strafe = Utilities.deadband(strafe);
        // Square the strafe stick
        strafe = Math.copySign(Math.pow(strafe, 2.0), strafe);

        double rotation = -RobotContainer.driveController.getRawAxis(4);
        rotation = Utilities.deadband(rotation, 0.040);
        // Square the rotation stick
        rotation = Math.copySign(Math.pow(rotation, 2.0), rotation);

        boolean fieldOrientedFlag;

        fieldOrientedFlag = !RobotContainer.driveController.getRawButton(6);

        //Reset Command for Gyro
        if(RobotContainer.driveController.getRawButtonPressed(5)) {
            RobotContainer.driveTrain.resetYaw();
        }

        RobotContainer.driveTrain.drive(new Translation2d(forward, strafe), rotation, fieldOrientedFlag);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}
