package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import org.checkerframework.dataflow.constantpropagation.Constant;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

import edu.wpi.first.wpilibj.I2C;

public class SwerveDrive extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private static final double kPgain = 0.040;
    private static final double kDgain = 0.0;

    private final AHRS gyro = new AHRS(I2C.Port.kOnboard);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new Rotation2d(0));

    public SwerveDrive() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(pose, getRotation2d());
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(),
                backRight.getState());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }


    //Below is taken from Old Code to get auton working hopefully
    public void driveHeading(Translation2d translation, double heading) {

        double angle = getAngle();
        double currentAngularRate = getAngularRate();
        double angle_error = angleDelta(heading, angle);
        double yawCommand = -angle_error * kPgain - (currentAngularRate) * kDgain;

        drive(translation, yawCommand, true);
    }

    static public double angleDelta(double src, double dest) {
        double delta = (dest - src) % 360.0;
        if (Math.abs(delta) > 180) {
            delta = delta - (Math.signum(delta) * 360);
        }
        return delta;
    }

    public double getAngle() {
        return -gyro.getAngle();
    }

    public double getAngularRate() {
        return -gyro.getRate();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldOriented) {
        rotation *= 2.0 / Math.hypot(Constants.DriveConstants.kWheelBase, Constants.DriveConstants.kTrackWidth);
        ChassisSpeeds speeds;
        SmartDashboard.putBoolean("Field-Oriented", fieldOriented);
        if (fieldOriented) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation,
                    Rotation2d.fromDegrees(getAngle()));
        } else {
            speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        }

        //FIXME Need to sort out to drive in Auton I think
        // SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        // frontLeft.setTargetVelocity(states[0].speedMetersPerSecond, states[0].angle.getRadians());
        // frontRight.setTargetVelocity(states[1].speedMetersPerSecond, states[1].angle.getRadians());
        // backLeft.setTargetVelocity(states[2].speedMetersPerSecond, states[2].angle.getRadians());
        // backRight.setTargetVelocity(states[3].speedMetersPerSecond, states[3].angle.getRadians());
    }
}