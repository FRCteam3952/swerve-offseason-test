// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.staticsubsystems.RobotGyro;
import frc.robot.Constants.PortConstants;

/**
 * Represents a swerve drive style drivetrain.
 */
public class DriveTrainSubsystem extends SubsystemBase {
    public static final double MAX_SPEED = 3.0; // 3 meters per second
    public static final double MAX_ANGULAR_SPEED = Math.PI; // 1/2 rotation per second


    // Location of each swerve drive, relative to motor center. +X -> moving to front of robot, +Y -> moving to left of robot. IMPORTANT.
    private final Translation2d frontLeftLocation = new Translation2d(0.381, 0.381);
    private final Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
    private final Translation2d backLeftLocation = new Translation2d(-0.381, 0.381);
    private final Translation2d backRightLocation = new Translation2d(-0.381, -0.381);

    private final SwerveModule frontLeft = new SwerveModule(
            PortConstants.FRONT_LEFT_DRIVE_MOTOR_ID,
            PortConstants.FRONT_LEFT_ROTATION_MOTOR_ID,
            PortConstants.FRONT_LEFT_ROTATION_CANCODER_ID
    );
    private final SwerveModule frontRight = new SwerveModule(
            PortConstants.FRONT_RIGHT_DRIVE_MOTOR_ID,
            PortConstants.FRONT_RIGHT_ROTATION_MOTOR_ID,
            PortConstants.FRONT_RIGHT_ROTATION_CANCODER_ID
    );
    private final SwerveModule backLeft = new SwerveModule(
            PortConstants.BACK_LEFT_DRIVE_MOTOR_ID,
            PortConstants.BACK_LEFT_ROTATION_MOTOR_ID,
            PortConstants.BACK_LEFT_ROTATION_CANCODER_ID
    );
    private final SwerveModule backRight = new SwerveModule(
            PortConstants.BACK_RIGHT_DRIVE_MOTOR_ID,
            PortConstants.BACK_RIGHT_ROTATION_MOTOR_ID,
            PortConstants.BACK_RIGHT_ROTATION_CANCODER_ID
    );

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, RobotGyro.getRotation2d(),
            new SwerveModulePosition[] {
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    backLeft.getPosition(),
                    backRight.getPosition()
    });

    public DriveTrainSubsystem() {
        RobotGyro.resetGyroAngle();
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        var swerveModuleStates = kinematics.toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, RobotGyro.getRotation2d()) : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_SPEED);
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Updates the field relative position of the robot.
     */
    public void updateOdometry() {
        odometry.update(RobotGyro.getRotation2d(), new SwerveModulePosition[]{frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()});
    }
}
