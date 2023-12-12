// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SwerveModule {
    private static final double MODULE_MAX_ANGULAR_VELOCITY = DriveTrainSubsystem.MAX_ANGULAR_SPEED;
    private static final double MODULE_MAX_ANGULAR_ACCELERATION = 2 * Math.PI; // radians per second squared

    private static final double SWERVE_MODULE_PLS_NO_EXPLODE_MAX_SPEED = 0.3;
    private static final double SWERVE_MODULE_PLS_NO_EXPLODE_MIN_SPEED = -0.3;

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final CANCoder turningEncoder;

    // Gains are for example purposes only - must be determined for your own robot!
    private final PIDController drivePIDController = new PIDController(1, 0, 0);

    // Gains are for example purposes only - must be determined for your own robot!
    private final ProfiledPIDController turningPIDController = new ProfiledPIDController(1, 0, 0,
            new TrapezoidProfile.Constraints(MODULE_MAX_ANGULAR_VELOCITY, MODULE_MAX_ANGULAR_ACCELERATION));

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(1, 3);
    private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
     *
     * @param driveMotorCANID      CAN ID for the drive motor.
     * @param turningMotorCANID    CAN ID for the turning motor.
     * @param turningEncoderCANID DIO input for the turning encoder channel A
     */
    public SwerveModule(int driveMotorCANID, int turningMotorCANID, int turningEncoderCANID) {
        driveMotor = new CANSparkMax(driveMotorCANID, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorCANID, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = new CANCoder(turningEncoderCANID);

        // Set the distance per pulse for the drive encoder. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        // driveEncoder.setDistancePerPulse(2 * Math.PI * WHEEL_RADIUS / ENCODER_RESOLUTION); // EXPERIMENT LATER.

        // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
        // This is the angle through an entire rotation (2 * pi) divided by the
        // encoder resolution.

        // WE NEED RADIANS I THINK
        // MAGIC NUMBER FROM DOCUMENTATION COMMENT ON METHOD COPY-PASTED. THIS SHOULD NOT BE TOUCHED UNLESS MASSIVE BREAK.
        turningEncoder.configFeedbackCoefficient(0.087890625 * Math.PI / 180, "rad", SensorTimeBase.PerSecond);
        // turningEncoder.setDistancePerPulse(2 * Math.PI / ENCODER_RESOLUTION); // template code, ignore.

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(turningEncoder.getPosition()));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(turningEncoder.getPosition()));
    }

    public void setVoltages(double driveVoltage, double turnVoltage) {
        driveMotor.setVoltage(driveVoltage);
        turningMotor.setVoltage(turnVoltage);
    }

    public void stop() {
        setVoltages(0, 0);
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(turningEncoder.getPosition()));

        // Calculate the drive output from the drive PID controller.
        final double driveOutput = drivePIDController.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond);

        final double driveFeedforward = this.driveFeedforward.calculate(state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = turningPIDController.calculate(turningEncoder.getPosition(), state.angle.getRadians());

        final double turnFeedforward = this.turnFeedforward.calculate(turningPIDController.getSetpoint().velocity);
        System.out.println("drive output: " + (driveOutput + driveFeedforward));
        System.out.println("turn output: " + (turnOutput + turnFeedforward));
        //driveMotor.setVoltage(MathUtil.clamp(driveOutput + driveFeedforward, SWERVE_MODULE_PLS_NO_EXPLODE_MIN_SPEED, SWERVE_MODULE_PLS_NO_EXPLODE_MAX_SPEED));
        //turningMotor.setVoltage(MathUtil.clamp(turnOutput + turnFeedforward, SWERVE_MODULE_PLS_NO_EXPLODE_MIN_SPEED, SWERVE_MODULE_PLS_NO_EXPLODE_MAX_SPEED));
    }
}
