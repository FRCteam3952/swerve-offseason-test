package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.ManualDrive;
import frc.robot.controllers.FlightJoystick;
import frc.robot.subsystems.swerve.DriveTrainSubsystem;
import frc.robot.subsystems.staticsubsystems.LimeLight;
import frc.robot.subsystems.staticsubsystems.RobotGyro;
import frc.robot.Constants.OperatorConstants;

public class RobotContainer {
    private final DriveTrainSubsystem driveTrain = new DriveTrainSubsystem();

    public final FlightJoystick driverController = new FlightJoystick(new CommandJoystick(OperatorConstants.RIGHT_JOYSTICK_PORT));

    public RobotContainer() {
        configureBindings();

        LimeLight.poke();
        RobotGyro.poke();
    }

    private void configureBindings() {

    }

    public void onRobotInit() {

    }

    public void onAutonInit() {

    }

    public void onTeleopInit() {
        this.driveTrain.setDefaultCommand(new ManualDrive(this.driveTrain, this.driverController));
    }
}
