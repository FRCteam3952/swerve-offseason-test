package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controllers.FlightJoystick;
import frc.robot.subsystems.swerve.DriveTrainSubsystem;

public class ManualDrive extends CommandBase {
    private final DriveTrainSubsystem driveTrain;
    private final FlightJoystick joystick;

    public ManualDrive(DriveTrainSubsystem driveTrain, FlightJoystick joystick) {
        this.driveTrain = driveTrain;
        this.joystick = joystick;

        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        this.driveTrain.drive(this.joystick.getVerticalMovement(), this.joystick.getHorizontalMovement(), this.joystick.getRotation(), true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
