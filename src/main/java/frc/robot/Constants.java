package frc.robot;

public final class Constants {
    public static class OperatorConstants {
        public static final int RIGHT_JOYSTICK_PORT = 1;
        public static class ControllerConstants {
        }
    }

    public static class PortConstants {
        public static final int FRONT_LEFT_DRIVE_MOTOR_ID           = 0;
        public static final int FRONT_RIGHT_DRIVE_MOTOR_ID          = 0;
        public static final int BACK_LEFT_DRIVE_MOTOR_ID            = 0;
        public static final int BACK_RIGHT_DRIVE_MOTOR_ID           = 0;
        public static final int FRONT_LEFT_ROTATION_MOTOR_ID        = 0;
        public static final int FRONT_RIGHT_ROTATION_MOTOR_ID       = 0;
        public static final int BACK_LEFT_ROTATION_MOTOR_ID         = 0;
        public static final int BACK_RIGHT_ROTATION_MOTOR_ID        = 0;
        public static final int FRONT_LEFT_ROTATION_CANCODER_ID     = 0;
        public static final int FRONT_RIGHT_ROTATION_CANCODER_ID    = 0;
        public static final int BACK_LEFT_ROTATION_CANCODER_ID      = 0;
        public static final int BACK_RIGHT_ROTATION_CANCODER_ID     = 0;

    }

    public static class DriveConstants {

    }

    public static class RobotConstants {
        public static final double SIDE_LENGTH_INCHES = 30; // square

        public static final double DIAGONAL_LENGTH_INCHES = Math.sqrt(2) * SIDE_LENGTH_INCHES;
        public static final double SWERVE_MODULE_INSET_FROM_CORNER_CM = 9; // CM
    }

    private Constants() {
        throw new UnsupportedOperationException("This is a utility class and cannot be instantiated");
    }
}
