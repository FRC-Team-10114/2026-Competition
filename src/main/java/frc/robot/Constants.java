package frc.robot;

public class Constants {

    public static final class SwerveModuleConstants {
        public static final String[] ModuleName = {
                "ForntLeft",
                "FrontRight",
                "BackLeft",
                "BackRight"
        };
    }

    public static final class IntakeConstants {
        public static final int INTAKE_MOTOR_ID = 13;
    }

    public static final class VisionConstants {
        public static final double MAX_GYRO_RATE = 1080;
    }

    public static final class HoodConstants {
        private final double GEAR_RATIO = 1; // 假設減速比 50:1
        private final double POSITION_CONVERSION_FACTOR = (2 * Math.PI) / GEAR_RATIO;
    }
}
