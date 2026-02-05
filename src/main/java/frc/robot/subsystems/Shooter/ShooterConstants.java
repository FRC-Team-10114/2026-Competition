package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class ShooterConstants {
        public static Transform3d robotToTurret = new Transform3d(0.2, 0.0, 0.44, Rotation3d.kZero);

        public static final double HARD_MIN_RADS = Units.degreesToRadians(-210.0);
        public static final double HARD_MAX_RADS = Units.degreesToRadians(210.0);

        public static final double SOFT_MIN_RADS = Units.degreesToRadians(-190.0);
        public static final double SOFT_MAX_RADS = Units.degreesToRadians(190.0);

        public static final double Hood_MAX_RADS = Units.degreesToRadians(58);
        public static final double Hood_MIN_RADS = Units.degreesToRadians(35);

        public static final double Hood_GEAR_RATIO = (1.0 / 0.0181);
        public static final double HoodCancoder_GEAR_RATIO_TOMotor = (1.0 / 0.0956);

        public static final double Flywheel_GEAR_RATIO = 1.0 / (12.0 / 16.0);
}
