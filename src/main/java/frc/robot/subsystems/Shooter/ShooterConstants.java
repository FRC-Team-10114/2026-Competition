package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;

public class ShooterConstants {
        public static Transform3d robotToTurret = new Transform3d(0.2, 0.0, 0.44, Rotation3d.kZero);

        public static final double HARD_MIN_RADS = Units.degreesToRadians(-360.0);
        public static final double HARD_MAX_RADS = Units.degreesToRadians(360.0);

        public static final double SOFT_MIN_RADS = Units.degreesToRadians(-340.0);
        public static final double SOFT_MAX_RADS = Units.degreesToRadians(340.0);

        public static final Angle Hood_MAX_RADS = Degree.of(55); // 上限63
        public static final Angle Hood_MIN_RADS = Degree.of(28); // 下限25

        public static final double Hood_GEAR_RATIO = (1.0 / 0.0181);
        public static final double HoodCancoder_GEAR_RATIO_TOMotor = (1.0 / 0.0956);

        public static final double Flywheel_GEAR_RATIO = 1.0 / (12.0 / 16.0);
}
