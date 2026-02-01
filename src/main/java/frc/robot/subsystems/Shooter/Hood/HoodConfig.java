package frc.robot.subsystems.Shooter.Hood;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.subsystems.Shooter.ShooterConstants;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotation;

import com.revrobotics.spark.config.SparkFlexConfig;

public class HoodConfig {
    
    public static SparkFlexConfig hoodConfig = new SparkFlexConfig();

    static {

        hoodConfig
                .idleMode(IdleMode.kBrake)
                .inverted(false)
                .smartCurrentLimit(40)
                .apply(hoodConfig);
        hoodConfig.softLimit
                .forwardSoftLimit(ShooterConstants.Hood_MAX_RADS)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(ShooterConstants.Hood_MIN_RADS)
                .reverseSoftLimitEnabled(true)
                .apply(hoodConfig.softLimit);
        hoodConfig.encoder
                .positionConversionFactor(Radians.convertFrom(ShooterConstants.Hood_GEAR_RATIO, Rotation))
                .apply(hoodConfig.encoder);
        hoodConfig.closedLoop
                .p(5.56)
                .i(0)
                .d(0.01)
                .apply(hoodConfig.closedLoop);
    }
}
