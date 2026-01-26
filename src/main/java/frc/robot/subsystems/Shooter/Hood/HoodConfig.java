package frc.robot.subsystems.Shooter.Hood;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class HoodConfig {
    
    public static SparkMaxConfig hoodConfig = new SparkMaxConfig();

    static {

        hoodConfig
                .idleMode(IdleMode.kBrake)
                .inverted(true)
                .smartCurrentLimit(25)
                .apply(hoodConfig);
        hoodConfig.softLimit
                .forwardSoftLimit(0.0)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(0.0)
                .reverseSoftLimitEnabled(true);
        hoodConfig.encoder
                .velocityConversionFactor(0)
                .positionConversionFactor(0)
                .apply(hoodConfig.encoder);
        hoodConfig.closedLoop
                .p(0.1)
                .i(0)
                .d(0)
                .apply(hoodConfig.closedLoop);
    }
}
