package frc.robot.subsystems.Shooter.Hood;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class HoodConfig {
    
    static {
        SparkMaxConfig hoodConfig = new SparkMaxConfig();

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
        
    }
}
