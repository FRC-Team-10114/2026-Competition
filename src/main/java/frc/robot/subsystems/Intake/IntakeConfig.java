package frc.robot.subsystems.Intake;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeConfig {
    
    public static SparkMaxConfig MaxIntakeConfig = new SparkMaxConfig();

    public static TalonFXConfiguration TalonIntakeConfig = new TalonFXConfiguration();

    static {
        MaxIntakeConfig
                .idleMode(IdleMode.kBrake)
                .inverted(false)
                .smartCurrentLimit(30);

        TalonIntakeConfig.CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(50)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(30.0);

        TalonIntakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        TalonIntakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }
}