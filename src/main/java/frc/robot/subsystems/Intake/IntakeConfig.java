package frc.robot.subsystems.Intake;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeConfig {
    
    // ✅ 修正 1：加上 = new SparkMaxConfig(); 進行初始化
    public static SparkMaxConfig MaxIntakeConfig = new SparkMaxConfig();

    // ✅ 修正 2：加上 = new TalonFXConfiguration(); 進行初始化
    public static TalonFXConfiguration TalonIntakeConfig = new TalonFXConfiguration();

    static {
        // 設定 Spark Max
        MaxIntakeConfig
                .idleMode(IdleMode.kBrake)
                .inverted(false)
                .smartCurrentLimit(30);
        // ❌ 刪除 .apply(MaxIntakeConfig); 
        // 這一行在 Config 設定檔裡是不需要的，套用設定是在 IntakeIOSpark.java 裡做的

        // 設定 Talon FX
        TalonIntakeConfig.CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(50)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(30.0);

        TalonIntakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        TalonIntakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }
}