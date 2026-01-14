package frc.robot.subsystems.Shooter.Roller;

// --- 1. CTRE Phoenix 6 函式庫 ---
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.AngularVelocityUnit;
// --- 2. WPILib Units 函式庫 (2025/2026 結構) ---
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
// 靜態引入 Units 類別，這樣才能直接用 RotationsPerSecond
import static edu.wpi.first.units.Units.*; 

public class RollerHardware implements RollerIO {
    
    private final TalonFX shooter = new TalonFX(15);
    
    private final StatusSignal<AngularVelocity> velocitySignal = shooter.getVelocity();

    private final MotionMagicTorqueCurrentFOC m_request = new MotionMagicTorqueCurrentFOC(0);
    
    public static final double MotorToRPM = 1.0; 

    public RollerHardware() {
        this.configureMotors();
    }

    public void configureMotors() {
        TalonFXConfiguration configs = new TalonFXConfiguration();

        // 電流限制
        configs.CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(70.0)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40.0);


        configs.CurrentLimits.withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(80));

        // 馬達設定
        configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // PID & FF
        configs.Slot0.kS = 0.25;
        configs.Slot0.kV = 0.12;
        configs.Slot0.kA = 0.01;
        configs.Slot0.kP = 0.11;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.0;

        // Motion Magic
        configs.MotionMagic.MotionMagicAcceleration = 400; 
        configs.MotionMagic.MotionMagicJerk = 4000; 

        // 齒輪比
        configs.Feedback.SensorToMechanismRatio = MotorToRPM;

        shooter.getConfigurator().apply(configs);
    }
// --- 關鍵修改 1: 輸入直接接受單位物件 ---
    @Override
    public void setRPM(AngularVelocity RPM) {
        // 這一行是精髓！
        // .in(RotationsPerSecond) 會自動把你的 RPM 轉成 Phoenix 6 需要的 RPS
        double targetRPS = RPM.in(RotationsPerSecond);

        shooter.setControl(m_request.withVelocity(targetRPS));
    }

    public AngularVelocity getSpeed() {
        // 1. 刷新訊號
        velocitySignal.refresh();
        
        // 2. 轉換單位
        // velocitySignal.getValue() -> 拿到 Double (RPS)
        // RotationsPerSecond.of() -> 轉成 WPILib 的速度物件
        return RotationsPerSecond.of(velocitySignal.getValueAsDouble());
    }
}