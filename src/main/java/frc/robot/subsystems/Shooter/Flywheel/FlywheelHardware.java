package frc.robot.subsystems.Shooter.Flywheel;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
// 修正 1: 改用速度控制專用的 Request
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import static edu.wpi.first.units.Units.*;

public class FlywheelHardware implements FlywheelIO {

    private final TalonFX shooter = new TalonFX(15);

    private final StatusSignal<AngularVelocity> velocitySignal = shooter.getVelocity();

    private final VelocityTorqueCurrentFOC m_request = new VelocityTorqueCurrentFOC(0);

    public FlywheelHardware() {
        this.configureMotors();
    }

    public void configureMotors() {
        TalonFXConfiguration configs = new TalonFXConfiguration();

        // 修正 3: 電流限制寫法精簡化 (你原本寫了兩次 SupplyLimit)
        // Stator (定子電流): 限制加速時的爆發力 -> 設 60-80A 防止燒馬達
        // Supply (供應電流): 限制電池端的耗電 -> 設 40A 防止像上次那樣電壓驟降
        configs.CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(80.0)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40.0);

        // 馬達設定
        configs.MotorOutput.NeutralMode = NeutralModeValue.Coast; // Roller 通常用 Coast，停下來比較滑順
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // 修正 4: PID 參數 (Torque 模式專用)
        // 這些數值需要重新用 SysId 測量，或者手動調整
        // 這裡給的是 "經驗法則" 的預估值，比你原本的大很多

        configs.Slot0.kP = 5.0; // 誤差 1 RPS，給 5 安培修正 (比 0.11 有力多了)
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.0;

        // Torque 模式下，kV 通常很小或為 0 (TalonFX 內部會處理反電動勢)
        // 這裡的 kV 只是用來對抗空氣阻力和摩擦力
        configs.Slot0.kV = 0.05;

        // kS (靜摩擦): 要給多少安培才推得動？
        configs.Slot0.kS = 0.5;

        // 修正 5: 移除 MotionMagic 設定
        // 因為我們現在是用 Velocity 模式，不需要設定 CruiseVelocity 和 Acceleration
        // 如果你希望加速不要太快，可以用 configs.ClosedLoopRamps.TorqueClosedLoopRampPeriod

        shooter.getConfigurator().apply(configs);
    }

    @Override
    public void setRPM(AngularVelocity RPM) {
        // 單位轉換正確
        double targetRPS = RPM.in(RotationsPerSecond);

        // 使用 Velocity Request
        shooter.setControl(m_request.withVelocity(targetRPS));
    }

    public AngularVelocity getSpeed() {
        velocitySignal.refresh();
        return velocitySignal.getValue(); // Phoenix 6 的 Units 版本會直接回傳 AngularVelocity 物件
    }
}