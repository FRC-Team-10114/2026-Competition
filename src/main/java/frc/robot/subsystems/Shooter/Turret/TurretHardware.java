package frc.robot.subsystems.Shooter.Turret;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.util.MathHelper.EncoderWithGearRatio;
import frc.robot.util.MathHelper.RobustCRTCalculator;

public class TurretHardware implements TurretIO {
    private final TalonFX Turretd = new TalonFX(20);

    private final CANcoder encoder1, encoder2;

    private final double metersPerangle = 1.0;

    private static final double HARD_MIN_RADS = ShooterConstants.HARD_MIN_RADS;
    private static final double HARD_MAX_RADS = ShooterConstants.HARD_MAX_RADS;

    private static final double SOFT_MIN_RADS = ShooterConstants.SOFT_MIN_RADS;
    private static final double SOFT_MAX_RADS = ShooterConstants.SOFT_MAX_RADS;

    public enum ShootState {
        ACTIVE_SHOOTING, // 準備開火：使用全範圍 (Hard Limits)
        TRACKING // 純追蹤/待機：使用縮小範圍 (Soft Limits)
    }

    private double lastSetpointRads = 0.0;

    private final MotionMagicVoltage m_request = new MotionMagicVoltage(Degree.of(0));

    public TurretHardware() {
        this.encoder1 = new CANcoder(21);
        this.encoder2 = new CANcoder(22);
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

        configs.SoftwareLimitSwitch
                // 反向限位 (防止往下撞壞)
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(HARD_MIN_RADS)

                // 正向限位 (防止往上飛出去) - 強烈建議開啟保護硬體
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(HARD_MAX_RADS);

        // 馬達設定
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // PID & FF
        configs.Slot0.kS = 0.25;
        configs.Slot0.kV = 0.12;
        configs.Slot0.kA = 0.01;
        configs.Slot0.kP = 0.11;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.0;

        // Motion Magic
        configs.MotionMagic.withMotionMagicCruiseVelocity(DegreesPerSecond.of(360))
                .withMotionMagicAcceleration(DegreesPerSecondPerSecond.of(720));

        // 齒輪比
        configs.Feedback.SensorToMechanismRatio = metersPerangle;

        Turretd.getConfigurator().apply(configs);
    }

    public void setAngle(Rotation2d robotHeading, Angle targetRad, ShootState state) {
        Turretd.setControl(
                m_request.withPosition(this.TurretCalculate(robotHeading, targetRad, state)));
    }

    public void resetAngle() {

        EncoderWithGearRatio encoder1 = new EncoderWithGearRatio(
                this.encoder1.getPosition().getValueAsDouble(),
                30);

        EncoderWithGearRatio encoder2 = new EncoderWithGearRatio(
                this.encoder2.getPosition().getValueAsDouble(),
                31);

        double angle = RobustCRTCalculator.calculateAbsolutePosition(encoder1, encoder2);
        Turretd.getConfigurator().setPosition(angle);
    }

    public Angle TurretCalculate(Rotation2d robotHeading, Angle targetRad, ShootState state) {

        // 1. 根據狀態決定目前的 "合法範圍"
        double currentMin, currentMax;

        if (state == ShootState.ACTIVE_SHOOTING) {
            // 射擊時：火力全開，使用硬體極限
            currentMin = HARD_MIN_RADS;
            currentMax = HARD_MAX_RADS;
        } else {
            // 追蹤時：保守一點，使用軟體限制
            currentMin = SOFT_MIN_RADS;
            currentMax = SOFT_MAX_RADS;
        }

        // 1. 先把單位 (Measure) 轉成幾何 (Rotation2d)
        Rotation2d targetRotation = Rotation2d.fromRadians(targetRad.in(Radians));

        // 2. 現在兩個都是 Rotation2d 了，可以相減
        Rotation2d relativeGoal = targetRotation.minus(robotHeading);

        // 3. 取出弧度數值
        double baseAngleRads = relativeGoal.getRadians();

        // 3. 窮舉候選角度 (解繞核心)
        double bestAngle = 0.0;
        boolean foundValidAngle = false;

        for (int i = -2; i <= 2; i++) {
            double candidate = baseAngleRads + (Math.PI * 2.0 * i);

            // 使用動態決定的 currentMin/Max 來過濾
            if (candidate >= currentMin && candidate <= currentMax) {

                if (!foundValidAngle) {
                    bestAngle = candidate;
                    foundValidAngle = true;
                } else {
                    // 如果有多個合法角度，選離上次目標最近的
                    if (Math.abs(lastSetpointRads - candidate) < Math
                            .abs(lastSetpointRads - bestAngle)) {
                        bestAngle = candidate;
                    }
                }
            }
        }

        // 防呆：如果在 Soft Limit 找不到角度 (極罕見)，試著放寬到 Hard Limit 找一次
        // 避免因為 Soft Limit 設太窄導致追蹤丟失
        if (!foundValidAngle && state == ShootState.TRACKING) {
            return TurretCalculate(robotHeading, targetRad, ShootState.ACTIVE_SHOOTING);
        }

        if (!foundValidAngle) {
            // 真的完全轉不到 (超出硬體極限)
            return Radians.of(lastSetpointRads);
        }

        lastSetpointRads = bestAngle;
        return Radians.of(bestAngle);
    }
}
