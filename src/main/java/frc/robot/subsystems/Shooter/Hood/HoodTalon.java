package frc.robot.subsystems.Shooter.Hood;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.google.gson.annotations.Until;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.Intake.IntakeConstants.ArmConstants;
import frc.robot.subsystems.Shooter.ShooterConstants;

public class HoodTalon implements HoodIO {

    private final TalonFX HoodMotor;

    private final StatusSignal<Angle> HoodPosition;

    private final CANcoder Hoodcancoder;

    private final MotionMagicVoltage m_request = new MotionMagicVoltage(Degree.of(0));

    public HoodTalon() {

        this.HoodMotor = new TalonFX(22);
        this.Hoodcancoder = new CANcoder(55);
        this.HoodPosition = HoodMotor.getPosition();

        configureMotors();
    }

    public void configureMotors() {
        TalonFXConfiguration configs = new TalonFXConfiguration();

        // 電流限制
        configs.CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(70.0)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40.0);

        // 馬達設定
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        configs.SoftwareLimitSwitch
                // 反向限位 (防止往下撞壞)
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(Degree.of(1.44))

                // 正向限位 (防止往上飛出去) - 強烈建議開啟保護硬體
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(Degree.of(30));
        configs.Feedback
                .withFeedbackRemoteSensorID(55)// 這個 TalonFX 要讀取 ID 30 的 CANcoder 作為回饋
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)// 指定要使用 Fused CANcoder（整合了磁性角度與速度的
                                                                                  // sensor）作為回饋信號 簡單來説就是跟馬達encoder結合
                .withSensorToMechanismRatio(ShooterConstants.HoodCancoder_GEAR_RATIO)// 齒輪比1
                .withRotorToSensorRatio( ShooterConstants.Hood_GEAR_RATIO);// 將馬達軸的旋轉換算成手臂旋轉
        // PID & FF
        configs.Slot0.kS = 0.0;
        configs.Slot0.kV = 0.0;
        configs.Slot0.kA = 0.0;
        configs.Slot0.kG = 0.0;
        // 誤差一圈給60v 也就是1°給0.16v
        configs.Slot0.kP = 60.0;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.0;
        configs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        // Motion Magic
        configs.MotionMagic.withMotionMagicCruiseVelocity(DegreesPerSecond.of(720))
                .withMotionMagicAcceleration(DegreesPerSecondPerSecond.of(1080));
        HoodMotor.getConfigurator().apply(configs);
    }

    @Override
    public void setAngle(Angle angle) {
        HoodMotor.setControl(m_request.withPosition(angle));
    }

    @Override
    public double getAngle() {
        this.HoodPosition.refresh();

        return this.HoodPosition.getValue().in(Degrees);
    }

    @Override
    public void reset() {
        // 將當前位置設為 0 圈
        this.HoodMotor.setPosition(Degree.of(0));
    }
}
