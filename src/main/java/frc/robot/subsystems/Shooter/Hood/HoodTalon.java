package frc.robot.subsystems.Shooter.Hood;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.Intake.IntakeConstants.ArmConstants;
import frc.robot.subsystems.Shooter.ShooterConstants;

public class HoodTalon implements HoodIO {

    private final TalonFX HoodMotor;

    private final StatusSignal<Angle> HoodPosition;

    private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    public HoodTalon() {

        this.HoodMotor = new TalonFX(22);
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

        configs.CurrentLimits.withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amp.of(60));
        // 馬達設定
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

configs.SoftwareLimitSwitch
        // 反向限位 (防止往下撞壞)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(Units.degreesToRotations(1.44)) 
        
        // 正向限位 (防止往上飛出去) - 強烈建議開啟保護硬體
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(Units.degreesToRotations(30));
        // PID & FF
        configs.Slot0.kS = 0.0;
        configs.Slot0.kV = 0.0;
        configs.Slot0.kA = 0.0;
        configs.Slot0.kP = 120.0;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.0;

        // Motion Magic
        configs.MotionMagic.withMotionMagicCruiseVelocity(2).withMotionMagicAcceleration(3);

        // 齒輪比
        configs.Feedback.SensorToMechanismRatio = ShooterConstants.Hood_GEAR_RATIO;

        HoodMotor.getConfigurator().apply(configs);
    }

    @Override
    public void setAngle(Angle angle) {
        HoodMotor.setControl(
                m_request.withPosition(Units.radiansToRotations(angle.in(Radians))));
    }

    @Override
    public double getAngle() {
        return this.HoodMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void reset() {
        // 將當前位置設為 0 圈
        this.HoodMotor.setPosition(0.0);
    }
}
