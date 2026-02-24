package frc.robot.subsystems.Shooter.Turret;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.IDs;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.util.MathHelper.PositionWithGearRatio;
import frc.robot.util.MathHelper.RobustCRTCalculator;

public class TurretIOTalon extends TurretIO {
    private final TalonFX turretMotor;
    private final CANcoder master, slave;

    // 依照 Spark 版本，你的齒輪比應該是這個（如果硬體相同）
    private final double gearRatio = (96.0 / 16.0) * 3.0;

    private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    private final VoltageOut m_voltageRequest = new VoltageOut(0);

    // 🟢 【新增】SysIdRoutine
    private final SysIdRoutine sysIdRoutine;

    public TurretIOTalon() {
        this.turretMotor = new TalonFX(IDs.Shooter.TURRET_MOTOR);
        this.master = new CANcoder(IDs.Shooter.TURRET_MASTER_CANCODER);
        this.slave = new CANcoder(IDs.Shooter.TURRET_SLAVE_CANCODER);

        // ==========================================
        // 🟢 【新增】初始化 SysId 測試機制 (TalonFX 版本)
        // ==========================================
        this.sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        // 1. 給電壓 (使用 Phoenix 6 的 VoltageOut)
                        (voltage) -> turretMotor.setControl(m_voltageRequest.withOutput(voltage.in(Volts))),
                        // 2. 記錄狀態
                        log -> {
                            log.motor("TurretMotor")
                                    .voltage(Volts.of(turretMotor.getMotorVoltage().getValueAsDouble()))
                                    .angularPosition(turretMotor.getPosition().getValue()) // Talon 直接回傳 Angle 單位
                                    .angularVelocity(turretMotor.getVelocity().getValue()); // Talon 直接回傳 Velocity 單位
                        },
                        // 3. 虛擬子系統綁定
                        new SubsystemBase() {
                            @Override
                            public String getName() {
                                return "TurretSysId";
                            }
                        }));

        this.configureMotors();
        this.resetAngle();
        initStartingPosition();
    }

    private void initStartingPosition() {

        StatusSignal<Angle> currentPos = turretMotor.getPosition();

        currentPos.waitForUpdate(0.25);

        m_request.Position = currentPos.getValueAsDouble();
    }

    // @Override
    // public void CANcoderConfig() {
    // var cfg = new CANcoderConfiguration();

    // double targetSensorRotations = Units.degreesToRotations(25.0) *
    // sensorToMechRatio;

    // cfg.MagnetSensor.MagnetOffset = 0.128662109375 + targetSensorRotations;

    // cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    // cfg.MagnetSensor.SensorDirection =
    // SensorDirectionValue.CounterClockwise_Positive;

    // Hoodcancoder.getConfigurator().apply(cfg);
    // }

    public void configureMotors() {
        TalonFXConfiguration configs = new TalonFXConfiguration();

        // 電流限制
        configs.CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(70.0)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40.0);

        configs.SoftwareLimitSwitch
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(Units.radiansToRotations(ShooterConstants.HARD_MIN_RADS))
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(Units.radiansToRotations(ShooterConstants.HARD_MAX_RADS));

        // 馬達設定
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // PID & FF (建議保留 Slot0 供 Motion Magic 使用)
        configs.Slot0.kS = 0.25;
        configs.Slot0.kV = 0.12;
        configs.Slot0.kA = 0.01;
        configs.Slot0.kP = 0.11;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.0;

        // Motion Magic 設定
        configs.MotionMagic.withMotionMagicCruiseVelocity(DegreesPerSecond.of(360))
                .withMotionMagicAcceleration(DegreesPerSecondPerSecond.of(720));

        // 🟢 【關鍵】設定齒輪比，讓 getPosition 直接讀到旋轉台角度
        configs.Feedback.SensorToMechanismRatio = gearRatio;

        turretMotor.getConfigurator().apply(configs);
    }

    // ==========================================
    // 🟢 【新增】SysId 指令組 (從 Spark 版本移植)
    // ==========================================
    public Command sysIdQuasistaticForward() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
                .until(() -> this.getAngle().in(Radians) > ShooterConstants.SOFT_MAX_RADS);
    }

    public Command sysIdQuasistaticReverse() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
                .until(() -> this.getAngle().in(Radians) < ShooterConstants.SOFT_MIN_RADS);
    }

    public Command sysIdDynamicForward() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward)
                .until(() -> this.getAngle().in(Radians) > ShooterConstants.SOFT_MAX_RADS);
    }

    public Command sysIdDynamicReverse() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse)
                .until(() -> this.getAngle().in(Radians) < ShooterConstants.SOFT_MIN_RADS);
    }

    public Command sysid() {
        return Commands.sequence(
                sysIdQuasistaticForward(),
                new WaitCommand(1.5),
                sysIdQuasistaticReverse(),
                new WaitCommand(1.5),
                sysIdDynamicForward(),
                new WaitCommand(1.5),
                sysIdDynamicReverse());
    }

    @Override
    public void setAngle(Rotation2d robotHeading, Angle targetRad, ShootState state) {
        turretMotor.setControl(m_request.withPosition(Calculate(robotHeading, targetRad, state)));
    }

    @Override
    public void resetAngle() {
        turretMotor.setPosition(0.0);
    }

    @Override
    public Angle getAngle() {
        // 這依然回傳 Angle 物件，但在視覺上明確了我們關注的是 Radians
        return Radians.of(turretMotor.getPosition().getValue().in(Radians));
    }

    @Override
    public boolean isAtSetPosition() {
        // 誤差小於 2 度視為到達
        return Math.abs(turretMotor.getClosedLoopError().getValueAsDouble()) < (40.0 / 360.0);
    }
}