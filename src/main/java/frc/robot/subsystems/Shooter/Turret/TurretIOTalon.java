package frc.robot.subsystems.Shooter.Turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.logging.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.IDs;
import frc.robot.subsystems.Shooter.ShooterConstants;

public class TurretIOTalon extends TurretIO {
    private final TalonFX turretMotor;
    private final CANcoder turretCaNcoder;
    private final StatusSignal<Angle> turretPosition;

    private final double gearRatio = (96.0 / 16.0) * 3.0;

    private final PositionVoltage m_request = new PositionVoltage(0);
    private final VoltageOut voltagRequire = new VoltageOut(0.0);
    public Angle goal;

    private final SysIdRoutine sysIdRoutine;

    public TurretIOTalon() {
        this.turretMotor = new TalonFX(IDs.Shooter.TURRET_MOTOR, "canivore");
        this.turretCaNcoder = new CANcoder(IDs.Shooter.TURRET_Cancoder, "canivore");
        this.turretPosition = turretMotor.getPosition();

      SignalLogger.setPath("/U/");

        this.sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(Volts.of(0.5).per(Second), Volts.of(3),
                        null, (state) -> SignalLogger.writeString("state", state.toString())),
                new SysIdRoutine.Mechanism(
                        (volts) -> this.turretMotor.setControl(voltagRequire.withOutput(volts.in(Volts))),
                        null,
                        // 🟢 修正 1：給予一個虛擬的 SubsystemBase，避免 IO 層轉型失敗當機
                        new SubsystemBase() {
                            @Override
                            public String getName() {
                                return "TurretSysId";
                            }
                        }));

        this.CANcoderConfig();
        configureMotors();
    }

    public void CANcoderConfig() {
        var cfg = new CANcoderConfiguration();
        cfg.MagnetSensor.MagnetOffset = 0.448486328125;
        cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        turretCaNcoder.getConfigurator().apply(cfg);
    }

    public void configureMotors() {
        TalonFXConfiguration configs = new TalonFXConfiguration();

        configs.CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(70.0)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40.0);

        configs.SoftwareLimitSwitch
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(ShooterConstants.HARD_MIN_LIMIT)
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(ShooterConstants.HARD_MAX_LIMIT);

        configs.Feedback
                .withFeedbackRemoteSensorID(IDs.Shooter.TURRET_Cancoder)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                .withRotorToSensorRatio(-36.0)
                .withSensorToMechanismRatio(0.5);

        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        configs.Slot0.kS = 0.63542;
        configs.Slot0.kV = 1.5255;
        configs.Slot0.kA = 0.13204;
        configs.Slot0.kP = 42.0;
        configs.Slot0.kD = 1.5;

        turretMotor.getConfigurator().apply(configs);
    }

    @Override
    public void setAngle(Rotation2d robotHeading, Angle targetRad, ShootState state) {
        turretMotor.setControl(m_request.withPosition(Calculate(robotHeading, targetRad, state)));
    }

    @Override
    public Angle getAngle() {
        this.turretPosition.refresh();
        return Radians.of(turretPosition.getValue().in(Radians));
    }

    @Override
    public Angle getAnglegoal() {
        return goal;
    }

    @Override
    public boolean isAtSetPosition() {
        return Math.abs(turretMotor.getClosedLoopError().getValueAsDouble()) < (40.0 / 360.0);
    }

    // ==========================================
    // 🟢 【終極修正版】SysId 測試指令
    // ==========================================
    @Override
    public Command sysid() {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    SignalLogger.start();
                    turretMotor.getPosition().setUpdateFrequency(250);
                    turretMotor.getVelocity().setUpdateFrequency(250);
                    turretMotor.getMotorVoltage().setUpdateFrequency(250);
                }),

                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
                        .until(() -> this.getAngle().in(Radians) > ShooterConstants.SOFT_MAX_LIMIT),

                new WaitCommand(1.5),

                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
                        .until(() -> this.getAngle().in(Radians) < ShooterConstants.SOFT_MIN_LIMIT),

                new WaitCommand(1.5),

                // 4. Dynamic Forward (快速往前推)
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward)
                        .until(() -> this.getAngle().in(Radians) > ShooterConstants.SOFT_MAX_LIMIT),

                new WaitCommand(1.5),

                // 5. Dynamic Reverse (快速往後拉)
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse)
                        .until(() -> this.getAngle().in(Radians) < ShooterConstants.SOFT_MIN_LIMIT),

                // 🟢 6. 測試結束：關閉紀錄器，並將更新率降回正常的 50Hz
                Commands.runOnce(() -> {
                    System.err.println("🛑 SysId 紀錄結束！");
                    SignalLogger.stop();
                    turretMotor.getPosition().setUpdateFrequency(50);
                    turretMotor.getVelocity().setUpdateFrequency(50);
                    turretMotor.getMotorVoltage().setUpdateFrequency(50);
                })
        );
    }
}