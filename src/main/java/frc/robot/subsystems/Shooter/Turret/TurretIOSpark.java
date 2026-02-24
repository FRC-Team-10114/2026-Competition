package frc.robot.subsystems.Shooter.Turret;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.util.MathHelper.PositionWithGearRatio;
import frc.robot.util.MathHelper.RobustCRTCalculator;

public class TurretIOSpark extends TurretIO {

        private final double gearRatio = 96.0 / 16.0 * 3.0;
        private final double positionFactor = (1.0 / gearRatio) * (2.0 * Math.PI);
        private final double velocityFactor = positionFactor / 60.0;

        private final SparkFlex turretMotor;
        private final SparkClosedLoopController turretController;
        private final RelativeEncoder turretEncoder;

        private double currentTargetRads = 0.0;
        private final CANcoder masterCANcoder, slaveCANcoder;

        // 🟢 【新增】宣告 SysIdRoutine 物件
        private final SysIdRoutine sysIdRoutine;

        public TurretIOSpark() {
                this.turretMotor = new SparkFlex(40, MotorType.kBrushless);
                this.turretController = turretMotor.getClosedLoopController();
                this.turretEncoder = turretMotor.getEncoder();

                this.masterCANcoder = new CANcoder(21);
                this.slaveCANcoder = new CANcoder(22);

                resetAngle();

                configureMotors();

                // ==========================================
                // 🟢 【新增】初始化 SysId 測試機制
                // ==========================================
                this.sysIdRoutine = new SysIdRoutine(
                                new SysIdRoutine.Config(),
                                new SysIdRoutine.Mechanism(
                                                // 1. 告訴 SysId 怎麼給馬達電壓
                                                (voltage) -> this.turretMotor.setVoltage(voltage.in(Volts)),

                                                // 2. 告訴 SysId 怎麼讀取當下的馬達狀態
                                                log -> {
                                                        log.motor("TurretMotor")
                                                                        .voltage(Volts.of(this.turretMotor
                                                                                        .getAppliedOutput()
                                                                                        * this.turretMotor
                                                                                                        .getBusVoltage()))
                                                                        .angularPosition(Radians.of(this.turretEncoder
                                                                                        .getPosition()))
                                                                        .angularVelocity(RadiansPerSecond
                                                                                        .of(this.turretEncoder
                                                                                                        .getVelocity()));
                                                },

                                                // 3. 🟢 【關鍵修正】：不要填 null，現場 new 一個空的 Subsystem 給它
                                                new edu.wpi.first.wpilibj2.command.SubsystemBase() {
                                                        @Override
                                                        public String getName() {
                                                                return "TurretSysId";
                                                        }
                                                }));
        }

        // 1. Quasistatic Forward (慢慢往前推)
        public Command sysIdQuasistaticForward() {
                return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
                                // 🟢 完美寫法：直接拿弧度出來，跟你的極限常數比較！
                                .until(() -> this.getAngle().in(Radians) > ShooterConstants.SOFT_MAX_RADS);
        }

        // 2. Quasistatic Reverse (慢慢往後拉)
        public Command sysIdQuasistaticReverse() {
                return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
                                // 🟢 完美寫法：倒退時，跟你的最小極限比較！
                                .until(() -> this.getAngle().in(Radians) < ShooterConstants.SOFT_MIN_RADS);
        }

        // 3. Dynamic Forward (快速往前推)
        public Command sysIdDynamicForward() {
                return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward)
                                .until(() -> this.getAngle().in(Radians) > ShooterConstants.SOFT_MAX_RADS);
        }

        // 4. Dynamic Reverse (快速往後拉)
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
                this.turretController.setSetpoint(this.Calculate(robotHeading, targetRad, state).in(Radians),
                                ControlType.kPosition);
                double target = this.Calculate(robotHeading, targetRad, state).in(Radians);
                this.currentTargetRads = target;
        }

        @Override
        public void resetAngle() {
                PositionWithGearRatio master = new PositionWithGearRatio(
                                this.masterCANcoder.getPosition().getValueAsDouble(), 30);
                PositionWithGearRatio slave = new PositionWithGearRatio(
                                this.slaveCANcoder.getPosition().getValueAsDouble(), 31);

                double angle = RobustCRTCalculator.calculateAbsolutePosition(master, slave);

                turretEncoder.setPosition(0.0);
        }

        @Override
        public Angle getAngle() {
                return Radians.of(this.turretEncoder.getPosition());
        }

        @Override
        public boolean isAtSetPosition() {
                double currentRads = this.turretEncoder.getPosition();
                double errorRads = Math.abs(this.currentTargetRads - currentRads);

                return !(errorRads > Units.degreesToRadians(40.0));
        }

        public void configureMotors() {
                var turretConfig = new SparkFlexConfig();

                turretConfig
                                .smartCurrentLimit(40)
                                .idleMode(IdleMode.kBrake)
                                .inverted(false);

                turretConfig.softLimit
                                .reverseSoftLimitEnabled(true)
                                .reverseSoftLimit(ShooterConstants.HARD_MIN_RADS)
                                .forwardSoftLimitEnabled(true)
                                .forwardSoftLimit(ShooterConstants.HARD_MAX_RADS);

                turretConfig.closedLoop
                                .pid(0.8, 0.0, 0.0);
                turretConfig.closedLoop.feedForward
                                .kV(0.0)
                                .kS(0.0) // 大約是 0.0145。讓馬達隨時保持「準備好要動」的狀態
                                .kA(0.0);

                turretConfig.encoder
                                .positionConversionFactor(positionFactor)
                                .velocityConversionFactor(velocityFactor);

                this.turretMotor.configure(
                                turretConfig,
                                ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);
        }
}