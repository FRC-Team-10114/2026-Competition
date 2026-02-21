package frc.robot.subsystems.Shooter.Turret;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
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

        public TurretIOSpark() {
                this.turretMotor = new SparkFlex(40, MotorType.kBrushless);
                this.turretController = turretMotor.getClosedLoopController();
                this.turretEncoder = turretMotor.getEncoder();

                this.masterCANcoder = new CANcoder(21);
                this.slaveCANcoder = new CANcoder(22);

                configureMotors();
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
                                this.masterCANcoder.getPosition().getValueAsDouble(),
                                30);

                PositionWithGearRatio slave = new PositionWithGearRatio(
                                this.slaveCANcoder.getPosition().getValueAsDouble(),
                                31);

                double angle = RobustCRTCalculator.calculateAbsolutePosition(master, slave);
                turretEncoder.setPosition(0.0);
        }

        @Override
        public Angle getAngle() {
                return Radians.of(this.turretEncoder.getPosition());
        }

        @Override
        public boolean isAtSetPosition() {
                // 1. 取得目前角度 (因為設定了 Factor，這裡是弧度)
                double currentRads = this.turretEncoder.getPosition();

                // 2. 計算誤差 (弧度)
                double errorRads = Math.abs(this.currentTargetRads - currentRads);

                // 3. 將 2 度轉成弧度來比較
                // 2 degrees ~= 0.035 radians
                return !(errorRads > Units.degreesToRadians(40.0));
        }

        public void configureMotors() {
                var turretConfig = new SparkFlexConfig();

                turretConfig
                                .smartCurrentLimit(40) // 建議 40A 就夠了
                                .idleMode(IdleMode.kBrake)
                                .inverted(false);

                turretConfig.softLimit
                                .reverseSoftLimitEnabled(true)
                                .reverseSoftLimit(ShooterConstants.HARD_MIN_RADS) // 硬體保護
                                .forwardSoftLimitEnabled(true)
                                .forwardSoftLimit(ShooterConstants.HARD_MAX_RADS);

                turretConfig.closedLoop
                                .pid(0.8, 0, 0); // P 值可能需要調大一點，因為單位是 Radians

                // 【修正 2】 單位修正：從度轉為弧度
                turretConfig.closedLoop.maxMotion
                                .cruiseVelocity(Units.degreesToRadians(360))
                                .maxAcceleration(Units.degreesToRadians(720));

                turretConfig.encoder
                                .positionConversionFactor(positionFactor)
                                .velocityConversionFactor(velocityFactor); // 【修正 1】 套用正確的速度因子

                this.turretMotor.configure(
                                turretConfig,
                                ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);
        }
}
