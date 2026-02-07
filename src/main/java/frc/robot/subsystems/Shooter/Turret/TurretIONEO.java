package frc.robot.subsystems.Shooter.Turret;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;

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
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.util.MathHelper.EncoderWithGearRatio;
import frc.robot.util.MathHelper.RobustCRTCalculator;

public class TurretIONEO extends TurretIO {

    private final double gearRatio = 1.0;

    private final SparkFlex turretMotor;
    private final SparkClosedLoopController turretController;
    private final RelativeEncoder turretEncoder;

    private final CANcoder masterCANcoder, slaveCANcoder;

    public TurretIONEO() {
        this.turretMotor = new SparkFlex(20, MotorType.kBrushless);
        this.turretController = turretMotor.getClosedLoopController();
        this.turretEncoder = turretMotor.getEncoder();

        this.masterCANcoder = new CANcoder(21);
        this.slaveCANcoder = new CANcoder(22);
    }

    @Override
    public void setAngle(Rotation2d robotHeading, Angle targetRad, ShootState state) {
        this.turretController.setSetpoint(this.Calculate(robotHeading, targetRad, state).in(Radians),
                ControlType.kPosition);
    }

    @Override
    public void resetAngle() {
        EncoderWithGearRatio master = new EncoderWithGearRatio(
                this.masterCANcoder.getPosition().getValueAsDouble(),
                30);

        EncoderWithGearRatio slave = new EncoderWithGearRatio(
                this.slaveCANcoder.getPosition().getValueAsDouble(),
                31);

        double angle = RobustCRTCalculator.calculateAbsolutePosition(master, slave);
        turretEncoder.setPosition(angle);
    }

    public void configureMotors() {
        var turretConfig = new SparkFlexConfig();

        turretConfig
                .smartCurrentLimit(50)
                .idleMode(IdleMode.kBrake)
                .inverted(false)
                .apply(turretConfig);
        turretConfig.softLimit
                .reverseSoftLimitEnabled(true)
                .reverseSoftLimit(ShooterConstants.HARD_MIN_RADS)
                .forwardSoftLimitEnabled(true)
                .forwardSoftLimit(ShooterConstants.HARD_MAX_RADS)
                .apply(turretConfig.softLimit);

        turretConfig.closedLoop
                .pid(0.11, 0, 0);

        turretConfig.closedLoop.maxMotion
                .cruiseVelocity(DegreesPerSecond.convertFrom(360, DegreesPerSecond))
                .maxAcceleration(DegreesPerSecond.convertFrom(720, DegreesPerSecond))
                .apply(turretConfig.closedLoop.maxMotion);

        turretConfig.encoder
                .velocityConversionFactor(gearRatio);

        this.turretMotor.configure(
                turretConfig, 
                ResetMode.kResetSafeParameters, 
                PersistMode.kPersistParameters);
    }
}
