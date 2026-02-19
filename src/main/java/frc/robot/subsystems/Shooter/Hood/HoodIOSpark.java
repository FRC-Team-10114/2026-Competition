package frc.robot.subsystems.Shooter.Hood;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.IDs;
import frc.robot.subsystems.Shooter.ShooterConstants;

public class HoodIOSpark implements HoodIO {

    private final SparkFlex motor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pidController;
    private final CANcoder cancoder;

    public HoodIOSpark() {
        motor = new SparkFlex(IDs.Shooter.HOOD_MOTOR, MotorType.kBrushless);

        encoder = motor.getEncoder();
        pidController = motor.getClosedLoopController();

        cancoder = new CANcoder(IDs.Shooter.HOOD_CANCODER);
        
        this.configure();
        this.CANcoderConfig();
    }

    @Override
    public void CANcoderConfig() {
        var CANcoderConfig = new CANcoderConfiguration();
        CANcoderConfig.MagnetSensor.MagnetOffset = 0.227294921875 + Units.degreesToRotations(35);// 磁力感測器的偏移量
        CANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        cancoder.getConfigurator().apply(CANcoderConfig);
    }

    @Override
    public void configure() {
        var hoodConfig = new SparkFlexConfig();

        hoodConfig
                .idleMode(IdleMode.kBrake)
                .inverted(false)
                .smartCurrentLimit(40)
                .apply(hoodConfig);
        hoodConfig.softLimit
                .forwardSoftLimit(ShooterConstants.Hood_MAX_RADS.in(Radians))
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(ShooterConstants.Hood_MIN_RADS.in(Radians))
                .reverseSoftLimitEnabled(true)
                .apply(hoodConfig.softLimit);
        hoodConfig.encoder
                .positionConversionFactor(Radians.convertFrom(ShooterConstants.Hood_GEAR_RATIO, Rotation))
                .apply(hoodConfig.encoder);
        hoodConfig.closedLoop
                .p(5.56)
                .i(0)
                .d(0.01)
                .allowedClosedLoopError(0.5, ClosedLoopSlot.kSlot0)
                .apply(hoodConfig.closedLoop);

        motor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setAngle(Angle angle) {
        pidController.setSetpoint(angle.in(Degree), ControlType.kPosition);
    }

    public void reset() {
        this.encoder.setPosition(Radians.convertFrom(cancoder.getPosition().getValueAsDouble(), Rotations));
    }

    @Override
    public double getAngle() {
        return this.encoder.getPosition();
    }

    @Override
    public boolean isAtSetPosition() {
        return this.pidController.isAtSetpoint();
    }
}
