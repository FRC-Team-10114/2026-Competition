package frc.robot.subsystems.Shooter.Hood;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;

public class HoodIONEO implements HoodIO {

    private final SparkFlex motor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pidController;
    private final CANcoder cancoder;

    public HoodIONEO() {
        motor = new SparkFlex(17, MotorType.kBrushless);

        encoder = motor.getEncoder();
        pidController = motor.getClosedLoopController();

        cancoder = new CANcoder(20);

        motor.configure(HoodConfig.hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.CANcoderConfig();
    }

    public void CANcoderConfig() {
        var CANcoderConfig = new CANcoderConfiguration();
        CANcoderConfig.MagnetSensor.MagnetOffset = 0.227294921875 + Units.degreesToRotations(35);// 磁力感測器的偏移量
        CANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        cancoder.getConfigurator().apply(CANcoderConfig);
    }

    @Override
    public void setAngle(Angle angle) {
        pidController.setSetpoint(angle.in(Radians), ControlType.kPosition);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void reset() {
        this.encoder.setPosition(Radians.convertFrom(cancoder.getPosition().getValueAsDouble(), Rotations));
    }

    @Override
    public double getAngle() {
        return this.encoder.getPosition();
    }
}
