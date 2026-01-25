package frc.robot.subsystems.Shooter.Turret;

import static edu.wpi.first.units.Units.Amp;
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

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.MathHelper.EncoderWithGearRatio;
import frc.robot.MathHelper.RobustCRTCalculator;

public class TurretHardware implements TurretIO {
    private final TalonFX Turretd = new TalonFX(20);

    private final CANcoder encoder1, encoder2;
        
    private final double metersPerangle = 1.0;

    private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
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

            

        configs.CurrentLimits.withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amp.of(80));
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
        configs.MotionMagic.withMotionMagicCruiseVelocity(1).withMotionMagicAcceleration(2.8);

        // 齒輪比
        configs.Feedback.SensorToMechanismRatio = metersPerangle;

        Turretd.getConfigurator().apply(configs);
    }

    public void setAngleRadians(double angle) {
        Turretd.setControl(
                m_request.withPosition(Units.radiansToRotations(angle)));
    }

    public void setControl(Angle angle) {
        this.setAngleRadians(angle.in(Radians));
    }

    public void resetAngle() {

        EncoderWithGearRatio encoder1 = new EncoderWithGearRatio(
            this.encoder1.getPosition().getValueAsDouble(),
            30
        );

        EncoderWithGearRatio encoder2 = new EncoderWithGearRatio(
            this.encoder2.getPosition().getValueAsDouble(),
            31
        );

        double angle = RobustCRTCalculator.calculateAbsolutePosition(encoder1, encoder2);
        Turretd.getConfigurator().setPosition(angle);
    }
}
