package frc.robot.subsystems.Shooter.Hood;

import static edu.wpi.first.units.Units.*; // 引入所有單位

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.Shooter.ShooterConstants;

public class HoodTalon implements HoodIO {

    private final TalonFX HoodMotor;
    private final CANcoder Hoodcancoder;
    private final StatusSignal<Angle> HoodPosition;

    private final MotionMagicVoltage m_request = new MotionMagicVoltage(Degree.of(0));

    private final double rotorToSensorRatio = ShooterConstants.HoodCancoder_GEAR_RATIO_TOMotor;

    private final double sensorToMechRatio = ShooterConstants.Hood_GEAR_RATIO
            / ShooterConstants.HoodCancoder_GEAR_RATIO_TOMotor;

    public HoodTalon() {
        this.HoodMotor = new TalonFX(22);
        this.Hoodcancoder = new CANcoder(55);
        this.HoodPosition = HoodMotor.getPosition();

        CANcoderConfig();

        configureMotors();

        initStartingPosition();
    }

    private void initStartingPosition() {

        StatusSignal<Angle> currentPos = HoodMotor.getPosition();

        currentPos.waitForUpdate(0.25);

        m_request.Position = currentPos.getValueAsDouble();
    }

    public void CANcoderConfig() {
        var cfg = new CANcoderConfiguration();

        double targetSensorRotations = Units.degreesToRotations(25.0) * sensorToMechRatio;

        cfg.MagnetSensor.MagnetOffset = 0.397705078125 + targetSensorRotations;

        cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        Hoodcancoder.getConfigurator().apply(cfg);
    }

    public void configureMotors() {
        TalonFXConfiguration configs = new TalonFXConfiguration();

        configs.CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(70.0)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40.0);

        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        configs.SoftwareLimitSwitch
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(Degree.of(28)) // 下限25
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(Degree.of(55)); // 上限63

        configs.Feedback
                .withFeedbackRemoteSensorID(55)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                .withRotorToSensorRatio(rotorToSensorRatio)
                .withSensorToMechanismRatio(sensorToMechRatio);

        configs.Slot0.kP = 200.0;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.0;

        configs.Slot0.kG = 0.0;
        configs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        configs.MotionMagic
                .withMotionMagicCruiseVelocity(DegreesPerSecond.of(720))
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
}