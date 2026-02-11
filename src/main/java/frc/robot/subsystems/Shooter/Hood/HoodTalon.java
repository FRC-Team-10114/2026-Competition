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
import frc.robot.Constants.IDs;
import frc.robot.subsystems.Shooter.ShooterConstants;

public class HoodTalon implements HoodIO {

    private final TalonFX HoodMotor;
    private final CANcoder Hoodcancoder;
    private final StatusSignal<Angle> HoodPosition;

    private final MotionMagicVoltage m_request = new MotionMagicVoltage(Degree.of(0));

    private final double rotorToSensorRatio = ShooterConstants.HoodCancoder_GEAR_RATIO_TOMotor;

    private final double sensorToMechRatio = ShooterConstants.Hood_GEAR_RATIO
            / ShooterConstants.HoodCancoder_GEAR_RATIO_TOMotor;

    private double latestTargetDegrees = 0.0;

    public HoodTalon() {
        this.HoodMotor = new TalonFX(IDs.Shooter.HOOD_MOTOR, "canivore");
        this.Hoodcancoder = new CANcoder(IDs.Shooter.HOOD_CANCODER, "canivore");
        this.HoodPosition = HoodMotor.getPosition();

        CANcoderConfig();

        configure();

        initStartingPosition();
    }

    private void initStartingPosition() {

        StatusSignal<Angle> currentPos = HoodMotor.getPosition();

        currentPos.waitForUpdate(0.25);

        m_request.Position = currentPos.getValueAsDouble();
    }

    @Override
    public void CANcoderConfig() {
        var cfg = new CANcoderConfiguration();

        double targetSensorRotations = Units.degreesToRotations(25.0) * sensorToMechRatio;

        cfg.MagnetSensor.MagnetOffset = 0.078857421875 + targetSensorRotations;

        cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        Hoodcancoder.getConfigurator().apply(cfg);
    }

    public void configure() {
        var hoodConfig = new TalonFXConfiguration();

        hoodConfig.CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(70.0)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40.0);

        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        hoodConfig.SoftwareLimitSwitch
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(ShooterConstants.Hood_MIN_RADS)
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(ShooterConstants.Hood_MAX_RADS);

        hoodConfig.Feedback
                .withFeedbackRemoteSensorID(55)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                .withRotorToSensorRatio(rotorToSensorRatio)
                .withSensorToMechanismRatio(sensorToMechRatio);

        hoodConfig.Slot0.kP = 180.0;
        hoodConfig.Slot0.kI = 0.0;
        hoodConfig.Slot0.kD = 0.0;

        hoodConfig.Slot0.kG = 0.0;
        hoodConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        hoodConfig.MotionMagic
                .withMotionMagicCruiseVelocity(DegreesPerSecond.of(1080))
                .withMotionMagicAcceleration(DegreesPerSecondPerSecond.of(3600));

        HoodMotor.getConfigurator().apply(hoodConfig);
    }

    @Override
    public void setAngle(Angle angle) {
        this.latestTargetDegrees = angle.in(Degrees);
        HoodMotor.setControl(m_request.withPosition(angle));
    }

    @Override
    public double getAngle() {
        this.HoodPosition.refresh();

        return this.HoodPosition.getValue().in(Degrees);
    }
    @Override
    public boolean isAtSetPosition() {
        // 3. 務必刷新訊號，確保讀到的是最新位置
        this.HoodPosition.refresh();

        // 4. 取得當前實際角度
        double currentDegrees = this.HoodPosition.getValue().in(Degrees);

        // 5. 計算誤差絕對值 (目標 - 實際)
        double error = Math.abs(this.latestTargetDegrees - currentDegrees);

        // 6. 只要誤差小於 2 度，就回傳 true (允許開火)
        return error < 2.0;
    }
}