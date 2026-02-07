package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.IDs;

public class ClimberIOSpark implements ClimberIO {

    private final SparkFlex climberMotor;
    private final RelativeEncoder climberEncoder;
    private final SparkClosedLoopController climberController;

    public ClimberIOSpark() {
        this.climberMotor = new SparkFlex(IDs.Climber.CLIMBER_MOTOR, MotorType.kBrushless);
        this.climberEncoder = climberMotor.getEncoder();
        this.climberController = climberMotor.getClosedLoopController();

        configure();
    }

    @Override
    public void setPosition(Distance meter) {
        this.climberController.setSetpoint(meter.baseUnitMagnitude(), ControlType.kMAXMotionPositionControl);
    }

    @Override
    public Distance getPosition() {
        return Meters.of(this.climberEncoder.getPosition());
    }

    @Override
    public void resetPosition() {
        this.climberEncoder.setPosition(0.0);
    }

    @Override
    public void configure() {
        var climberConfig = new SparkFlexConfig();

        climberConfig
                .idleMode(IdleMode.kBrake)
                .inverted(false)
                .smartCurrentLimit((int) ClimberConstants.STATOR_CURRENT_LIMIT.baseUnitMagnitude())
                .apply(climberConfig);
        climberConfig.encoder
                .positionConversionFactor(ClimberConstants.POSITION_CONVERSION_FACTOR)
                .inverted(false)
                .apply(climberConfig.encoder);
        climberConfig.closedLoop
                .pid(0.1, 0.0, 0.0)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .apply(climberConfig.closedLoop);
        climberConfig.closedLoop.maxMotion
                .maxAcceleration(ClimberConstants.MAX_ACCELERATION.baseUnitMagnitude())
                .cruiseVelocity(ClimberConstants.CRUISE_VELOCITY.baseUnitMagnitude())
                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
                .apply(climberConfig.closedLoop.maxMotion);
        climberConfig.softLimit
                .forwardSoftLimit(ClimberConstants.FORWARD_LIMIT.baseUnitMagnitude())
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(ClimberConstants.REVERSE_LIMIT.baseUnitMagnitude())
                .reverseSoftLimitEnabled(true)
                .apply(climberConfig.softLimit);
        
        this.climberMotor.configure(
            climberConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters);
    }
}
