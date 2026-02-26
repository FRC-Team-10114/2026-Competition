package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;

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

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.IDs;

public class ClimberIOSpark implements ClimberIO {

    private final SparkFlex master, slave;
    private final RelativeEncoder climberEncoder;
    private final SparkClosedLoopController climberController;

    public ClimberIOSpark() {
        this.master = new SparkFlex(IDs.Climber.CLIMBER_MOTOR, MotorType.kBrushless);
        this.slave = new SparkFlex(IDs.Climber.CLIMBER_MOTOR, MotorType.kBrushless);
        this.climberEncoder = master.getEncoder();
        this.climberController = master.getClosedLoopController();

        configure();
    }

    @Override
    public void setPosition(Angle round) {
        this.climberController.setSetpoint(round.baseUnitMagnitude(), ControlType.kMAXMotionPositionControl);
    }

    @Override
    public Angle getPosition() {
        return Rotations.of(this.climberEncoder.getPosition());
    }

    @Override
    public void resetPosition() {
        this.climberEncoder.setPosition(0.0);
    }

    @Override
    public void configure() {
        var masterConfig = new SparkFlexConfig();

        masterConfig
                .idleMode(IdleMode.kBrake)
                .inverted(false)
                .smartCurrentLimit((int) ClimberConstants.STATOR_CURRENT_LIMIT.baseUnitMagnitude())
                .apply(masterConfig);
        masterConfig.encoder
                .positionConversionFactor(ClimberConstants.POSITION_CONVERSION_FACTOR)
                .inverted(false)
                .apply(masterConfig.encoder);
        masterConfig.closedLoop
                .pid(0.1, 0.0, 0.0)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .apply(masterConfig.closedLoop);
        masterConfig.closedLoop.maxMotion
                .maxAcceleration(ClimberConstants.MAX_ACCELERATION.baseUnitMagnitude())
                .cruiseVelocity(ClimberConstants.CRUISE_VELOCITY.baseUnitMagnitude())
                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
                .apply(masterConfig.closedLoop.maxMotion);
        masterConfig.softLimit
                .forwardSoftLimit(ClimberConstants.FORWARD_LIMIT.baseUnitMagnitude())
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(ClimberConstants.REVERSE_LIMIT.baseUnitMagnitude())
                .reverseSoftLimitEnabled(true)
                .apply(masterConfig.softLimit);

        var slaveConfig = new SparkFlexConfig();

        slaveConfig
                .idleMode(IdleMode.kBrake)
                .inverted(false)
                .smartCurrentLimit((int) ClimberConstants.STATOR_CURRENT_LIMIT.baseUnitMagnitude())
                .apply(masterConfig);
        slaveConfig.encoder
                .positionConversionFactor(ClimberConstants.POSITION_CONVERSION_FACTOR)
                .inverted(false)
                .apply(masterConfig.encoder);
        slaveConfig.closedLoop
                .pid(0.1, 0.0, 0.0)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .apply(masterConfig.closedLoop);
        slaveConfig.closedLoop.maxMotion
                .maxAcceleration(ClimberConstants.MAX_ACCELERATION.baseUnitMagnitude())
                .cruiseVelocity(ClimberConstants.CRUISE_VELOCITY.baseUnitMagnitude())
                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
                .apply(masterConfig.closedLoop.maxMotion);
        slaveConfig.softLimit
                .forwardSoftLimit(ClimberConstants.FORWARD_LIMIT.in(Rotation))
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(ClimberConstants.REVERSE_LIMIT.in(Rotations))
                .reverseSoftLimitEnabled(true)
                .apply(masterConfig.softLimit);
        
        this.master.configure(
            masterConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters);
    }
}
