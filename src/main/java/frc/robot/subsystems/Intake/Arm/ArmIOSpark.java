package frc.robot.subsystems.Intake.Arm;

import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.units.measure.Angle;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.IntakeConstants;

public class ArmIOSpark implements ArmIO {

    private final SparkMax armMotor;

    private final RelativeEncoder armEncoder;

    private final SparkClosedLoopController armContoller;

    private final CANcoder armAbsoluteEncoder;

    public ArmIOSpark() {

        this.armMotor = new SparkMax(IntakeConstants.INTAKE_ARM_MOTOR_ID, MotorType.kBrushless);
        this.armEncoder = armMotor.getEncoder();
        this.armContoller = armMotor.getClosedLoopController();

        this.armAbsoluteEncoder = new CANcoder(IntakeConstants.INTAKE_ROLLER_MOTOR_ID);

        configure();
        resetEncoder();
    }

    @Override
    public void setPosition(Angle position) {
        this.armContoller.setSetpoint(position.baseUnitMagnitude(), ControlType.kMAXMotionPositionControl);
    }

    @Override
    public Angle getPosition() {
        return Radian.ofBaseUnits(this.armEncoder.getPosition());
    }

    @Override
    public void resetEncoder() {
        this.armEncoder
                .setPosition(Radian.convertFrom(this.armAbsoluteEncoder.getPosition().getValueAsDouble(), Rotation));
    }

    @Override
    public void configure() {

        var armConfig = new SparkMaxConfig();

        armConfig
                .idleMode(IdleMode.kBrake)
                .inverted(false)
                .smartCurrentLimit((int) IntakeConstants.INTAKE_ARM_SUPPLY_CURRENT_LIMIT.baseUnitMagnitude())
                .apply(armConfig);
        armConfig.encoder
                .velocityConversionFactor(IntakeConstants.INTAKE_ARM_VELOCITY_CONVERSION_FACOTR)
                .positionConversionFactor(IntakeConstants.INTAKE_ARM_POSITION_CONVERSION_FACTOR)
                .apply(armConfig.encoder);
        armConfig.closedLoop
                .pid(
                        IntakeConstants.INTAKE_ARM_PID[0],
                        IntakeConstants.INTAKE_ARM_PID[1],
                        IntakeConstants.INTAKE_ARM_PID[2])
                .apply(armConfig.closedLoop);
        armConfig.closedLoop.maxMotion
                .cruiseVelocity(IntakeConstants.INTAKE_ARM_CRUISE_VELOCITY)
                .maxAcceleration(IntakeConstants.INTAKE_ARM_MAX_ACCELERATION)
                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
                .apply(armConfig.closedLoop.maxMotion);
        armConfig.softLimit
                .forwardSoftLimit(IntakeConstants.INTAKE_ARM_FORWARD_LIMIT.baseUnitMagnitude())
                .reverseSoftLimit(IntakeConstants.INTAKE_ARM_REVERSE_LIMIT.baseUnitMagnitude())
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimitEnabled(true)
                .apply(armConfig.softLimit);
        
        armMotor.configure(
                armConfig, 
                ResetMode.kResetSafeParameters, 
                PersistMode.kPersistParameters);
    }

}
