package frc.robot.subsystems.Intake.Arm;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.IntakeConstants;

public class ArmIOTalon implements ArmIO {
    
    private final TalonFX armMotor;

    private final StatusSignal<Angle> armPosition;

    private final CANcoder armAbsoluteEncoder;

    private final MotionMagicVoltage armOutput;

    public ArmIOTalon() {

        this.armMotor = new TalonFX(IntakeConstants.INTAKE_ARM_MOTOR_ID, "rio");
        this.armPosition = armMotor.getPosition();

        this.armAbsoluteEncoder = new CANcoder(IntakeConstants.INTKAE_ARM_CANCODER_ID, "rio");

        this.armOutput = new MotionMagicVoltage(0.0);

        configure();
        resetEncoder();
    }

    @Override
    public void setPosition(Angle position) {
        this.armMotor.setControl(this.armOutput.withPosition(position).withEnableFOC(true));
    }

    @Override
    public Angle getPosition() {
        this.armPosition.refresh();
        return this.armPosition.getValue();
    }

    @Override
    public void resetEncoder() {
        this.armMotor.getConfigurator().setPosition(Radians.convertFrom(this.armAbsoluteEncoder.getPosition().getValueAsDouble(), Rotation));
    }

    @Override
    public void configure() {
        
        var armConfig = new TalonFXConfiguration();

        armConfig.CurrentLimits
                .withStatorCurrentLimit(IntakeConstants.INTAKE_ARM_STATOR_CURRENT_LIMIT.baseUnitMagnitude())
                .withSupplyCurrentLimit(IntakeConstants.INTAKE_ARM_SUPPLY_CURRENT_LIMIT.baseUnitMagnitude())
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true);
        armConfig.MotionMagic
                .withMotionMagicAcceleration(IntakeConstants.INTAKE_ARM_MAX_ACCELERATION)
                .withMotionMagicCruiseVelocity(IntakeConstants.INTAKE_ARM_CRUISE_VELOCITY);
        armConfig.Feedback
                .withFusedCANcoder(armAbsoluteEncoder)
                .withSensorToMechanismRatio(IntakeConstants.INTAKE_ARM_POSITION_CONVERSION_FACTOR);
        armConfig.Slot0
                .withKP(IntakeConstants.INTAKE_ARM_PID[0])
                .withKI(IntakeConstants.INTAKE_ARM_PID[1])
                .withKD(IntakeConstants.INTAKE_ARM_PID[2])
                .withGravityType(GravityTypeValue.Arm_Cosine);
        armConfig.HardwareLimitSwitch
                .withForwardLimitAutosetPositionEnable(true)
                .withForwardLimitAutosetPositionValue(IntakeConstants.INTAKE_ARM_FORWARD_LIMIT)
                .withReverseLimitAutosetPositionEnable(true)
                .withReverseLimitAutosetPositionValue(IntakeConstants.INTAKE_ARM_REVERSE_LIMIT);

        armMotor.getConfigurator().apply(armConfig);
    }
}
