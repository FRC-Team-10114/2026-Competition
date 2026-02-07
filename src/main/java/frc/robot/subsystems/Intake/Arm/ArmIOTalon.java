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
import frc.robot.Constants.IDs;
import frc.robot.subsystems.Intake.IntakeConstants.ArmConstants;

public class ArmIOTalon implements ArmIO {
    
    private final TalonFX armMotor;

    private final StatusSignal<Angle> armPosition;

    private final CANcoder armAbsoluteEncoder;

    private final MotionMagicVoltage armOutput;

    public ArmIOTalon() {

        this.armMotor = new TalonFX(IDs.Intake.ARM_MOTOR, "rio");
        this.armPosition = armMotor.getPosition();

        this.armAbsoluteEncoder = new CANcoder(IDs.Intake.ARM_CANCODER, "rio");

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
                .withStatorCurrentLimit(ArmConstants.STATOR_CURRENT_LIMIT.baseUnitMagnitude())
                .withSupplyCurrentLimit(ArmConstants.SUPPLY_CURRENT_LIMIT.baseUnitMagnitude())
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true);
        armConfig.MotionMagic
                .withMotionMagicAcceleration(ArmConstants.MAX_ACCELERATION)
                .withMotionMagicCruiseVelocity(ArmConstants.CRUISE_VELOCITY);
        armConfig.Feedback
                .withFusedCANcoder(armAbsoluteEncoder)
                .withSensorToMechanismRatio(ArmConstants.POSITION_CONVERSION_FACTOR);
        armConfig.Slot0
                .withKP(ArmConstants.PID[0])
                .withKI(ArmConstants.PID[1])
                .withKD(ArmConstants.PID[2])
                .withGravityType(GravityTypeValue.Arm_Cosine);
        armConfig.HardwareLimitSwitch
                .withForwardLimitAutosetPositionEnable(true)
                .withForwardLimitAutosetPositionValue(ArmConstants.FORWARD_LIMIT)
                .withReverseLimitAutosetPositionEnable(true)
                .withReverseLimitAutosetPositionValue(ArmConstants.REVERSE_LIMIT);

        armMotor.getConfigurator().apply(armConfig);
    }
}
