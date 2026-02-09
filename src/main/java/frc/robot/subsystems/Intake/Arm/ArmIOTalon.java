package frc.robot.subsystems.Intake.Arm;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.IDs;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeConstants.ArmConstants;
import frc.robot.subsystems.Shooter.ShooterConstants;

public class ArmIOTalon implements ArmIO {
    
    private final TalonFX armMotor;

    private final StatusSignal<Angle> armPosition;

    // private final CANcoder armAbsoluteEncoder;

    private final PositionVoltage armOutput;

    public ArmIOTalon() {

        this.armMotor = new TalonFX(IDs.Intake.ARM_MOTOR, "canivore");
        this.armPosition = armMotor.getPosition();

        this.armOutput = new PositionVoltage(0.0);

        configure();
        resetEncoder();
    }

    @Override
    public void setPosition(double position) {
        this.armMotor.setControl(this.armOutput.withPosition(position));
    }

    @Override
    public double getPosition() {
        this.armPosition.refresh();
        return this.armPosition.getValueAsDouble();
    }

    @Override
    public void resetEncoder() {
        this.armMotor.getConfigurator().setPosition(0);
    }

    @Override
    public void configure() {
        
        var armConfig = new TalonFXConfiguration();

        armConfig.Feedback.SensorToMechanismRatio = ArmConstants.GEAR_RATIO;
        armConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        armConfig.CurrentLimits
                .withStatorCurrentLimit(ArmConstants.STATOR_CURRENT_LIMIT.baseUnitMagnitude())
                .withSupplyCurrentLimit(ArmConstants.SUPPLY_CURRENT_LIMIT.baseUnitMagnitude())
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true);
        // armConfig.MotionMagic
        //         .withMotionMagicAcceleration(ArmConstants.MAX_ACCELERATION)
        //         .withMotionMagicCruiseVelocity(ArmConstants.CRUISE_VELOCITY);
        // // armConfig.Feedback
        //         .withFusedCANcoder(armAbsoluteEncoder)
        //         .withSensorToMechanismRatio(ArmConstants.POSITION_CONVERSION_FACTOR);
        armConfig.Slot0
                .withKP(ArmConstants.PID[0])
                .withKI(ArmConstants.PID[1])
                .withKD(ArmConstants.PID[2])
                .withGravityType(GravityTypeValue.Arm_Cosine);
        armConfig.HardwareLimitSwitch
                .withForwardLimitAutosetPositionEnable(false)
                .withForwardLimitAutosetPositionValue(128)
                .withReverseLimitAutosetPositionEnable(false)
                .withReverseLimitAutosetPositionValue(0);

        armMotor.getConfigurator().apply(armConfig);
    }
}
