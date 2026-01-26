package frc.robot.subsystems.Intake.Roller;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.IntakeConstants;

public class RollerIOTalon implements RollerIO{

    private final TalonFX rollerMotor;
    private final StatusSignal<AngularVelocity> rollerVelocity;
    private final MotionMagicVelocityVoltage rollerOutput;

    public RollerIOTalon() {
        this.rollerMotor = new TalonFX(IntakeConstants.INTAKE_ROLLER_MOTOR_ID, "rio");
        this.rollerVelocity = rollerMotor.getVelocity();
        this.rollerOutput = new MotionMagicVelocityVoltage(0.0);
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        this.rollerMotor.setControl(this.rollerOutput.withVelocity(velocity));
    }

    @Override
    public AngularVelocity getVelocity() {
        this.rollerVelocity.refresh();
        return this.rollerVelocity.getValue();
    }

    @Override
    public void configure() {
        var rollerConfig = new TalonFXConfiguration();

        rollerConfig.Feedback
                .withSensorToMechanismRatio(IntakeConstants.INTAKE_ROLLER_VELOCITY_CONVERSION_FACOTR);
        rollerConfig.MotionMagic
                .withMotionMagicAcceleration(IntakeConstants.INTAKE_ROLLER_MAX_ACCELERATION)
                .withMotionMagicCruiseVelocity(IntakeConstants.INTAKE_ROLLER_CRUISE_VELOCITY);
        rollerConfig.MotorOutput
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);
        rollerConfig.CurrentLimits
                .withStatorCurrentLimit(IntakeConstants.INTAKE_ROLLER_STATOR_CURRENT_LIMIT.baseUnitMagnitude())
                .withSupplyCurrentLimit(IntakeConstants.INTAKE_ROLLER_SUPPLY_CURRENT_LIMIT.baseUnitMagnitude());
        rollerConfig.Slot0
                .withKP(IntakeConstants.INTAKE_ROLLER_PID[0])
                .withKI(IntakeConstants.INTAKE_ROLLER_PID[1])
                .withKD(IntakeConstants.INTAKE_ROLLER_PID[2])
                .withGravityType(GravityTypeValue.Elevator_Static);

        rollerMotor.getConfigurator().apply(rollerConfig);
    }
}
