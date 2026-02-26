package frc.robot.subsystems.Intake.Roller;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.IDs;
import frc.robot.subsystems.Intake.IntakeConstants.RollerConstants;

public class RollerIOTalon implements RollerIO{

    private final TalonFX rollerMotor;
    private final StatusSignal<AngularVelocity> rollerVelocity;
    private final VoltageOut output;

    public RollerIOTalon() {
        this.rollerMotor = new TalonFX(IDs.Intake.ROLLER_MOTOR, "canivore");
        this.rollerVelocity = rollerMotor.getVelocity();
        this.output = new VoltageOut(Volts.of(0));
        configure();
    }

    @Override
    public void setVoltage(Voltage voltage) {
        this.rollerMotor.setControl(output.withOutput(voltage));
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
                .withSensorToMechanismRatio(RollerConstants.VELOCITY_CONVERSION_FACOTR);
        rollerConfig.MotorOutput
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast);
        rollerConfig.CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(RollerConstants.STATOR_CURRENT_LIMIT.in(Amps))
                .withSupplyCurrentLimit(RollerConstants.SUPPLY_CURRENT_LIMIT.in(Amps));
        rollerConfig.Slot0
                .withKP(RollerConstants.PID[0])
                .withKI(RollerConstants.PID[1])
                .withKD(RollerConstants.PID[2])
                .withGravityType(GravityTypeValue.Elevator_Static);

        rollerMotor.getConfigurator().apply(rollerConfig);
    }
}
