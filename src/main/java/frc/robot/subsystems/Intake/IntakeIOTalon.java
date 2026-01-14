package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.IntakeConstants;


public class IntakeIOTalon implements IntakeIO {
    
    private final TalonFX intakeMotor;

    private final StatusSignal<AngularVelocity> intakeVelocity;

    private final VoltageOut voltageOut;

    public IntakeIOTalon() {

        this.intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);
        this.intakeVelocity = this.intakeMotor.getVelocity();
        this.voltageOut = new VoltageOut(0.0);
    }
    
    @Override
    public void configure() {
        TalonFXConfiguration configs = new TalonFXConfiguration();

        configs.CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(60)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40.0);

        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        intakeMotor.getConfigurator().apply(configs);
    }

    @Override
    public void setControl(double velocity, boolean isFOC) {
        this.intakeMotor.setControl(voltageOut.withOutput(velocity * 12)
                                    .withEnableFOC(isFOC));
    }

    @Override
    public double getVelocity() {
        intakeVelocity.refresh();
        return this.intakeVelocity.getValueAsDouble();
    }
}
