package frc.robot.subsystems.Hopper.Spindexer;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.IDs;
import frc.robot.subsystems.Hopper.HopperConstant.SpindexerConstants;

public class SpindexerIOHardware implements SpindexerIO {

    private final TalonFX mechineMotor;

    private final VoltageOut output;

    public SpindexerIOHardware() {
        this.mechineMotor = new TalonFX(IDs.Hopper.SPINDEXER_MOTOR, "canivore");
        this.output = new VoltageOut(Volts.of(0)); 

        configure();
    }

    @Override
    public void run(double vlot) {
        this.mechineMotor.setControl(output.withOutput(Volts.of(vlot)));
    }

    @Override
    public void stop() {
        this.mechineMotor.stopMotor();
    }
    @Override
    public double getStatorCurrent(){
        return this.mechineMotor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void configure() {
        var mechineConfig = new TalonFXConfiguration();

        mechineConfig.MotorOutput
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast);
        mechineConfig.CurrentLimits
                .withStatorCurrentLimit(SpindexerConstants.MECHINE_STATOR_CURRENT_LIMIT)
                .withSupplyCurrentLimit(SpindexerConstants.MECHINE_SUPPLY_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true);
        
        mechineMotor.getConfigurator().apply(mechineConfig);
    }
    
}
