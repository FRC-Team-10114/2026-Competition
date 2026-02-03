package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.Arm.ArmIO;
import frc.robot.subsystems.Intake.Roller.RollerIO;

public class IntakeSubsystem extends SubsystemBase {
    
    private final ArmIO arm;

    private final RollerIO roller;

    public IntakeSubsystem(ArmIO arm, RollerIO roller) {
        this.arm = arm;
        this.roller = roller;
    }

    public void intake() {
        roller.setVoltage(Volts.of(8));
        arm.setPosition(Radians.of(Math.PI));
    }

    public void outtake() {
        roller.setVoltage(Volts.of(-8));
    }

    public void back() {
        roller.setVoltage(Volts.of(0.0));
        arm.setPosition(Radians.of(0.0));
    }
}
