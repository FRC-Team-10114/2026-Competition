package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.Arm.ArmIO;
import frc.robot.subsystems.Intake.Arm.ArmIOTalon;
import frc.robot.subsystems.Intake.Roller.RollerIO;
import frc.robot.subsystems.Intake.Roller.RollerIOTalon;

public class IntakeSubsystem extends SubsystemBase {

    private final ArmIO arm;

    private final RollerIO roller;

    public IntakeSubsystem(ArmIO arm, RollerIO roller) {
        this.arm = arm;
        this.roller = roller;
    }

    public static IntakeSubsystem create() {
        return new IntakeSubsystem(
                new ArmIOTalon(),
                new RollerIOTalon());
    }

    @Override
    public void periodic() {
        Logger.recordOutput("intakearmangle", this.arm.getPosition());

    }

    public void rollerStart() {
        this.roller.setVoltage(Volts.of(6));
    }

    public void rollerEnd() {
        this.roller.setVoltage(Volts.of(0));
    }

    public void armDown() {
        this.arm.setPosition(125);
    }

    public void armUp() {
        this.arm.setPosition(0.0);
    }
}
