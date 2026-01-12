package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class superstructure {
    
    private final IntakeSubsystem intake;

    public superstructure(IntakeSubsystem intake) {
        this.intake = intake;
    }

    public Command intake() {
        return Commands.runOnce(() -> this.intake.intake(), this.intake);
    }

    public Command outtake() {
        return Commands.runOnce(() -> this.intake.outtake(), this.intake);
    }

    public Command stopIntakeMotor() {
        return Commands.runOnce(() -> this.stopIntakeMotor(), this.intake);
    }
}
