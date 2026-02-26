package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drivetrain.AutoAlign;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper.HopperSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.util.RobotEvent.Event.*;

public class superstructure extends SubsystemBase {

    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final HopperSubsystem hopper;
    private final LED led;
    private final AutoAlign autoAlign;
    private final List<ShootingStateTrue> ShootingStateTrue = new ArrayList<>();
    private final List<ShootingStateFalse> ShootingStateFalse = new ArrayList<>();

    public superstructure(
            ShooterSubsystem shooter,
            IntakeSubsystem intake,
            HopperSubsystem hopper,
            LED led,
            AutoAlign autoAlign) {
        this.shooter = shooter;
        this.intake = intake;
        this.hopper = hopper;
        this.led = led;
        this.autoAlign = autoAlign;
    }

    public Command autoshooter() {
        return shootCommand();
    }

    // Intake Methods

    public Command intake() {
        return Commands.parallel(
                Commands.runOnce(intake::armDown),
                Commands.runOnce(intake::rollerStart));
    }

    public Command stopintake() {
        return Commands.parallel(
                Commands.runOnce(intake::rollerEnd),
                Commands.runOnce(intake::armUp));
    }


    public Command shootCommand() {
        return Commands.run(() -> {
            this.setShootingStateTrue();

            if (shooter.isAtSetPosition()) {
                hopper.warmUpforshoot();
                this.shooter.shoot();
            } else {
                this.shooter.shoot();
                hopper.warmUpforshoot();
            }
        });
    }

    public Command stopShoot() {
        return Commands.parallel(
                Commands.runOnce(this::setShootingStateFalse),
                Commands.runOnce(this.shooter::stopShoot),
                Commands.runOnce(hopper::waiting));
    }
    

    public Command DriveToTrench() {
        return this.autoAlign.DriveToTrench();
    }

    public void TriggerShootingStateTrue(ShootingStateTrue event) {
        ShootingStateTrue.add(event);
    }

    public void TriggerShootingStateFalse(ShootingStateFalse event) {
        ShootingStateFalse.add(event);
    }

    public void setShootingStateTrue() {
        for (ShootingStateTrue listener : ShootingStateTrue) {
            listener.ShootingStateTrue();
        }
    }

    public void setShootingStateFalse() {
        for (ShootingStateFalse listener : ShootingStateFalse) {
            listener.ShootingStateFalse();
        }
    }

    @Override
    public void periodic() {
    }
}