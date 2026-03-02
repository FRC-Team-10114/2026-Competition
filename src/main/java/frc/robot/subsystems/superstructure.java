package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.logging.LogManager;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Climber.ClimberSubsystem;
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
    private final ClimberSubsystem Climber;
    private final LED led;
    private final AutoAlign autoAlign;
    private final List<ShootingStateTrue> ShootingStateTrue = new ArrayList<>();
    private final List<ShootingStateFalse> ShootingStateFalse = new ArrayList<>();

    public superstructure(
            ShooterSubsystem shooter,
            IntakeSubsystem intake,
            HopperSubsystem hopper,
            LED led,
            AutoAlign autoAlign,
            ClimberSubsystem Climber) {
        this.shooter = shooter;
        this.intake = intake;
        this.hopper = hopper;
        this.led = led;
        this.autoAlign = autoAlign;
        this.Climber = Climber;
    }

    public Command autoclimb() {
        return Commands.sequence(Commands.parallel(ClimbPrepare(), stopintake(), autoAlign.FindClimbPath()), Climb());
    }

    public Command ClimbPrepare() {
        return this.Climber.up();
    }

    public Command Climb() {
        return this.Climber.climbup();
    }

    public Command Climbdown() {
        return this.Climber.climbup();
    }

    public Command autoshooter() {
        return shootCommand();
    }

    // Intake Methods

    public Command Climberup() {
        return this.Climber.up();
    }

    public Command Climberdown() {
        return this.Climber.down();
    }

    public Command intake() {
        return this.intake.intake();
    }

    public Command stopintake() {
        return Commands.parallel(
                Commands.runOnce(intake::rollerEnd),
                Commands.runOnce(intake::armup));
    }

public Command shootCommand() {
        return Commands.parallel(
            
            this.intake.shootintake(),

            // 2. 執行射擊與供彈判斷的 Command
            Commands.run(() -> {
                this.setShootingStateTrue();
                
                this.shooter.shoot();

                if (shooter.isAtSetPosition()) {
                    hopper.warmUpforshoot();
                } else {
                    hopper.stopAll(); 
                }
            }, this.shooter, this.hopper)
        );
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
        Logger.recordOutput("climber", this.Climber.getPosition());
    }
}