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
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.util.RobotEvent.Event.*;

public class superstructure extends SubsystemBase {

    private final CommandSwerveDrivetrain drive;

    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final HopperSubsystem hopper;
    private final AutoAlign autoAlign;
    private final List<ShootingStateTrue> ShootingStateTrue = new ArrayList<>();
    private final List<ShootingStateFalse> ShootingStateFalse = new ArrayList<>();

    public superstructure(
            CommandSwerveDrivetrain drive,
            ShooterSubsystem shooter,
            IntakeSubsystem intake,
            HopperSubsystem hopper,
            AutoAlign autoAlign) {
        this.drive = drive;
        this.shooter = shooter;
        this.intake = intake;
        this.hopper = hopper;
        this.autoAlign = autoAlign;
    }

    public Command autoshooter(){
       return shootCommand();
    }

    // Intake Methods

    public Command intake() {
        return Commands.parallel(
            Commands.runOnce(intake::rollerStart),
            Commands.runOnce(intake::armDown),
            Commands.runOnce(hopper::warmUp, hopper));
    }
    public Command stopintake(){
        return Commands.parallel(Commands.runOnce(intake::rollerEnd),
        Commands.runOnce(hopper::stopSpin, hopper));
    }

public Command shootCommand() {
    // 使用 run() 來建立一個持續執行的 Command
    return Commands.run(() -> {
        this.setShootingStateTrue();

        if (shooter.isAtSetPosition()) {
            // ✅ 轉速/角度到位 -> 進彈 (射擊)
            hopper.load();
            hopper.warmUp();
        } else {
            hopper.warmUp(); 
        }
    }, hopper)
    
    .finallyDo(() -> {
        this.setShootingStateFalse();
        hopper.stopAll();
    });
}


    public Command stopShoot() {
        return Commands.parallel(
            Commands.runOnce(hopper::stopAll),
            Commands.runOnce(this::setShootingStateFalse)
        );
    }

    // Hopper Methods

    public Command warmUpCommand() {
        return Commands.runOnce(hopper::warmUp);
    }

    public Command stopWashCommand() {
        return Commands.runOnce(hopper::stopSpin);
    }

    public Command loadCommand() {
        return Commands.startEnd(
                hopper::load,
                hopper::stopTrigger);
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
        this.hopper.warmUp();
    }
}
