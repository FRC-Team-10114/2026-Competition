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
        // 1. 設定射擊狀態 (通知 Shooter 開始加速/瞄準)
        // 建議這個放在 initialize 或這裡都可以，確保狀態是 True
        this.setShootingStateTrue();

        // 2. 【關鍵】在每一幀動態檢查是否到位
        if (shooter.isAtSetPosition()) {
            // ✅ 轉速/角度到位 -> 進彈 (射擊)
            hopper.load();
            hopper.warmUp();
        } else {
            // ⏳ 還沒到位 -> 只是預熱/攪拌 (防止卡彈)
            hopper.warmUp(); 
        }
    }, hopper) // ⚠️ 非常重要：必須宣告 require hopper 子系統，防止與其他 hopper 指令衝突
    
    // 3. 當放開按鈕或指令結束時 -> 復原狀態並停馬達
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
