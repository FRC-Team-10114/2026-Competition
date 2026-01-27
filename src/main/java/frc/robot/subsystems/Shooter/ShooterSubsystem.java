package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Radians;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.ShooterCalculator.ShootingState;
import frc.robot.subsystems.Shooter.ShooterCalculator.ShootState;
import frc.robot.subsystems.RobotStatus;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIO;
import frc.robot.subsystems.Shooter.Hood.HoodIO;
import frc.robot.subsystems.Shooter.Turret.TurretIO;

public class ShooterSubsystem extends SubsystemBase {

    private final HoodIO hood;
    private final FlywheelIO flywheel;
    private final TurretIO turret;
    private final ShooterCalculator shooterCalculator;
    private final CommandSwerveDrivetrain drive;

    private ShootState currentShootState = ShootState.TRACKING;

    private Angle currentTurretTargetAngle = Degree.of(0);

    private final RobotStatus robotStatus;

    // 建構子需要傳入 Robot Rotation Supplier，因為 Turret 解繞需要它
    public ShooterSubsystem(HoodIO hood, FlywheelIO flywheel, TurretIO turret, ShooterCalculator shooterCalculator,CommandSwerveDrivetrain drive,RobotStatus robotStatus) {
        this.hood = hood;
        this.flywheel = flywheel;
        this.turret = turret;
        this.shooterCalculator = shooterCalculator;
        this.drive = drive;
        this.robotStatus = robotStatus;

        this.turret.resetAngle();
    }

    @Override
    public void periodic() {
        ShootingState state = this.shootergoal();

        if (state != null) {

            Rotation2d targetFieldAngle = state.turretFieldAngle();

            Angle targetAngleUnit = Radians.of(targetFieldAngle.getRadians());

            this.setTurretAngle(drive.getRotation(),targetAngleUnit);
        }
    }

    public ShootingState shootergoal(){
        if(robotStatus.getArea() == RobotStatus.Area.CENTER){
            return  this.shooterCalculator.calculateShootingToAlliance();
        }else{
            return this.shooterCalculator.calculateShootingToHub();
        }
    }

    // --- 設定狀態的 Command ---

    public void setShootingState(boolean shooting) {
        if (shooting) {
            this.currentShootState = ShootState.ACTIVE_SHOOTING;
        } else {
            this.currentShootState = ShootState.TRACKING;
        }
    }

    public Command aimCommand() {
        return this.startEnd(
                () -> setShootingState(true), // 按下開始：進入射擊模式 (開放極限)
                () -> setShootingState(false) // 放開結束：回到追蹤模式 (縮小範圍)
        );
    }


    public void setHoodAngle(Angle targetRad) {
        this.hood.setAngle(targetRad);
    }

    public void setRollerVelocity(AngularVelocity velocity) {
        this.flywheel.setRPM(velocity);
    }

    public void setTurretAngle(Rotation2d robotAngle, Angle targetRad) {

        Angle setpoint = this.shooterCalculator.TurretCalculate(robotAngle, targetRad, currentShootState);

        this.turret.setControl(setpoint);

    }
}