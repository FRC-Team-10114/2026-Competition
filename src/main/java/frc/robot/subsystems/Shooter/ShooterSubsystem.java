package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import com.google.gson.annotations.Until;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.ShooterCalculator.ShootingState;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelHardware;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIO;
import frc.robot.subsystems.Shooter.Hood.HoodIO;
import frc.robot.subsystems.Shooter.Hood.HoodTalon;
import frc.robot.subsystems.Shooter.Turret.TurretIO;
import frc.robot.subsystems.Shooter.Turret.TurretIONEO;
import frc.robot.subsystems.Shooter.Turret.TurretIO.ShootState;
import frc.robot.util.RobotStatus.RobotStatus;

public class ShooterSubsystem extends SubsystemBase {

    private final HoodIO hood;
    private final FlywheelIO flywheel;
    private final TurretIO turret;
    private final ShooterCalculator shooterCalculator;
    private final CommandSwerveDrivetrain drive;

    private ShootState currentShootState = ShootState.TRACKING;

    private final RobotStatus robotStatus;

    private double flywheelRPS = 0.0;

    private boolean Targetactive = true;

    private boolean Isshooting = false;

    private boolean InTrench = false;

    private Angle m_targetAngle = Degrees.of(25);

    private Angle turretAngle = Radians.of(0);

    public ShooterSubsystem(HoodIO hood, FlywheelIO flywheel, TurretIO turret, ShooterCalculator shooterCalculator,
            CommandSwerveDrivetrain drive, RobotStatus robotStatus) {
        this.hood = hood;
        this.flywheel = flywheel;
        this.turret = turret;
        this.shooterCalculator = shooterCalculator;
        this.drive = drive;
        this.robotStatus = robotStatus;
    }

    public static ShooterSubsystem create(CommandSwerveDrivetrain drive, RobotStatus status) {
        return new ShooterSubsystem(
                new HoodTalon(),
                new FlywheelHardware(),
                new TurretIONEO(),
                new ShooterCalculator(drive, status),
                drive,
                status);
    }

    @Override
    public void periodic() {
        SetShooterGoal();
        Logger.recordOutput("HoodgoalAngle", m_targetAngle);
        Logger.recordOutput("flywheelRPS", flywheelRPS);
        Logger.recordOutput("HoodAngle", this.hood.getAngle());
        Logger.recordOutput("InTrench", InTrench);
        Logger.recordOutput("Turretangle", this.turret.getAngle());
        Logger.recordOutput("Turretgoal", turretAngle);
        Logger.recordOutput("shooterisAtSetPosition", isAtSetPosition());
        hood.setAngle(m_targetAngle);
    }

    public void TrueIsshooting() {
        Isshooting = true;
    }

    public void FalseIsshooting() {
        Isshooting = false;
    }

    public void TrueTargetactive() {
        Targetactive = true;
    }

    public void FalseTargetactive() {
        Targetactive = false;
    }

    public void TrueInTrench() {
        InTrench = true;
    }

    public void FalsInTrench() {
        InTrench = false;
    }

    public boolean SpinAllTime() {
        if (Targetactive == true && robotStatus.isInMyAllianceZone() == true) {
            return true;
        } else {
            return false;
        }
    }

    public ShootingState shooterTargetChoose() {
        if (robotStatus.getArea() == RobotStatus.Area.CENTER) {
            return this.shooterCalculator.calculateShootingToAlliance();
        } else {
            return this.shooterCalculator.calculateShootingToHub();
        }
    }

    public void SetShooterGoal() {
        ShootingState state = this.shooterTargetChoose();

        Rotation2d targetFieldAngle = state.turretFieldAngle();

        Angle TurretTarget = Radians.of(targetFieldAngle.getRadians());

        Angle HoodTarget = state.HoopAngle();

        AngularVelocity FlywheelRPS = state.FlywheelRPS();

        // this.setHoodAngle(HoodTarget);

        // this.setRollerRPS(FlywheelRPS);

        // this.setTurretAngle(drive.getRotation(), TurretTarget);
    }

    public void setHoodAngle(Angle targetRad) {
        if (InTrench) {
            this.hood.setAngle(ShooterConstants.Hood_MIN_RADS);
        } else {
            this.hood.setAngle(targetRad);
        }
    }

    public void setRollerRPS(AngularVelocity velocity) {
        if (SpinAllTime() || Isshooting) {
            this.flywheel.setRPS(velocity);
        }
    }

    public void setTurretAngle(Rotation2d robotAngle, Angle targetRad) {

        this.turret.setAngle(robotAngle, targetRad, currentShootState);

    }
    public boolean isAtSetPosition(){
        return (this.hood.isAtSetPosition() && this.turret.isAtSetPosition() && this.flywheel.isAtSetPosition());
    }

    // TEST METHOD

    public void hoodUp() {
        m_targetAngle = m_targetAngle.plus(Degrees.of(1));
    }

    public void hoodDown() {
        m_targetAngle = m_targetAngle.minus(Degrees.of(1));
    }

    public void flywheelup() {
        this.flywheelRPS += 1;
        this.flywheel.setRPS(RotationsPerSecond.of(flywheelRPS));
    }

    public void flywheeldown() {
        this.flywheelRPS -= 1;
        this.flywheel.setRPS(RotationsPerSecond.of(flywheelRPS));
    }

    public void turretup() {
        this.turretAngle = turretAngle.plus(Radians.of(Units.degreesToRadians(5.0)));
        this.turret.setAngle(new Rotation2d(0), turretAngle , currentShootState);
    }

    public void turretdown() {
        this.turretAngle = turretAngle.minus(Radians.of(Units.degreesToRadians(5.0)));
        this.turret.setAngle(new Rotation2d(0), turretAngle, currentShootState);
    }

    public void setShootingState() {
        if (Isshooting) {
            this.currentShootState = ShootState.ACTIVE_SHOOTING;
        } else {
            this.currentShootState = ShootState.TRACKING;
        }
    }
}