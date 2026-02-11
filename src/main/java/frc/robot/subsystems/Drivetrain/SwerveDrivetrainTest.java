    package frc.robot.subsystems.Drivetrain;

    import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
    import com.ctre.phoenix6.hardware.TalonFX;

    import edu.wpi.first.units.measure.Current;
    import edu.wpi.first.units.measure.Voltage;
    import edu.wpi.first.wpilibj2.command.CommandScheduler;
    import edu.wpi.first.wpilibj2.command.SubsystemBase;
    import frc.robot.Constants.SwerveModuleConstants;

    public class SwerveDrivetrainTest extends SubsystemBase {

        private final TalonFX driveMotor, steerMotor;

        private final StatusSignal<Current> 
                driveStator, driveSupply, driveTorque, 
                steerStator, steerSupply, steerTorque;

        private final StatusSignal<Voltage> driveVoltage, steerVoltage;

        private final int ModuleIndex;

        public SwerveDrivetrainTest(
                CommandSwerveDrivetrain drivetrain,
                int moduleIndex) {
            this.ModuleIndex = moduleIndex;
            this.driveMotor = drivetrain.getModule(moduleIndex).getDriveMotor();
            this.steerMotor = drivetrain.getModule(moduleIndex).getSteerMotor();
            
            this.driveStator = driveMotor.getStatorCurrent();
            this.driveSupply = driveMotor.getSupplyCurrent();
            this.driveTorque = driveMotor.getTorqueCurrent();
            this.driveVoltage = driveMotor.getMotorVoltage();

            this.steerStator = steerMotor.getStatorCurrent();
            this.steerSupply = steerMotor.getSupplyCurrent();
            this.steerTorque = steerMotor.getTorqueCurrent();
            this.steerVoltage = steerMotor.getMotorVoltage();

            CommandScheduler.getInstance().registerSubsystem(this);
        }

        @Override
        public void periodic() {
            refreshSignal();

            logMotorCurrents();
        }

        public void refreshSignal() {

            BaseStatusSignal.refreshAll(
                this.driveStator,
                this.driveSupply,
                this.driveTorque,
                this.driveVoltage,
                this.driveVoltage,
                this.steerStator,
                this.steerStator,
                this.steerSupply,
                this.steerSupply,
                this.steerTorque,
                this.steerVoltage
            );
        }


        public void logMotorCurrents() {

            Logger.recordOutput("Drivetrain/CurrentTest/"+ SwerveModuleConstants.ModuleName[ModuleIndex] +"/DriveMotor/StatorCurrent", driveStator.getValueAsDouble());
            Logger.recordOutput("Drivetrain/CurrentTest/"+ SwerveModuleConstants.ModuleName[ModuleIndex] +"/DriveMotor/SupplyCurrent", driveSupply.getValueAsDouble());
            Logger.recordOutput("Drivetrain/CurrentTest/"+ SwerveModuleConstants.ModuleName[ModuleIndex] +"/DriveMotor/TorqueCurrent", driveTorque.getValueAsDouble());
            Logger.recordOutput("Drivetrain/CurrentTest/"+ SwerveModuleConstants.ModuleName[ModuleIndex] +"/DriveMotor/Voltage", driveVoltage.getValueAsDouble());

            Logger.recordOutput("Drivetrain/CurrentTest/"+ SwerveModuleConstants.ModuleName[ModuleIndex] +"/SteerMotor/StatorCurrent", steerStator.getValueAsDouble());
            Logger.recordOutput("Drivetrain/CurrentTest/"+ SwerveModuleConstants.ModuleName[ModuleIndex] +"/SteerMotor/SupplyCurrent", steerSupply.getValueAsDouble());
            Logger.recordOutput("Drivetrain/CurrentTest/"+ SwerveModuleConstants.ModuleName[ModuleIndex] +"/SteerMotor/TorqueCurrent", steerTorque.getValueAsDouble());
            Logger.recordOutput("Drivetrain/CurrentTest/"+ SwerveModuleConstants.ModuleName[ModuleIndex] +"/SteerMotor/Voltage", steerVoltage.getValueAsDouble());


            if (driveStator.getValueAsDouble() > 60.0) {
                Logger.recordOutput("Drivetrain/Warnings/" + SwerveModuleConstants.ModuleName[ModuleIndex] + "/DriveMotorHighCurrentStall", true);
            } else {
                Logger.recordOutput("Drivetrain/Warnings/" + SwerveModuleConstants.ModuleName[ModuleIndex] + "/DriveMotorHighCurrentStall", false);
            }

            if (steerStator.getValueAsDouble() > 60.0) {
                Logger.recordOutput("Drivetrain/Warnings/" + SwerveModuleConstants.ModuleName[ModuleIndex] + "/SteerMotorHighCurrentStall", true);
            } else {
                Logger.recordOutput("Drivetrain/Warnings/" + SwerveModuleConstants.ModuleName[ModuleIndex] + "/SteerMotorHighCurrentStall", false);
            }
        }
    }