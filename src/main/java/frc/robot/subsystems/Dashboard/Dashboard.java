package frc.robot.subsystems.Dashboard;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.FMS.Signal;

public class Dashboard extends SubsystemBase {
    
    private double matchTime;
    
    private final Signal fms;

    public Dashboard(Signal fms) {
        matchTime = -1;
        this.fms = fms;
    }

    @Override
    public void periodic() {
        matchTime = DriverStation.getMatchTime();
        this.countingDown();
        this.isActive();
    }

    public void countingDown() {
        Logger.recordOutput("matchTime", this.matchTime);
    }

    public void isActive() {
        Logger.recordOutput("isActive", fms.Active());
    }
}
