package frc.robot.subsystems.Dashboard;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.FMS.Signal;

public class Dashboard extends SubsystemBase {
    
    private double matchTime;
    private final Signal fms;

    private boolean endgameAlertTriggered = false;

    public Dashboard(Signal fms) {
        this.fms = fms;
        this.matchTime = -1;
    }

    @Override
    public void periodic() {
        updateMatchData();
        logDashboardData();
    }

    private void updateMatchData() {
        matchTime = DriverStation.getMatchTime();
    }

    private void logDashboardData() {
        Logger.recordOutput("Dashboard/MatchTime", matchTime);
        Logger.recordOutput("Dashboard/IsFMSConnected", fms.Active());

        Logger.recordOutput("Dashboard/BatteryVoltage", RobotController.getBatteryVoltage());

        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            Logger.recordOutput("Dashboard/Alliance", alliance.get().toString());
        } else {
            Logger.recordOutput("Dashboard/Alliance", "Unknown");
        }

        checkTimeAlerts();
    }

    private void checkTimeAlerts() {
        if (DriverStation.isTeleop() && matchTime > 0 && matchTime <= 20.0) {
            Logger.recordOutput("Dashboard/Alerts/Endgame", true); // 可以綁定 Dashboard 讓畫面閃爍
        } else {
            Logger.recordOutput("Dashboard/Alerts/Endgame", false);
        }
    }
}