package frc.robot.subsystems.Dashboard;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.FMS.Signal;

public class Dashboard extends SubsystemBase {

    private double matchTime, roundTime;
    private final Signal fms;

    private boolean endgameAlertTriggered = false;

    private boolean timeAlert = false;

    private final double MatchTime = 140.0;
    private final double TRANSITION = 10.0;
    private final double Round = 25.0;

    public Dashboard(Signal fms) {
        this.fms = fms;
        this.matchTime = -1;
        this.roundTime = -1;
    }

    @Override
    public void periodic() {
        updateMatchData();
        updateRoundTime();
        logDashboardData();
    }

    private void updateMatchData() {
        matchTime = DriverStation.getMatchTime();
    }

    private void logDashboardData() {
        Logger.recordOutput("Dashboard/MatchTime", matchTime);
        Logger.recordOutput("Dashboard/RoundTime", roundTime);
        Logger.recordOutput("Dashboard/IsFMSConnected", fms.Active());
        Logger.recordOutput("Dashboard/TimeAlert", timeAlert);

        Logger.recordOutput("Dashboard/BatteryVoltage", RobotController.getBatteryVoltage());

        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            Logger.recordOutput("Dashboard/Alliance", alliance.get().toString());
        } else {
            Logger.recordOutput("Dashboard/Alliance", "Unknown");
        }

        timeAlert();
    }

    private void checkTimeAlerts() {
        if (DriverStation.isTeleop() && matchTime > 0 && matchTime <= 20.0) {
            Logger.recordOutput("Dashboard/Alerts/Endgame", true); // 可以綁定 Dashboard 讓畫面閃爍
        } else {
            Logger.recordOutput("Dashboard/Alerts/Endgame", false);
        }
    }

    private void updateRoundTime() {
        if (DriverStation.isAutonomous()) {
            roundTime = DriverStation.getMatchTime();
            return;
        } else if (fms.isTRANSITION()) {
            roundTime = DriverStation.getMatchTime() - matchTime;
            return;
        } else if (DriverStation.getMatchTime() <= 30) {
            roundTime = DriverStation.getMatchTime();
        } else {
            for (int round = 1; round <= 4; round++) {
                if (fms.isInRound(round)) {
                    roundTime = DriverStation.getMatchTime() - matchTime - TRANSITION - (round - 1) * Round;
                    return;
                }
            }
        }
    }

    public void timeAlert() {
        if (roundTime <= 10) {
            alertBlink();
        } else {
            timeAlert = false;
        }
    }

    private void alertBlink() {
        timeAlert = (int) (Timer.getFPGATimestamp() * 2) % 2 == 0;
    }
}