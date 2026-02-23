// package frc.robot.commands;

// import org.photonvision.common.hardware.VisionLEDMode;

// import com.fasterxml.jackson.annotation.ObjectIdGenerators.None;
// import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.commands.PathPlannerAuto;

// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.subsystems.superstructure;
// import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
// import frc.robot.util.RobotStatus.RobotStatus;

// public class AutoChooser {
//     private final CommandSwerveDrivetrain drive;
//     private final superstructure superstructure;
//     private final RobotStatus robotStatus;
//     public final SendableChooser<ShowTime> ShowtimeChooser = new SendableChooser<>();
//     public final SendableChooser<AutoStart> AutoStartChooser = new SendableChooser<>();
//     public final SendableChooser<IfGoCenter> IfGoCenterChooser = new SendableChooser<>();
//     public final SendableChooser<IfGoclimb> IfGoClimbChooser = new SendableChooser<>();

//     public AutoChooser(CommandSwerveDrivetrain drive, superstructure superstructure, RobotStatus robotStatus) {
//         this.drive = drive;
//         this.superstructure = superstructure;
//         this.robotStatus = robotStatus;
//         this.SetNamedCommands();
//         this.configureAutoChoosers();

//     }

//     public void SetNamedCommands() {
//         NamedCommands.registerCommand("intakeDown", superstructure.intake());
//         NamedCommands.registerCommand("intakestop", superstructure.stopintake());
//         NamedCommands.registerCommand("HoodDown", Commands.runOnce(() -> robotStatus.SetSafeHood(true), robotStatus));
//         NamedCommands.registerCommand("StopHoodDown",
//                 Commands.runOnce(() -> robotStatus.SetSafeHood(false), robotStatus));
//         NamedCommands.registerCommand("shoot", superstructure.autoshooter().withTimeout(2.0));
//         NamedCommands.registerCommand("NoStopShoot", superstructure.autoshooter());
//         NamedCommands.registerCommand("stopshoot", superstructure.stopShoot());
//         NamedCommands.registerCommand("isIn",
//                 Commands.run(() -> System.out.println("Stop!!!!!!!")).until(robotStatus::isInTrench));
//     }

//     public enum ShowTime {
//         None, DoubleCenter, CleanAllCenter
//     }

//     public enum AutoStart {
//         LEFT, CENTER, RIGHT, NONE
//     }

//     public enum IfGoCenter {
//         GO_CENTER,
//         stay
//     }

//     public enum IfGoclimb {
//         climb,
//         ReverseClimb,
//         EndAtCenter,
//         None
//     }

//     public void configureAutoChoosers() {

//         AutoStartChooser.setDefaultOption("None", AutoStart.NONE);
//         AutoStartChooser.addOption("Start: Left", AutoStart.LEFT);
//         AutoStartChooser.addOption("Start: Right", AutoStart.RIGHT);
//         AutoStartChooser.addOption("Start: CENTER", AutoStart.CENTER);
//         SmartDashboard.putData("Start Position", AutoStartChooser);

//         ShowtimeChooser.setDefaultOption("None", ShowTime.None);
//         ShowtimeChooser.addOption("None", ShowTime.None);
//         ShowtimeChooser.addOption("DoubleCenter", ShowTime.DoubleCenter);
//         ShowtimeChooser.addOption("CleanAllCenter", ShowTime.CleanAllCenter);

//         IfGoCenterChooser.setDefaultOption("Go Center", IfGoCenter.GO_CENTER);
//         IfGoCenterChooser.addOption("Stay", IfGoCenter.stay);
//         SmartDashboard.putData("If Go Center", IfGoCenterChooser);

//         IfGoClimbChooser.setDefaultOption("Go Climb", IfGoclimb.climb);
//         IfGoClimbChooser.addOption("Reverse Climb", IfGoclimb.ReverseClimb);
//         IfGoClimbChooser.addOption("End At Center", IfGoclimb.EndAtCenter);
//         IfGoClimbChooser.addOption("None", IfGoclimb.None);
//         SmartDashboard.putData("End", IfGoClimbChooser);
//     }

//     public Command auto() {
//         AutoStart startPose = AutoStartChooser.getSelected();
//         ShowTime issShowTime = ShowtimeChooser.getSelected();
//         IfGoCenter IfGoCenter = IfGoCenterChooser.getSelected();
//         IfGoclimb IfGoclimb = IfGoClimbChooser.getSelected();

//         Command AutoCommand = Commands.none();
//         switch (issShowTime) {
//             case DoubleCenter:
//                 switch (startPose) {
//                     case LEFT:
//                         AutoCommand = new PathPlannerAuto("Left_DoubleCenter");
//                         break;
//                     case RIGHT:
//                         AutoCommand = new PathPlannerAuto("Right_DoubleCenter");
//                         break;
//                 }
//                 break;

//             default:
//                 break;
//         }
//         Command start = Commands.none();
//         switch (startPose) {
//             case LEFT:
//                 switch (IfGoCenter) {
//                     case GO_CENTER:
//                         start = new PathPlannerAuto("Left_center_deploy");
//                         break;
//                     case stay:
//                         start = new PathPlannerAuto("Left_Deploy");
//                 }
//                 break;
//             case RIGHT:
//                 switch (IfGoCenter) {
//                     case GO_CENTER:
//                         start = new PathPlannerAuto("Right_center_hum");
//                         break;
//                     case stay:
//                         start = new PathPlannerAuto("Right_hum");
//                 }
//             case CENTER:
//                 start = new PathPlannerAuto("Center");
//         }
//         Command end = Commands.none();
//         switch (IfGoclimb) {
//             case climb:
//                 switch (startPose) {
//                     case LEFT:
//                         end = new PathPlannerAuto("Left_End_Climb");
//                         break;
//                     case RIGHT:
//                         end = new PathPlannerAuto("Right_End_Climb");
//                         break;
//                     case CENTER:
//                         end = new PathPlannerAuto("Center_End_Climb");
//                 }
//             case ReverseClimb:
//                 switch (startPose) {
//                     case LEFT:
//                         end = new PathPlannerAuto("Left_End_ReverseClimb");
//                         break;
//                     case RIGHT:
//                         end = new PathPlannerAuto("Right_End_ReverseClimb");
//                         break;
//                     case CENTER:
//                         end = new PathPlannerAuto("Center_End_ReverseCilmb");
//                 }
//             case EndAtCenter:
//                     switch (startPose) {
//                         case LEFT:
//                             end = new PathPlannerAuto("Left_End_Center");
//                             break;
//                         case RIGHT:
//                             end = new PathPlannerAuto("Right_End_Center");
//                     }
//         }
//         if (AutoCommand != null) {
//             return AutoCommand;
//         } else {
//             return Commands.sequence(
//                     start,
//                     end);
//         }
//     }
// }
