package frc.robot.util;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.plays.DoNothing;
import frc.robot.auto.plays.BothAlliance.BP4_Cone_NoDock;
import frc.robot.auto.plays.BothAlliance.BScoreGiven;
import frc.robot.auto.plays.BothAlliance.BScoreGivenLeave;
import frc.robot.auto.plays.BothAlliance.BScoreGivenLeaveDock;
import frc.robot.auto.plays.BothAlliance.BottomLeaveCommunity;
import frc.robot.auto.plays.BothAlliance.BottomSetPose;
import frc.robot.auto.plays.BothAlliance.MidLeaveCommunity;
import frc.robot.auto.plays.BothAlliance.MidScoreGiven;
import frc.robot.auto.plays.BothAlliance.MidScoreGivenLeave;
import frc.robot.auto.plays.BothAlliance.MidScoreGivenLeaveDock;
import frc.robot.auto.plays.BothAlliance.MidSetPose;
import frc.robot.auto.plays.BothAlliance.TP1P2Pickup_Cone_NoDock;
import frc.robot.auto.plays.BothAlliance.TP1P2_LowerCone_Dock;
import frc.robot.auto.plays.BothAlliance.TP1P2_LowerCone_NoDock;
import frc.robot.auto.plays.BothAlliance.TP1_Cone_Dock;
import frc.robot.auto.plays.BothAlliance.TP1_Cone_DockFar;
import frc.robot.auto.plays.BothAlliance.TP1_Cone_NoDock;
import frc.robot.auto.plays.BothAlliance.TP1_Cube_Dock;
import frc.robot.auto.plays.BothAlliance.TP1_Cube_NoDock;
import frc.robot.auto.plays.BothAlliance.TScoreGiven;
import frc.robot.auto.plays.BothAlliance.TScoreGivenLeave;
import frc.robot.auto.plays.BothAlliance.TScoreGivenLeaveDock;
import frc.robot.auto.plays.BothAlliance.TopLeaveCommunity;
import frc.robot.auto.plays.BothAlliance.TopSetPose;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Manipulator;

public class AutoChooser {
    private ShuffleboardTab preMatch;
    private SendableChooser<Command> autoChooser = new SendableChooser();

    public AutoChooser(DrivetrainSubsystem mDrivetrainSubsystem, Manipulator mManipulator){
        preMatch = Shuffleboard.getTab("Match");
        //autoChooser = new SendableChooser<>();
        

        //auto plays
        autoChooser.setDefaultOption("Do Nothing", new DoNothing());
        autoChooser.addOption("3: Set Pose", new BottomSetPose(mDrivetrainSubsystem));
        autoChooser.addOption("3: Leave Community", new BottomLeaveCommunity(mDrivetrainSubsystem));
        autoChooser.addOption("3: Score Given", new BScoreGiven(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("3: Score Given Leave", new BScoreGivenLeave(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("3: Score Given Leave Dock", new BScoreGivenLeaveDock(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("3: Score Cone and Cube High", new BP4_Cone_NoDock(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("2: Set Pose", new MidSetPose(mDrivetrainSubsystem));
        autoChooser.addOption("2: Leave Community", new MidLeaveCommunity(mDrivetrainSubsystem));
        autoChooser.addOption("2: Score Given", new MidScoreGiven(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("2: Score Given Leave", new MidScoreGivenLeave(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("2: Score Given Leave Dock", new MidScoreGivenLeaveDock(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("1: Set Pose", new TopSetPose(mDrivetrainSubsystem));
        autoChooser.addOption("1: Leave Community", new TopLeaveCommunity(mDrivetrainSubsystem));
        autoChooser.addOption("1: Score Given", new TScoreGiven(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("1: Score Given Leave", new TScoreGivenLeave(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("1: Score Given Leave Dock", new TScoreGivenLeaveDock(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("1: Score 2 Cube Dock", new TP1_Cube_Dock(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("1: Score 2 Cube", new TP1_Cube_NoDock(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("1-Cone: Score Cone and Cube High Dock", new TP1_Cone_Dock(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("1-Cone: Score Cone and Cube High Dock Far", new TP1_Cone_DockFar(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("1-Cone: Score Cone and Cube High Pickup Third", new TP1P2Pickup_Cone_NoDock(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("1-Cone: Score Cone and Cube High", new TP1_Cone_NoDock(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("1-LowerCone: Score 3", new TP1P2_LowerCone_NoDock(mDrivetrainSubsystem, mManipulator));
        //autoChooser.addOption("1-LowerCone: Score 3 Dock", new TP1P2_LowerCone_Dock(mDrivetrainSubsystem, mManipulator));


        preMatch.add("Auto Play", autoChooser).withSize(2, 1).withPosition(4, 5);
    }

    public Command getSelected(){
        return autoChooser.getSelected();
    }

    public static PathPlannerTrajectory openTrajectoryFile(String name, PathConstraints constraints){
        PathPlannerTrajectory t = PathPlanner.loadPath(name, constraints);
        return t;
    }
    public static PathPlannerTrajectory openTrajectoryFileForAlliance(String name, PathConstraints constraints){
        PathPlannerTrajectory t = PathPlanner.loadPath(name, constraints);
        return PathPlannerTrajectory.transformTrajectoryForAlliance(t, DriverStation.getAlliance());
    }
}
