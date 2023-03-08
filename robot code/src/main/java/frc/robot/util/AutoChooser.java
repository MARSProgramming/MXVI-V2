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
        autoChooser.addOption("BOTTOM: Set Pose", new BottomSetPose(mDrivetrainSubsystem));
        autoChooser.addOption("BOTTOM: Leave Community", new BottomLeaveCommunity(mDrivetrainSubsystem));
        autoChooser.addOption("BOTTOM: Score Given", new BScoreGiven(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("BOTTOM: Score Given Leave", new BScoreGivenLeave(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("BOTTOM: Score Given Leave Dock", new BScoreGivenLeaveDock(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("MIDDLE: Set Pose", new MidSetPose(mDrivetrainSubsystem));
        autoChooser.addOption("MIDDLE: Leave Community", new MidLeaveCommunity(mDrivetrainSubsystem));
        autoChooser.addOption("MIDDLE: Score Given", new MidScoreGiven(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("MIDDLE: Score Given Leave", new MidScoreGivenLeave(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("MIDDLE: Score Given Leave Dock", new MidScoreGivenLeaveDock(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("TOP: Set Pose", new TopSetPose(mDrivetrainSubsystem));
        autoChooser.addOption("TOP: Leave Community", new TopLeaveCommunity(mDrivetrainSubsystem));
        autoChooser.addOption("TOP: Score Given", new TScoreGiven(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("TOP: Score Given Leave", new TScoreGivenLeave(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("TOP: Score Given Leave Dock", new TScoreGivenLeaveDock(mDrivetrainSubsystem, mManipulator));
        
        preMatch.add(autoChooser).withSize(2, 1).withPosition(0, 0);
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
