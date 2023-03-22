package frc.robot.auto.plays.BothAlliance;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.AutoBalance;
import frc.robot.commands.Drive.DriveAtPath;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.commands.Drive.ZeroGyroscope;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Manipulator;
import frc.robot.util.AutoChooser;

public class MidScoreGivenLeaveDock extends SequentialCommandGroup{
    public MidScoreGivenLeaveDock(DrivetrainSubsystem drivetrain, Manipulator mManipulator){
        addRequirements(drivetrain);

        PathPlannerTrajectory LeaveCommunity = AutoChooser.openTrajectoryFileForAlliance("BLUE_MiddleLeaveCommunity", new PathConstraints(1, 0.5));
        PathPlannerTrajectory Dock = AutoChooser.openTrajectoryFileForAlliance("BLUE_MiddleMarker_M-C", new PathConstraints(1, 0.5));
        addCommands(
            new ZeroGyroscope(drivetrain, 180).withTimeout(0.1),
            new ResetDrivePose(drivetrain, LeaveCommunity.getInitialHolonomicPose()),
            mManipulator.goToCubeShootHigh().withTimeout(1.0).deadlineWith(mManipulator.getGrasper().runTestCurrent()),
            mManipulator.getGrasper().runSpitMode().withTimeout(0.2),
            mManipulator.goToZero().withTimeout(1),
            new DriveAtPath(drivetrain, LeaveCommunity, false, false, 5),
            new DriveAtPath(drivetrain, Dock, false, false, 10),
            new AutoBalance(drivetrain)
        );
    }
}
