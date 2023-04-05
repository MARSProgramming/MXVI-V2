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

public class BScoreGivenLeaveDock extends SequentialCommandGroup{
    private DrivetrainSubsystem mDrivetrain;
    public BScoreGivenLeaveDock(DrivetrainSubsystem drivetrain, Manipulator mManipulator){
        mDrivetrain = drivetrain;
        addRequirements(drivetrain);
        PathPlannerTrajectory LeaveCommunity = AutoChooser.openTrajectoryFileForAlliance("BLUE_BottomMarker_M-C", new PathConstraints(1.5, 0.5));
        addCommands(
            new ZeroGyroscope(drivetrain, 180).withTimeout(0.1),
            new ResetDrivePose(drivetrain, LeaveCommunity.getInitialHolonomicPose()),
            mManipulator.goToCubeShootHigh().withTimeout(1).deadlineWith(mManipulator.getGrasper().runTestCurrent()),
            mManipulator.getGrasper().runSpitMode().withTimeout(0.5),
            new DriveAtPath(drivetrain, LeaveCommunity, true, false, 100).deadlineWith(mManipulator.goToZero()),
            new AutoBalance(drivetrain)
         );
    }
}
