package frc.robot.auto.plays.Blue;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.DriveAtPath;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.commands.Drive.ZeroGyroscope;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Manipulator;
import frc.robot.util.AutoChooser;

public class BLUE_TOP_SCORELEAVE extends SequentialCommandGroup{
    public BLUE_TOP_SCORELEAVE(DrivetrainSubsystem drivetrain, Manipulator mManipulator){
        addRequirements(drivetrain);

        PathPlannerTrajectory LeaveCommunity = AutoChooser.openTrajectoryFileForBlue("BLUE_TOP_LEAVE", new PathConstraints(1.0, 0.5));
        addCommands(
            new ZeroGyroscope(drivetrain, 180).withTimeout(0.1),
            new ResetDrivePose(drivetrain, LeaveCommunity.getInitialHolonomicPose()),
            mManipulator.swapAutoScoreCommand().withTimeout(0.1),
            mManipulator.goToScoreHigh().withTimeout(4),
            mManipulator.swapAutoScoreCommand().withTimeout(0.1),
            new DriveAtPath(drivetrain, LeaveCommunity, false, false, 100)
        );
    }
}
