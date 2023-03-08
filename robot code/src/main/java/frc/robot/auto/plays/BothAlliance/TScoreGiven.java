package frc.robot.auto.plays.BothAlliance;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.commands.Drive.ZeroGyroscope;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Manipulator;
import frc.robot.util.AutoChooser;

public class TScoreGiven extends SequentialCommandGroup{
    private DrivetrainSubsystem mDrivetrain;

    PathPlannerTrajectory LeaveCommunity = AutoChooser.openTrajectoryFileForAlliance("BLUE_TopLeaveCommunity", new PathConstraints(0.5, 1));
    public TScoreGiven(DrivetrainSubsystem drivetrain, Manipulator mManipulator){
        mDrivetrain = drivetrain;
        addRequirements(drivetrain);

        addCommands(
            new ZeroGyroscope(drivetrain, 180).withTimeout(0.1),
            new ResetDrivePose(drivetrain, LeaveCommunity.getInitialHolonomicPose()),
            mManipulator.goToShoot().withTimeout(3).deadlineWith(mManipulator.getGrasper().runTestCurrent()),
            mManipulator.getGrasper().runSpitMode().withTimeout(2),
            mManipulator.goToZero()
         );
    }
}
