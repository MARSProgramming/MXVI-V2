package frc.robot.auto.plays.BothAlliance;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.AutoBalance;
import frc.robot.commands.Drive.DriveAtPath;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.commands.Drive.ZeroGyroscope;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Manipulator;
import frc.robot.util.AutoChooser;

public class TP1_Cone_NoDock extends SequentialCommandGroup{
    public TP1_Cone_NoDock(DrivetrainSubsystem drivetrain, Manipulator mManipulator){
        addRequirements(drivetrain, mManipulator);

        PathPlannerTrajectory MarkertoP1 = AutoChooser.openTrajectoryFileForAlliance("BLUE_TopMarker_M-P1Cone", new PathConstraints(3, 1));
        PathPlannerTrajectory P1toMarker = AutoChooser.openTrajectoryFileForAlliance("BLUE_TopMarker_P1-MCone", new PathConstraints(3, 1));
        addCommands(
            new ZeroGyroscope(drivetrain, 180).withTimeout(0.03),
            new ResetDrivePose(drivetrain, MarkertoP1.getInitialHolonomicPose()).withTimeout(0.03),
            mManipulator.swapAutoScoreCommand().withTimeout(0.03),
            mManipulator.goToScoreHigh().withTimeout(4.0),
            new DriveAtPath(drivetrain, MarkertoP1, false, false, 5.0).deadlineWith(
                mManipulator.goToCubeIntake(), mManipulator.getGrasper().runTestMode()
            ),
            new ParallelCommandGroup(
                mManipulator.getGrasper().runTestCurrent().withTimeout(3),
                new DriveAtPath(drivetrain, P1toMarker, false, false, 4.0).deadlineWith(
                    mManipulator.goToZero().withTimeout(0.5).andThen(mManipulator.goToCubeShootHigh())
                )
            ),
            mManipulator.getGrasper().runSpitMode().withTimeout(0.3),
            mManipulator.goToZero()
        );
    }
}
