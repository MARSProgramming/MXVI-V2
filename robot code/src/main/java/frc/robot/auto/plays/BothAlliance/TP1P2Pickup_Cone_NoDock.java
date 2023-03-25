package frc.robot.auto.plays.BothAlliance;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Drive.AutoBalance;
import frc.robot.commands.Drive.DriveAtPath;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.commands.Drive.ZeroGyroscope;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Manipulator;
import frc.robot.util.AutoChooser;

public class TP1P2Pickup_Cone_NoDock extends SequentialCommandGroup{
    public TP1P2Pickup_Cone_NoDock(DrivetrainSubsystem drivetrain, Manipulator mManipulator){
        addRequirements(drivetrain, mManipulator);

        PathPlannerTrajectory MarkertoP1 = AutoChooser.openTrajectoryFile("BLUE_TopMarker_M-P1Cone", new PathConstraints(3, 1.5));
        PathPlannerTrajectory P1toMarker = AutoChooser.openTrajectoryFile("BLUE_TopMarker_P1-MCone_Three", new PathConstraints(3, 2.0));
        PathPlannerTrajectory MarkerToP2 = AutoChooser.openTrajectoryFile("BLUE_TopMarker_M-P2_Fast", new PathConstraints(3, 2.0));
        PathPlannerTrajectory MarkerToP2_Part2 = AutoChooser.openTrajectoryFile("BLUE_TopMarker_M-P2_Fast_Part2", new PathConstraints(1.0, 1.0));
        addCommands(
            new ZeroGyroscope(drivetrain, 180).withTimeout(0.03),
            new ResetDrivePose(drivetrain, MarkertoP1.getInitialHolonomicPose()).withTimeout(0.03),
            mManipulator.swapAutoScoreCommand().withTimeout(0.03),
            mManipulator.goToScoreHigh().withTimeout(3.0),
            mManipulator.swapAutoScoreCommand().withTimeout(0.03),
            mManipulator.goToCubeIntake().withTimeout(0.3),
            new DriveAtPath(drivetrain, MarkertoP1, false, false, 5.0).deadlineWith(
                mManipulator.goToCubeIntake(), mManipulator.getGrasper().runTestMode()
            ),
            new DriveAtPath(drivetrain, P1toMarker, false, false, 2.9).deadlineWith(
                mManipulator.goToZero().withTimeout(0.3).andThen(mManipulator.goToCubeShootHigh()),
                mManipulator.getGrasper().runTestCurrent().withTimeout(2.3).andThen(mManipulator.getGrasper().runSpitMode().withTimeout(0.1))
            ),
            new SequentialCommandGroup(
                new DriveAtPath(drivetrain, MarkerToP2, false, false, 3.7),
                new DriveAtPath(drivetrain, MarkerToP2_Part2, false, false, 4.0).deadlineWith(
                    mManipulator.getGrasper().runTestMode()
                )
            ).deadlineWith(mManipulator.goToHighIntake()),
            mManipulator.goToZero().alongWith(mManipulator.getGrasper().runTestCurrent())
        );
    }
}
