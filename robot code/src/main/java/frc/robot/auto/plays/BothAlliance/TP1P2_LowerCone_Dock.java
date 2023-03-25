package frc.robot.auto.plays.BothAlliance;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Drive.DriveAtPath;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.commands.Drive.ZeroGyroscope;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Manipulator;
import frc.robot.util.AutoChooser;

public class TP1P2_LowerCone_Dock extends SequentialCommandGroup{
    public TP1P2_LowerCone_Dock(DrivetrainSubsystem drivetrain, Manipulator mManipulator){
        addRequirements(drivetrain, mManipulator);

        PathPlannerTrajectory MarkertoP1 = AutoChooser.openTrajectoryFileForAlliance("BLUE_TopLowerCone_M-P1", new PathConstraints(4, 3));
        PathPlannerTrajectory P1toMarker = AutoChooser.openTrajectoryFileForAlliance("BLUE_TopLowerCone_P1-M", new PathConstraints(3, 2));
        PathPlannerTrajectory MarkerToP2 = AutoChooser.openTrajectoryFileForAlliance("BLUE_TopLowerCone_M-P2", new PathConstraints(3.5, 2.5));
        PathPlannerTrajectory MarkerToP2_Part2 = AutoChooser.openTrajectoryFileForAlliance("BLUE_TopLowerCone_P2-M", new PathConstraints(3.5, 3));
        addCommands(
            new ZeroGyroscope(drivetrain, 180).withTimeout(0.03),
            new ResetDrivePose(drivetrain, MarkertoP1.getInitialHolonomicPose()).withTimeout(0.03),
            mManipulator.swapAutoScoreCommand().withTimeout(0.03),
            mManipulator.goToScoreHigh().withTimeout(2.3),
            mManipulator.goToIntake().withTimeout(0.3),
            new DriveAtPath(drivetrain, MarkertoP1, false, false, 1.5).deadlineWith(
                mManipulator.goToIntake(), mManipulator.getGrasper().setPercentOutputCommand(1)
            ),
            new DriveAtPath(drivetrain, P1toMarker, false, false, 2.6).alongWith
            (new WaitCommand(1.5).deadlineWith(mManipulator.goToZero(), mManipulator.getGrasper().runTestMode()).andThen(mManipulator.goToScoreHigh().withTimeout(2.7))),
            new DriveAtPath(drivetrain, MarkerToP2, false, false, 2.6).deadlineWith(
                new WaitCommand(0.4).andThen(mManipulator.goToCloseCubeIntake()), mManipulator.getGrasper().setPercentOutputCommand(1)
            ),
            new DriveAtPath(drivetrain, MarkerToP2_Part2, false, false, 2.0).deadlineWith(
                mManipulator.getGrasper().runTestCurrent(), mManipulator.goToZero().withTimeout(0.5).andThen(mManipulator.goToCubeShootHigh())
            ),
            mManipulator.getGrasper().runSpitMode().withTimeout(0.5),
            mManipulator.swapAutoScoreCommand().withTimeout(0.03)
        );
    }
}
