package frc.robot.auto.plays.BothAlliance;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Drive.AutoBalance;
import frc.robot.commands.Drive.DriveAtPath;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.commands.Drive.ZeroGyroscope;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Manipulator;
import frc.robot.util.AutoChooser;

public class TP1P2_LowerCone_Mid_Dock extends SequentialCommandGroup{
    public TP1P2_LowerCone_Mid_Dock(DrivetrainSubsystem drivetrain, Manipulator mManipulator){
        addRequirements(drivetrain, mManipulator);

        PathPlannerTrajectory MarkertoP1 = AutoChooser.openTrajectoryFileForAlliance("BLUE_TopLowerCone_M-P1Mid", new PathConstraints(2.0, 1.6));
        PathPlannerTrajectory P1toMarker = AutoChooser.openTrajectoryFileForAlliance("BLUE_TopLowerCone_P1-MMid", new PathConstraints(3.5, 2.5));
        PathPlannerTrajectory MarkerToP2 = AutoChooser.openTrajectoryFileForAlliance("BLUE_TopLowerCone_M-P2Mid", new PathConstraints(3.0, 2.0));
        PathPlannerTrajectory MarkerToP2_Part2 = AutoChooser.openTrajectoryFileForAlliance("BLUE_TopLowerCone_P2-MMidDock", new PathConstraints(4, 3));
        PathPlannerTrajectory Dock = AutoChooser.openTrajectoryFileForAlliance("BLUE_TopLowerCone_M-C", new PathConstraints(1.5, 1));
        addCommands(
            new ZeroGyroscope(drivetrain, 180).withTimeout(0.02),
            new ResetDrivePose(drivetrain, MarkertoP1.getInitialHolonomicPose()),
            mManipulator.swapAutoScoreCommand().withTimeout(0.02),
            mManipulator.goToScoreMid().withTimeout(1.3),
            new DriveAtPath(drivetrain, MarkertoP1, false, false, 5.0).deadlineWith(
                mManipulator.goToIntake(), mManipulator.getGrasper().setPercentOutputCommand(1)
            ),
            new DriveAtPath(drivetrain, P1toMarker, false, false, 2.1).alongWith
            (new WaitCommand(1.1).deadlineWith(mManipulator.goToZero(), mManipulator.getGrasper().setPercentOutputCommand(0.6)).andThen(mManipulator.goToScoreMid().withTimeout(1.4))),
            new DriveAtPath(drivetrain, MarkerToP2, false, false, 2.6).deadlineWith(
                mManipulator.goToCloseCubeIntake(), mManipulator.getGrasper().setPercentOutputCommand(1)
            ),
            new DriveAtPath(drivetrain, MarkerToP2_Part2, false, false, 5.0).deadlineWith(
                (mManipulator.getGrasper().runTestCurrent().withTimeout(1.6).deadlineWith(mManipulator.goToZero().withTimeout(0.5).andThen(mManipulator.goToShoot()))).andThen(mManipulator.getGrasper().setPercentOutputCommand(-1).withTimeout(0.2))
            ),
            //new DriveAtPath(drivetrain, Dock, false, false, 3.0).alongWith(
            //mManipulator.swapAutoScoreCommand().withTimeout(0.03)),
            new AutoBalance(drivetrain).alongWith(mManipulator.swapAutoScoreCommand().withTimeout(0.03))
        );
    }
}
