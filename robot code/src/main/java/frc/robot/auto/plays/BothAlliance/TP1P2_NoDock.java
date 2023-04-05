package frc.robot.auto.plays.BothAlliance;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Drive.DriveAtPath;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.commands.Drive.ZeroGyroscope;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Manipulator;
import frc.robot.util.AutoChooser;

public class TP1P2_NoDock extends SequentialCommandGroup{
    public TP1P2_NoDock(DrivetrainSubsystem drivetrain, Manipulator mManipulator){
        addRequirements(drivetrain, mManipulator);

        PathPlannerTrajectory MarkertoP1 = AutoChooser.openTrajectoryFileForAlliance("BLUE_TopMarker_M-P1", new PathConstraints(2, 0.75));
        PathPlannerTrajectory P1toMarker = AutoChooser.openTrajectoryFileForAlliance("BLUE_TopMarker_P1-M", new PathConstraints(2.5, 1.5));
        PathPlannerTrajectory MarkertoP2 = AutoChooser.openTrajectoryFileForAlliance("BLUE_TopMarker_M-P2", new PathConstraints(2, 1.5));
        PathPlannerTrajectory P2toMarker = AutoChooser.openTrajectoryFileForAlliance("BLUE_TopMarker_P2-M", new PathConstraints(2.5, 1.5));
        addCommands(
            new ZeroGyroscope(drivetrain, 180).withTimeout(0.1),
            new ResetDrivePose(drivetrain, MarkertoP1.getInitialHolonomicPose()),
            //mManipulator.goToShoot().withTimeout(3).deadlineWith(mManipulator.getGrasper().runTestCurrent()),
            //mManipulator.getGrasper().runSpitMode().withTimeout(0.3),
            new DriveAtPath(drivetrain, MarkertoP1, false, false, 3.5).deadlineWith(
                //mManipulator.goToCubeIntake(),
                //mManipulator.getGrasper().runTestMode()
            ),
            new ParallelCommandGroup(
                new DriveAtPath(drivetrain, P1toMarker, false, false, 4.0).deadlineWith(
                    //mManipulator.getGrasper().runTestCurrent()
                )//,
                //mManipulator.goToZero().withTimeout(2.0).andThen(mManipulator.goToCubeShootHigh().withTimeout(2.0))
            ),
           // mManipulator.getGrasper().runSpitMode().withTimeout(0.5),
            //mManipulator.goToZero().withTimeout(1)
            new DriveAtPath(drivetrain, MarkertoP2, false, false, 3.5).alongWith(
            ),
            new DriveAtPath(drivetrain, P2toMarker, false, false, 5).alongWith(
                mManipulator.goToZero()
            )
                // Move arm (if necessary) to position game piece for scoring
                // Open claw
                // Move arm into "default" position
        );
    }
}
