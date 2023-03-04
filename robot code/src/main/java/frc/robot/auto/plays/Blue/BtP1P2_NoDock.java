package frc.robot.auto.plays.Blue;

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

public class BtP1P2_NoDock extends SequentialCommandGroup{
    public BtP1P2_NoDock(DrivetrainSubsystem drivetrain, Manipulator mManipulator){
        addRequirements(drivetrain, mManipulator);

        PathPlannerTrajectory MarkertoP1 = AutoChooser.openTrajectoryFile("BLUE_TopMarker_M-P1", new PathConstraints(2.5, 0.75));
        PathPlannerTrajectory P1toMarker = AutoChooser.openTrajectoryFile("BLUE_TopMarker_P1-M", new PathConstraints(3, 0.75));
        PathPlannerTrajectory MarkertoP2 = AutoChooser.openTrajectoryFile("BLUE_TopMarker_M-P2", new PathConstraints(2.5, 0.75));
        PathPlannerTrajectory P2toMarker = AutoChooser.openTrajectoryFile("BLUE_TopMarker_P2-M", new PathConstraints(2.5, 0.75));
        addCommands(
            new ZeroGyroscope(drivetrain, 180).withTimeout(0.1),
            new ResetDrivePose(drivetrain, 1.81, 4.31, 180),
            mManipulator.goToShoot().withTimeout(3).deadlineWith(mManipulator.getGrasper().runTestCurrent()),
            mManipulator.getGrasper().runSpitMode().withTimeout(0.3),
            new DriveAtPath(drivetrain, MarkertoP1, 0, 4.5).deadlineWith(
                mManipulator.goToCubeIntake(),
                mManipulator.getGrasper().runTestMode()
            ),
            new ParallelCommandGroup(new DriveAtPath(drivetrain, P1toMarker, 0, 4.0).deadlineWith(
                mManipulator.getGrasper().runTestCurrent()
            ),
            mManipulator.goToZero().withTimeout(2.0).andThen(mManipulator.goToCubeShootHigh().withTimeout(2.0))
            ),
            mManipulator.getGrasper().runSpitMode().withTimeout(0.5),
            mManipulator.goToZero()


                
            /*new ParallelCommandGroup(
                new DriveAtPath(drivetrain, MarkertoP2, 0, 10)
                // Code for extending intake
                // Code for retracting intake
                // Move arm to retrieve game piece, and open claw
                // Once in position, close claw
                // Make any readjustments necessary for making sure the piece is secure
            ),
            new ParallelCommandGroup(
            new DriveAtPath(drivetrain, P2toMarker, 0, 10)
                // Move arm (if necessary) to position game piece for scoring
                // Open claw
                // Move arm into "default" position
            )*/
        );
    }
}
