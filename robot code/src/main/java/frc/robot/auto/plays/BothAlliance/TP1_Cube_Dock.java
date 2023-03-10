package frc.robot.auto.plays.BothAlliance;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Manipulator;
import frc.robot.commands.Drive.AutoBalance;
import frc.robot.commands.Drive.DriveAtPath;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.commands.Drive.ZeroGyroscope;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AutoChooser;

public class TP1_Cube_Dock extends SequentialCommandGroup{
    public TP1_Cube_Dock(DrivetrainSubsystem drivetrain, Manipulator mManipulator){
        addRequirements(drivetrain, mManipulator);

        PathPlannerTrajectory MarkertoP1 = AutoChooser.openTrajectoryFileForAlliance("BLUE_TopMarker_M-P1", new PathConstraints(1.5, 0.5));
        PathPlannerTrajectory P1toMarker = AutoChooser.openTrajectoryFileForAlliance("BLUE_TopMarker_P1-M", new PathConstraints(3, 1));
        PathPlannerTrajectory CSPath = AutoChooser.openTrajectoryFileForAlliance("BLUE_TopMarker_M-CNoLeave", new PathConstraints(1.5, 1));
        addCommands(
            new ZeroGyroscope(drivetrain, 180).withTimeout(0.1),
            new ResetDrivePose(drivetrain, MarkertoP1.getInitialHolonomicPose()).withTimeout(0.1),
            mManipulator.goToShoot().withTimeout(3).deadlineWith(mManipulator.getGrasper().runTestCurrent()),
            mManipulator.getGrasper().runSpitMode().withTimeout(0.3),
            new DriveAtPath(drivetrain, MarkertoP1, false, false, 5.0).deadlineWith(
                mManipulator.goToCubeIntake(),
                mManipulator.getGrasper().runTestMode()
            ),
            new ParallelCommandGroup(
                new DriveAtPath(drivetrain, P1toMarker, false, false, 4.0).deadlineWith(
                    mManipulator.getGrasper().runTestCurrent()
                ),
                mManipulator.goToZero().withTimeout(2.0).andThen(mManipulator.goToCubeShootHigh().withTimeout(1.0))
            ),
            mManipulator.getGrasper().runSpitMode().withTimeout(0.5),
            mManipulator.goToZero().withTimeout(1).alongWith(
            new DriveAtPath(drivetrain, CSPath, false, false, 5.0)),
            new AutoBalance(drivetrain)
        


                
            /*new ParallelCommandGroup(
                new DriveAtPath(drivetrain, MarkertoP2, false, false, 10)
                // Code for extending intake
                // Code for retracting intake
                // Move arm to retrieve game piece, and open claw
                // Once in position, close claw
                // Make any readjustments necessary for making sure the piece is secure
            ),
            new ParallelCommandGroup(
            new DriveAtPath(drivetrain, P2toMarker, false, false, 10)
                // Move arm (if necessary) to position game piece for scoring
                // Open claw
                // Move arm into "default" position
            )*/
        );
    }
}
