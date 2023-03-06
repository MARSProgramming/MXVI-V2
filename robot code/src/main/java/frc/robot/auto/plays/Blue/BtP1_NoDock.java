package frc.robot.auto.plays.Blue;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Manipulator;
import frc.robot.commands.Drive.DriveAtPath;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.commands.Drive.ZeroGyroscope;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AutoChooser;

public class BtP1_NoDock extends SequentialCommandGroup{
    public BtP1_NoDock(DrivetrainSubsystem drivetrain, Manipulator mManipulator){
        addRequirements(drivetrain, mManipulator);

        PathPlannerTrajectory MarkertoP1 = AutoChooser.openTrajectoryFile("BLUE_TopMarker_M-P1", new PathConstraints(1.5, 0.5));
        PathPlannerTrajectory P1toMarker = AutoChooser.openTrajectoryFile("BLUE_TopMarker_P1-M", new PathConstraints(3, 0.75));
        addCommands(
            new ZeroGyroscope(drivetrain, 180).withTimeout(0.1),
            new ResetDrivePose(drivetrain, 1.83, 4.34, 180),
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
                mManipulator.goToZero().withTimeout(2.0).andThen(mManipulator.goToCubeShootHigh().withTimeout(2.0))
            ),
            mManipulator.getGrasper().runSpitMode().withTimeout(0.5),
            mManipulator.goToZero().withTimeout(1)
        );
    }
}
