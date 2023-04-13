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

public class BLUE_TOP_2HIGH extends SequentialCommandGroup{
    public BLUE_TOP_2HIGH(DrivetrainSubsystem drivetrain, Manipulator mManipulator){
        addRequirements(drivetrain);

        PathPlannerTrajectory P1 = AutoChooser.openTrajectoryFileForBlue("BLUE_TOP_G-P1", new PathConstraints(2, 1.5));
        PathPlannerTrajectory ScoreP1 = AutoChooser.openTrajectoryFileForBlue("BLUE_TOP_P1-G", new PathConstraints(2, 1.5));
        addCommands(
            new ZeroGyroscope(drivetrain, 180).withTimeout(0.03),
            new ResetDrivePose(drivetrain, P1.getInitialHolonomicPose()),
            mManipulator.swapAutoScoreCommand().withTimeout(0.03),
            mManipulator.goToScoreHigh().withTimeout(2.5),
            new DriveAtPath(drivetrain, P1, false, false, 5).deadlineWith(
                mManipulator.goToCloseCubeIntake().alongWith(mManipulator.getGrasper().setPercentOutputCommand(1))
            ),
            new DriveAtPath(drivetrain, ScoreP1, false, false, 3).alongWith(
                mManipulator.goToZero().until(() -> drivetrain.getPose().getX() < 3.6).deadlineWith(mManipulator.getGrasper().runTestCurrent()).andThen(mManipulator.goToScoreHigh().withTimeout(2.2))
            ),
            mManipulator.swapAutoScoreCommand().withTimeout(0.03),
            mManipulator.goToZero()
        );
    }
}
