package frc.robot.auto.plays.BothAlliance;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.DriveAtPath;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.commands.Drive.ZeroGyroscope;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Manipulator;
import frc.robot.util.AutoChooser;

public class MidScoreGiven extends SequentialCommandGroup{
    public MidScoreGiven(DrivetrainSubsystem drivetrain, Manipulator mManipulator){
        addRequirements(drivetrain);

        PathPlannerTrajectory LeaveCommunity = AutoChooser.openTrajectoryFileForAlliance("BLUE_MiddleLeaveCommunity", new PathConstraints(1, 0.5));
        addCommands(
            new ZeroGyroscope(drivetrain, 180).withTimeout(0.1),
            new ResetDrivePose(drivetrain, LeaveCommunity.getInitialHolonomicPose()),
            mManipulator.goToCubeShootHigh().withTimeout(3.0),
            mManipulator.getGrasper().runSpitMode().withTimeout(1),
            mManipulator.goToZero()
        );
    }
}
