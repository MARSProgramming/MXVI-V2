
package frc.robot.commands.Drive;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveAtPath extends CommandBase {
    private final DrivetrainSubsystem mDrivetrainSubsystem;
    private PathPlannerTrajectory mTrajectory;
    private HolonomicDriveController mController;
    private Timer mTimer;
    private Rotation2d mEndRotation;
    private double timeout;

    public DriveAtPath(DrivetrainSubsystem subsystem, PathPlannerTrajectory traj, double rotation, double timeout) {
        mTrajectory = traj;
        mDrivetrainSubsystem = subsystem;
        mController = subsystem.getDrivePathController();
        mTimer = new Timer();
        mEndRotation = new Rotation2d(Math.toRadians(rotation));
        this.timeout = timeout;
        mController.setTolerance(new Pose2d(0.3, 0.3, new Rotation2d(0.3)));
        
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mTimer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        var state = (PathPlannerState) mTrajectory.sample(mTimer.get());
        mDrivetrainSubsystem.drive(mController.calculate(mDrivetrainSubsystem.getPose(), mTrajectory.sample(mTimer.get()), state.holonomicRotation));
        SmartDashboard.putNumber("desiredX", mTrajectory.sample(mTimer.get()).poseMeters.getX());
        SmartDashboard.putNumber("desiredY", mTrajectory.sample(mTimer.get()).poseMeters.getY());
        SmartDashboard.putNumber("desiredrot", state.holonomicRotation.getDegrees());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mTimer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return mController.atReference() || mTimer.get() > timeout;
    }
}
