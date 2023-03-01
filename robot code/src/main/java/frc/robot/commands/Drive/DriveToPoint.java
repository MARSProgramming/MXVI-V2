package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveToPoint extends CommandBase{
    private final DrivetrainSubsystem mDrivetrainSubsystem;
    private Trajectory mTrajectory;
    private HolonomicDriveController mController;
    private Rotation2d mEndRotation;
    private Pose2d mTarget;

    public DriveToPoint(DrivetrainSubsystem subsystem, Pose2d target) {
        mDrivetrainSubsystem = subsystem;
        mController = subsystem.getDrivePathController();
        mTarget = target;
        mController.setTolerance(new Pose2d(0.1, 0.1, new Rotation2d(0.1)));
        
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        ChassisSpeeds cs = mController.calculate(mDrivetrainSubsystem.getPose(), mTarget, 0.1, new Rotation2d(0));
        SmartDashboard.putNumber("desiredX", mTarget.getX());
        SmartDashboard.putNumber("desiredY", mTarget.getY());
        SmartDashboard.putNumber("desiredrot", 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }
}
