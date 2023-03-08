package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ResetDrivePose extends CommandBase{
    private final DrivetrainSubsystem mDrivetrainSubsystem;
    private double mX;
    private double mY;
    private Rotation2d mRotation;
    private Pose2d pose;
    public ResetDrivePose(DrivetrainSubsystem driveSubsystem, double x, double y, double rotation){
        mDrivetrainSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
        mRotation = new Rotation2d(rotation);
        pose = new Pose2d(mX, mY, mRotation);
    }

    public ResetDrivePose(DrivetrainSubsystem driveSubsystem, Pose2d pose){
        mDrivetrainSubsystem = driveSubsystem;
        this.pose = pose;
        mRotation = pose.getRotation();
        addRequirements(driveSubsystem);
    }
    @Override
    public void initialize(){
        mDrivetrainSubsystem.setPose(pose, mRotation);
    }
    @Override
    public boolean isFinished(){
        return mDrivetrainSubsystem.getPose().getTranslation().getDistance(new Translation2d(mX, mY)) < 0.01;
    }
}
