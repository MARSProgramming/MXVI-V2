package frc.robot.commands.Drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoBalance extends CommandBase{
    private DrivetrainSubsystem mDrivetrainSubsystem;
    public AutoBalance(DrivetrainSubsystem dt){
        mDrivetrainSubsystem = dt;
    }

    @Override
    public void execute(){
        if(mDrivetrainSubsystem.getRoll() < 85 && mDrivetrainSubsystem.getRoll() > 60){
            mDrivetrainSubsystem.drive(new ChassisSpeeds(-0.03 * (88 - mDrivetrainSubsystem.getRoll()), 0, 0));
        }
        else if(mDrivetrainSubsystem.getRoll() < 0){
            mDrivetrainSubsystem.drive(new ChassisSpeeds(0.03 * (88 + mDrivetrainSubsystem.getRoll()), 0, 0));
        }
        else{
            mDrivetrainSubsystem.drive(new ChassisSpeeds());
        }
    }
}
