package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoAlignAdjust extends CommandBase{
    private DrivetrainSubsystem mDrivetrainSubsystem;
    private double adjust;

    public AutoAlignAdjust(DrivetrainSubsystem dt, double d){
        mDrivetrainSubsystem = dt;
        adjust = d;
    }
    
    @Override
    public void initialize(){
        mDrivetrainSubsystem.alignAdjust(adjust);
    }
}
