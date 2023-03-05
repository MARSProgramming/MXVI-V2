package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ZeroGyroscope extends CommandBase{
    private DrivetrainSubsystem mDT;
    private double rotation; 

    public ZeroGyroscope(DrivetrainSubsystem dt, double angle){ 
        mDT = dt; 
        rotation = angle; 
        addRequirements(dt); 
    }

    @Override
    public void initialize(){
        mDT.resetYaw(rotation); 
    }
}
