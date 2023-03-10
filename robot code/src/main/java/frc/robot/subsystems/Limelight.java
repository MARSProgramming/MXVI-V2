package frc.robot.subsystems;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase{
    double[] botpose;
    private DrivetrainSubsystem dt;
    public Limelight(DrivetrainSubsystem drive){
        dt = drive;
    }

    public void resetPose(){
        if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1.0 && (dt.getPose().getX() < 3.6 || dt.getPose().getX() > 13)){
            if(dt.getPose().getX() > 13){
                dt.setVisionMeasurementStdDevs(new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.03, 0.1, 0.07));
            }
            if(dt.getPose().getX() < 3){
                dt.setVisionMeasurementStdDevs(new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.01, 0.01, 0.07));
            }
            String key = DriverStation.getAlliance() == Alliance.Blue ? "botpose_wpiblue" : "botpose_wpired";
            botpose = NetworkTableInstance.getDefault().getTable("limelight").getEntry(key).getDoubleArray(new double[6]);
            double x = botpose[0];
            double y = botpose[1];
            double z = botpose[2];
            dt.addVisionMeasurement(new Pose2d(x, y, dt.getGyroscopeRotation()));
        }
    }
}
