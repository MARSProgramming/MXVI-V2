package frc.robot.util;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class UtilityFunctions {
    //detecting stalling
    private static double[] lastPositions;
    public static boolean isStalling(double encoderPosition, double threshold) {
        
        int index = 0;
        lastPositions[index] = encoderPosition;
        index = index + 1;
        index = index % 10;

        if (lastPositions.length > 0) {
            double pos_sum = 0.0;
            for (int i = 0; i < lastPositions.length; i++) {
                pos_sum += lastPositions[i];
            }
           
            double posAvg = pos_sum / lastPositions.length;
            return (Math.abs(encoderPosition - posAvg) > threshold);
    
        } else {
            return false;
        }
    } 

    //score while moving
    public static boolean releaseForSWM(Pose2d current, Pose2d target, double linearVelocity, double height){
        double time = current.getTranslation().getDistance(target.getTranslation())/linearVelocity;
        
        double timeFromRobotToScoring = Math.sqrt(height * 2 / 386.2205);
        return Math.abs(time - timeFromRobotToScoring) < 0.3;
    }
 }

