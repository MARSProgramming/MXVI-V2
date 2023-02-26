package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UtilityFunctions extends SubsystemBase{
    
    public UtilityFunctions() {

    }
    
    private static double[] lastPositions = new double[10];
    private static int index = 0;
    public static boolean isStalling(double encoderPosition, double threshold) {
        
        lastPositions[index] = encoderPosition;
        index = index + 1;
        index = index % 10;

        if (lastPositions.length > 0) {
            double pos_sum = 0.0;
            for (int i = 0; i < lastPositions.length; i++) {
                pos_sum += lastPositions[i];
            }
           
            double posAvg = pos_sum / lastPositions.length;
            return (Math.abs(encoderPosition - posAvg) < threshold);
    
        } else {
            return false;
        }
        

    } 
    @Override
    public void periodic(){
        SmartDashboard.putNumberArray("lastpositions", lastPositions);
    }
 }

