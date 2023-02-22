package frc.robot.util;

public class UtilityFunctions {
    
    public UtilityFunctions() {

    }
    
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
 }

