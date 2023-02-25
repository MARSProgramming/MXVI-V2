package frc.robot.util.Autoscoring;

import com.ctre.phoenix.Util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoScore extends Util{
     Location[][] Grid = new Location[7][3];
     int j = 0;
     int q = 0;
   
    public AutoScore(){
        for(int r = 0; r < Grid.length; r++){
            for(int h = 0; h < Grid[r].length; h++){
                Grid[r][h].setRow(r);
                Grid[r][h].setHeight(h);
                SmartDashboard.putBoolean("", Grid[r][h].getSelected());
            }
        }
    }

    public String decidename(int a, int b){
        String s = " ";
        String r = " ";
    if(a % 2 == 0 || a % 5 == 0 || a % 8 == 0){
        s = " Square";
    }
    else 
    {
        s = " Cube";
    }
    if(b == 1){
        r = " Low";
    }

    if(b == 2){
        r = " Middle";
    }

    if(b == 3){
        r = " High";
    }

    return s + r;
    }

    public void LocationChangeUp(){
        
        for(int r = 0; r < Grid.length; r++){
            for(int h = 0; h < Grid[r].length; h++){
                if(Grid[r][h].equals(Grid[j][q])){
                    Grid[r][h].setSelected(true);
                }
                else
                {
                    Grid[r][h].setSelected(false);
                }
            }
        }
    }
    public void Score(){

    }



}
