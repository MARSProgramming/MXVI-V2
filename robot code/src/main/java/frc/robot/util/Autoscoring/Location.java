package frc.robot.util.Autoscoring;

import com.ctre.phoenix.Util;

public class Location extends Util{
    private int Row = 0;
    private int Height = 0;
    private boolean selected = false;
    public Location(int r, int H){
        Row = r;
        Height = H;
    }

    public void setRow(int a){
        Row = a;
    }

    public void setSelected(boolean a){
        selected = a;
    }

    public boolean getSelected(){
        return selected;
    }
    
    public void setHeight(int a){
        Height = a;
    }

    public int getRow(){
        return Row;
    }

    public int getHeight(){
        return Height;
    }

    

}
