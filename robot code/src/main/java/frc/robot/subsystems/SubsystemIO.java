package frc.robot.subsystems;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map.Entry;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.Pivot;
import frc.robot.Constants.Wrist;

public class SubsystemIO extends SubsystemBase{
    private HashMap<Field, SimpleWidget> entries;
    public SubsystemIO(){
        ShuffleboardTab subsystemIOTab;// = Shuffleboard.getTab("SubsystemIO");
        entries = new HashMap<>();

        Class<?>[] minisystems = {Pivot.class, Elevator.class, Wrist.class};
        
        for(Class<?> minisystem : minisystems){
            Field[] fields = minisystem.getDeclaredFields();
            subsystemIOTab = Shuffleboard.getTab(minisystem.getSimpleName());
            for (Field field : fields) {
                if (java.lang.reflect.Modifier.isStatic(field.getModifiers()) && field.getType() == double.class) {
                    field.setAccessible(true);
                    double value = 0;
                    try{
                        value = field.getDouble(null);
                    }
                    catch(IllegalAccessException a){
                        System.out.println("Access Exception when reading subsystem IO");
                    }
                    if(field.getName() != "kP" && field.getName() != "kD" && field.getName() != "kI" && field.getName() != "kF"){
                        entries.put(field, subsystemIOTab.addPersistent(minisystem.getSimpleName() + ": " + field.getName(), "double", value).withSize(2, 1));
                    }
                }
            }
        }
    }

    @Override
    public void periodic(){
        for(Entry<Field, SimpleWidget> entry : entries.entrySet()){
            try{
                entry.getKey().setDouble(null, entry.getValue().getEntry().getDouble(0));
            }
            catch(IllegalAccessException a){
                System.out.println("Access Exception when writing subsystem IO");
            }
        }
    }
}
