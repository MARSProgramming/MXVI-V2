package frc.robot.util;
import java.io.File;  
import java.io.FileNotFoundException;  
import java.util.Scanner; 

public class AutoFileParser {
  File file; 
  Scanner reader; 
  
  public AutoFileParser(String autoPlay){ 

    file = new File(autoPlay); 
    try {
        reader = new Scanner(file);
    } catch (FileNotFoundException e) {
        e.printStackTrace();
        System.out.println("File not found"); 
    } 
  }




  


      }
