package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

enum LEDState {
  RAINBOW,
  RED,
  PURPLE,
  FLASHING_PURPLE,
  YELLOW,
  FLASHING_YELLOW
  }  

public class LED extends SubsystemBase {
  LEDState state = LEDState.RAINBOW;
  AddressableLED m_led = new AddressableLED(0);
  AddressableLEDBuffer m_ledBuffer;

  public LED() {
    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(60);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
    //leds.start();

  }
//TODO this methad is not called by any class
  public CommandBase setRainbow(){
    return runOnce(
      () -> {
        state = LEDState.RAINBOW;
      }
    );
  }

  public CommandBase setRed(){
    return runOnce(
      () -> {
        state = LEDState.RED;
      }
    );
  }

  
  public CommandBase setYellow(){
    return runOnce(
      () -> {
        state = LEDState.YELLOW;
      }
    );
  }

  public CommandBase setYellowFlashing(){
    return runOnce(
      () -> {
        state = LEDState.FLASHING_YELLOW;
      }
    );
  }

  public CommandBase setPurple(){
    return runOnce(
      () -> {
        state = LEDState.PURPLE;
      }
    );
  }

  public CommandBase setPurpleFlashing(){
    return runOnce(
      () -> {
        state = LEDState.FLASHING_PURPLE;
      }
    );
  }

  public CommandBase swapYellowPurple(){
    return runOnce(
      () -> {
        if(state == LEDState.PURPLE){
          state = LEDState.YELLOW;
        }
        else if(state == LEDState.YELLOW){
          state = LEDState.PURPLE;
        }
        else{
          state = LEDState.YELLOW;
        }
      }
    );
  }

  public void startFlashing(){
        if(state == LEDState.PURPLE){
          state = LEDState.FLASHING_PURPLE;
        }
        else if(state == LEDState.YELLOW){
          state = LEDState.FLASHING_YELLOW;
        }
        else{
          state = LEDState.RAINBOW;
        }
      }
    
      public void stopFlashing(){
        if(state == LEDState.FLASHING_PURPLE){
          state = LEDState.PURPLE;
        }
        else if(state == LEDState.FLASHING_YELLOW){
          state = LEDState.YELLOW;
        }
        else{
          state = LEDState.RAINBOW;
        }
      }

  @Override
  public void periodic() {
        /*for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 0, 255, 0);
   }
   
   m_led.setData(m_ledBuffer);*/
   if (state == LEDState.RAINBOW) {
    rainbow();
   } 
   else if (state == LEDState.RED) {
    red();
   }
   else if (state == LEDState.PURPLE) {
    purple();
   }
   else if (state == LEDState.FLASHING_PURPLE) {
    purple_flash();
  }
   else if (state == LEDState.YELLOW) {
    yellow();
   }
   else if (state == LEDState.FLASHING_YELLOW){
    yellow_flash();
    }
   m_led.setData(m_ledBuffer);
  }

  double m_rainbowFirstPixelHue = 0;

  private void rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, (int) hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }

  private void red() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 34, 178, 23);
   }
   
   m_led.setData(m_ledBuffer);
    }

    private void purple() {
      // For every pixel
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        m_ledBuffer.setRGB(i, 0, 128, 128);
      }
      m_led.setData(m_ledBuffer);
    }
    
    private int iterations = 0;

    private void purple_flash() {

      // For every pixel
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        if ((iterations / 25) % 2 == 0) {
          m_ledBuffer.setRGB(i, 0, 128, 128);
        } else {
          m_ledBuffer.setRGB(i, 0,0,0);
        }

     }
     iterations++;
    }

    private void yellow_flash() {

      // For every pixel
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        if ((iterations / 25) % 2 == 0) {
          m_ledBuffer.setRGB(i, 255, 255, 34);
        } else {
          m_ledBuffer.setRGB(i, 0,0,0);
        }

     }
     iterations++;
    }
      private void yellow() {
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          // Sets the specified LED to the RGB values for red
          m_ledBuffer.setRGB(i, 255, 255, 34);
       }
       
       m_led.setData(m_ledBuffer);
        }
    
  }

