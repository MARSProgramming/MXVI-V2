package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

public class LED {
  private static final int LED_COUNT = 60;
  private static final int LED_PORT = 0; // replace with the actual port number

  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;
  private boolean running = false;

  public LED() {
    led = new AddressableLED(LED_PORT);
    ledBuffer = new AddressableLEDBuffer(LED_COUNT);
    led.setLength(LED_COUNT);
    led.setData(ledBuffer);
  }

  public void start() {
    running = true;
    Thread t = new Thread(() -> {
      while (running) {
        flamePattern();
      }
      reset();
    });
    t.start();
  }

  public void stop() {
    running = false;
  }

  private void reset() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 0, 0, 0);
    }
    led.setData(ledBuffer);
  }

  private void flamePattern() {
    int[] colors = new int[LED_COUNT];
    for (int i = 0; i < LED_COUNT; i++) {
      double rand = Math.random() * 0.2 + 0.8; // random brightness between 0.8 and 1.0
      colors[i] = (int) (rand * 255);
    }
    for (int i = 0; i < LED_COUNT - 1; i++) {
      colors[i] = (colors[i] + colors[i + 1]) / 2;
    }
    colors[LED_COUNT - 1] = colors[LED_COUNT - 2]; // last LED has same color as second-to-last
    for (int i = 0; i < LED_COUNT; i++) {
      int color = colors[i];
      ledBuffer.setRGB(i, color, color / 3, 0); // red, orange, and yellow
    }
    led.setData(ledBuffer);
    Timer.delay(0.05); // adjust delay time to change flame speed
  }
}

