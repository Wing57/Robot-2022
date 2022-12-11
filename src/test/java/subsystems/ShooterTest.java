package subsystems;

import static org.junit.Assert.assertEquals;

import frc.robot.subsystems.Shooter;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

public class ShooterTest {

  public static final double DELTA = 0.01;
  Shooter shooter;

  @Before
  public void setup() {
    shooter = new Shooter();
  }

  @After
  public void close() throws Exception {
    shooter.shutdown();
  }

  // Should be zero cuz fake, important part is that pid method setShooterRPM outputs a valie
  @Test
  public void setRPMTest() {
    shooter.setShooterRPM(100);
    assertEquals(0, shooter.getShooterRPM(), DELTA);
  }
}
