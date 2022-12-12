package subsystems;

import static org.junit.Assert.assertEquals;

import frc.robot.Vision;
import org.junit.Before;
import org.junit.Test;

public class LimelightTest {

  public static final double DELTA = 0.01;

  Vision vision;

  @Before
  public void setup() {
    vision = Vision.getInstance();
  }

  @Test
  public void hasTargetTest() {
    vision.hasValidTarget();
    assertEquals(false, vision.hasValidTarget());
  }

  // When using a fake limelight, all readable double values return 2.147483647E9 (in other words,
  // invalid)
  @Test
  public void getXOffSetTest() {
    vision.getHorizontalOffset();
    assertEquals(2.147483647E9, vision.getHorizontalOffset(), DELTA);
  }
}
