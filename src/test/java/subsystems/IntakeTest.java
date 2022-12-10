package subsystems;

import static org.junit.Assert.assertEquals;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import frc.robot.Constants.Ctake;
import frc.robot.subsystems.Intake;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

public class IntakeTest {

  // Nothing SOlenoid related will work unless its with the 2023 VSC version with the simulated
  // solenoids
  public static final double Delta = 0.01;

  Intake intake;
  DoubleSolenoid piston;

  @Before
  public void setup() {
    intake = new Intake();
    piston = new DoubleSolenoid(Ctake.MODULE_NUMBER, PneumaticsModuleType.REVPH, 1, 2);
    piston.set(Value.kReverse);
  }

  @After
  public void shutdown() throws Exception {
    intake.close();
  }

  @Test
  public void intakeTest() {
    intake.setIntakeMotor(0.5);
    assertEquals(0.5, intake.getIntakeMotor(), Delta);
  }
}
