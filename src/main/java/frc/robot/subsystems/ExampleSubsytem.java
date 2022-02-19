package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ExampleSubsytem extends SubsystemBase {
    private final DoubleSolenoid piston;

    public ExampleSubsytem() {
        Compressor comp = new Compressor(Constants.PNEUMATICS_MODULE_ID,
                PneumaticsModuleType.REVPH);
        piston = new DoubleSolenoid(Constants.PNEUMATICS_MODULE_ID,
                PneumaticsModuleType.REVPH,
                Constants.PISTON_FORWARD_CHANNEL,
                Constants.PISTON_REVERSE_CHANNEL);
    }

    public void togglePiston() {
        Value oppositeValue = piston.get() == Value.kForward ? Value.kReverse : Value.kForward;
        piston.set(oppositeValue);
    }
}
