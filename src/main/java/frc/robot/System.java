package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class System {
  public Swerve swerve = new Swerve();

  public System() {
    // Call startup here
  }

  public void configureTeleopDrive(
      DoubleSupplier xTranslation,
      DoubleSupplier yTranslation,
      DoubleSupplier zRotation,
      BooleanSupplier boost) {
    swerve.setDefaultCommand(
        swerve.teleopDriveCommand(xTranslation, yTranslation, zRotation, boost));
  }

  public Command getAuto() {
    return new WaitCommand(15);
  }
}
