package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
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
    return swerve.driveToPoint(new Pose2d(3, 0, new Rotation2d()), Rotation2d.fromDegrees(90));
    // return swerve.driveToPoint(new Pose2d(3, 2, new Rotation2d()), new Rotation2d(1.6));
  }
}
