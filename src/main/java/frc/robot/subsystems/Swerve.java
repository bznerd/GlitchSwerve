package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kSwerve;
import frc.robot.utilities.ChassisLimiter;
import frc.robot.utilities.MAXSwerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Swerve extends SubsystemBase {
  private final MAXSwerve frontLeft =
      new MAXSwerve(
          kSwerve.CANID.frontLeftDrive, kSwerve.CANID.frontLeftSteer, kSwerve.Offsets.frontLeft);

  private final MAXSwerve backLeft =
      new MAXSwerve(
          kSwerve.CANID.backLeftDrive, kSwerve.CANID.backLeftSteer, kSwerve.Offsets.backLeft);

  private final MAXSwerve backRight =
      new MAXSwerve(
          kSwerve.CANID.backRightDrive, kSwerve.CANID.backRightSteer, kSwerve.Offsets.backRight);

  private final MAXSwerve frontRight =
      new MAXSwerve(
          kSwerve.CANID.frontRightDrive, kSwerve.CANID.frontRightSteer, kSwerve.Offsets.frontRight);

  private final Gyro gyro = new ADXRS450_Gyro();
  private final SwerveDrivePoseEstimator poseEstimator;
  private final ChassisLimiter limiter;
  private Rotation2d gyroOffset = new Rotation2d();
  private ChassisSpeeds chassisVelocity = new ChassisSpeeds();

  public Swerve() {
    limiter = new ChassisLimiter(kSwerve.maxTransAccel, kSwerve.maxAngAccel);
    poseEstimator =
        new SwerveDrivePoseEstimator(
            kSwerve.kinematics,
            getGyroRaw(),
            new SwerveModulePosition[] {
              frontLeft.getPositon(),
              backLeft.getPositon(),
              backRight.getPositon(),
              frontRight.getPositon()
            },
            new Pose2d());
  }

  // ---------- Commands ----------

  public Command teleopDriveCommand(
      DoubleSupplier xTranslation,
      DoubleSupplier yTranslation,
      DoubleSupplier zRotation,
      BooleanSupplier boost) {
    return this.run(
        () ->
            driveFO(
                joystickToChassis(
                    xTranslation.getAsDouble(),
                    yTranslation.getAsDouble(),
                    zRotation.getAsDouble(),
                    boost.getAsBoolean()),
                kSwerve.OI.closedLoop));
  }

  // ---------- Public interface methods ----------

  // Drive field-oriented (optional flag for closed loop velocity control)
  public void driveFO(ChassisSpeeds fieldRelativeSpeeds, boolean closedLoopDrive) {
    var relativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getGyro());
    drive(relativeSpeeds, closedLoopDrive);
  }

  // Drive chassis-oriented (optional flag for closed loop velocity control)
  public void drive(ChassisSpeeds speeds, boolean closedLoopDrive) {
    speeds = limiter.calculate(speeds);
    var targetStates = kSwerve.kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        targetStates,
        speeds,
        kSwerve.kModule.maxDriveSpeed,
        kSwerve.maxTransSpeed,
        kSwerve.maxAngSpeed);

    chassisVelocity = kSwerve.kinematics.toChassisSpeeds(targetStates);
    setStates(targetStates, closedLoopDrive);
  }

  // Retrieve the pose estimation pose
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  // Set an initial pose for the pose estimator
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(getGyroRaw(), getPositions(), pose);
  }

  // Zero out the gyro (current heading becomes 0)
  public void zeroGyro() {
    gyroOffset = getGyroRaw();
  }

  // Reset gyro using pose estimator
  public void matchGyroToPose() {
    gyroOffset = getGyroRaw().minus(getPose().getRotation());
  }

  // ---------- Private hardware interface methods ----------

  // Get direct gyro reading as Rotation2d
  private Rotation2d getGyroRaw() {
    return new Rotation2d(gyro.getAngle());
  }

  // Get software offset gyro angle
  private Rotation2d getGyro() {
    return new Rotation2d(gyro.getAngle()).minus(gyroOffset);
  }

  // Retrieve the positions (angle and distance traveled) for each swerve module
  private SwerveModulePosition[] getPositions() {
    return new SwerveModulePosition[] {
      frontLeft.getPositon(), backLeft.getPositon(), backRight.getPositon(), frontRight.getPositon()
    };
  }

  // Set desired states (angle and velocity) to each module with an optional flag to enable closed
  // loop control on velocity
  private void setStates(SwerveModuleState[] states, boolean closedLoopDrive) {
    frontLeft.setTargetState(states[0], closedLoopDrive);
    backLeft.setTargetState(states[1], closedLoopDrive);
    backRight.setTargetState(states[2], closedLoopDrive);
    frontRight.setTargetState(states[3], closedLoopDrive);
  }

  // ---------- Periodic ----------

  // Update pose estimator and log data
  @Override
  public void periodic() {
    poseEstimator.update(getGyroRaw(), getPositions());
  }

  // ---------- Helpers ----------
  private ChassisSpeeds joystickToChassis(
      double xTranslation, double yTranslation, double zRotation, boolean boost) {
    // Square inputs for controlabitly
    xTranslation *= xTranslation;
    yTranslation *= yTranslation;
    zRotation *= zRotation;

    // Create a velocity vector (full speed is a unit vector)
    var translationVelocity = VecBuilder.fill(xTranslation, yTranslation);

    // Multiply velocity vector by max speed
    translationVelocity.times(kSwerve.maxTransSpeed);

    // Contrain velocities to boost gain
    if (!boost) {
      translationVelocity.times(kSwerve.OI.translationGain);
      zRotation *= kSwerve.OI.rotationGain;
    }

    // Construct chassis speeds and return
    return new ChassisSpeeds(
        translationVelocity.get(0, 0), translationVelocity.get(1, 0), zRotation);
  }
}
