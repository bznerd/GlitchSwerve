package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kSwerve;
import frc.robot.utilities.ChassisLimiter;
import frc.robot.utilities.MAXSwerve;
import edu.wpi.first.math.geometry.Rotation2d;

public class Swerve extends SubsystemBase {
  private final MAXSwerve frontLeft = new MAXSwerve(
    kSwerve.CANID.frontLeftDrive,
    kSwerve.CANID.frontLeftSteer,
    kSwerve.Offsets.frontLeft);

  private final MAXSwerve backLeft = new MAXSwerve(
    kSwerve.CANID.backLeftDrive,
    kSwerve.CANID.backLeftSteer,
    kSwerve.Offsets.backLeft);
  
  private final MAXSwerve backRight = new MAXSwerve(
    kSwerve.CANID.backRightDrive,
    kSwerve.CANID.backRightSteer,
    kSwerve.Offsets.backRight);

  private final MAXSwerve frontRight = new MAXSwerve(
    kSwerve.CANID.frontRightDrive,
    kSwerve.CANID.frontRightSteer,
    kSwerve.Offsets.frontRight);

  private final Gyro gyro = new ADXRS450_Gyro();
  private final SwerveDrivePoseEstimator poseEstimator;
  private final ChassisLimiter limiter;
  private Rotation2d gyroOffset = new Rotation2d();
  private ChassisSpeeds chassisVelocity = new ChassisSpeeds();

  public Swerve() {
    limiter = new ChassisLimiter(kSwerve.maxTransAccel, kSwerve.maxAngAccel);
    poseEstimator = new SwerveDrivePoseEstimator(
      kSwerve.kinematics, getGyroRaw(),
      new SwerveModulePosition[] {
        frontLeft.getPositon(),
        backLeft.getPositon(),
        backRight.getPositon(),
        frontRight.getPositon()},
      new Pose2d());
  }

  public void driveFO(ChassisSpeeds fieldRelativeSpeeds, boolean closedLoopDrive) {
    var relativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getGyro());
    drive(relativeSpeeds, closedLoopDrive);
  }

  public void drive(ChassisSpeeds speeds, boolean closedLoopDrive) {
    speeds = limiter.calculate(speeds);
    var targetStates = kSwerve.kinematics.toSwerveModuleStates(speeds);
    kSwerve.kinematics.desaturateWheelSpeeds(
      targetStates,
      speeds,
      kSwerve.kModule.maxDriveSpeed,
      kSwerve.maxTransSpeed,
      kSwerve.maxAngSpeed);

    chassisVelocity = kSwerve.kinematics.toChassisSpeeds(targetStates);
    setStates(targetStates, closedLoopDrive);
  }

  private Rotation2d getGyroRaw() {
    return new Rotation2d(gyro.getAngle());
  }

  private Rotation2d getGyro() {
    return new Rotation2d(gyro.getAngle()).minus(gyroOffset);
  }

  private SwerveModulePosition[] getPositions() {
    return new SwerveModulePosition[] {
      frontLeft.getPositon(),
      backLeft.getPositon(),
      backRight.getPositon(),
      frontRight.getPositon()
    };
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(getGyroRaw(), getPositions(), pose);
  }

  public void zeroGyro() {
    gyroOffset = getGyroRaw();
  }

  public void matchGyroToPose() {
    gyroOffset = getGyroRaw().minus(getPose().getRotation());
  }

  private void setStates(SwerveModuleState[] states, boolean closedLoopDrive) {
    frontLeft.setTargetState(states[0], closedLoopDrive);
    backLeft.setTargetState(states[1], closedLoopDrive);
    backRight.setTargetState(states[2], closedLoopDrive);
    frontRight.setTargetState(states[3], closedLoopDrive);
  }

  @Override
  public void periodic() {
    poseEstimator.update(getGyro(), getPositions());
  }
}
