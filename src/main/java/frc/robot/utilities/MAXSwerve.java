package frc.robot.utilities;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import frc.robot.Constants.kSwerve.kModule;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class MAXSwerve {
  private SwerveModuleState targetState = new SwerveModuleState();

  private final CANSparkMax driveNEO;
  private final CANSparkMax steerNEO;

  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder steerEncoder;

  private final SparkMaxPIDController drivePID;
  private final SparkMaxPIDController steerPID;

  private final double chassisOffset;

  private final SimpleMotorFeedforward driveFF;

  public MAXSwerve(int driveCANId, int steerCANId, double offset) {
    chassisOffset = offset;

    driveNEO = new CANSparkMax(driveCANId, MotorType.kBrushless);
    steerNEO = new CANSparkMax(steerCANId, MotorType.kBrushless);

    driveEncoder = driveNEO.getEncoder();
    steerEncoder = steerNEO.getAbsoluteEncoder(Type.kDutyCycle);

    driveEncoder.setPositionConversionFactor(kModule.drivingEncoderPositionFactor);
    steerEncoder.setPositionConversionFactor(kModule.steeringEncoderPositionFactor);

    driveEncoder.setVelocityConversionFactor(kModule.drivingEncoderVelocityFactor);
    steerEncoder.setVelocityConversionFactor(kModule.steeringEncoderVelocityFactor);

    steerEncoder.setInverted(kModule.steeringEncoderInverted);
    
    drivePID = driveNEO.getPIDController();
    drivePID.setFeedbackDevice(driveEncoder);
    steerPID = steerNEO.getPIDController();
    steerPID.setFeedbackDevice(steerEncoder);

    driveFF = new SimpleMotorFeedforward(kModule.drivingS, kModule.drivingV, kModule.drivingA);

    drivePID.setOutputRange(kModule.drivingMinOutput, kModule.drivingMaxOutput);
    steerPID.setOutputRange(kModule.steeringMinOutput, kModule.steeringMaxOutput);

    steerPID.setPositionPIDWrappingEnabled(true);
    steerPID.setPositionPIDWrappingMaxInput(kModule.steeringEncoderPositionPIDMaxInput);
    steerPID.setPositionPIDWrappingMinInput(kModule.steeringEncoderPositionPIDMinInput);

    drivePID.setP(kModule.drivingP);
    drivePID.setI(kModule.drivingI);
    drivePID.setD(kModule.drivingD);
    
    steerPID.setP(kModule.drivingP);
    steerPID.setI(kModule.drivingI);
    steerPID.setD(kModule.drivingD);

    driveNEO.setIdleMode(IdleMode.kBrake);
    steerNEO.setIdleMode(IdleMode.kBrake);

    driveNEO.setSmartCurrentLimit(kModule.drivingMotorCurrentLimit);
    steerNEO.setSmartCurrentLimit(kModule.steeringMotorCurrentLimit);

    driveNEO.burnFlash();
    steerNEO.burnFlash();

    targetState.angle = new Rotation2d(steerEncoder.getPosition());
  }

  public Rotation2d getCorrectedSteer() {
    return new Rotation2d(steerEncoder.getPosition() + chassisOffset);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
      driveEncoder.getVelocity(),
       getCorrectedSteer());
  }

  public SwerveModulePosition getPositon() {
    return new SwerveModulePosition(
      driveEncoder.getPosition(),
      getCorrectedSteer());
  }

  public void setTargetState(SwerveModuleState desiredState, boolean closedLoopDrive) {
    SwerveModuleState optimizedState = SwerveModuleState.optimize(
      new SwerveModuleState(
        desiredState.speedMetersPerSecond,
        desiredState.angle.minus(new Rotation2d(chassisOffset))),
      new Rotation2d(steerEncoder.getPosition())
    );

    if (closedLoopDrive){
      drivePID.setReference(
        optimizedState.speedMetersPerSecond,
        ControlType.kVelocity,
        0,
        driveFF.calculate(optimizedState.speedMetersPerSecond));
    } 
    else {
      driveNEO.setVoltage(driveFF.calculate(optimizedState.speedMetersPerSecond));
    }

    steerPID.setReference(optimizedState.angle.getRadians(), ControlType.kPosition);

    targetState = optimizedState;
  }

  public void resetEncoder() {
    driveEncoder.setPosition(0);
  }

  public void setBrakeMode(boolean brake) {
    if (brake) {
      driveNEO.setIdleMode(IdleMode.kBrake);
    }
    else {
      driveNEO.setIdleMode(IdleMode.kCoast);
    }
  }
}
