package frc.robot.subsystems.climb;

import edu.wpi.first.math.MathUtil;

public class ClimbIOSim implements ClimbIO {
	private double targetMotorRotations = 0.0;
	private double simMotorRotations = 0.0;
	private double simVelocityRps = 0.0;

	@Override
	public void setTargetPositionRotations(double targetPositionRotations) {
		targetMotorRotations = targetPositionRotations;
	}

	@Override
	public void stop() {
		simVelocityRps = 0.0;
		targetMotorRotations = simMotorRotations;
	}

	@Override
	public void updateInputs(ClimbIOInputs inputs) {
		double errorRotations = targetMotorRotations - simMotorRotations;
		double desiredVelocityRps =
				MathUtil.clamp(errorRotations * ClimbConstants.SIM_RESPONSE_ALPHA * 50.0,
						-ClimbConstants.SIM_MAX_RPS,
						ClimbConstants.SIM_MAX_RPS);

		simVelocityRps = desiredVelocityRps;
		simMotorRotations += simVelocityRps * 0.02;

		if (Math.abs(errorRotations) <= ClimbConstants.POSITION_TOLERANCE_ROTATIONS) {
			simMotorRotations = targetMotorRotations;
			simVelocityRps = 0.0;
		}

		double appliedVolts = MathUtil.clamp((simVelocityRps / ClimbConstants.SIM_MAX_RPS) * 12.0, -12.0, 12.0);

		inputs.connected = true;
		inputs.motorPositionRotations = simMotorRotations;
		inputs.motorVelocityRps = simVelocityRps;
		inputs.targetPositionRotations = targetMotorRotations;
		inputs.appliedVolts = appliedVolts;
		inputs.currentAmps = Math.abs(appliedVolts) * 2.0;
		inputs.brakeModeConfigured = true;
	}
}
