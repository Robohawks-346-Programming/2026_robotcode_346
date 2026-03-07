package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
	@AutoLog
	public static class ClimbIOInputs {
		public boolean connected = false;

		public double motorPositionRotations = 0.0;
		public double motorVelocityRps = 0.0;
		public double targetPositionRotations = 0.0;

		public double appliedVolts = 0.0;
		public double currentAmps = 0.0;
		public boolean brakeModeConfigured = false;
	}

	public default void updateInputs(ClimbIOInputs inputs) {
	}

	public default void setTargetPositionRotations(double targetPositionRotations) {
	}

	public default void stop() {
	}
}
