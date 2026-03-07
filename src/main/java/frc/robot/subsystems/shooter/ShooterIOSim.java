package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;

public class ShooterIOSim implements ShooterIO {
	private double target2InchRps = 0.0;
	private double target3InchRps = 0.0;
	private double neoPercent = 0.0;
	private double rollerPercent = 0.0;

	private double simVel2InchRps = 0.0;
	private double simVel3Inch1Rps = 0.0;
	private double simVel3Inch2Rps = 0.0;

	private static final double SIM_RESPONSE_ALPHA = 0.15;

	private static double rpmToRps(double rpm) {
		return rpm / 60.0;
	}

	@Override
	public void setTargets(double twoInchRpm, double threeInchRpm, double neoPercent, double rollerPercent) {
		target2InchRps = rpmToRps(twoInchRpm);
		target3InchRps = rpmToRps(threeInchRpm);
		this.neoPercent = MathUtil.clamp(neoPercent, -100.0, 100.0);
		this.rollerPercent = MathUtil.clamp(rollerPercent, -100.0, 100.0) * ShooterConstants.FEEDER_ROLLER_DIR;
	}

	@Override
	public void stop() {
		target2InchRps = 0.0;
		target3InchRps = 0.0;
		neoPercent = 0.0;
		rollerPercent = 0.0;
	}

	@Override
	public void updateInputs(ShooterIOInputs inputs) {
		double target2Inch = target2InchRps * ShooterConstants.TALON_2_INCH_DIR;
		double target3Inch1 = target3InchRps * ShooterConstants.TALON_3_INCH_1_DIR;
		double target3Inch2 = target3InchRps * ShooterConstants.TALON_3_INCH_2_DIR;

		simVel2InchRps += SIM_RESPONSE_ALPHA * (target2Inch - simVel2InchRps);
		simVel3Inch1Rps += SIM_RESPONSE_ALPHA * (target3Inch1 - simVel3Inch1Rps);
		simVel3Inch2Rps += SIM_RESPONSE_ALPHA * (target3Inch2 - simVel3Inch2Rps);

		inputs.connected2Inch = true;
		inputs.connected3Inch1 = true;
		inputs.connected3Inch2 = true;
		inputs.connectedNeo550 = true;
		inputs.connectedRoller = true;

		inputs.velocity2InchRps = simVel2InchRps;
		inputs.velocity3Inch1Rps = simVel3Inch1Rps;
		inputs.velocity3Inch2Rps = simVel3Inch2Rps;

		inputs.appliedVolts2Inch = MathUtil.clamp(simVel2InchRps / 100.0 * 12.0, -12.0, 12.0);
		inputs.appliedVolts3Inch1 = MathUtil.clamp(simVel3Inch1Rps / 100.0 * 12.0, -12.0, 12.0);
		inputs.appliedVolts3Inch2 = MathUtil.clamp(simVel3Inch2Rps / 100.0 * 12.0, -12.0, 12.0);

		inputs.currentAmps2Inch = Math.abs(inputs.appliedVolts2Inch) * 3.0;
		inputs.currentAmps3Inch1 = Math.abs(inputs.appliedVolts3Inch1) * 3.0;
		inputs.currentAmps3Inch2 = Math.abs(inputs.appliedVolts3Inch2) * 3.0;

		inputs.neoAppliedPercent = neoPercent;
		inputs.rollerAppliedPercent = rollerPercent;
		inputs.appliedVoltsRoller = (rollerPercent / 100.0) * 12.0;
		inputs.currentAmpsRoller = Math.abs(inputs.appliedVoltsRoller) * 2.0;
		 
	}
}
 
