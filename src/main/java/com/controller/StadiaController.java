package com.controller;

import edu.wpi.first.wpilibj.XboxController;

public class StadiaController extends XboxController {
	public StadiaController(int port) {
		super(port);
	}

	@Override
	public double getLeftX() {
		return super.getRawAxis(0);
	}

	@Override
	public double getRightX() {
		return super.getRawAxis(3);
	}

	@Override
	public double getLeftY() {
		return super.getRawAxis(1);
	}

	@Override
	public double getRightY() {

		return super.getRawAxis(4);
	}


}
