package org.littletonrobotics.frc2026.Mock;

import frc.lib.util.TunableNumber;

public class LoggedTunableNumber {
    private TunableNumber tunableNumber;

    public LoggedTunableNumber(String name) {
        this.tunableNumber = new TunableNumber(name, 0, true);
    }

    public LoggedTunableNumber(String name, double defaultValue) {
        this.tunableNumber = new TunableNumber(name, defaultValue, true);
    }

    public void initDefault(double defaultValue) {
        tunableNumber.setDefault(defaultValue);
    }
    public double get() {
        return tunableNumber.get();
    }
    public boolean hasChanged(int hash) {
        return tunableNumber.hasChanged();
    }

    public boolean hasChanged() {
        return tunableNumber.hasChanged();
    }
}
