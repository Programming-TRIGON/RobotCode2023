package frc.trigon.robot.utilities;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.trigon.robot.Robot;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class KeyboardController extends CommandGenericHID implements Loggable {
    /**
     * Construct an instance of a device.
     *
     * @param port The port index on the Driver Station that the device is plugged into.
     */
    public KeyboardController(int port) {
        super(port);
    }

    @Log(methodName="getAsBoolean")
    public Trigger esc() {
        return button(1);
    }

    @Log(methodName="getAsBoolean")
    public Trigger f1() {
        return button(2);
    }

    @Log(methodName="getAsBoolean")
    public Trigger f2() {
        return button(3);
    }

    @Log(methodName="getAsBoolean")
    public Trigger f3() {
        return button(4);
    }

    @Log(methodName="getAsBoolean")
    public Trigger f4() {
        return button(5);
    }

    @Log(methodName="getAsBoolean")
    public Trigger f5() {
        return button(6);
    }

    @Log(methodName="getAsBoolean")
    public Trigger f6() {
        return button(7);
    }

    @Log(methodName="getAsBoolean")
    public Trigger f7() {
        return button(8);
    }

    @Log(methodName="getAsBoolean")
    public Trigger f8() {
        return button(9);
    }

    @Log(methodName="getAsBoolean")
    public Trigger f9() {
        return button(10);
    }

    @Log(methodName="getAsBoolean")
    public Trigger f10() {
        return button(11);
    }

    @Log(methodName="getAsBoolean")
    public Trigger f11() {
        return button(12);
    }

    @Log(methodName="getAsBoolean")
    public Trigger f12() {
        return button(13);
    }

    @Log(methodName="getAsBoolean")
    public Trigger del() {
        return button(14);
    }

    @Log(methodName="getAsBoolean")
    public Trigger backtick() {
        return button(15);
    }

    @Log(methodName="getAsBoolean")
    public Trigger one() {
        return button(16);
    }

    @Log(methodName="getAsBoolean")
    public Trigger two() {
        return button(17);
    }

    @Log(methodName="getAsBoolean")
    public Trigger three() {
        return button(18);
    }

    @Log(methodName="getAsBoolean")
    public Trigger four() {
        return button(19);
    }

    @Log(methodName="getAsBoolean")
    public Trigger five() {
        return button(20);
    }

    @Log(methodName="getAsBoolean")
    public Trigger six() {
        return button(21);
    }

    @Log(methodName="getAsBoolean")
    public Trigger seven() {
        return button(22);
    }

    @Log(methodName="getAsBoolean")
    public Trigger eight() {
        return button(23);
    }

    @Log(methodName="getAsBoolean")
    public Trigger nine() {
        return button(24);
    }

    @Log(methodName="getAsBoolean")
    public Trigger zero() {
        return button(25);
    }

    @Log(methodName="getAsBoolean")
    public Trigger minus() {
        return button(26);
    }

    @Log(methodName="getAsBoolean")
    public Trigger equals() {
        return button(27);
    }

    @Log(methodName="getAsBoolean")
    public Trigger backspace() {
        return button(28);
    }

    @Log(methodName="getAsBoolean")
    public Trigger tab() {
        return button(29);
    }

    @Log(methodName="getAsBoolean")
    public Trigger q() {
        return button(30);
    }

    @Log(methodName="getAsBoolean")
    public Trigger w() {
        return button(31);
    }

    @Log(methodName="getAsBoolean")
    public Trigger e() {
        return button(32);
    }

    @Log(methodName="getAsBoolean")
    public Trigger r() {
        return getButtonFromBitOfAxis(0, 0);
    }

    @Log(methodName="getAsBoolean")
    public Trigger t() {
        return getButtonFromBitOfAxis(1, 0);
    }

    @Log(methodName="getAsBoolean")
    public Trigger y() {
        return getButtonFromBitOfAxis(2, 0);
    }

    @Log(methodName="getAsBoolean")
    public Trigger u() {
        return getButtonFromBitOfAxis(3, 0);
    }

    @Log(methodName="getAsBoolean")
    public Trigger i() {
        return getButtonFromBitOfAxis(4, 0);
    }

    @Log(methodName="getAsBoolean")
    public Trigger o() {
        return getButtonFromBitOfAxis(5, 0);
    }
    
    @Log(methodName="getAsBoolean")
    public Trigger p() {
        return getButtonFromBitOfAxis(6, 0);
    }

    @Log(methodName="getAsBoolean")
    public Trigger a() {
        return getButtonFromBitOfAxis(7, 0);
    }

    @Log(methodName="getAsBoolean")
    public Trigger s() {
        return getButtonFromBitOfAxis(0, 1);
    }

    @Log(methodName="getAsBoolean")
    public Trigger d() {
        return getButtonFromBitOfAxis(1, 1);
    }

    @Log(methodName="getAsBoolean")
    public Trigger f() {
        return getButtonFromBitOfAxis(2, 1);
    }

    @Log(methodName="getAsBoolean")
    public Trigger g() {
        return getButtonFromBitOfAxis(3, 1);
    }

    @Log(methodName="getAsBoolean")
    public Trigger h() {
        return getButtonFromBitOfAxis(4, 1);
    }

    @Log(methodName="getAsBoolean")
    public Trigger j() {
        return getButtonFromBitOfAxis(5, 1);
    }

    @Log(methodName="getAsBoolean")
    public Trigger k() {
        return getButtonFromBitOfAxis(6, 1);
    }

    @Log(methodName="getAsBoolean")
    public Trigger l() {
        return getButtonFromBitOfAxis(7, 1);
    }

    @Log(methodName="getAsBoolean")
    public Trigger semicolon() {
        return getButtonFromBitOfAxis(0, 2);
    }

    @Log(methodName="getAsBoolean")
    public Trigger apostrophe() {
        return getButtonFromBitOfAxis(1, 2);
    }

    @Log(methodName="getAsBoolean")
    public Trigger leftShift() {
        return getButtonFromBitOfAxis(2, 2);
    }

    @Log(methodName="getAsBoolean")
    public Trigger z() {
        return getButtonFromBitOfAxis(3, 2);
    }

    @Log(methodName="getAsBoolean")
    public Trigger x() {
        return getButtonFromBitOfAxis(4, 2);
    }

    @Log(methodName="getAsBoolean")
    public Trigger c() {
        return getButtonFromBitOfAxis(5, 2);
    }

    @Log(methodName="getAsBoolean")
    public Trigger v() {
        return getButtonFromBitOfAxis(6, 2);
    }

    @Log(methodName="getAsBoolean")
    public Trigger b() {
        return getButtonFromBitOfAxis(7, 2);
    }

    @Log(methodName="getAsBoolean")
    public Trigger n() {
        return getButtonFromBitOfAxis(0, 3);
    }

    @Log(methodName="getAsBoolean")
    public Trigger m() {
        return getButtonFromBitOfAxis(1, 3);
    }

    @Log(methodName="getAsBoolean")
    public Trigger comma() {
        return getButtonFromBitOfAxis(2, 3);
    }

    @Log(methodName="getAsBoolean")
    public Trigger period() {
        return getButtonFromBitOfAxis(3, 3);
    }

    @Log(methodName="getAsBoolean")
    public Trigger forwardSlash() {
        return getButtonFromBitOfAxis(4, 3);
    }

    @Log(methodName="getAsBoolean")
    public Trigger rightShift() {
        return getButtonFromBitOfAxis(5, 3);
    }

    @Log(methodName="getAsBoolean")
    public Trigger leftCtrl() {
        return getButtonFromBitOfAxis(6, 3);
    }

    @Log(methodName="getAsBoolean")
    public Trigger leftAlt() {
        return getButtonFromBitOfAxis(7, 3);
    }

    @Log(methodName="getAsBoolean")
    public Trigger rightAlt() {
        return getButtonFromBitOfAxis(0, 4);
    }

    @Log(methodName="getAsBoolean")
    public Trigger rightCtrl() {
        return getButtonFromBitOfAxis(1, 4);
    }

    @Log(methodName="getAsBoolean")
    public Trigger left() {
        return getButtonFromBitOfAxis(2, 4);
    }

    @Log(methodName="getAsBoolean")
    public Trigger right() {
        return getButtonFromBitOfAxis(3, 4);
    }

    @Log(methodName="getAsBoolean")
    public Trigger up() {
        return getButtonFromBitOfAxis(4, 4);
    }

    @Log(methodName="getAsBoolean")
    public Trigger down() {
        return getButtonFromBitOfAxis(5, 4);
    }

    @Log(methodName="getAsBoolean")
    public Trigger numpad0() {
        return getButtonFromBitOfAxis(6, 4);
    }

    @Log(methodName="getAsBoolean")
    public Trigger numpad1() {
        return getButtonFromBitOfAxis(7, 4);
    }

    @Log(methodName="getAsBoolean")
    public Trigger numpad2() {
        return getButtonFromBitOfAxis(0, 5);
    }

    @Log(methodName="getAsBoolean")
    public Trigger numpad3() {
        return getButtonFromBitOfAxis(1, 5);
    }

    @Log(methodName="getAsBoolean")
    public Trigger numpad4() {
        return getButtonFromBitOfAxis(2, 5);
    }

    @Log(methodName="getAsBoolean")
    public Trigger numpad5() {
        return getButtonFromBitOfAxis(3, 5);
    }

    @Log(methodName="getAsBoolean")
    public Trigger numpad6() {
        return getButtonFromBitOfAxis(4, 5);
    }

    @Log(methodName="getAsBoolean")
    public Trigger numpad7() {
        return getButtonFromBitOfAxis(5, 5);
    }

    @Log(methodName="getAsBoolean")
    public Trigger numpad8() {
        return getButtonFromBitOfAxis(6, 5);
    }

    @Log(methodName="getAsBoolean")
    public Trigger numpad9() {
        return getButtonFromBitOfAxis(7, 5);
    }

    private Trigger getButtonFromBitOfAxis(int bit, int axis) {
        return new Trigger(() -> getBitsFromAxis(axis)[bit]);
    }

    private boolean[] getBitsFromAxis(int axis) {
        boolean[] bits = new boolean[8];
        double rawValue = (getRawAxis(axis) + 1) / 2 * 256;
        int value = (int) (
                Robot.IS_REAL ?
                Math.ceil(rawValue): Math.round(rawValue)
        );
        for(int i = 0; i < bits.length; i++) {
            bits[i] = value % 2 == 1;
            value /= 2;
        }
        return bits;
    }
}
