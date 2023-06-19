package frc.trigon.robot.utilities;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.trigon.robot.Robot;

public class KeyboardController extends CommandGenericHID {
    /**
     * Construct an instance of a device.
     *
     * @param port The port index on the Driver Station that the device is plugged into.
     */
    public KeyboardController(int port) {
        super(port);
    }

    public Trigger esc() {
        return button(1);
    }

    public Trigger f1() {
        return button(2);
    }

    public Trigger f2() {
        return button(3);
    }

    public Trigger f3() {
        return button(4);
    }

    public Trigger f4() {
        return button(5);
    }

    public Trigger f5() {
        return button(6);
    }

    public Trigger f6() {
        return button(7);
    }

    public Trigger f7() {
        return button(8);
    }

    public Trigger f8() {
        return button(9);
    }

    public Trigger f9() {
        return button(10);
    }

    public Trigger f10() {
        return button(11);
    }

    public Trigger f11() {
        return button(12);
    }

    public Trigger f12() {
        return button(13);
    }

    public Trigger del() {
        return button(14);
    }

    public Trigger backtick() {
        return button(15);
    }

    public Trigger one() {
        return button(16);
    }

    public Trigger two() {
        return button(17);
    }

    public Trigger three() {
        return button(18);
    }

    public Trigger four() {
        return button(19);
    }

    public Trigger five() {
        return button(20);
    }

    public Trigger six() {
        return button(21);
    }

    public Trigger seven() {
        return button(22);
    }

    public Trigger eight() {
        return button(23);
    }

    public Trigger nine() {
        return button(24);
    }

    public Trigger zero() {
        return button(25);
    }

    public Trigger minus() {
        return button(26);
    }

    public Trigger equals() {
        return button(27);
    }

    public Trigger backspace() {
        return button(28);
    }

    public Trigger tab() {
        return button(29);
    }

    public Trigger q() {
        return button(30);
    }

    public Trigger w() {
        return button(31);
    }

    public Trigger e() {
        return button(32);
    }

    public Trigger r() {
        return getButtonFromBitOfAxis(0, 0);
    }

    public Trigger t() {
        return getButtonFromBitOfAxis(1, 0);
    }

    public Trigger y() {
        return getButtonFromBitOfAxis(2, 0);
    }

    public Trigger u() {
        return getButtonFromBitOfAxis(3, 0);
    }

    public Trigger i() {
        return getButtonFromBitOfAxis(4, 0);
    }

    public Trigger o() {
        return getButtonFromBitOfAxis(5, 0);
    }
    
    public Trigger p() {
        return getButtonFromBitOfAxis(6, 0);
    }

    public Trigger a() {
        return getButtonFromBitOfAxis(7, 0);
    }

    public Trigger s() {
        return getButtonFromBitOfAxis(0, 1);
    }

    public Trigger d() {
        return getButtonFromBitOfAxis(1, 1);
    }

    public Trigger f() {
        return getButtonFromBitOfAxis(2, 1);
    }

    public Trigger g() {
        return getButtonFromBitOfAxis(3, 1);
    }

    public Trigger h() {
        return getButtonFromBitOfAxis(4, 1);
    }

    public Trigger j() {
        return getButtonFromBitOfAxis(5, 1);
    }

    public Trigger k() {
        return getButtonFromBitOfAxis(6, 1);
    }

    public Trigger l() {
        return getButtonFromBitOfAxis(7, 1);
    }

    public Trigger semicolon() {
        return getButtonFromBitOfAxis(0, 2);
    }

    public Trigger apostrophe() {
        return getButtonFromBitOfAxis(1, 2);
    }

    public Trigger leftShift() {
        return getButtonFromBitOfAxis(2, 2);
    }

    public Trigger z() {
        return getButtonFromBitOfAxis(3, 2);
    }

    public Trigger x() {
        return getButtonFromBitOfAxis(4, 2);
    }

    public Trigger c() {
        return getButtonFromBitOfAxis(5, 2);
    }

    public Trigger v() {
        return getButtonFromBitOfAxis(6, 2);
    }

    public Trigger b() {
        return getButtonFromBitOfAxis(7, 2);
    }

    public Trigger n() {
        return getButtonFromBitOfAxis(0, 3);
    }

    public Trigger m() {
        return getButtonFromBitOfAxis(1, 3);
    }

    public Trigger comma() {
        return getButtonFromBitOfAxis(2, 3);
    }

    public Trigger period() {
        return getButtonFromBitOfAxis(3, 3);
    }

    public Trigger forwardSlash() {
        return getButtonFromBitOfAxis(4, 3);
    }

    public Trigger rightShift() {
        return getButtonFromBitOfAxis(5, 3);
    }

    public Trigger leftCtrl() {
        return getButtonFromBitOfAxis(6, 3);
    }

    public Trigger leftAlt() {
        return getButtonFromBitOfAxis(7, 3);
    }

    public Trigger rightAlt() {
        return getButtonFromBitOfAxis(0, 4);
    }

    public Trigger rightCtrl() {
        return getButtonFromBitOfAxis(1, 4);
    }

    public Trigger left() {
        return getButtonFromBitOfAxis(2, 4);
    }

    public Trigger right() {
        return getButtonFromBitOfAxis(3, 4);
    }

    public Trigger up() {
        return getButtonFromBitOfAxis(4, 4);
    }

    public Trigger down() {
        return getButtonFromBitOfAxis(5, 4);
    }

    public Trigger numpad0() {
        return getButtonFromBitOfAxis(6, 4);
    }

    public Trigger numpad1() {
        return getButtonFromBitOfAxis(7, 4);
    }

    public Trigger numpad2() {
        return getButtonFromBitOfAxis(0, 5);
    }

    public Trigger numpad3() {
        return getButtonFromBitOfAxis(1, 5);
    }

    public Trigger numpad4() {
        return getButtonFromBitOfAxis(2, 5);
    }

    public Trigger numpad5() {
        return getButtonFromBitOfAxis(3, 5);
    }

    public Trigger numpad6() {
        return getButtonFromBitOfAxis(4, 5);
    }

    public Trigger numpad7() {
        return getButtonFromBitOfAxis(5, 5);
    }

    public Trigger numpad8() {
        return getButtonFromBitOfAxis(6, 5);
    }

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
