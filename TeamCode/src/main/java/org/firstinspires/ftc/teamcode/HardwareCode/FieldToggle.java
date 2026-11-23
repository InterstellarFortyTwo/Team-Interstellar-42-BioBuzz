package org.firstinspires.ftc.teamcode.HardwareCode;

public class FieldToggle {
    boolean fieldtoggle = false;
    public FieldToggle(boolean fieldToggle) {
        this.fieldtoggle = fieldToggle;
    }
    public void setFieldToggle(boolean newFieldToggle) {fieldtoggle = newFieldToggle; }
    public boolean getFieldToggle() {return fieldtoggle;}
}
