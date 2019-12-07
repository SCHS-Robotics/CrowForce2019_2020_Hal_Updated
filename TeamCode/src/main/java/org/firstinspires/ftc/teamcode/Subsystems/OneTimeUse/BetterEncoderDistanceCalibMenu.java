package org.firstinspires.ftc.teamcode.Subsystems.OneTimeUse;

import com.SCHSRobotics.HAL9001.system.source.GUI.Cursor;
import com.SCHSRobotics.HAL9001.system.source.GUI.GUI;
import com.SCHSRobotics.HAL9001.system.source.GUI.GuiLine;
import com.SCHSRobotics.HAL9001.system.source.GUI.Menu;
import com.SCHSRobotics.HAL9001.system.subsystems.cursors.DefaultCursor;
import com.SCHSRobotics.HAL9001.util.calib.EncoderDistanceCalib;
import com.SCHSRobotics.HAL9001.util.exceptions.NotBooleanInputException;
import com.SCHSRobotics.HAL9001.util.math.Units;
import com.SCHSRobotics.HAL9001.util.misc.Button;
import com.SCHSRobotics.HAL9001.util.misc.CustomizableGamepad;

public class BetterEncoderDistanceCalibMenu extends Menu {

    public BetterEncoderDistanceCalibMenu(GUI gui, Cursor cursor, GuiLine[] startingLines, int selectionZoneWidth, int selectionZoneHeight) {
        super(gui, cursor, startingLines, selectionZoneWidth, selectionZoneHeight);
    }

    //How fast the distance input should be increased.
    private enum SpeedMode{
        FAST(25), MEDIUM(10), SLOW(1), PRECISION(.1);

        public double increment;
        SpeedMode(double increment) {
            this.increment = increment;
        }
    }
    private BetterEncoderDistanceCalibMenu.SpeedMode speedMode;

    //The customizable gamepad that stores all the inputs for the program.
    private CustomizableGamepad inputs;
    //The name of the button used to toggle the speed mode.
    private static final String SPEED_MODE_TOGGLE = "SpeedModeToggle";
    //The EncoderDistanceCalib subsystem that this menu interacts with.
    private BetterEncoderCalibProgram calib;
    //The distance value that has been entered into the menu.
    private double distance;
    //The distance unit being entered into the menu.
    private Units unit;

    /**
     * Constructor for BetterEncoderDistanceCalibMenu.
     *
     * @param gui - The GUI being used to render the menu.
     * @param unit - The unit of distance being entered into the menu.
     * @param speedToggleButton - The button used to toggle the increment/decrement speed.
     * @param calib - The EncoderDistanceCalib subsystem associated with this menu.
     */
    public BetterEncoderDistanceCalibMenu(GUI gui, Units unit, Button speedToggleButton, BetterEncoderCalibProgram calib){
        super(gui, new DefaultCursor(gui.robot, new DefaultCursor.Params()), new GuiLine[]{new GuiLine("<#>", ""), new GuiLine("###", "Done  " + "Increment: " + "Fast")},3,2);

        speedMode = BetterEncoderDistanceCalibMenu.SpeedMode.FAST;
        distance = 0;

        this.unit = unit;
        this.calib = calib;

        inputs = new CustomizableGamepad(gui.robot);
        if(!speedToggleButton.isBoolean){
            throw new NotBooleanInputException("SpeedToggleButton must be a boolean button");
        }
        inputs.addButton(SPEED_MODE_TOGGLE, speedToggleButton);

        GuiLine[] newerLines = {
                new GuiLine("<#>", "I traveled: " + "0" + unit.abreviation),
                lines.get(1)
        };
        setLines(newerLines);
    }

    @Override
    protected void init() {

    }

    @Override
    protected void open() {

    }

    @Override
    public void onSelect() {
        if(cursor.y == 1){
            calib.numberSelected(Units.convert(distance, unit, Units.METERS));
        }
    }

    @Override
    public void onButton(String name, Button button) {
        if(name.equals(DefaultCursor.LEFT) || name.equals(DefaultCursor.RIGHT)){
            if(cursor.y == 0 && cursor.x == 0){
                distance -= speedMode.increment;
                cursor.setX(1);
                updateLinesForIncrement();
            }
            else if(cursor.y == 0 && cursor.x == 2){
                distance += speedMode.increment;
                cursor.setX(1);
                updateLinesForIncrement();
            }
        }
        if(name.equals(DefaultCursor.UP)){
            if(cursor.x == 0 || cursor.x == 2){
                cursor.setX(1);
            }
        }
    }

    @Override
    protected void render() {
        if(inputs.getBooleanInput(SPEED_MODE_TOGGLE)){
            switch (speedMode){
                case FAST:
                    speedMode = BetterEncoderDistanceCalibMenu.SpeedMode.MEDIUM;
                    setLines(new GuiLine[]{lines.get(0), new GuiLine(lines.get(1).selectionZoneText, lines.get(1).postSelectionText.substring(0, 6) + "Medium")});
                    break;
                case MEDIUM:
                    speedMode = BetterEncoderDistanceCalibMenu.SpeedMode.SLOW;
                    setLines(new GuiLine[]{lines.get(0), new GuiLine(lines.get(1).selectionZoneText, lines.get(1).postSelectionText.substring(0, 6) + "Slow")});
                    break;
                case SLOW:
                    speedMode = BetterEncoderDistanceCalibMenu.SpeedMode.PRECISION;
                    setLines(new GuiLine[]{lines.get(0), new GuiLine(lines.get(1).selectionZoneText, lines.get(1).postSelectionText.substring(0, 6) + "Precision")});
                    break;
                case PRECISION:
                    speedMode = BetterEncoderDistanceCalibMenu.SpeedMode.FAST;
                    setLines(new GuiLine[]{lines.get(0), new GuiLine(lines.get(1).selectionZoneText, lines.get(1).postSelectionText.substring(0, 6) + "Fast")});
                    break;
            }
        }
        displayLines(lines);
    }

    @Override
    protected void initLoopRender() {

    }

    @Override
    protected void onStart() {

    }

    @Override
    protected void stop() {

    }

    /**
     * Updates the displayed distance value.
     */
    private void updateLinesForIncrement(){
        setLines(new GuiLine[]{
                new GuiLine(lines.get(0).selectionZoneText, "I traveled: " + distance + unit.abreviation),
                lines.get(1)
        });
    }
}
