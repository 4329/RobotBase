package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;

public class DPadButton extends Button
{

    XboxController controller;
    Direction direction;

    public DPadButton(XboxController controller, Direction direction)
    {
        this.controller = controller;
        this.direction = direction;
    }

    public static enum Direction
    {
        UP(0), UPRIGHT(45), RIGHT(90), DOWNRIGHT(135), DOWN(180), DOWNLEFT(235), LEFT(270), UPLEFT(315);

        private int direction;

        private Direction(int direction)
        {
            this.direction = direction;
        }

        public int getValue()
        {
            return this.direction;
        }
    }

    public boolean get()
    {
        int dPadValue = controller.getPOV();
        return (dPadValue == direction.getValue());
    }

}