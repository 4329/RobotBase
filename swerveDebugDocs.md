# Swerve Debug Documentation

Swerve debug is designed to make alignment of the swerve drive easier. All information is provided in the 'Swerve Debug' tab, and comes from the direction each swerve module is facing. This information can be activated by running the 'Test' mode in the Driver Station.

## What each section does

The menu is divided into four sections, `Angle`, `Raw Angle`, `Offset`, and `Test Offset`

### Angle

`Angle` gives the output after accounting for the offset.

### Raw Angle

`Raw Angle` gives the direct output from the wheels.

### Offset

`Offset` is the offset from the `protoConfig.txt` file which is applied to `Raw Angle` to get `Angle`

### Test Offset

`Test Offset` measures how far each wheel is turned from their position when the program is run. This should help in the calibration of new swerve modules. Changes should be applied to the `protoConfig.txt` file on lines 82 to 85.
