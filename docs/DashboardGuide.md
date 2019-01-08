# How To Incoporate Dashboard Features Into Your Subsystem
*This guide will go over the required steps for getting the NetworkTable "SmartDashboard" working in your subsystem.*

## Step 1: Send the value to the table
Use the `SmartDashboard` method:
```cpp
SmartDashboard::PutNumber("subsystem/type/name", m_myDevice->GetValue());
```
The example above describes how to put a value into the table. A good naming scheme for the key is to use the name of the subsystem (if none, use "misc"), then the type (such as voltages, currents, velocities, etc), followed by the specific name (such as leftvoltage, rightcurrent, drivevelocity). The next parameter is the value to send to the table.

Some options include:
 - `SmartDashboard::PutBoolean()`
 - `SmartDashboard::PutNumber()`
 - `SmartDashboard::PutString()`

More options for SmartDashboard are shown [here](http://first.wpi.edu/FRC/roborio/release/docs/cpp/classfrc_1_1SmartDashboard.html).

## Step 2: Add key to GreyDash
Currently under development.
