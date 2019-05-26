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
Open the `configlisteners.json` file in GreyDash's source code. You will see two sections: one for chart listeners and one for indicator listeners.


### Chart Listeners
Here is a template for a chart listener:
```json
{
    "settings": {
        "tooltip": false,
        "minValue": -14,
        "maxValue": 14,
        "show": true
    },
    "title": "My subsystem's voltage",
    "keys": [
        "/SmartDashboard/subsystem/type/name"
    ]
},
```

Let's define these flags:

 - `settings` is the container for the chart settings. This will change how the chart looks.
 - `tooltip` will show the value at the time where your cursor is over the chart.
 - `minValue` is the lowest value the chart can show.
 - `maxValue` is the highest value the chart can show.
 - `show` determines whether the chart will be shown.
 - `title` is the description shown above the chart.
 - `keys` is the container for the SmartDashboard keys to be shown. You can list multiple for multiple lines. Please describe the colors in the title.


### Indicator Listeners
Here is a template for a indicator listener:
```json
{
    "settings": {
        "fixedDecimals": true,
        "show": true,
        "default": true,
        "debug": false
    },
    "title": "My subsystem's voltage",
    "unit": "v",
    "key": "/SmartDashboard/subsystem/type/name"
},
```

Let's define these flags:

 - `settings` is the container for the indicator settings. This will change how the indicator looks.
 - `fixedDecimals` will limit the number of decimals shown to 2 spaces. This is a very good idea to keep true
 - `show` determines whether the chart will be shown.
 - `default` determines whether the chart will be shown on the default tab.
 - `debug` determines whether the chart will be shown on the debug tab.
 - `title` is the description shown above the indicator.
 - `unit` is the unit to be shown after the number, like v for voltage or a for amperes
 - `key` is the SmartDashboard key.

## FAQ

 - Q: The dashboard looks wrong! What should I do?
   - A: Make sure the pynetworktables2js instance is running. If you are still having errors, open up the dev tools console in your browser.

 - Q: I'm not getting output!
   - A: Check that you are actually writing values to the key. Typos may be present in either the robot code or the json file.
