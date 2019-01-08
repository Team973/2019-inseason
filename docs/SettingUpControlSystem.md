# Setting Up Our Control System

### Roborio
- Open the Roborio Imaging Tool which can be installed [here](http://www.ni.com/download/first-robotics-software-2015/5112/en/)
- Connect to the roborio with a USB A to USB B cable (printer cable)
- Wait until the Imaging tool finds the roboRIO target
- Once it finds the roborio, type in `973` in the Team Number space
- Check `format target option`
- Select the newest image available
- Click the `Reformat` option
- Wait until the imaging process is done. Then proceed to the next step.

### Talon and Victor Addressing
- Go to the Silverlight Web UI at `roborio-973-frc.local` or IP address assigned after imaging the rio. If you are using the programming laptops, use Internet Explorer or Mozilla Firefox. Download Microsoft Silverlight [here](https://www.microsoft.com/getsilverlight/get-started/install/default?reason=unsupportedplatform).
- Install [Phoenix lifeboat](http://www.ctr-electronics.com/control-system/hro.html#product_tabs_technical_resources). Follow the instructions in the program for Installing Phoenix/Web-based Config and use a USB A to USB B cable (printer cable).
- Once it is done installing, there will be a list of available Talons and Victors for addressing. You won't be able to see any of the devices unless the tool has run.
- For each talon/victor, rename each talon to its specific purpose like `Left Drive Talon A Master`, and reindex each to its own specific ID. Refer to the wiring IO spreadsheet for the up-to-date list of which IDs to index each talon/victor.
- You might have to restart every time you index each talon/victor since new talons/victors default to ID 0.
- Update the FRC-specific firmware for each [talon](http://www.ctr-electronics.com/talon-srx.html#product_tabs_technical_resources) and [victor](http://www.ctr-electronics.com/victor-spx.html#product_tabs_technical_resources) to the newest version.
