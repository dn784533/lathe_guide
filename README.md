# lathe_guide
Guide to constructing the frame and carriage assembly of a vinyl record lathe. 

This repository includes all the FreeCAD files required, in three folders under Construction:
- Common - this has all the files for the general assembly.
- TEAX19C01 - this has the files required for construction of the cutter head using the TEAX19C01 'coin exciter' drivers.
- OffTheShelfParts - these files should not be 3D printed. They are included purely to illustrate what the final construction should look like. in FreeCAD, load the file \TEAX19C01\Assembly.FCStd.

Also included is an Arduino sketch for control of a stepper motor. The cutter head carriage, described in the construction guide and mounted on a linear rail, is controlled by this motor, and the Arduino is itself controlled via USB by a harness program 'Vinyl Burn' running on a Windows PC. Vinyl Burn enables the user to plan a record side, specifying the sound files to be used along with track- and side-based characteristics such as groove pitch, inter-track gap length, record radius, lock-groove radius and record rotational speed. When the user clicks the 'Cut record' icon, Vinyl Burn plays the selected sound files in turn through the PC's sound card, while sending the appropriate signals to the Arduino to control the linear rail motor.

Vinyl Burn is available from the repository vinyl_burn.
