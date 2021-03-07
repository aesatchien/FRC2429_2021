2021 0207 readying the roborio:

1) Upgrade the firmware and the image on the roborio with the roborio imaging tool.
	As of 2/7/2021 this is 2021v3

2) Download python for the roborio onto the computer - be connected to the internet for this
	If you have the robotpy package installed, then robotpy-installer.exe should be installed
	
	robotpy-installer download-python    (downloads and caches the files in your wpilib 2021 robotpy dir)
	robotpy-installer download robotpy[all]   (gets the rest of it)

3) Install it on the roborio - disconnect from internet but connect to the roborio vis USB
	
	robotpy-installer install-python  (only has to be done once)
	robotpy-installer install robotpy[all]
	
And then we're done getting it ready.  Next, we need to deploy.

4) python robot.py deploy     - copies everything over, but recursively gets the sim folder as well.  need to remove before deploy*+


Solidworks 2019:
Download Link: www.solidworks.com/SEK.  Select YES to the first question (you already have a serial number).  
Select Student Team. Choose 2019-2020 version, Serial is:
SolidWorks CAD serial number: 9020009174873759W6N3Z7FH  
CAM, electrical and simulation automatically get installed with this one now)