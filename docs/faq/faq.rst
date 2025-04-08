Frequently Asked Questions
==================================================

Texture not loading
-------------
Download the whole repo and not only the world file from github.

python.exe not found (on Windows)
-------------
Usually an installation issue, can have any number of causes. Clean out any installed version of Python, downloading the official installer from python.org and checking “Add to path” when installing. 
If Python is already installed, it can be added to path so that Webots knows where to find it.
If Python is installed without administrator access, it is usually somewhere in the users AppData folder, it should be added to the user path. 
Otherwise, if it is somewhere in Program Files, it should be added to system path. In this case, it is sufficient to close any Webots and terminal windows and reopen them, it should just work. 
If it is not in AppData or Program Files, it is just the shortcut in the start menu! It should not be necessary to modify the Webots preferences, as long as path is correctly configured. 
On Windows, it is also recommended to disable the Python app alias that opens the Windows Store. Adding “.exe” is usually not required in Windows (it appends this automatically). 

numpy / matplotlib not found
-------------
Usually a problem with conflicting Python installations. 
Install a self-contained Python virtual environment, that has its own clean Python runtime and pip packages, and point Webots directly to that executable (something like python path: /path/to/.venv/bin/python3, using .venv/bin/pip3 to install packages, maybe chmod +x is also required on macOS). 

cf_camera is not showing
-------------
Go to overlays / camera / show cf_camera.

Odd visuals
-------------
Everything is black and the object outline is in white. If you pressed **shift+w** its on wireframe display, so to get it back press **shift+p** to get to plane rendering.

Drone starts updside down
-------------
You overwrote the initial conditions by saving in webots. Just copy paste the world file from the github repo and reload.

Camera view not showing
-------------
Right-click on the robot and in the scene tree, click overlays / camera devices / show cf_camera. If still not visible, click overlay on the bar and untick hide all cameras.

Webots is super slow
-------------
Remove shadows to make it quicker. Tools / preferences / OpenGL / disable shadows, disable anti-aliasing.


====================================================================================

If your question still persists, raise your hand and ask a TA.
