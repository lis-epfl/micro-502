Frequently Asked Questions
==================================================

Getting and updating the course repo
-------------
**First-time setup:**

1. Clone the repo: in VSCode, open the Command Palette (``Ctrl+Shift+P`` / ``Cmd+Shift+P``), type *Git: Clone*, and paste the repo URL.
2. Create your own branch: open the Source Control panel (the icon with three circles connected by lines), click the branch name in the bottom-left status bar, then select *Create new branch* and give it a name (e.g. ``my-work``). This is where you save your own code.

**Getting weekly solution releases:**

1. Fetch new changes: in the Source Control panel, click the ``...`` menu → *Pull, Push* → *Fetch From All Remotes*.
2. Switch to ``main``: click the branch name in the status bar and select ``main``.
3. Pull the latest solutions: click the ``...`` menu → *Pull*.
4. Optionally, switch back to your branch and merge ``main`` into it to bring in the solutions: click the ``...`` menu → *Branch* → *Merge Branch…* → select ``main``.

Texture not loading
-------------
Download the whole repo and not only the world file from github.

PROTO import errors
-------------
If a PROTO fails to import, check the Webots version in the EXTERNPROTO URL (e.g., R2023a/R2023b) and make sure it matches your installed Webots version. If it does not match, edit the version string in the URL to your version.

python.exe not found (on Windows)
-------------
Usually an installation issue, can have any number of causes. Clean out any installed version of Python, downloading the official installer from python.org and checking “Add to path” when installing. 
If Python is already installed, it can be added to path so that Webots knows where to find it.
If Python is installed without administrator access, it is usually somewhere in the users AppData folder, it should be added to the user path. 
Otherwise, if it is somewhere in Program Files, it should be added to system path. In this case, it is sufficient to close any Webots and terminal windows and reopen them, it should just work. 
If it is not in AppData or Program Files, it is just the shortcut in the start menu! It should not be necessary to modify the Webots preferences, as long as path is correctly configured. 
On Windows, it is also recommended to disable the Python app alias that opens the Windows Store. Adding “.exe” is usually not required in Windows (it appends this automatically). 

CreateProcess failed with error 193 (on Windows)
-------------
Install or move the project to a directory path that contains no spaces, then restart Webots.

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

Changing control keys
-------------
If the drone does not react as you expect, it usually means the key code does not match the key you pressed. In action_from_keyboard (controllers/main/main.py), letter keys use ord('W'), ord('A'), etc., while special keys (arrows, Shift, etc.) use Webots key codes (e.g., Up Arrow can be key == 315). Get those codes from the Webots Keyboard API or asking to ChatGPT.

Webots is super slow
-------------
Remove shadows to make it quicker. Tools / preferences / OpenGL / disable shadows, disable anti-aliasing. Also disable ambient occlusion. If it is still too slow, switch to wireframe rendering (Shift+W). To return to normal rendering, press Shift+P.

Crazyflie does not power on
----------------------------
Check if the battery connector cables are good and that all decks are properly connected. If the battery cables are broken, please directly contact a TA and DO NOT attempt to fix it yourself.


====================================================================================

If your question still persists, raise your hand and ask a TA.
 
