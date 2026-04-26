# App

* [X] Three minor design changes:
  * [X] Traffic light alignment - needs to be vertically aligned with text
  * [X] Task switcher animation - needs to crossfade
  * [X] Task switcher scrolling - currently doesn't allow scrolling; fix this bug
* [X] Major design change - follow appdesign.jpg
  * [X] Polish design - all of these fixes follow appdesign.jpg
    * [X] Fix model area - add disconnection screen (combined w/ controller area), model upside down, flickering on window resize, follow the picure, compass should be stuck to the box (move w/ it about the x, y, z axis when dragged),
    * [X] Fix controller area - disconnection screen should be prominent, opmode start button should be circular, controls not aligned correctly
    * [X] Fix camera switcher - doesn't allow for unselection, add more space/padding
    * [X] Fix widgets - shouldn't be at the top of the screen, move to above model (keeping it the same shape) w/ 2x2 grid of rectangles
  * [X] Fix task switcher focus - each task should have own iframe element, onswitch should not reload, only if task gets reselected should reload
  * [X] Extra polishes:
    * [X] Compass degrees is off center
    * [X] Task 1.2 upload buttons hover border is getting cut off slightly at the bottom
    * [X] OpMode and Telemetry tabs should be combined into one
    * [X] Fix controls!!
      * [X] Fix layout
      * [X] Fix oversizing (w/ dual mode on >2230px)
      * [X] Fix highlight
  * [X] Add dual-control
* [X] Add settings panel
  * [X] Create rough panel
  * [X] Implement swift app correctly w/o separate program (in-app native panel)
  * [X] Implement macos 26 design
  * [X] Add icon to settings opener in context menu
* [ ] Implement bluetooth connection
  * [ ] Enable bluetooth on control hub
    * [ ] Disable system block
    * [ ] Disable optional connection popup
    * [ ] Fix other connection issues
    * [ ] Write post-connection verification in wifi style
  * [ ] Send/recieve messages fast and relaibly
  * [ ] Auto-connect
* [X] Connect to cameras
  * [X] Fix camera behaviour when 1 top and 1 bottom camera is selected on opposite sides
  * [ ] Fix usb video and macos webcam switching
  * [X] Add screenshot buttons for tasks
* [ ] Implement new photogrammetry
* [X] App internals fixes:
  * [X] Transition to "production WSGI server"
  * [X] Add server start detection to eliminate false starts that require a reload
* [X] Create frontend for the AI model
* [X] Implement the AI model locally into the app
* [ ] Add Task Counter (cmd+t and in menu bar) with separate floating always-on-top window
* [X] Add debug menu bar item to launch developer tools and to reload
  * [X] fix the freaking icons
  * [X] fix the hidden dev mode launcher

# Photogrammetry

* [ ] Get manual measurement working
* [ ] Get AI-fallback real photogrammetry working
* [ ] Reimplement new methods w/ method switcher

# Crab Detect

* [X] Finish prototype annotations (300 images; cdphotos-p1 & p2)
* [X] Export correctly
* [X] Train AI model
* [X] Test and package the model
* [ ] Gather expanded data
* [ ] Train on expanded data
