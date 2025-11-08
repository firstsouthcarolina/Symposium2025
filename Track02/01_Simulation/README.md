# Track 02 — Simulation

Canva Presentation Link
- https://www.canva.com/design/DAG389LOtUA/9SNSv_9u-5idSTSn5Ljl5Q/view?utm_content=DAG389LOtUA&utm_campaign=designshare&utm_medium=link2&utm_source=uniquelinks&utlId=hb7ae2fe314

What this is
- A hands-on set of small simulations and visual mechanisms that explain how robot motors and simple control loops work.
- Build visuals using WPILib tools so non-programmers (mentors, students, and visitors) can understand ideas visually without reading lots of code.

Who this is for
- Programming teams that don't have access to the physical robot
- Teams that want to prototype subsystems interacting in a superstructure
- Teams that want to provide visualizations to non-programmers

Quick start (3 steps)
1. Clone this repository
2. Open the WPILib's Version of Visual Studio Code
3. Do either of the following, depending on where you are with the presentation
  a. Open the `simplesimulation` folder for SmartDashboard and Epilogue
  b. Open the `advantagekit` folder for AdvantageKit 
  - Click the `WPILib` button within the WPILib VS Code window
  - Select the `WPILib: Simulate Robot Code` option
  - Select the `Sim GUI` or `Use Real Driver Station` option
5. Run the demo. Watch the visualizations in either SimGUI or AdvantageScope

What's included
- Simulation — runnable examples that model motor and mechanism behavior.
- DCMotor Controls with ProfiledPID — shows how a motor follows a target using a PID controller with motion profiling.
- Physics-based Simulation — simple physical models that behave like a real mechanism.
- Mechanisms — visual representations to help non-programmers understand motion.

Goals
1. Show how simulated motors connect to existing subsystem code.
2. Feed simulated motor data into visualizations so results are easy to see.
3. Build clear visual mechanisms so non-programmers can understand the workflow.


Glossary
- Simulation: a computer model that imitates real-world motor/robot behavior.
- PID: a control method that adjusts motor output to reach and hold a target using Proportional, Integral, and Derivative gains.
- Profiled PID: PID combined with a smooth motion plan.
- Mechanism: a visible part of a robot (arm, turret, pivot) used to explain motion.
