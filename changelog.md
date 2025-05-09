# Changelog

*Changelog created using the [Simple Changelog](https://marketplace.visualstudio.com/items?itemName=tobiaswaelde.vscode-simple-changelog) extension for VS Code.*

## [1.0] - 2025-02-28
### Added
- First use of Change Log for Deadpool
- Changed Deadpool Bindings to match more with Operator panel including centering and adding Climb
- Added Back Driver insted of Singleplayerdriver
- Changed CLimb to use a boolean called climbstart to stop accidental uses of climb
- Changed elevator setpoints due to bolts on elevator 
- Added back Idle High Positions instead of using L2
- Lots of Auto changes to fix weird git pull 
- Refactored isL methods
- Event Markers do not work
- Tuning positining for Three coral Right and Left
- Added Algae Intake, Needs wrist tuning 

## [1.1] - 2025-02-28
### Removed
- Limelight Auto Max Speed Constant
- Limelight Center Reef Auto Commands (Unused)
### Changed
- Changed Pathplanner Autonomous to fit 987 field
- Changed bouncing on first coral intake at human player
- Idle High for Auto
- Limelight configurations in robot.java
- Climber encoder positions 
- Deadpool Panel Controls
### Added
- Working Algae Intake

## [1.2] - 2025-03-3
### Changed
- Changed Pathplanner Autonomous to fit 7426 field
- Optimized Three Coral works on most cases
- Change Three Coral to work on Optimized Three Coral
- Changed Human Player Heights and Adjusted Algae Intaking

## [1.3] - 2025-03-4
### Changed
- Optimized Pathplanner Autonomous
- Tested Algae Intake
### Added
- Added centering for algae intake
- Added center auto

## [1.4] - 2025-03-5
### Changed
- Autos for Left and right to match with new wheels
- AUTOSSSSSS WITH FRESSSSSHHH AND NEWWW WHEEEELSSSSS NEWWWW WHEEELSSS
- Setpoints to match with broken elevator (Done) 
- Cleaned path planner getting rid of unsued autos
- Autos have been attempted with broken elevator (Can be optimized)
- Clean up at Ventura (Add back L4 shots)

- Notes (Elevator Left Camera is slightly blurry - Check for symptoms)

- Left (Blue) (Good)
- Left (Red) (Bad) (Flip)
- Right (Blue) (Test)
- Right (Red) (Bad) (Flip)

## [1.5] - 2025-03-10
### Changed
- After Comp push
- Auto Changes
- Probably tuned setpoints
### Removed
- Single Driver

## [1.6] - 2025-03-10
### Added
- New path based on not turning in path planner
- Id Restrictions to Limelights
- New Auto Commands

## [1.7] - 2025-03-11
### Added
- Utah Right Side
- Utah Left Side
## Adding Soon
- Another version of Utah Left and Right Side
### Removed
- Small Climber everything else is same 

## [1.8] - 2025-03-19
### Added
- Vegas Left Side
- Vegas Left Side
- Command for Auto centering right reef
## Adding Soon
- Center Auto with Algae Scoring
## Changed
- Lowered all is L4 values to better match with rigging 
- Max Speed for auto
### Removed
- Deprecated initSendable methods
- Mentions of Limelight.AutoMaxSpeed
- Mentions of "Big" climber

## [1.8] - 2025-03-19
### Added
- Added indivdual commands for changing pipleines for less confusion
- Added Idle High Command for time save at start of auto
## Changes
- Calibrated Human Player Centering
## NEEDS TO CHANGE 
- Currrent Pipeline system
- REZERO/RECALIBRATE all pipelines to avoid a different center command
- Weird Outake bug

## [1.9] - 2025-03-24
## Changes
- Changed Elevator Slot of Algae to match with coral 2 -> 1
- Vegas Right tuning
- Calibrated X and Y on all tags
- CoralOutAuto to use low brake and time
- moved Limelight.Autostage++ to reset high cause intake was weird
## Removed
- Pipeline commands as its done in autonmous periodic
## FYI
- At comp calibrate XY and tune Ta for human player

## [2.0] - 2025-03-25
## Changes
- Changed Wrist PID slower on idle for better shots
- Tuned positons for Vegas autos 
- Tuned outake speed
## Added 
- Algae Centering command
- Autonomous Algae commands
- Position to shoot barge backwards

