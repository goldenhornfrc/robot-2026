## Quick context

This is a WPILib Java robot project (Java 17) built with GradleRIO. It uses the AdvantageKit logging/junction system, vendor libraries (vendor JSONs under `vendordeps/`), and the Command-based structure from WPILib.

Keep answers tightly focused: reference files below when suggesting edits, follow the IO abstraction pattern, and prefer small, well-scoped changes that compile and respect Spotless formatting.

## Big-picture architecture (what to know first)
- This is a project built on top of a template from Mechanical Advantage's AdvantageKit, which provides a structured starting point for WPILib robot code with built-in logging and replay capabilities.
- Entry points: `src/main/java/frc/robot/Robot.java` (robot lifecycle) and `RobotContainer.java` (subsystem construction, default commands, button bindings).
- Subsystems follow an IO abstraction pattern: an interface `XxxIO` with platform implementations named `XxxIOTalonFX` (real hardware) and lightweight `XxxIO` anonymous classes for SIM/HEADLESS use. See examples in `src/main/java/frc/robot/subsystems/*` and `RobotContainer.java` where the concrete implementation is chosen by `Constants.currentMode`.
- Device/config constants are generated: `src/main/java/frc/robot/generated/TunerConstants.java` and `BuildConstants` produced by the `gversion` plugin. Don’t edit generated files directly.
- Logging/telemetry uses AdvantageKit / Junction. `Robot` configures `WPILOGWriter`, `NT4Publisher`, and replay sources depending on `Constants.currentMode`.

## Project-specific workflows & commands (Windows PowerShell)

- Build (includes Spotless formatting run):

  .\gradlew.bat build

- Deploy to roboRIO (team number comes from `.wpilib/wpilib_preferences.json` or pass via Gradle property):

  .\gradlew.bat deploy -Pteam=1234

- Run AdvantageKit replay helper (project defines `replayWatch`):

  .\gradlew.bat replayWatch

- Note: `spotlessApply` is a build dependency (formatting runs before compile). If formatting changes are unexpected, run:

  .\gradlew.bat spotlessApply

## Patterns and conventions to follow

- IO abstraction: Always add a small `XxxIO` interface first. Provide a `XxxIOTalonFX` (real hardware), a SIM `XxxIO` anonymous implementation (see `RobotContainer`), and update DI in `RobotContainer` to select implementations by `Constants.currentMode`.
- Commands: Use WPILib Command-based style; default commands are set with `setDefaultCommand()` inside `RobotContainer.configureButtonBindings()`.
- Unit tests use JUnit 5 (`test` configuration). Keep tests small and avoid hardware assumptions.
- Formatting: The project enforces Google Java Format + Spotless; make code style-compliant edits or run `spotlessApply`.

## Integration points & external dependencies

- Vendor JNI/libs and versions are declared by the JSON files in `vendordeps/` and pulled by GradleRIO (see `build.gradle`). Changing vendor libraries requires updating those JSONs and Gradle config.
- Native/simulation differences: many subsystems use `IO` interfaces so you can swap real TalonFX implementations with simulation no-op implementations—follow the existing pattern.
- Deployable static resources are under `src/main/deploy` and copied to `/home/lvuser/deploy` on the roboRIO.

## Files to inspect for common tasks (examples)

- `build.gradle` — GradleRIO configuration, JVM args, artifact configuration, `spotless`, and `replayWatch` task.
- `src/main/java/frc/robot/Robot.java` — startup, AdvantageKit wiring, and lifecycle.
- `src/main/java/frc/robot/RobotContainer.java` — where subsystems are constructed and button bindings live.
- `src/main/java/frc/robot/generated/TunerConstants.java` and generated `BuildConstants` — device IDs and build metadata (auto-generated).

## Quick dos & don'ts for code edits

- Do: Follow the IO pattern when adding hardware. Update `RobotContainer` to choose implementations by `Constants.currentMode`.
- Do: Run `.\gradlew.bat build` locally to validate compile and formatting.
- Don't: Edit generated files under `generated/` or the `TunerConstants` source directly — change the source of generation instead.

## If you need to add telemetry or logging

- Use existing AdvantageKit/Logger patterns (see `Robot.java`): add data receivers or use `AutoLogOutputManager` for classes that hold robot state.

## When in doubt, inspect these places first

- `RobotContainer.java` (behavior wiring)
- `build.gradle` (build, tasks, and formatting enforcement)
- `src/main/deploy/` (runtime files deployed to roboRIO)

---

If any area above looks incomplete or you'd like me to expand an example (for example: adding a new subsystem with tests, or step-by-step simulation run), tell me which part and I will iterate.
