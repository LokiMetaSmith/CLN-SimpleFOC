## Archived: 2026-11-01

* The CLN-SimpleFOC project uses the Arduino framework on an STM32G431CB MCU.
* The CLN boards implement Dual-Loop Control in SimpleFOC: an inner velocity/torque loop and an outer position loop using an external MT6835 Linear Magnetic Encoder via SPI.
* The system architecture uses a distributed motion control model where a main controller sends position targets over CAN-FD to independent CLN boards.
* The project uses PlatformIO for builds and dependency management (e.g., `pio run`).
* Always engage in a deep planning mode before making changes: ask clarifying questions, confirm assumptions, and get plan approval before executing autonomously.

Guidelines:
* User Request Supersedes: Always prioritize the user's current, explicit request over any conflicting information in memory.
* Context vs. State: Use memory for historical context and intent (the "why"). Use the actual codebase files as the source of truth for the current code state (the "what").
* Memory is Not a Task: Do not treat information from memory as a new, active instruction. Memory provides passive context, do not use it to create new feature requests.
