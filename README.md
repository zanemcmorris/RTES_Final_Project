Admin todo list:
- Exercise 6 business
- -- Determine deadlines in the system
- -- -- Target 500hz for control loop
- -- -- Acquire sensor data at 1666hz or more? Faster is better if it's free-ish with DMA
- -- -- BLE checkin time of 1ms or less? (EDF?)

Hardware todo list
- Assemble drone body and test motors -- Assembled one frame. Looking into 3D printing another
- Find a battery! -- Ordered another one
- Buy another FCU? -- Ordered another one

Software Todo List
- LSM6DSR (IMU) Starting Driver -- Almost Done -- Zane 4/3
- LSM6DSR Improved driver (non-blocking DMA-driven IO)
- Barometer Driver -- Almost Done -- Zane 4/3
- Barometer Improved Driver (non-blocking DMA-driven IO)

- Find and learn a quaternion library?

- Control loop implementation
- Set up timers and spin motors 

- Unit testing?
- BLE integration? -- Dillon & Likhita Started
- Tooling? (python graphing and other tooling to interperet live data) -- WIP -- Zane, plotter/plotter.py
- Look into whether Simulink is something we could use

Stretch Goals
- Program sick ass maneuvers?


Work done For 4/3/26
- Fixed IMU communication by removing dumb byte
- Set up USART and CH340 to USB to view serial output
- Verified plotter application over USART
- Set up LS22HH barometer driver
- Added barometer plot to plotter
- Enabled FreeRTOS and implemented tasks for LEDHeartbeat and IMU-sampling-and-printing. 
