# Nano-sim Simulator

<div align="center">

<img src="https://img.shields.io/badge/release-0.1.0-0071E3?style=flat-square" alt="Release 0.1.0">
<img src="https://img.shields.io/badge/status-early_prototype-F5A623?style=flat-square" alt="Status: early prototype">
<img src="https://img.shields.io/badge/platforms-macOS%20%C2%B7%20Linux-8E8E93?style=flat-square" alt="Platforms: macOS, Linux">
<img src="https://img.shields.io/badge/license-Apache_2.0-8E8E93?style=flat-square" alt="License: Apache 2.0">

</div>

<br>

<img src="assets/nano-sim-cover.png" alt="Nano-sim — Autonomous Driving Simulator" width="100%">

<div align="center">

<br>

<a href="../../releases"><img src="https://img.shields.io/badge/Download-0071E3?style=for-the-badge&labelColor=0071E3" alt="Download"></a>
&nbsp;
<a href="https://hyeokk.com/projects/nano-sim/docs"><img src="https://img.shields.io/badge/Documentation-1D1D1F?style=for-the-badge&labelColor=1D1D1F" alt="Documentation"></a>

</div>

## About

Nano-sim is a Unity-based, closed-loop **autonomous driving simulator** built around a
small-scale RC car. It is made for research use — generating synthetic perception data
(RGB, segmentation, depth, 2D/3D LiDAR, IMU/GNSS), training and evaluating driving
policies, and testing perception and control in a repeatable virtual environment that
mirrors a real bench setup.

> **Release 0.1.0 — early prototype.**
> Core simulation and sensors are working. UI polish, window resizing, and in-app exit
> controls are still in progress and will arrive in later updates.

<br>

## Download

Get the latest build from the [**Releases**](../../releases) page.

| Platform | File |
| :-- | :-- |
| macOS · Apple silicon / Intel | `Nano-sim-macOS.zip` |
| Linux · x86_64 | `Nano-sim-linux.zip` |

<br>

## Install & Run

The app is **not code-signed** (research prototype), so your OS will block it the first
time you open it. This is expected — follow the steps for your platform.

### macOS

1. Unzip `Nano-sim-macOS.zip`.
2. **Right-click** (or Control-click) `Nano-sim.app` → **Open** → **Open** again.
   You only need to do this once.

<details>
<summary>macOS says <em>"Nano-sim.app is damaged and can't be opened"</em></summary>

<br>

Clear the quarantine flag in Terminal, then open it normally:

```bash
xattr -cr /path/to/Nano-sim.app
```

</details>

### Linux

1. Unzip `Nano-sim-linux.zip`. **Keep all files together** — the executable,
   `UnityPlayer.so`, and the `Nano-sim_Data/` folder must stay in the same directory.
2. Make the binary executable and run it:
   ```bash
   chmod +x ./Nano-sim.x86_64
   ./Nano-sim.x86_64
   ```

<details>
<summary>It doesn't start</summary>

<br>

Launch it from a terminal to see any error output, and make sure your GPU drivers
(Vulkan / OpenGL) are installed.

</details>

<br>

## Requirements

- macOS (Apple silicon or Intel) **or** Linux x86_64
- A GPU with up-to-date drivers

<br>

## Documentation

Full documentation — setup, sensors, the Python/TCP API, and dataset tooling:

**→ [hyeokk.com/projects/nano-sim/docs](https://hyeokk.com/projects/nano-sim/docs)**

<br>

## License

Licensed under the Apache License 2.0 — see [LICENSE](LICENSE) and [NOTICE](NOTICE).

Copyright 2026 Junhyeok Lee

<sub>Product names referenced in the app (camera / LiDAR / vehicle) identify the real
hardware the simulator models. They are trademarks of their respective owners. This is an
independent, non-commercial research project and is **not affiliated with, sponsored by,
or endorsed by** any of them.</sub>
