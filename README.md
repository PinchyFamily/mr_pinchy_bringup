# mr_pinchy_bringup

ROS 2 (Humble) bringup package for **Mr Pinchy** — vehicle-specific launch files, parameter configs, and hardware setup.

## Prerequisites

- ROS 2 Humble
- A built workspace containing [ublox_dgnss](https://github.com/aussierobots/ublox_dgnss)

## Building

```bash
cd ~/ws
colcon build --packages-select mr_pinchy_bringup
source install/setup.bash
```

## Launch files

| Launch file | Description |
|---|---|
| `gnss.launch.py` | u-blox GNSS driver + NavSatFix converter. Publishes `/fix` (`sensor_msgs/NavSatFix`) and raw UBX topics. |

### GNSS

```bash
ros2 launch mr_pinchy_bringup gnss.launch.py
```

Arguments:

| Argument | Default | Description |
|---|---|---|
| `log_level` | `INFO` | ROS log level |
| `gnss_params_file` | `config/gnss.yaml` | Path to GNSS parameter overrides |

Verify output:

```bash
ros2 topic echo /fix
```

## Configuration

Vehicle-specific parameters live in `config/`. Edit these instead of modifying launch files.

| File | Purpose |
|---|---|
| `gnss.yaml` | u-blox device family, measurement rates, USB message outputs |

### Secrets

Credentials (e.g. NTRIP for RTK corrections) should **not** be committed. Use one of:

- Environment variables referenced in launch files
- A `config/secrets.yaml` file added to `.gitignore`

A template is provided at `config/secrets.yaml.example` (when RTK is configured).

## Hardware

### GNSS (u-blox)

The u-blox receiver appears as `/dev/gnss0` → `/dev/ttyACM0` over USB. No udev rules are needed for basic operation, but if the device index changes you can add a rule:

```
# /etc/udev/rules.d/99-ublox.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="*", SYMLINK+="ublox_gnss", MODE="0666"
```

Then reload with `sudo udevadm control --reload-rules && sudo udevadm trigger`.
