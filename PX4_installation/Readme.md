# PX4 SITL + AirSim (WSL2) + PyCharm Offboard — Setup Guide

This guide covers: installing PX4 SITL on Ubuntu/WSL2, launching it with the
`none_iris` target, connecting it to AirSim running on the Windows host, and
enabling a broadcast MAVLink link so a Python/MAVSDK script running in
PyCharm on Windows can do offboard control.

Environment assumed: **PX4 SITL runs inside WSL2 (Ubuntu)**, **AirSim runs on
the Windows host**.

---

## 1. Install PX4 SITL (minimal dependencies)

Only the packages actually needed to build and run SITL — no Gazebo,
jMAVSim, or NuttX cross-toolchain, since AirSim is the simulator and you're
not flashing real hardware.

```bash
sudo apt update
sudo apt install -y git python3-pip python3-venv

git clone --recursive https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
# optional: pin a known-good release instead of main
# git checkout v1.14.0

bash ./Tools/setup/ubuntu.sh --no-sim-tools --no-nuttx
```

- `--no-sim-tools` skips Gazebo/jMAVSim (not needed — AirSim replaces them).
- `--no-nuttx` skips the ARM cross-toolchain (only needed for real flight
  controllers).

**Log out and back in** (or open a fresh terminal) after this — the setup
script changes group membership and `PATH`.

If the build later fails to find files, run:
```bash
git submodule update --init --recursive
```

---

## 2. Launch PX4 SITL with `none_iris`

`none_iris` builds PX4's flight stack **without** a bundled simulator, so it
just exposes MAVLink/TCP and waits for AirSim to connect.

```bash
cd ~/PX4-Autopilot
export PX4_SIM_HOST_ADDR="<vEthernet (WSL) IPv4>"   # see §4 below
make px4_sitl_default none_iris
```

`PX4_SIM_HOST_ADDR` is read by the rcS startup script — if it's set, PX4
dials its simulator TCP connection out to that address instead of
`localhost`. This is what lets PX4 (in WSL2) reach AirSim (on Windows),
since WSL2 and the Windows host don't share a loopback.

**Start order matters:** start **AirSim first** (it opens the TCP server on
port 4560), **then** launch PX4 (it connects out to AirSim as the client).

---

## 3. Enable broadcast on MAVLink instance #0 (for PyCharm on Windows)

By default, PX4's instance #0 MAVLink link (the GCS-facing link, UDP port
14550) is started with the `-f` (forward) flag, which only forwards
messages between local MAVLink instances — it doesn't broadcast on the
network. That's fine when your script runs inside WSL2 alongside PX4, but
if you want to run your Python/MAVSDK script in **PyCharm on the Windows
host**, PX4 needs to actually broadcast on the WSL virtual network segment
so Windows can receive it without a hardcoded target IP.

Edit this line:

```
PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink
```

(Edit the **source** file under `ROMFS/`, not just the built copy under
`build/px4_sitl_default/etc/init.d-posix/px4-rc.mavlink` — the build copy
gets regenerated from the ROMFS source on every `make`, so an edit there
alone won't survive a rebuild. Edit both if you want the fix to apply
immediately without a full rebuild.)

Find:
```
mavlink start -x -u $udp_gcs_port_local -r 4000000 -f $mavlink_network_interface_arg
```

Replace `-f` with `-p`:
```
mavlink start -x -u $udp_gcs_port_local -r 4000000 -p $mavlink_network_interface_arg
```

- `-f` = enable message forwarding between local MAVLink instances.
- `-p` = enable broadcast mode — PX4 broadcasts on the local network
  segment instead of only forwarding locally, so a listener on the Windows
  host side of the WSL vEthernet adapter (e.g. MAVSDK's
  `udpin://0.0.0.0:14550` in your script) can pick it up.

Rebuild after editing the ROMFS source:
```bash
cd ~/PX4-Autopilot
make px4_sitl_default none_iris
```

---

## 4. Get the WSL vEthernet IPv4 address

On **Windows**, open PowerShell or Command Prompt:

```powershell
ipconfig
```

Look for the adapter named **`vEthernet (WSL)`** and note its **IPv4
Address**. This is the address Windows uses to reach WSL2, and (from the
WSL side) it's also the gateway address for reaching the Windows host — use
the same value for both `PX4_SIM_HOST_ADDR` (§2) and `LocalHostIp` in
settings.json (§5).

> **Note:** in WSL2's default NAT networking mode, this address can change
> after a reboot. If you keep losing connectivity after restarting your PC,
> re-run `ipconfig` and update `PX4_SIM_HOST_ADDR` / `LocalHostIp`
> accordingly — or add `networkingMode=mirrored` under `[wsl2]` in
> `%UserProfile%\.wslconfig` (Windows 11 22H2+), which lets WSL2 share the
> Windows host's network identity and avoids the address changing at all.

---

## 5. `.bashrc` shortcut

Add this function to `~/.bashrc` in WSL2:

```bash
px4() {
    export PX4_SIM_HOST_ADDR="<vEthernet (WSL) IPv4 from ipconfig>"
    cd ~/PX4-Autopilot
    make px4_sitl_default none_iris
}
```

Then:
```bash
source ~/.bashrc
```

From now on, typing `px4` in any terminal exports the right host address,
moves into the repo, and launches SITL.

---

## 6. AirSim `settings.json`

Location: `Documents\AirSim\settings.json` on the **Windows** side.

```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "ViewMode": "SpringArmChase",
  "ClockType": "SteppableClock",
  "Vehicles": {
    "PX4": {
      "VehicleType": "PX4Multirotor",
      "UseSerial": false,
      "LockStep": true,
      "UseTcp": true,
      "TcpPort": 4560,
      "ControlIp": "remote",
      "ControlPortLocal": 14540,
      "ControlPortRemote": 14580,
      "LocalHostIp": "<vEthernet (WSL) IPv4 from ipconfig>",
      "X": 0,
      "Y": 0,
      "Z": 0,
      "Yaw": 180,
      "Sensors": {
        "Barometer": {
          "SensorType": 1,
          "Enabled": true,
          "PressureFactorSigma": 0.0001825
        }
      }
    }
  }
}
```

### Field reference

| Field | Meaning |
|---|---|
| `TcpPort: 4560` | Port AirSim opens as a TCP server for the core sim↔PX4 link (sensors/actuators). PX4 dials into this. |
| `ControlIp: "remote"` | Tells AirSim to resolve the UDP control channel's target from whatever address the incoming TCP connection came from, instead of a hardcoded IP — this is what makes the config resilient to WSL2's address changing between reboots. |
| `ControlPortLocal: 14540` / `ControlPortRemote: 14580` | The MAVLink channel between PX4 and AirSim itself (RC/actuator/sensor injection) — separate from the GCS/offboard link your Python script uses on 14550. |
| `LocalHostIp` | The Windows-side `vEthernet (WSL)` address — tells AirSim to open its TCP port on that adapter instead of localhost, so WSL2 can reach it. |

Use the *documented* field names (`ControlPortLocal` / `ControlPortRemote`)
— older examples floating around use `ControlPort` / `UdpPort`, which
AirSim doesn't actually parse, so they silently do nothing.

---

## 7. Startup order & test

1. Start **AirSim** first (opens TCP server on 4560).
2. Run `px4` in WSL2 (connects out to AirSim via `PX4_SIM_HOST_ADDR`).
3. Watch the `pxh>` console — it should report the simulator host it
   connected to (not `localhost`) and, once GPS streams in, print healthy
   global/home position checks.
4. From PyCharm on Windows, connect MAVSDK to
   `udpin://0.0.0.0:14550` (as in the original script) — with the `-p`
   broadcast fix from §3, this should now receive PX4's telemetry without
   any extra manual `mavlink start` command.

---

## Troubleshooting

- **No TCP connection at all:** confirm `PX4_SIM_HOST_ADDR` and
  `LocalHostIp` are the *same* address, and re-check `ipconfig` — it may
  have changed since your last reboot.
- **TCP connects but no telemetry to Windows:** this is the symptom the
  `-f` → `-p` broadcast fix (§3) addresses — confirm the ROMFS source
  edit was actually rebuilt (`grep -n "\-p \$mavlink_network_interface_arg"
  build/px4_sitl_default/etc/init.d-posix/px4-rc.mavlink` should show the
  edited line).
- **Works, then breaks after a Windows reboot:** you're on WSL2's default
  NAT networking mode and the vEthernet address rotated — either update
  the address in both places each time, or switch to `mirrored` mode in
  `.wslconfig` (§4).
- **Windows Firewall:** make sure inbound TCP 4560 and UDP 14540/14550/14580
  are allowed on the `vEthernet (WSL)` adapter.
- **`PX4_SIM_HOST_ADDR` seems to have no effect:** confirm your
  PX4-Autopilot checkout is recent enough to read it — `grep -n
  "PX4_SIM_HOST_ADDR" ROMFS/px4fmu_common/init.d-posix/rcS`. If it's
  missing, update the repo.
