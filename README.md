# victron-ve-direct

Reads a Victron Energy device over the VE.Direct serial-text protocol (USB-to-serial dongle) and broadcasts one JSON telemetry line per frame over TCP.

## Supported devices

- **SmartSolar MPPT** — full range (75|10 through 250|100)
- **BlueSolar MPPT** — full range (75|10 through 150|100)
- **BMV battery monitors** — BMV-700, BMV-702, BMV-700H, BMV-712 Smart
- **Phoenix Inverter** — 12/24/48V, 250 VA through 1200 VA
- Any other VE.Direct device — raw fields are still emitted

## Network interface

| Port | Protocol | Description |
|------|----------|-------------|
| 8563 | TCP | Data stream — one newline-delimited JSON line per frame |
| 8562 | TCP | Status query — send `status\n`, get a key=value reply |

```bash
# Stream live telemetry
nc 127.0.0.1 8563

# Query status
echo status | nc 127.0.0.1 8562
```

## JSON output

One object per VE.Direct frame, emitted on stdout and broadcast to all connected TCP clients.

```json
{
  "ts_us": 1751200000000000,
  "frame": 42,
  "device": {
    "pid": "0xA067",
    "name": "SmartSolar MPPT 100|50",
    "serial": "HQ2241A3JKL",
    "fw": "161"
  },
  "V": 12.540,
  "I": 0.150,
  "VPV": 18.200,
  "PPV": 4,
  "CS": 5,
  "CS_name": "Float",
  "MPPT": 2,
  "MPPT_name": "MPP tracker active",
  "ERR": 0,
  "ERR_name": "No error",
  "LOAD": "ON",
  "IL": 0.150,
  "H19": 123.45,
  "H20": 0.12,
  "H21": 45,
  "H22": 0.98,
  "H23": 52,
  "HSDS": 5
}
```

All voltages are in V, currents in A, power in W, energy in kWh. Enum fields (`CS`, `MPPT`, `ERR`) include a `_name` companion string. Any fields not recognised by the formatter are collected into a `"raw"` object so no data is silently dropped.

## Discovery (mDNS/DNS-SD)

On startup the process announces both TCP ports on the LAN via Avahi, so you don't need to know its IP:

```bash
avahi-browse -rt _victron-data._tcp
avahi-browse -rt _victron-status._tcp
```

This needs `avahi-daemon` running on the Pi (installed and enabled by default on Raspberry Pi OS). If it isn't running, `victron_ve_direct` logs a warning at startup and continues normally — mDNS is discovery-only, not required for the TCP protocol to work. Disable it or change the advertised name via the `[mdns]` section in `config.ini`.

## Build

Requires a C++20 compiler, CMake 3.16+, `pkg-config`, and Avahi client development headers (`libavahi-client-dev` on Debian/Raspberry Pi OS).

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

Binary: `build/victron_ve_direct`

## Run

```bash
./build/victron_ve_direct [--config config.ini]
```

The default config path is `config.ini` in the working directory. Copy and edit before running:

```bash
cp config.ini my-config.ini
# edit my-config.ini: set device.port to your serial adapter
./build/victron_ve_direct --config my-config.ini
```

## Configuration

`config.ini`:

```ini
[device]
port        = /dev/ttyUSB0   ; or /dev/ttyACM0, /dev/victron-serial
baud        = 19200           ; VE.Direct is always 19200 8N1
retry_secs  = 5              ; wait before reconnect after disconnect

[output]
ctrl_port   = 8562
data_port   = 8563

[mdns]
enabled     = true                ; announce _victron-data._tcp / _victron-status._tcp via avahi-daemon
name        = victron-ve-direct   ; service instance name shown in discovery
```

The VE.Direct protocol is always 19200 8N1 — do not change the baud rate.

## Debian package

Build the `.deb` on the target machine (e.g. Raspberry Pi):

```bash
cd victron-ve-direct
dpkg-buildpackage -us -uc -b
```

The package lands one level up: `../victron-ve-direct_1.0.0+..._armhf.deb`

Install:

```bash
sudo dpkg -i ../victron-ve-direct_*.deb
```

The package:
- Creates a dedicated `victron-ve-direct` system user with `dialout` group membership
- Installs config to `/etc/victron-ve-direct/config.ini`
- Installs udev rules that create a stable `/dev/victron-serial` symlink for Victron FTDI (VID 0403:6001) and SiLabs CP210x (VID 10c4:ea60) adapters
- Registers and starts a systemd service

### From the APT repository

CI publishes to a signed APT repository (shared with other aipicam Raspberry Pi packages) hosted on Cloudflare R2, with two channels:

- **`main`** — pushing a `v*` tag publishes the clean release version here.
- **`nightly`** — every push (to any branch, and PRs) publishes a dev build here, versioned with a `+<UTC timestamp>-1` suffix.

```bash
curl -fsSL https://repo.aipicam.com/pubkey.asc | sudo gpg --dearmor -o /usr/share/keyrings/aipicam.gpg

# stable releases
echo "deb [signed-by=/usr/share/keyrings/aipicam.gpg] https://repo.aipicam.com main main" | sudo tee /etc/apt/sources.list.d/aipicam.list

# or nightly builds instead
echo "deb [signed-by=/usr/share/keyrings/aipicam.gpg] https://repo.aipicam.com nightly main" | sudo tee /etc/apt/sources.list.d/aipicam.list

sudo apt-get update
sudo apt-get install victron-ve-direct
```

Builds run on GitHub's native `ubuntu-24.04-arm` hosted runner (no QEMU). Uses the same `R2_ACCOUNT_ID`, `R2_ACCESS_KEY_ID`, `R2_SECRET_ACCESS_KEY`, `GPG_PRIVATE_KEY`, and `GPG_KEY_ID` repo secrets described in [pi-block-cpu-cores](../pi-block-cpu-cores)'s README, since it publishes into the same shared repo.

## systemd service

```bash
# Enable and start
systemctl enable --now victron-ve-direct

# Check status
systemctl status victron-ve-direct

# Follow logs
journalctl -fu victron-ve-direct
```

Edit `/etc/victron-ve-direct/config.ini` and `systemctl restart victron-ve-direct` to apply changes.

If your Victron adapter is detected by the udev rules, you can use the stable symlink instead of a numbered device:

```ini
port = /dev/victron-serial
```
