# Wasp Destroyer 9000

Host-side software for an ELEGOO Conqueror tank that streams camera video to a
laptop and detects wasp nests with the Roboflow `wasps-nest/1` model.

**Status:** live viewer with local wasp-nest detection works. Robot control
comes next; movement stays disabled until the command channel is tested.
Hardware and protocol notes: [docs/hardware.md](docs/hardware.md).

## Setup

Requires **Python 3.10–3.13** (Roboflow's SDK does not support 3.14 yet).

### macOS
```bash
brew install python@3.12
python3.12 -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"
cp .env.example .env        # then paste your Roboflow API key into .env
```

### Windows (PowerShell)
```powershell
py -3.12 -m venv .venv
.venv\Scripts\Activate.ps1
pip install -e ".[dev]"
copy .env.example .env      # then paste your Roboflow API key into .env
```

Get an API key at roboflow.com → Settings → API Keys. `.env` is gitignored —
never commit it.

## Live viewer

1. Start the local inference server (see below) — once per boot; it auto-restarts after that.
2. Power on the tank and join its Wi-Fi (`ELEGOO-68EE8F55EBF8`, no password).
3. Run:

```bash
python -m wd9000.viewer
```

| Key | Action |
|---|---|
| `q` / `Esc` | quit |
| `d` | pause/resume detection |
| `s` | save annotated snapshot to `output/` |

Develop without the robot:

```bash
python -m wd9000.viewer --source samples/paper_wasp_nest.jpg   # still image as fake stream
python -m wd9000.viewer --source my_clip.mp4                   # video file (loops)
python -m wd9000.viewer --source 0 --backend mock              # laptop webcam, fake detections
```

The HUD shows camera FPS, inference FPS, end-to-end detection latency and any
connection errors. The stream reconnects automatically if the robot drops out.

## Web app (camera + spray test)

A local web page with two tabs: **Camera** (live feed with nest-detection boxes)
and **Spray test** (point-and-click servo testing for the spray actuator).

```bash
python -m wd9000.web                  # then open http://localhost:8000
python -m wd9000.web --no-detect      # camera without Docker/detection
```

It only accepts connections from the Mac it runs on. The Spray test tab needs
`firmware/servo_spray_test` on the UNO (see [firmware/README.md](firmware/README.md)).
Try it without the robot: run `python -m wd9000.robot.sim` in another terminal, then
`python -m wd9000.web --source samples/paper_wasp_nest.jpg --backend mock --robot 127.0.0.1:10100`.

## Robot command channel (bench test)

`wd9000.robot` talks to the ESP32's factory command server (TCP port 100,
JSON). The kit's ESP32 source has a 1 Hz heartbeat failsafe, but our robot's
ESP32 doesn't send heartbeats (see [docs/hardware.md](docs/hardware.md)), so
safety timeouts live in the UNO firmware.
Drive commands are refused unless `robot.motion_enabled: true` in
`config/settings.yaml` (default `false`).

Bench test with just the boards (no motors or servos needed). Close the ELEGOO
phone app first; the ESP32 accepts one command client at a time:

```bash
python scripts/robot_bench.py              # heartbeat, LED cycle with timing, ultrasonic read, stop
python scripts/robot_bench.py --failsafe   # also checks that the ESP32 drops a hung client
```

Without hardware, run the simulator in a second terminal and point the bench
test at it:

```bash
python -m wd9000.robot.sim                 # listens on 127.0.0.1:10100
python scripts/robot_bench.py --host 127.0.0.1 --port 10100 --failsafe
```

## Spray actuator test (over Wi-Fi)

Flash `firmware/servo_spray_test` to the UNO, set the shield switch to **cam**,
join the robot's Wi-Fi, then use the web app's **Spray test** tab (below) or
the terminal console:

```bash
python scripts/spray_test.py
```

Servo 1 is on the left, servo 2 on the right. Angles: 90 = rest (level), 0 = pressing;
the firmware mirrors servo 2. Wiring, power and the full walkthrough are in
[firmware/README.md](firmware/README.md). `robot_bench.py` needs the factory
firmware, so restore it before running that.

## Run inference on an image

```bash
python scripts/infer_image.py samples/paper_wasp_nest.jpg            # uses settings.yaml backend
python scripts/infer_image.py samples/paper_wasp_nest.jpg --show     # also open a window
python scripts/infer_image.py samples/paper_wasp_nest.jpg --raw      # print raw JSON
python scripts/infer_image.py samples/paper_wasp_nest.jpg --backend mock
```

Annotated images are written to `output/`.

The model has two classes: `nests` and `wasps`. It does best on whole nests
seen from a distance; close-ups of nest cells are mostly missed.

## Inference backends

Set `inference.backend` in [config/settings.yaml](config/settings.yaml), or
pass `--backend`:

| Backend | Needs | Notes |
|---|---|---|
| `local` (default) | Docker + API key | Roboflow Inference server on `localhost:9001`. ~80 ms/frame on an M-series Mac. |
| `cloud` | internet + API key | Roboflow hosted API. First call after idle can take ~30–50 s (cold start); later calls ~0.5 s. |
| `mock`  | nothing | Random fake detections for UI/state-machine development. |

### Local server (Docker)

Install and open Docker Desktop, then once:

```bash
docker run -d --name wd9000-inference --restart unless-stopped \
  -p 9001:9001 -v wd9000-inference-cache:/tmp/cache \
  -e AUTO_LOADER_CACHE_EXPIRATION_MINUTES=5256000 \
  roboflow/roboflow-inference-server-cpu:latest
```

(On Windows PowerShell put it on one line, without the `\`.)

While still on normal internet, run one inference so the model weights are
downloaded into the `wd9000-inference-cache` volume:

```bash
python scripts/infer_image.py samples/paper_wasp_nest.jpg --backend local
```

After that it works offline, e.g. on the robot's Wi-Fi.
`AUTO_LOADER_CACHE_EXPIRATION_MINUTES` (10 years) matters for that: by default the
server forgets which cached files belong to the model after 24 hours, then
asks Roboflow again. With no internet, every frame then fails with
`500 ... /model/add` (`RetryError: Connectivity error` in
`docker logs wd9000-inference`). If you made the container without it, recreate it
(the downloaded model is kept in the volume):

```bash
docker rm -f wd9000-inference
# then the docker run command above, while on normal internet,
# then the infer_image.py command above
```

Check the server with `curl http://localhost:9001/info`. Stop it with
`docker stop wd9000-inference`; start it again with `docker start wd9000-inference`.

## Tests

```bash
pytest
```

Tests run offline and need no hardware or API key.

## Sample images

`samples/` contains photos from Wikimedia Commons, resized to 1280 px wide:

- `paper_wasp_nest.jpg`: ["European paper wasp (nest)"](https://commons.wikimedia.org/wiki/File:European_paper_wasp_(nest).jpg)
  by Tdolphin~plwiki, [CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0)
- `umbrella_wasp_nest.jpg`: ["Umbrella Paper Wasp (Polistes sp.) Nest - Guelph, Ontario 2013-11-06 (01)"](https://commons.wikimedia.org/wiki/File:Umbrella_Paper_Wasp_(Polistes_sp.)_Nest_-_Guelph,_Ontario_2013-11-06_(01).jpg)
  by Ryan Hodnett, [CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0)
