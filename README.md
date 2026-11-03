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
  roboflow/roboflow-inference-server-cpu:latest
```

(On Windows PowerShell put it on one line, without the `\`.)

While still on normal internet, run one inference so the model weights are
downloaded into the `wd9000-inference-cache` volume:

```bash
python scripts/infer_image.py samples/paper_wasp_nest.jpg --backend local
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
