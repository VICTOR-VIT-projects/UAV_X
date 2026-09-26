# UAV-X: installation and reproduction (exact, ordered steps)

Tested on Windows 11 with Python 3.10.11 and Webots R2025a.

## 1. Install the prerequisites

1. **Python 3.10+.** Install from <https://www.python.org/downloads/> and
   tick "Add python.exe to PATH".
2. **Webots R2025a** (open source, Apache 2.0). Install from
   <https://cyberbotics.com/#download>.
3. **ffmpeg** (needed only to record the video):
   `winget install Gyan.FFmpeg`

## 2. Install the Python packages

Open a terminal in the `uavx_sim` folder:

```powershell
cd "UAV_X FINAL\uavx_sim"
python -m pip install -r requirements.txt
```

`ultralytics` is optional. Without it, everything runs except onboard
person detection.

## 3. Verify the installation

```powershell
python -m unittest discover -s tests
```

Expected output: `Ran 65 tests ... OK` (takes about 1–3 minutes).

## 4. Reproduce the reported results (no Webots needed)

| Step | Command | Expected result |
|---|---|---|
| 1 | `python tools/run_headless.py` | Scripted demo scenario (seed 7). All constraints `true`. Imagery complete at 634.8 s. |
| 2 | `python tools/recompute_metrics.py results scenario` | `ALL PASS`: metrics re-derived independently from the raw logs. |
| 3 | `python tools/evaluate.py` | The benchmark on the saved scenarios (~15 min). Adaptive-spare passes 30/30 (dev) and 30/30 (held-out) on E1. Writes `results/evaluation.md`. |
| 4 | `python tools/make_report_xlsx.py` then `python tools/make_report_pdf.py` | Rebuilds the reports in `reports/`. |

## 5. Run the simulation in Webots

1. Open Webots.
2. Choose **File → Open World** and open
   `uavx_sim/worlds/uavx_stage3.wbt`.
3. Press **Play**. The GCS dashboard window opens alongside Webots.

On the first launch, Webots needs internet access to download its assets.

## 6. Record the demo video (optional)

Keep the Webots window visible. A minimised window records a black 3D view.

```powershell
$env:UAVX_RECORD_MOVIE="$PWD\media\uav_x.mp4"
& "$env:LOCALAPPDATA\Programs\Webots\msys64\mingw64\bin\webots.exe" --batch --mode=fast worlds/uavx_stage3.wbt
```
