"""Composite the Webots 3D recording with the dashboard frames.

    python tools/compose_video.py <movie_3d.mp4> <frames_dir> <out.mp4> [frame_rate]
                                  [intro=<png>] [outro=<png>] ...

The dashboard frames are 1920x1080 with the top-left 1280x720 left for the
3D view. Their caption band (the bottom CAP_H px of that region) is laid over
the 3D view; where there is no caption the band is filled with magenta,
which is keyed out so the 3D view shows through. The 3D movie drives the
output timing, the dashboard regions are overlaid at their own rate, and
the optional cards are held for CARD_S seconds before / after.
Requires ffmpeg on PATH. Removes the intermediate files on success.
"""

import os
import shutil
import subprocess
import sys

CAP_H = 136
TOP_H = 46
CARD_S = {"intro": 9, "outro": 12}


def main():
    pos = [a for a in sys.argv[1:] if "=" not in a]
    cards = [a.split("=", 1) for a in sys.argv[1:] if "=" in a]
    movie3d, frames, out = pos[:3]
    rate = pos[3] if len(pos) > 3 else "1000/96"
    ff = shutil.which("ffmpeg")
    if not ff:
        print(f"ffmpeg not found: kept {movie3d} and {frames} for manual compositing")
        return 1
    y_cap = 720 - CAP_H
    inputs = ["-framerate", rate, "-i", os.path.join(frames, "%06d.jpg"), "-i", movie3d]
    filt = ("[1:v]scale=1280:720,pad=1920:1080:0:0:color=0x0d1117[base];"
            "[0:v]split=4[a][b][c][d];[a]crop=640:1080:1280:0[right];[b]crop=1280:360:0:720[bottom];"
            f"[d]crop=1280:{TOP_H}:0:0[top];"
            f"[c]crop=1280:{CAP_H}:0:{y_cap},colorkey=0xFF00FF:0.35:0.05[cap];"
            "[base][right]overlay=1280:0:eof_action=repeat[t1];"
            "[t1][bottom]overlay=0:720:eof_action=repeat[t2];"
            f"[t2][cap]overlay=0:{y_cap}:eof_action=repeat[t3];"
            "[t3][top]overlay=0:0:eof_action=repeat,fps=30,format=yuv420p,setsar=1[main]")
    intro = [p for k, p in cards if k == "intro"]
    outro = [p for k, p in cards if k == "outro"]
    seq = []
    idx = 2
    for kind, paths in (("intro", intro), ("outro", outro)):
        for p in paths:
            inputs += ["-loop", "1", "-framerate", "30", "-t", str(CARD_S[kind]), "-i", p]
            filt += (f";[{idx}:v]scale=1920:1080,fps=30,format=yuv420p,setsar=1,"
                     f"fade=in:st=0:d=0.5,fade=out:st={CARD_S[kind] - 0.5}:d=0.5[k{idx}]")
            seq.append((kind, f"[k{idx}]"))
            idx += 1
    order = [s for k, s in seq if k == "intro"] + ["[main]"] + [s for k, s in seq if k == "outro"]
    if len(order) > 1:
        filt += ";" + "".join(order) + f"concat=n={len(order)}:v=1:a=0[v]"
    else:
        filt += ";[main]null[v]"
    cmd = [ff, "-y", "-v", "error"] + inputs + [
        "-filter_complex", filt, "-map", "[v]", "-r", "30",
        "-c:v", "libx264", "-preset", "medium", "-crf", "20", "-movflags", "+faststart", out]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print("ffmpeg failed:\n" + r.stderr[-2000:])
        return r.returncode
    if os.environ.get("UAVX_KEEP_INTERMEDIATE") == "1":
        print(f"composited video written to {out} (intermediates kept)")
        return 0
    os.remove(movie3d)
    shutil.rmtree(frames, ignore_errors=True)
    for _, p in cards:
        try:
            os.remove(p)
        except OSError:
            pass
    print(f"composited video written to {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
