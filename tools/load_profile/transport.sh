#!/bin/bash
# Receive cost per image format: raw 30 Hz vs raw 10 Hz vs 320x240 vs JPEG (+ decode), one process each.
#   tools/load_profile/transport.sh <bag> <outdir> [seconds]
# Same command on the car gives the Orin numbers. Decides what the image hub publishes (0.2 / 4.2).
BAG=$1; OUT=$(realpath -m "$2"); SECS=${3:-15}
source "$(dirname "$0")/env.sh"
mkdir -p "$OUT"
python3 "$HERE/transport_pub.py" "$BAG" --seconds $((SECS + 8)) > "$OUT/pub.log" 2>&1 &
PUB=$!
for m in raw raw_bgr raw10 small jpeg jpeg_decode jpeg_decode_half; do
  python3 "$HERE/transport_sub.py" $m "$SECS" > "$OUT/$m.log" 2>&1 &
done
wait $PUB
wait
head -1 "$OUT/pub.log"
grep -h '^RESULT' "$OUT"/*.log | sed 's/^RESULT //' | python3 -c "
import json, sys
order = 'raw raw_bgr raw10 small jpeg jpeg_decode jpeg_decode_half'.split()
rows = {r['label']: r for r in map(json.loads, sys.stdin)}
print(f\"{'mode':18s} {'CPU % of a core':>16s} {'Hz':>6s}\")
for k in order:
    if k in rows:
        print(f\"{k:18s} {rows[k]['cpu_pct']:16.1f} {rows[k]['hz']:6.1f}\")"
