# 🧠 01 — AI Brain

**MobileNetV2 thermal inference engine, training pipelines, and model artifacts.**

## Key Stats
- **Architecture:** MobileNetV2 (edge-optimized, TensorFlow)
- **Accuracy:** 91.9% F1-Score
- **Latency:** 45ms on Raspberry Pi 4 (8GB)
- **Target:** Early-stage crop thermal anomaly detection

## Files

| File | Description |
|------|-------------|
| [`../ai_models/thermal_monitor.py`](../ai_models/thermal_monitor.py) | Live inference node |

## Usage

```bash
# Run thermal monitoring inference
python3 ai_models/thermal_monitor.py --input /dev/thermal0 --model models/mobilenetv2_thermal.h5
```

> Training data, augmented datasets, and model checkpoints to be stored here.
