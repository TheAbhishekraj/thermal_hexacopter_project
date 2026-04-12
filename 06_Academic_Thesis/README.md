# 🎓 06 — Academic Thesis

This folder houses all **LaTeX source files, research methodology documents, and publication drafts** for the PhD thesis submission.

## Thesis Title
**"Autonomous Thermal-Imaging Hexacopter for Precision Agriculture: A Digital Twin Approach"**

**Author:** Abhishek Raj  
**Department:** School of Mechanical Engineering  
**Status:** ✅ Validated | Sealed: February 15, 2026 | Tag: `v4.0-final-thesis-seal`

## Folder Structure

```
06_Academic_Thesis/
├── latex/                  # LaTeX source (.tex files)
│   ├── main.tex            # Master thesis document
│   ├── chapters/           # Individual chapter files
│   └── figures/            # Thesis figures and plots
├── methodology/            # Methodology notes and diagrams
├── publications/           # Journal/conference paper drafts
└── README.md               # This file
```

## Key Reference Documents

| Document | Location |
|----------|----------|
| Full Literature Review | [`../docs/lit_review.md`](../docs/lit_review.md) |
| Defense Presentation Script | [`../docs/DEFENSE_PRESENTATION.md`](../docs/DEFENSE_PRESENTATION.md) |
| PhD Master Guide | [`../docs/PHD_MASTER_GUIDE.md`](../docs/PHD_MASTER_GUIDE.md) |
| Completion Certificate | [`../COMPLETION_CERTIFICATE.md`](../COMPLETION_CERTIFICATE.md) |
| Presentation Outline | [`../docs/presentation_outline.txt`](../docs/presentation_outline.txt) |

## Methodology Summary

| Phase | Approach | Output |
|-------|----------|--------|
| Phase 1 | SITL in Gazebo (Bihar Maize Farm) | Validated 7-waypoint mission |
| Phase 2 | MobileNetV2 edge AI training | 91.9% F1-Score, 45ms latency |
| Phase 3 | HITL → Physical maiden voyage | ±0.06m stability, 100% success |

## BibTeX Citation

```bibtex
@phdthesis{raj2026hexacopter,
  author    = {Abhishek Raj},
  title     = {Autonomous Thermal-Imaging Hexacopter for Precision Agriculture: A Digital Twin Approach},
  school    = {School of Mechanical Engineering},
  year      = {2026},
  note      = {Platform cost: ₹1,28,900 | AI F1-Score: 91.9\% | Inference: 45ms}
}
```
