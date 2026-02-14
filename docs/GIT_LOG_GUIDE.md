# Git Historian Log & Time-Machine Guide

**The PhD Audit Trail: Development History & Version Control**

---

## 📜 The PhD Journey: Stage-by-Stage Development

This project followed a systematic, risk-mitigated development approach across three major stages:

### Stage 1: Digital Twin Foundation (Hover & Basic Navigation)
**Timeline:** Week 1-2  
**Objective:** Establish simulation environment and validate basic flight control

**Key Milestones:**
- ✅ Docker environment setup (hexacopter_lab_safe_copy image)
- ✅ Gazebo + PX4 SITL integration
- ✅ ROS 2 Humble workspace configuration
- ✅ Level 1 Hover Test: ±0.06m altitude stability
- ✅ Level 2 Box Mission: 4-corner waypoint navigation

**Commits (Example):**
```bash
git commit -m "feat: establish Gazebo simulation environment"
git commit -m "feat: implement Level 1 hover test with altitude hold"
git commit -m "fix: resolve GUI rendering with LIBGL_ALWAYS_SOFTWARE"
git commit -m "feat: validate Level 2 box mission waypoint navigation"
```

---

### Stage 2: Thermal AI Integration (Disease Detection)
**Timeline:** Week 3-4  
**Objective:** Integrate thermal sensing and edge AI disease detection

**Key Milestones:**
- ✅ Thermal camera sensor plugin (Gazebo SDF)
- ✅ MobileNetV2 disease detection node (thermal_monitor.py)
- ✅ ROS 2 topic integration (/agri/thermal/image_raw → /agri/crop_health/alerts)
- ✅ Simultaneous survey + thermal monitoring validation
- ✅ Performance benchmarking: 91.9% F1-score, 45ms latency

**Commits (Example):**
```bash
git commit -m "feat: add thermal camera sensor to hexacopter SDF model"
git commit -m "feat: implement MobileNetV2 thermal disease detection"
git commit -m "fix: register thermal_monitor in setup.py entry_points"
git commit -m "test: validate simultaneous survey and AI monitoring"
```

---

### Stage 3: Bihar Localization & Academic Finalization
**Timeline:** Week 5-6  
**Objective:** High-fidelity Bihar world integration and PhD documentation

**Key Milestones:**
- ✅ Bihar maize farm world (bihar_maize.sdf with GPS coordinates)
- ✅ visual_hexacopter_bihar.sh launch script
- ✅ Level 3 Survey Mission: 7/7 waypoints, 100% success
- ✅ Literature review: 3,012 words, 50+ citations
- ✅ Defense presentation: 15 slides with CEO speech
- ✅ Audit master script: Digital CEO system health verification

**Commits (Example):**
```bash
git commit -m "feat: create Bihar maize farm world with realistic crop spacing"
git commit -m "feat: implement visual_hexacopter_bihar.sh launch script"
git commit -m "docs: complete 3,000-word literature review with LaTeX equations"
git commit -m "docs: finalize 15-slide defense presentation with CEO speech"
git commit -m "chore: reorganize to world-class project structure"
```

---

## 🎯 Conventional Commits Standard

This project follows the **Conventional Commits** specification used by Google, Tesla, and major open-source projects.

### Commit Format

```
<type>(<scope>): <subject>

<body>

<footer>
```

### Commit Types

| Type | Purpose | When to Use | Example |
|------|---------|-------------|---------|
| **feat** | New feature | Adding functionality | `feat: integrate MobileNetV2 thermal detection` |
| **fix** | Bug fix | Resolving errors | `fix: resolve hexacopter flipping via VTOL mixer` |
| **docs** | Documentation | Writing/updating docs | `docs: complete 3,000-word literature review` |
| **test** | Testing | Adding/updating tests | `test: validate Level 3 survey mission` |
| **refactor** | Code restructuring | Improving code without changing behavior | `refactor: extract waypoint generation to utility` |
| **perf** | Performance improvement | Optimizing code | `perf: reduce AI inference latency to 45ms` |
| **chore** | Maintenance | Tooling, dependencies, cleanup | `chore: reorganize to standard folder structure` |
| **style** | Code style | Formatting, whitespace | `style: apply PEP8 to thermal_monitor.py` |
| **ci** | CI/CD changes | GitHub Actions, Docker | `ci: add automated build pipeline` |
| **build** | Build system | Dependencies, build config | `build: update ROS 2 package dependencies` |

### Scope (Optional)

Indicates the component affected:

- `(simulation)`: Gazebo, PX4 SITL
- `(ai)`: Thermal detection, MobileNetV2
- `(flight)`: Navigation, mission planning
- `(docs)`: Documentation files
- `(hardware)`: Physical hexacopter configuration

**Examples:**
```bash
git commit -m "feat(simulation): add Bihar maize farm world"
git commit -m "fix(ai): correct hotspot threshold calculation"
git commit -m "docs(defense): add CEO speech to presentation"
```

---

## 🕰️ The Time-Machine Guide: Git Checkout

Git allows you to "travel back in time" to any previous version of your research. This is critical for:

1. **Reproducing results** for peer review
2. **Debugging regressions** (when something breaks)
3. **Comparing approaches** (e.g., quadcopter vs. hexacopter)
4. **Demonstrating progress** to your supervisor

### Basic Time Travel Commands

**1. View Project History**
```bash
# See all commits with messages
git log --oneline --graph --all

# See detailed commit history
git log --stat

# See commits for specific file
git log -- docs/lit_review.md
```

**2. Travel to Specific Commit**
```bash
# View commit hash (first 7 characters)
git log --oneline

# Example output:
# a1b2c3d docs: finalize defense presentation
# e4f5g6h feat: integrate Bihar world
# i7j8k9l fix: resolve GUI rendering

# Travel to that commit (read-only)
git checkout e4f5g6h

# Return to latest version
git checkout main
```

**3. Create Branch from Past Commit**
```bash
# Create new branch from specific commit
git checkout -b experiment-quadcopter e4f5g6h

# Make changes, test, then compare
git diff main experiment-quadcopter
```

**4. Compare Two Versions**
```bash
# See what changed between commits
git diff e4f5g6h a1b2c3d

# See what changed in specific file
git diff e4f5g6h a1b2c3d -- workspace/src/agri_hexacopter/agri_hexacopter/thermal_monitor.py
```

---

## 📊 PhD Audit Trail: Key Commits

Here are the critical commits that your supervisor should review:

### 1. Foundation Commit
```bash
git log --grep="establish Gazebo simulation"
```
**What to Show:** Docker setup, PX4 SITL integration, first successful hover

### 2. Flight Validation Commit
```bash
git log --grep="validate Level 3 survey"
```
**What to Show:** 100% mission success, telemetry graphs, waypoint completion

### 3. AI Integration Commit
```bash
git log --grep="implement MobileNetV2"
```
**What to Show:** thermal_monitor.py code, 91.9% F1-score, 45ms latency

### 4. Bihar Localization Commit
```bash
git log --grep="Bihar maize farm"
```
**What to Show:** bihar_maize.sdf world, GPS coordinates, realistic crop spacing

### 5. Academic Finalization Commit
```bash
git log --grep="finalize defense presentation"
```
**What to Show:** Complete documentation suite, CEO speech, Q&A preparation

---

## 🔍 Advanced Git Techniques for PhD Research

### 1. Tagging Important Milestones

Use tags to mark significant achievements:

```bash
# Tag Level 1 validation
git tag -a v1.0-hover -m "Level 1: Hover validation complete"

# Tag AI integration
git tag -a v2.0-thermal-ai -m "Stage 2: Thermal AI integration complete"

# Tag defense-ready version
git tag -a v3.0-defense-ready -m "PhD defense ready: all documentation complete"

# List all tags
git tag -l

# Checkout specific tag
git checkout v2.0-thermal-ai
```

### 2. Stashing Work-in-Progress

Save uncommitted changes temporarily:

```bash
# Save current work
git stash save "WIP: testing new PID gains"

# List stashed changes
git stash list

# Restore stashed work
git stash pop
```

### 3. Cherry-Picking Commits

Apply specific commit to current branch:

```bash
# Apply commit from another branch
git cherry-pick a1b2c3d
```

### 4. Bisect for Debugging

Find which commit introduced a bug:

```bash
# Start bisect
git bisect start

# Mark current version as bad
git bisect bad

# Mark known good version
git bisect good v1.0-hover

# Git will checkout middle commit, test it
# Then mark as good or bad
git bisect good  # or git bisect bad

# Repeat until bug is found
git bisect reset  # when done
```

---

## 📁 Project Structure & Git Organization

### Recommended .gitignore

Create `.gitignore` to exclude unnecessary files:

```gitignore
# Build artifacts
ros2_ws/build/
ros2_ws/install/
ros2_ws/log/
*.pyc
__pycache__/

# IDE files
.vscode/
.idea/
*.swp

# Large binary files (use Git LFS)
*.bag
*.mp4
*.avi

# Temporary files
*.tmp
*.log
```

### Git LFS for Large Files

For large datasets (thermal images, flight logs):

```bash
# Install Git LFS
git lfs install

# Track large files
git lfs track "*.bag"
git lfs track "*.mp4"

# Commit .gitattributes
git add .gitattributes
git commit -m "chore: configure Git LFS for large files"
```

---

## 🎓 Supervisor Presentation: Git Demonstration

When presenting to your supervisor, demonstrate version control:

**1. Show Project Evolution**
```bash
# Visual timeline
git log --oneline --graph --all --decorate
```

**2. Show Code Changes**
```bash
# Compare first version vs. final
git diff v1.0-hover v3.0-defense-ready -- ros2_ws/src/agri_hexacopter/agri_hexacopter/thermal_monitor.py
```

**3. Show Reproducibility**
```bash
# Checkout defense-ready version
git checkout v3.0-defense-ready

# Run validation
python3 scripts/audit_master.py
```

---

## 🚀 Post-Defense: Publishing Your Research

### 1. Create GitHub Repository

```bash
# Add remote repository
git remote add origin https://github.com/yourusername/thermal-hexacopter-phd.git

# Push all branches and tags
git push -u origin main
git push --tags
```

### 2. Create Release

On GitHub, create a release for your defense version:

- Tag: `v3.0-defense-ready`
- Title: "PhD Defense Version: Autonomous Thermal-Imaging Hexacopter"
- Description: Include EXECUTIVE_SUMMARY.md content
- Attach: Presentation slides, walkthrough video

### 3. Archive on Zenodo

For permanent DOI (Digital Object Identifier):

1. Link GitHub repository to Zenodo
2. Create release on GitHub
3. Zenodo automatically archives and assigns DOI
4. Include DOI in publications

---

## 📋 Git Workflow Checklist

### Daily Development

- [ ] Pull latest changes: `git pull origin main`
- [ ] Create feature branch: `git checkout -b feat/new-feature`
- [ ] Make changes and test
- [ ] Stage changes: `git add <files>`
- [ ] Commit with conventional message: `git commit -m "feat: description"`
- [ ] Push to remote: `git push origin feat/new-feature`

### Before Defense

- [ ] Ensure all changes committed
- [ ] Tag defense version: `git tag -a v3.0-defense-ready -m "Defense ready"`
- [ ] Push tags: `git push --tags`
- [ ] Create backup: `git bundle create phd-backup.bundle --all`
- [ ] Test checkout: `git checkout v3.0-defense-ready && python3 scripts/audit_master.py`

### After Defense

- [ ] Tag final version: `git tag -a v4.0-final -m "Post-defense final version"`
- [ ] Create GitHub release
- [ ] Archive on Zenodo
- [ ] Update README with DOI badge

---

## 🔗 Useful Git Resources

- **Official Documentation:** https://git-scm.com/doc
- **Conventional Commits:** https://www.conventionalcommits.org/
- **Git LFS:** https://git-lfs.github.com/
- **GitHub Guides:** https://guides.github.com/
- **Zenodo Integration:** https://guides.github.com/activities/citable-code/

---

## 💡 Pro Tips for PhD Students

1. **Commit Often:** Small, frequent commits are better than large, infrequent ones
2. **Write Descriptive Messages:** Your future self will thank you
3. **Use Branches:** Experiment without breaking main branch
4. **Tag Milestones:** Easy reference for supervisor meetings
5. **Backup Regularly:** `git bundle` creates offline backups
6. **Document Everything:** Commit messages are part of your research narrative

---

**Remember:** Git is not just version control—it's your PhD research diary. Every commit tells the story of your journey from "Hello World" to "Ready for Defense."

**Status:** ✅ **Git Historian Log Complete**  
**Last Updated:** February 15, 2026  
**Version:** 1.0 (Defense-Ready)
