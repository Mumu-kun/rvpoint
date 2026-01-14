# Collaboration Guide

This document outlines the workflow for the 5-person development team.

## Team Structure

- **Team Size:** 5 members
- **Primary Branch:** `dev`
- **Development Environment:** VS Code Dev Container (mandatory for consistency)

## Development Workflow

### Initial Setup (One-time per team member)

1. **Install Prerequisites:**
   - [Docker Desktop](https://docs.docker.com/get-docker/)
   - [Visual Studio Code](https://code.visualstudio.com/)
   - [Dev Containers Extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

2. **Clone Repository:**
   ```bash
   git clone <repository-url>
   cd rvpoint
   ```

3. **Configure Git:**
   ```bash
   git config user.name "Your Name"
   git config user.email "your.email@university.edu"
   ```

4. **Checkout Dev Branch:**
   ```bash
   git checkout dev
   ```

5. **Open in Dev Container:**
   ```bash
   code .
   ```
   - Click "Reopen in Container" when prompted
   - Wait for initial build (~5-10 minutes)

### Daily Workflow

#### 1. Start Your Day

```bash
# Make sure you're on dev branch
git checkout dev

# Get latest changes from team
git pull origin dev

# Create your feature branch
git checkout -b feature/your-feature-name
```

#### 2. During Development

**Build and test frequently:**
```bash
# Native build (quick testing)
./scripts/build.sh

# RISC-V build
./scripts/build.sh --riscv

# With RVV
./scripts/build.sh --riscv --rvv

# Run tests
cd build && ctest --output-on-failure
```

**Commit small, logical changes:**
```bash
git add <files>
git commit -m "Brief description of what you changed"
```

#### 3. End of Day / Feature Complete

```bash
# Push your branch
git push origin feature/your-feature-name
```

#### 4. Merging to Dev

**Option A: Direct Merge (for small changes)**
```bash
git checkout dev
git pull origin dev
git merge feature/your-feature-name
git push origin dev
```

**Option B: Code Review (recommended for larger changes)**
- Let another team member review your code
- Discuss in team chat before merging
- One person merges after approval

### Branch Naming Conventions

- `feature/add-kdtree` - New features
- `fix/segfault-in-filter` - Bug fixes
- `refactor/point-cloud-api` - Code refactoring
- `docs/update-build-guide` - Documentation updates
- `test/add-rvv-tests` - Adding tests

### Commit Message Guidelines

Good commit messages:
```
Add passthrough filter implementation

- Implement scalar version in include/filters/passthrough.hpp
- Add unit tests in tests/test_passthrough.cpp
- Update CMakeLists.txt to build new files
```

Bad commit messages:
```
fixed stuff
WIP
asdfasdf
```

**Format:**
```
Brief summary (50 chars or less)

Detailed explanation if needed (wrap at 72 chars):
- What changed
- Why it changed
- Any caveats or notes
```

## Code Organization

### Directory Responsibilities

Each team member can "own" certain directories/features:

```
include/
├── core/           # Point types, containers (Member 1)
├── filters/        # Filtering algorithms (Member 2)
├── search/         # KD-tree, nearest neighbor (Member 3)
├── segmentation/   # Segmentation algorithms (Member 4)
└── backend/        # Scalar/RVV implementations (Member 5)

tests/
├── test_core.cpp       # Member 1
├── test_filters.cpp    # Member 2
├── test_search.cpp     # Member 3
├── test_segmentation.cpp # Member 4
└── test_backend.cpp    # Member 5
```

**But:** Everyone should feel free to contribute anywhere!

## Communication

### Before You Start Coding

1. **Check existing branches:**
   ```bash
   git fetch
   git branch -a
   ```

2. **Coordinate in team chat:**
   - "I'm working on the passthrough filter"
   - "Does anyone need the Point3D structure changed?"

### During Development

- **Ask questions in team chat** if stuck
- **Share progress** at end of day
- **Report blocking issues** immediately

### Before Merging to Dev

- **Test your code:**
  ```bash
  ./scripts/build.sh --riscv --rvv
  cd build && ctest
  ```

- **Check for conflicts:**
  ```bash
  git checkout dev
  git pull
  git checkout feature/your-branch
  git merge dev  # Resolve conflicts if any
  ```

- **Announce in chat:**
  - "Merging passthrough filter to dev"

## Conflict Resolution

### If Git Merge Conflicts Occur

1. **Don't panic!**

2. **Update your branch:**
   ```bash
   git checkout feature/your-branch
   git fetch origin
   git merge origin/dev
   ```

3. **Git will show conflicts:**
   ```
   Auto-merging include/point_cloud.hpp
   CONFLICT (content): Merge conflict in include/point_cloud.hpp
   ```

4. **Open conflicted files in VS Code:**
   - VS Code highlights conflicts
   - Choose "Accept Current Change", "Accept Incoming", or "Accept Both"
   - Or manually edit

5. **After resolving:**
   ```bash
   git add <resolved-files>
   git commit -m "Merge dev into feature/your-branch, resolve conflicts"
   ```

6. **Ask for help if stuck!**

## Code Review (Optional but Recommended)

### Quick Review Checklist

Before merging, another team member should check:

- [ ] Code compiles without warnings
- [ ] Tests pass
- [ ] Code follows existing style
- [ ] Includes tests for new features
- [ ] Comments explain non-obvious logic
- [ ] No debug `printf` statements left in

### How to Review

```bash
# Check out teammate's branch
git fetch
git checkout feature/their-branch

# Build and test
./scripts/build.sh --riscv
cd build && ctest

# Read the code
# Give feedback in chat or add comments
```

## Testing Strategy

### Everyone Should Test

**Before pushing:**
```bash
# Native build (fast)
./scripts/build.sh
cd build && ctest

# RISC-V build (thorough)
./scripts/build.sh --riscv --clean
cd build && ctest --verbose
```

### Automated Testing

- We removed CI/CD for now to focus on development
- Later, we can add GitHub Actions back for automatic testing

### Manual Testing

For complex features, test manually:
```bash
# Build your example
cd build/examples
./your_example

# Try different inputs
./your_example --config test1.cfg
```

## Development Environment Tips

### Container Tips

**Check if you're in container:**
```bash
echo $CONTAINER_NAME  # Should show something
```

**Rebuild container after Dockerfile changes:**
- Command Palette (F1) → "Dev Containers: Rebuild Container"

**Container takes too long to build:**
- First build is slow (~10 min)
- Subsequent builds are cached and faster
- Don't rebuild unless necessary

### VS Code Tips

**Useful shortcuts:**
- `Ctrl+Shift+P` (F1) - Command Palette
- `Ctrl+`` - Toggle terminal
- `Ctrl+P` - Quick file open
- `Ctrl+Shift+F` - Search across files

**CMake integration:**
- Status bar shows build target
- Click to configure/build
- Or use Command Palette → "CMake: Build"

### Git Tips

**See what changed:**
```bash
git status          # Files you modified
git diff            # Changes not yet staged
git diff --staged   # Changes staged for commit
```

**Undo mistakes:**
```bash
git checkout -- file.cpp    # Discard changes to file
git reset HEAD file.cpp     # Unstage file
git reset --hard origin/dev # DANGER: Reset to remote (loses local changes)
```

**See commit history:**
```bash
git log --oneline --graph --all
```

Or use Git Graph extension in VS Code!

## Common Issues

### "Build fails in container but works on my machine"

- You're probably not using the dev container
- Make sure you "Reopened in Container"

### "Can't push to dev branch"

- Someone else pushed before you
- Pull first: `git pull origin dev`
- Then push: `git push origin dev`

### "Container out of disk space"

```bash
# Clean up Docker
docker system prune -a

# Remove old build artifacts
rm -rf build
```

### "QEMU tests are slow"

- This is normal! Emulation is slower than native
- Use native builds for quick iteration
- Only test with QEMU before pushing to dev

### "Merge conflicts everywhere!"

- Communicate before making large changes
- Pull from dev frequently
- Ask team lead to help resolve

## Project Milestones

Keep track of team progress:

### Week 1-2: Foundation
- [ ] Core data structures (Point, PointCloud)
- [ ] Build system working for everyone
- [ ] Basic tests passing

### Week 3-4: Scalar Implementations
- [ ] Passthrough filter
- [ ] Statistical outlier removal
- [ ] Basic KD-tree
- [ ] Comprehensive tests

### Week 5-6: RVV Vectorization
- [ ] RVV backend implementation
- [ ] Vectorized filters
- [ ] Performance benchmarks

### Week 7-8: Advanced Features
- [ ] Complete KD-tree with search
- [ ] Segmentation algorithms
- [ ] Custom instructions (if time permits)

### Week 9-10: Polish
- [ ] Documentation
- [ ] Example programs
- [ ] Final performance testing
- [ ] Presentation preparation

## Getting Help

### Team Communication

- **Quick questions:** Team chat
- **Code review:** Share branch name in chat
- **Blocked on something:** Ask immediately, don't wait
- **Design decisions:** Discuss as a team

### External Resources

- **RISC-V Spec:** https://riscv.org/technical/specifications/
- **RVV Spec:** https://github.com/riscv/riscv-v-spec
- **CMake Docs:** https://cmake.org/documentation/
- **Google Test:** https://google.github.io/googletest/

### When Totally Stuck

1. Check `.devcontainer/README.md`
2. Check `docs/BUILD.md`
3. Ask team in chat
4. Search online (Stack Overflow, GitHub Issues)
5. Ask instructor/TA

## Remember

- 🤝 **Communication is key** - Keep team updated
- 🧪 **Test before pushing** - Don't break dev branch
- 📝 **Document as you go** - Future you will thank you
- 🚀 **Small commits frequently** - Easier to review and debug
- 💡 **Ask questions** - No question is stupid

Good luck team! 🎉
