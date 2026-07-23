# RVPoint Development Notes

## WSL2 Setup

- `env/setup.sh` - Single entry point (Git Bash as Administrator)
- `env/wsl.sh` - Quick launcher to enter rvpoint WSL in current directory
- `env/linux/install.sh` - Linux environment installer (called by setup.sh)

### Quick Start

```bash
# From Git Bash (as Administrator)
./env/setup.sh

# Enter rvpoint WSL in current directory
./env/wsl.sh
```

Or inside WSL2 Ubuntu (after manual import):
```bash
sudo bash env/linux/install.sh
source env/activate.sh
```

## Verification

```bash
env/verify_container.sh
# or scripts/verify_container.sh
```