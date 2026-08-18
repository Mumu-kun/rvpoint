#!/bin/bash
set -e

HOOK_DIR=".git/hooks"
PRE_PUSH="${HOOK_DIR}/pre-push"

echo "Setting up git hooks in ${HOOK_DIR}..."

if [ ! -d ".git" ]; then
    echo "Error: .git directory not found. Are you in the root of the repo?"
    exit 1
fi

mkdir -p "${HOOK_DIR}"

cat > "${PRE_PUSH}" << 'EOF'
#!/bin/bash
# Pre-push hook to run verification

echo "Running pre-push verification..."
PROJECT_ROOT=$(git rev-parse --show-toplevel)

# Run variable verification script
if [ -f "${PROJECT_ROOT}/scripts/verify_container.sh" ]; then
    bash "${PROJECT_ROOT}/scripts/verify_container.sh"
    RESULT=$?
    if [ $RESULT -ne 0 ]; then
        echo "❌ Verification failed! Push aborted."
        exit 1
    fi
else
    echo "⚠️ Warning: verify_container.sh not found. Skipping verification."
fi

echo "✅ Verification passed. Proceeding with push."
exit 0
EOF

chmod +x "${PRE_PUSH}"
echo "Git pre-push hook installed successfully!"
