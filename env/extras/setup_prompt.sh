#!/usr/bin/env bash
# env/extras/setup_prompt.sh
# ─────────────────────────────────────────────────────────────────────────────
# Installs a polished bash prompt (Tokyo Night palette) into ~/.bashrc.
# Run once inside WSL2:  bash env/extras/setup_prompt.sh
# ─────────────────────────────────────────────────────────────────────────────

MARKER="# >>> rvpoint prompt <<<"
BASHRC="$HOME/.bashrc"

# Remove any previous installation
if grep -q "$MARKER" "$BASHRC" 2>/dev/null; then
  sed -i "/$MARKER/,/# <<< rvpoint prompt >>>/d" "$BASHRC"
fi

cat >> "$BASHRC" << 'PROMPT_BLOCK'
# >>> rvpoint prompt <<<

# ── Tokyo Night palette (256-color approximations) ────────────────────────────
#   #f7768e  coral red       → 210
#   #9ece6a  lime green      → 149
#   #e0af68  warm amber      → 179
#   #7aa2f7  periwinkle blue → 111
#   #bb9af7  soft violet     → 141
#   #7dcfff  sky cyan        → 117
#   #565f89  muted slate     → 61   (decorators, separators)
#   #c8ccd4  soft white      → 252  (default text)

__c_reset='\[\e[0m\]'
__c_bold='\[\e[1m\]'
__c_dim='\[\e[2m\]'

__c_red='\[\e[38;5;210m\]'      # coral red    — root user / errors
__c_green='\[\e[38;5;149m\]'    # lime green   — normal user / pass
__c_amber='\[\e[38;5;179m\]'    # warm amber   — git branch
__c_blue='\[\e[38;5;111m\]'     # periwinkle   — path
__c_violet='\[\e[38;5;141m\]'   # soft violet  — hostname
__c_cyan='\[\e[38;5;117m\]'     # sky cyan     — venv
__c_slate='\[\e[38;5;61m\]'     # muted slate  — decorators / dim text
__c_white='\[\e[38;5;252m\]'    # soft white   — normal text

# ── Git branch + dirty flag ───────────────────────────────────────────────────
__prompt_git() {
  local branch
  branch=$(git symbolic-ref --short HEAD 2>/dev/null) || \
  branch=$(git rev-parse --short HEAD 2>/dev/null)   || return
  local dirty=""
  [[ -n "$(git status --porcelain 2>/dev/null)" ]] && dirty="*"
  echo "${branch}${dirty}"
}

# ── Active virtualenv label ───────────────────────────────────────────────────
__prompt_venv() {
  [[ -n "${VIRTUAL_ENV:-}" ]] && echo "($(basename "$VIRTUAL_ENV")) "
}

# ── Smart path (last 3 segments, ~ for home) ──────────────────────────────────
__prompt_path() {
  local p="${PWD/#$HOME/\~}"
  local depth
  depth=$(grep -o '/' <<< "$p" | wc -l)
  [[ $depth -gt 3 ]] && p="…/$(echo "$p" | rev | cut -d'/' -f1-3 | rev)"
  echo "$p"
}

# ── Assemble PS1 ──────────────────────────────────────────────────────────────
__build_ps1() {
  local exit_code=$?

  # User: coral for root, lime for everyone else
  local uc="$__c_green"
  [[ $EUID -eq 0 ]] && uc="$__c_red"

  # Failed-exit badge
  local badge=""
  [[ $exit_code -ne 0 ]] && badge="${__c_red}[✗ ${exit_code}] ${__c_reset}"

  # Git
  local git_info=""
  local branch
  branch=$(__prompt_git)
  [[ -n "$branch" ]] && git_info=" ${__c_slate}on ${__c_amber}${branch}${__c_reset}"

  # Venv
  local venv=""
  venv=$(__prompt_venv)
  [[ -n "$venv" ]] && venv="${__c_cyan}${venv}${__c_reset}"

  local short_path
  short_path=$(__prompt_path)

  # Line 1: ╭─  [badge] user@host  path  on branch
  # Line 2: ╰─  $
  PS1="\n"
  PS1+="${__c_slate}╭─${__c_reset} "
  PS1+="${venv}"
  PS1+="${badge}"
  PS1+="${uc}${__c_bold}\u${__c_reset}"
  PS1+="${__c_slate}@${__c_reset}"
  PS1+="${__c_violet}\h${__c_reset}"
  PS1+="  ${__c_blue}${short_path}${__c_reset}"
  PS1+="${git_info}"
  PS1+="\n"
  PS1+="${__c_slate}╰─${__c_reset} "
  PS1+="${uc}${__c_bold}\$${__c_reset} "
}

PROMPT_COMMAND='__build_ps1'
# Append history on every command (safe multi-window history)
PROMPT_COMMAND="${PROMPT_COMMAND};history -a"

# ── dir_colors (ls color tuning) ──────────────────────────────────────────────
export LS_COLORS='di=38;5;111:ln=38;5;117:so=38;5;141:pi=38;5;179:ex=38;5;149:bd=38;5;61:cd=38;5;61:su=38;5;210:sg=38;5;210:tw=38;5;111:ow=38;5;111:'

# ── grep colors ───────────────────────────────────────────────────────────────
export GREP_COLORS='ms=38;5;210:mc=38;5;210:sl=:cx=:fn=38;5;111:ln=38;5;179:bn=38;5;179:se=38;5;61'

# ── Aliases ───────────────────────────────────────────────────────────────────
alias ls='ls --color=auto -F'
alias ll='ls -lah --color=auto'
alias la='ls -A --color=auto'
alias grep='grep --color=auto'
alias ..='cd ..'
alias ...='cd ../..'
alias gs='git status -sb'
alias gd='git diff'
alias gl='git log --oneline --graph --decorate -20'
alias gco='git checkout'
alias ga='git add'
alias gc='git commit'

# ── History ───────────────────────────────────────────────────────────────────
HISTSIZE=20000
HISTFILESIZE=50000
HISTCONTROL=ignoreboth:erasedups
shopt -s histappend
shopt -s checkwinsize   # fix window resize artifacts

# ── Better less / man colors ──────────────────────────────────────────────────
export LESS='-R --use-color'
export LESS_TERMCAP_mb=$'\e[38;5;210m'   # begin blink
export LESS_TERMCAP_md=$'\e[38;5;111m'   # begin bold
export LESS_TERMCAP_me=$'\e[0m'          # reset bold/blink
export LESS_TERMCAP_so=$'\e[48;5;61m\e[38;5;252m'  # standout
export LESS_TERMCAP_se=$'\e[0m'          # end standout
export LESS_TERMCAP_us=$'\e[38;5;149m'   # underline
export LESS_TERMCAP_ue=$'\e[0m'          # end underline

# <<< rvpoint prompt <<<
PROMPT_BLOCK

echo ""
echo "✓ Prompt installed → $BASHRC"
echo "  Reload:  source ~/.bashrc"
echo ""
