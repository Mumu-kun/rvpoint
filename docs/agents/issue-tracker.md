# Issue Tracker: Local Markdown

Issues, tickets, and specifications for this repository live exclusively as local markdown files under `docs/plans/`. No remote GitHub issue tracking is used; GitHub involvement is strictly limited to committing progress to the git branch.

## Conventions

- **Directory Layout**: Features and specifications live under `docs/plans/<feature-slug>/` or as dated specs in `docs/plans/<date>-<slug>.md`.
- **Implementation Tickets**: Individual work tickets live at `docs/plans/<feature-slug>/issues/<NN>-<slug>.md`, numbered from `01`.
- **Triage State**: Recorded as a `Status: <role>` line near the top of each file (see `docs/agents/triage-labels.md`).
- **Comments**: Conversation history and progress notes append to the bottom under `## Comments`.

## When a skill says "publish to the issue tracker"

Write the specification or issue file under `docs/plans/`. Apply the appropriate `Status:` header (e.g. `Status: ready-for-agent`). Do NOT invoke `gh issue create`.

## When a skill says "fetch the relevant ticket"

Read the corresponding local markdown file under `docs/plans/`.

## Wayfinding Operations

- **Map**: `docs/plans/<feature-slug>/map.md`
- **Child Tickets**: `docs/plans/<feature-slug>/issues/<NN>-<slug>.md`
- **Claim**: Set `Status: claimed`
- **Resolve**: Append the answer under `## Answer` and set `Status: resolved`.

