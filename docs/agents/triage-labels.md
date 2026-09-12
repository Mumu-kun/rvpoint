# Triage Labels

The engineering skills speak in terms of five canonical triage roles. In this repository, because issues are tracked locally in markdown files, these roles map directly to `Status:` header values in each issue/spec file:

| Role in engineering skills | Status tag in local file | Meaning |
| :--- | :--- | :--- |
| `needs-triage` | `Status: needs-triage` | Maintainer needs to evaluate this item |
| `needs-info` | `Status: needs-info` | Waiting on more domain information |
| `ready-for-agent` | `Status: ready-for-agent` | Fully specified, ready for autonomous agent execution |
| `ready-for-human` | `Status: ready-for-human` | Requires human implementation or physical hardware interaction |
| `wontfix` | `Status: wontfix` | Explicitly rejected or closed |

When a skill mentions a role (e.g. "apply the AFK-ready triage label"), write `Status: ready-for-agent` into the file header.

