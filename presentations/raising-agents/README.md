# Raising Agents — Presentation

"Raising Agents" is a discussion-format presentation for the CCOM/JHC AI Working Group on using AI tools in technical workflows.

## Prerequisites

Install [Marp CLI](https://github.com/marp-team/marp-cli):

```bash
npm install -g @marp-team/marp-cli
```

Or use the [Marp for VS Code](https://marketplace.visualstudio.com/items?itemName=marp-team.marp-vscode) extension.

## Usage

All commands below assume you are in the presentation directory:

```bash
cd presentations/raising-agents
```

### Present in browser (with live reload)

```bash
marp -s --theme theme-ccom.css .
```

### Export to HTML

```bash
marp --theme theme-ccom.css slides.md -o slides.html
```

### Export to PDF

```bash
marp --theme theme-ccom.css slides.md --pdf -o slides.pdf
```

## VS Code Setup

If using the Marp VS Code extension, add to `.vscode/settings.json`:

```json
{
  "markdown.marp.themes": [
    "./presentations/raising-agents/theme-ccom.css"
  ]
}
```

## Files

| File | Purpose |
|------|---------|
| `slides.md` | Presentation content (Marp markdown) |
| `theme-ccom.css` | Custom theme with UNH CCOM/JHC brand colors |
