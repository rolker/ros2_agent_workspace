# AI in Our Work — Presentation

A discussion-format presentation for the CCOM/JHC working group on using AI tools in technical workflows.

## Prerequisites

Install [Marp CLI](https://github.com/marp-team/marp-cli):

```bash
npm install -g @marp-team/marp-cli
```

Or use the [Marp for VS Code](https://marketplace.visualstudio.com/items?itemName=marp-team.marp-vscode) extension.

## Usage

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
    "./presentations/ai-in-our-work/theme-ccom.css"
  ]
}
```

## Files

| File | Purpose |
|------|---------|
| `slides.md` | Presentation content (Marp markdown) |
| `theme-ccom.css` | Custom theme with UNH CCOM/JHC brand colors |
| `images/` | Screenshots, diagrams, and visual assets |
