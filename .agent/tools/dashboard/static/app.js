/* Agent Dashboard — frontend logic */

let activeTab = null;
let terminalInterval = null;
let sessions = [];

// --- Initialization ---

async function init() {
    await refreshSessions();
    connectSSE();
    // Refresh session list periodically (catches new/removed worktrees)
    setInterval(refreshSessions, 15000);
}

// --- Session Discovery ---

async function refreshSessions() {
    try {
        const resp = await fetch('/api/sessions');
        sessions = await resp.json();
    } catch (e) {
        sessions = [];
    }
    renderTabs();

    // Auto-select first tab if none active
    if (activeTab === null && sessions.length > 0) {
        switchTab(sessions[0].id);
    }
    // If active tab disappeared, select first
    if (activeTab !== null && !sessions.find(s => s.id === activeTab)) {
        if (sessions.length > 0) {
            switchTab(sessions[0].id);
        } else {
            activeTab = null;
        }
    }
}

// --- Tab Bar ---

function renderTabs() {
    const bar = document.getElementById('tab-bar');
    const noSessions = document.getElementById('no-sessions');

    if (sessions.length === 0) {
        bar.innerHTML = '';
        bar.appendChild(noSessions || createNoSessionsEl());
        return;
    }

    bar.innerHTML = '';
    for (const s of sessions) {
        const tab = document.createElement('div');
        tab.className = 'tab' + (s.id === activeTab ? ' active' : '');
        tab.dataset.session = s.id;
        tab.onclick = () => switchTab(s.id);

        const dot = document.createElement('span');
        dot.className = 'status-dot ' + s.agent_status;
        tab.appendChild(dot);

        const label = document.createElement('span');
        label.textContent = s.issue ? `#${s.issue}` : (s.skill || s.id);
        tab.appendChild(label);

        bar.appendChild(tab);
    }
}

function createNoSessionsEl() {
    const el = document.createElement('span');
    el.id = 'no-sessions';
    el.style.cssText = 'color: var(--text-dim); padding: 8px; font-style: italic;';
    el.textContent = 'No active sessions found. Waiting...';
    return el;
}

// --- Tab Switching ---

function switchTab(sessionId) {
    activeTab = sessionId;

    // Update tab active state
    document.querySelectorAll('.tab').forEach(t =>
        t.classList.toggle('active', t.dataset.session === sessionId));

    // Start terminal polling for this tab
    clearInterval(terminalInterval);
    refreshTerminal();
    terminalInterval = setInterval(refreshTerminal, 1500);

    // Load context and plan
    loadContext(sessionId);
    loadPlan(sessionId);
}

// --- Terminal Panel ---

async function refreshTerminal() {
    if (!activeTab) return;

    try {
        const resp = await fetch(`/api/terminal/${encodeURIComponent(activeTab)}`);
        const data = await resp.json();

        const el = document.getElementById('terminal-output');
        const wasScrolledToBottom = el.scrollTop + el.clientHeight >= el.scrollHeight - 20;

        el.textContent = data.output || '(no output)';

        // Auto-scroll if user was already at bottom
        if (wasScrolledToBottom) {
            el.scrollTop = el.scrollHeight;
        }

        // Update status badge
        const badge = document.getElementById('terminal-status');
        badge.className = 'status-badge ' + data.status;
        badge.textContent = data.status;
    } catch (e) {
        // Ignore fetch errors (server might be restarting)
    }
}

async function sendToAgent() {
    if (!activeTab) return;

    const input = document.getElementById('agent-input');
    const text = input.value.trim();
    if (!text) return;

    try {
        await fetch(`/api/terminal/${encodeURIComponent(activeTab)}/send`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ text }),
        });
        input.value = '';
    } catch (e) {
        // Ignore
    }
}

// Send on Enter key
document.getElementById('agent-input').addEventListener('keydown', (e) => {
    if (e.key === 'Enter') {
        sendToAgent();
    }
});

// --- Context Panel ---

async function loadContext(sessionId) {
    const el = document.getElementById('context-content');

    try {
        const resp = await fetch(`/api/context/${encodeURIComponent(sessionId)}`);
        const ctx = await resp.json();
        renderContext(ctx);
    } catch (e) {
        el.innerHTML = '<em>Failed to load context.</em>';
    }
}

function renderContext(ctx) {
    const el = document.getElementById('context-content');
    let html = '';

    // Issue summary
    if (ctx.issue) {
        html += '<div class="ctx-section">';
        html += '<div class="ctx-section-title">Issue</div>';
        html += `<div class="ctx-issue-title">${escapeHtml(ctx.issue.title)}</div>`;
        if (ctx.issue.labels && ctx.issue.labels.length > 0) {
            html += ctx.issue.labels.map(l =>
                `<span class="ctx-label">${escapeHtml(l)}</span>`
            ).join('');
        }
        if (ctx.issue.url) {
            html += `<div style="margin-top:4px"><a href="${escapeHtml(ctx.issue.url)}" `
                + `target="_blank" rel="noopener noreferrer" style="color:var(--blue);font-size:11px">View on GitHub</a></div>`;
        }
        html += '</div>';
    }

    // Session info
    const s = ctx.session;
    html += '<div class="ctx-section">';
    html += '<div class="ctx-section-title">Worktree</div>';
    html += kvRow('Branch', s.branch || '—');
    html += kvRow('Status', s.worktree_status, s.worktree_status === 'clean' ? 'pass' : 'fail');
    html += kvRow('Files changed', s.files_changed);
    html += kvRow('Agent', s.agent_status,
        s.agent_status === 'working' ? 'pass' : (s.agent_status === 'waiting' ? 'fail' : ''));
    html += '</div>';

    // Test results
    if (ctx.test_summary) {
        html += '<div class="ctx-section">';
        html += '<div class="ctx-section-title">Tests</div>';
        const ts = ctx.test_summary;
        html += kvRow('Overall', ts.overall_success ? 'passed' : 'failed',
            ts.overall_success ? 'pass' : 'fail');
        if (ts.layers) {
            for (const layer of ts.layers) {
                html += kvRow(layer.name, layer.result,
                    layer.result === 'passed' ? 'pass' : (layer.result === 'failed' ? 'fail' : ''));
            }
        }
        html += '</div>';
    }

    // Changed files
    if (ctx.changed_files && ctx.changed_files.length > 0) {
        html += '<div class="ctx-section">';
        html += '<div class="ctx-section-title">Changed Files</div>';
        html += '<ul class="ctx-file-list">';
        for (const line of ctx.changed_files) {
            const parts = line.split('\t');
            const status = parts[0] || '?';
            const file = parts.slice(1).join('\t') || line;
            html += `<li><span class="file-status ${escapeHtml(status[0])}">${escapeHtml(status[0])}</span> `
                + `${escapeHtml(file)}</li>`;
        }
        html += '</ul>';
        html += '</div>';
    }

    el.innerHTML = html || '<em>No context available.</em>';
}

function kvRow(key, value, cls) {
    const valClass = cls ? ` class="val ${cls}"` : ' class="val"';
    return `<div class="ctx-kv"><span class="key">${escapeHtml(key)}</span>`
        + `<span${valClass}>${escapeHtml(String(value))}</span></div>`;
}

// --- Plan Panel ---

async function loadPlan(sessionId) {
    const el = document.getElementById('plan-content');

    try {
        const resp = await fetch(`/api/plan/${encodeURIComponent(sessionId)}`);
        const data = await resp.json();

        if (data.plan && data.plan.content) {
            el.innerHTML = renderMarkdown(data.plan.content);
        } else {
            el.innerHTML = '<em>No plan found for this session.</em>';
        }
    } catch (e) {
        el.innerHTML = '<em>Failed to load plan.</em>';
    }
}

// --- Simple Markdown Renderer ---
// Handles: headers, bold, code blocks, inline code, lists, tables, links, paragraphs

function renderMarkdown(md) {
    let html = '';
    const lines = md.split('\n');
    let i = 0;
    let inCodeBlock = false;
    let codeContent = '';
    let inTable = false;
    let tableRows = [];

    while (i < lines.length) {
        const line = lines[i];

        // Code blocks
        if (line.startsWith('```')) {
            if (inCodeBlock) {
                html += `<pre><code>${escapeHtml(codeContent.trim())}</code></pre>`;
                codeContent = '';
                inCodeBlock = false;
            } else {
                // Flush table if open
                if (inTable) { html += renderTable(tableRows); inTable = false; tableRows = []; }
                inCodeBlock = true;
            }
            i++;
            continue;
        }

        if (inCodeBlock) {
            codeContent += line + '\n';
            i++;
            continue;
        }

        // Table detection
        if (line.includes('|') && line.trim().startsWith('|')) {
            // Check if next line is separator
            if (!inTable) {
                inTable = true;
                tableRows = [];
            }
            // Skip separator rows
            if (/^\|[\s\-:|]+\|$/.test(line.trim())) {
                i++;
                continue;
            }
            tableRows.push(line);
            i++;
            continue;
        } else if (inTable) {
            html += renderTable(tableRows);
            inTable = false;
            tableRows = [];
        }

        // Headers
        const headerMatch = line.match(/^(#{1,3})\s+(.+)/);
        if (headerMatch) {
            const level = headerMatch[1].length;
            html += `<h${level}>${inlineFormat(headerMatch[2])}</h${level}>`;
            i++;
            continue;
        }

        // Checkbox items (must be checked before unordered list)
        if (/^\s*[-*]\s+\[[ x]\]/i.test(line)) {
            while (i < lines.length && /^\s*[-*]\s+\[[ x]\]/i.test(lines[i])) {
                const checked = /\[x\]/i.test(lines[i]);
                const content = lines[i].replace(/^\s*[-*]\s+\[[ x]\]\s*/i, '');
                const icon = checked ? '&#9745;' : '&#9744;';
                html += `<div>${icon} ${inlineFormat(content)}</div>`;
                i++;
            }
            continue;
        }

        // Unordered list items
        if (/^\s*[-*]\s+/.test(line)) {
            html += '<ul>';
            while (i < lines.length && /^\s*[-*]\s+/.test(lines[i])) {
                const content = lines[i].replace(/^\s*[-*]\s+/, '');
                html += `<li>${inlineFormat(content)}</li>`;
                i++;
            }
            html += '</ul>';
            continue;
        }

        // Ordered list items
        if (/^\s*\d+\.\s+/.test(line)) {
            html += '<ol>';
            while (i < lines.length && /^\s*\d+\.\s+/.test(lines[i])) {
                const content = lines[i].replace(/^\s*\d+\.\s+/, '');
                html += `<li>${inlineFormat(content)}</li>`;
                i++;
            }
            html += '</ol>';
            continue;
        }

        // Empty line
        if (line.trim() === '') {
            i++;
            continue;
        }

        // Paragraph
        html += `<p>${inlineFormat(line)}</p>`;
        i++;
    }

    // Flush remaining
    if (inCodeBlock) {
        html += `<pre><code>${escapeHtml(codeContent.trim())}</code></pre>`;
    }
    if (inTable) {
        html += renderTable(tableRows);
    }

    return html;
}

function renderTable(rows) {
    if (rows.length === 0) return '';
    let html = '<table>';
    rows.forEach((row, idx) => {
        const cells = row.split('|').filter((_, i, a) => i > 0 && i < a.length - 1);
        const tag = idx === 0 ? 'th' : 'td';
        html += '<tr>';
        for (const cell of cells) {
            html += `<${tag}>${inlineFormat(cell.trim())}</${tag}>`;
        }
        html += '</tr>';
    });
    html += '</table>';
    return html;
}

function inlineFormat(text) {
    // Escape HTML first to prevent XSS
    text = escapeHtml(text);
    // Bold
    text = text.replace(/\*\*(.+?)\*\*/g, '<strong>$1</strong>');
    // Inline code
    text = text.replace(/`([^`]+)`/g, '<code>$1</code>');
    // Links — restrict to http/https schemes and add rel for tabnabbing protection
    text = text.replace(/\[([^\]]+)\]\(([^)]+)\)/g, (_, label, href) => {
        if (/^https?:\/\//i.test(href) || href.startsWith('#') || href.startsWith('/')) {
            return `<a href="${href}" target="_blank" rel="noopener noreferrer" style="color:var(--blue)">${label}</a>`;
        }
        return `${label} (${href})`;
    });
    return text;
}

// --- SSE Connection ---

function connectSSE() {
    const events = new EventSource('/api/events');

    events.addEventListener('status_change', (e) => {
        const data = JSON.parse(e.data);
        // Update the session's status in our local list
        const session = sessions.find(s => s.id === data.session);
        if (session) {
            session.agent_status = data.status;
            renderTabs();
        }
    });

    events.addEventListener('session_added', (e) => {
        // Refresh full session list
        refreshSessions();
    });

    events.addEventListener('session_removed', (e) => {
        refreshSessions();
    });

    events.onerror = () => {
        // EventSource auto-reconnects; nothing to do
    };
}

// --- Utilities ---

function escapeHtml(text) {
    const div = document.createElement('div');
    div.textContent = text;
    return div.innerHTML;
}

// --- Start ---
init();
