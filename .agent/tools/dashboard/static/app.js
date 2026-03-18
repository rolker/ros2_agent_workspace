/* Agent Dashboard — frontend logic */

let activeTab = null;
let terminalInterval = null;
let sessions = [];
let activeAppTab = null;
let appTerminalInterval = null;

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
        if (!resp.ok) throw new Error(resp.status);
        sessions = await resp.json();
        setConnected(true);
    } catch (e) {
        setConnected(false);
        sessions = [];
    }
    renderTabs();

    // Auto-select first agent tab if none active
    const agentSessions = sessions.filter(s => s.type !== 'app');
    if (activeTab === null && agentSessions.length > 0) {
        switchTab(agentSessions[0].id);
    }
    // If active tab disappeared, select first
    if (activeTab !== null && !agentSessions.find(s => s.id === activeTab)) {
        if (agentSessions.length > 0) {
            switchTab(agentSessions[0].id);
        } else {
            activeTab = null;
        }
    }

    // Refresh app sub-tabs if a tab is active
    if (activeTab !== null) {
        renderAppTabs();
    }
}

// --- Tab Bar ---

function renderTabs() {
    const bar = document.getElementById('tab-bar');
    const noSessions = document.getElementById('no-sessions');

    // Only show agent sessions (not app sessions) as top-level tabs
    const agentSessions = sessions.filter(s => s.type !== 'app');

    if (agentSessions.length === 0) {
        bar.innerHTML = '';
        bar.appendChild(noSessions || createNoSessionsEl());
        return;
    }

    bar.innerHTML = '';
    for (const s of agentSessions) {
        const tab = document.createElement('div');
        tab.className = 'tab' + (s.id === activeTab ? ' active' : '');
        tab.dataset.session = s.id;
        tab.onclick = () => switchTab(s.id);

        const dot = document.createElement('span');
        dot.className = 'status-dot ' + s.agent_status;
        tab.appendChild(dot);

        const label = document.createElement('span');
        label.textContent = formatTabLabel(s);
        tab.appendChild(label);

        bar.appendChild(tab);
    }
}

function formatTabLabel(session) {
    let base = session.issue ? `#${session.issue}` : (session.skill || session.id);
    // Add repo/layer context for layer worktrees
    if (session.type === 'layer') {
        const context = session.layer || session.repo;
        if (context) {
            base += ` (${context})`;
        }
    } else if (session.type === 'workspace') {
        base += ' (ws)';
    }
    return base;
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

    // Update app sub-tabs for this session's issue
    renderAppTabs();
}

// --- App Output Panel ---

function getAppSessionsForActiveTab() {
    if (!activeTab) return [];
    const activeSession = sessions.find(s => s.id === activeTab);
    if (!activeSession || !activeSession.issue) return [];
    return sessions.filter(s => s.type === 'app' && s.issue === activeSession.issue);
}

function renderAppTabs() {
    const appSessions = getAppSessionsForActiveTab();
    const panel = document.getElementById('app-panel');
    const tabsEl = document.getElementById('app-tabs');

    if (appSessions.length === 0) {
        panel.style.display = 'none';
        clearInterval(appTerminalInterval);
        activeAppTab = null;
        return;
    }

    panel.style.display = '';
    tabsEl.innerHTML = '';

    for (const s of appSessions) {
        const tab = document.createElement('span');
        tab.className = 'app-tab' + (s.id === activeAppTab ? ' active' : '');
        tab.dataset.appSession = s.id;
        tab.onclick = () => switchAppTab(s.id);

        const dot = document.createElement('span');
        dot.className = 'status-dot ' + s.agent_status;
        tab.appendChild(dot);

        const label = document.createTextNode(s.label);
        tab.appendChild(label);

        tabsEl.appendChild(tab);
    }

    // Auto-select first app tab if none active or current disappeared
    if (!activeAppTab || !appSessions.find(s => s.id === activeAppTab)) {
        switchAppTab(appSessions[0].id);
    }
}

function switchAppTab(appSessionId) {
    activeAppTab = appSessionId;

    // Update app tab active state
    document.querySelectorAll('.app-tab').forEach(t =>
        t.classList.toggle('active', t.dataset.appSession === appSessionId));

    // Update attach command
    document.getElementById('app-attach-cmd').textContent =
        `tmux attach -t ${appSessionId}`;

    // Start polling for this app session
    clearInterval(appTerminalInterval);
    refreshAppTerminal();
    appTerminalInterval = setInterval(refreshAppTerminal, 1500);
}

async function refreshAppTerminal() {
    if (!activeAppTab) return;

    try {
        const resp = await fetch(`/api/terminal/${encodeURIComponent(activeAppTab)}`);
        const data = await resp.json();

        const el = document.getElementById('app-output');
        const wasScrolledToBottom = el.scrollTop + el.clientHeight >= el.scrollHeight - 20;

        el.textContent = data.output || '(no output)';

        if (wasScrolledToBottom) {
            el.scrollTop = el.scrollHeight;
        }
    } catch (e) {
        // Ignore fetch errors
    }
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
            // escapeHtml() does not escape " — replace additionally to prevent
            // attribute breakout (same pattern as inlineFormat()).
            const safeUrl = escapeHtml(ctx.issue.url).replace(/"/g, '&quot;');
            html += `<div style="margin-top:4px"><a href="${safeUrl}" `
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

    // Build report
    if (ctx.build_report) {
        html += '<div class="ctx-section">';
        html += '<div class="ctx-section-title">Build</div>';
        html += `<pre style="font-size:11px;white-space:pre-wrap;color:var(--text-dim)">${escapeHtml(ctx.build_report)}</pre>`;
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
            // href is already HTML-escaped by the escapeHtml() call above (&->&amp; etc).
            // Only escape unquoted " to prevent attribute injection — avoid double-escaping &.
            const safeHref = href.replace(/"/g, '&quot;');
            return `<a href="${safeHref}" target="_blank" rel="noopener noreferrer" style="color:var(--blue)">${label}</a>`;
        }
        return `${label} (${href})`;
    });
    return text;
}

// --- Connection Status ---

function setConnected(connected) {
    const el = document.getElementById('connection-status');
    if (connected) {
        el.className = 'connected';
        el.title = 'Connected to server';
    } else {
        el.className = 'disconnected';
        el.title = 'Server unreachable';
    }
}

// --- SSE Connection ---

function connectSSE() {
    const events = new EventSource('/api/events');

    events.onopen = () => setConnected(true);

    events.addEventListener('status_change', (e) => {
        const data = JSON.parse(e.data);
        // Update the session's status in our local list
        const session = sessions.find(s => s.id === data.session);
        if (session) {
            session.agent_status = data.status;
            renderTabs();
            // Re-render app tabs if the status change is for an app session
            if (session.type === 'app') {
                renderAppTabs();
            }
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
        setConnected(false);
        // EventSource auto-reconnects automatically
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
