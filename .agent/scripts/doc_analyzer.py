#!/usr/bin/env python3

import os
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from datetime import datetime

# Configuration
# Explicitly listing workspaces based on previous context, but could be dynamic
BASE_DIR = Path(__file__).resolve().parent
WORKSPACES_ROOT = BASE_DIR.parent / "workspaces"
REPORT_FILE = BASE_DIR.parent / "scratchpad" / "documentation_quality_report.md"

# Scoring Weights
WEIGHT_README_EXISTS = 40
WEIGHT_README_SIZE = 20  # Up to 20 points for size > 50 lines
WEIGHT_HEADERS = 20  # 5 points per key header, max 20
WEIGHT_PKG_DESC = 10  # 10 points if description > 20 chars
WEIGHT_LICENSE = 10  # 10 points if license tag exists

KEY_HEADERS = [
    r"#\s+Installation",
    r"#\s+Usage",
    r"#\s+Nodes",
    r"#\s+Topics",
    r"#\s+Parameters",
    r"#\s+Services",
    r"#\s+Launch",
    r"#\s+Configuration",
]


def find_packages(root_dir):
    """Finds all directories containing a package.xml, excluding build dirs."""
    packages = []
    root_path = Path(root_dir)

    for path in root_path.rglob("package.xml"):
        # Exclude build, install, log directories
        parts = path.parts
        if "build" in parts or "install" in parts or "log" in parts:
            continue

        packages.append(path.parent)

    return packages


def analyze_package(pkg_path):
    """Analyzes a single package for documentation quality."""
    score = 0
    details = {
        "path": str(pkg_path),
        "name": pkg_path.name,
        "readme_exists": False,
        "readme_lines": 0,
        "headers_found": [],
        "pkg_desc_len": 0,
        "license_found": False,
        "issues": [],
    }

    # 1. Analyze package.xml
    try:
        tree = ET.parse(pkg_path / "package.xml")
        root = tree.getroot()
        name_tag = root.find("name")
        if name_tag is not None and name_tag.text:
            details["name"] = name_tag.text

        desc_tag = root.find("description")
        if desc_tag is not None and desc_tag.text:
            details["pkg_desc_len"] = len(desc_tag.text.strip())
            if details["pkg_desc_len"] > 20:
                score += WEIGHT_PKG_DESC
            else:
                details["issues"].append("Description too short")
        else:
            details["issues"].append("Missing description in package.xml")

        license_tag = root.find("license")
        if license_tag is not None and license_tag.text:
            details["license_found"] = True
            score += WEIGHT_LICENSE
        else:
            details["issues"].append("Missing license in package.xml")

    except Exception as e:
        details["issues"].append(f"Error parsing package.xml: {e}")

    # 2. Analyze README
    readme_candidates = ["README.md", "README", "readme.md", "readme"]
    readme_path = None
    for cand in readme_candidates:
        p = pkg_path / cand
        if p.exists():
            readme_path = p
            break

    if readme_path:
        details["readme_exists"] = True
        score += WEIGHT_README_EXISTS

        try:
            content = readme_path.read_text(errors="ignore")
            lines = content.splitlines()
            details["readme_lines"] = len(lines)

            # Size score
            size_score = min(WEIGHT_README_SIZE, int((len(lines) / 50) * WEIGHT_README_SIZE))
            score += size_score
            if len(lines) < 10:
                details["issues"].append("README is very short")

            # Header analysis
            for header_regex in KEY_HEADERS:
                if re.search(header_regex, content, re.IGNORECASE):
                    details["headers_found"].append(
                        header_regex.replace(r"#\s+", "").replace(r"\\", "")
                    )

            header_score = min(WEIGHT_HEADERS, len(details["headers_found"]) * 5)
            score += header_score
            if header_score < WEIGHT_HEADERS:
                found = len(details['headers_found'])
                details["issues"].append(
                    f"Missing key sections (Found {found}/4+ needed for max score)"
                )

        except Exception as e:
            details["issues"].append(f"Error reading README: {e}")
    else:
        details["issues"].append("No README found")

    details["score"] = score
    return details


def generate_report(results):
    """Generates a markdown report from analysis results."""
    # Sort by score (ascending) to show worst documentation first
    sorted_results = sorted(results, key=lambda x: x["score"])

    lines = []
    lines.append("# Documentation Quality Report")
    lines.append(f"Generated on: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append(f"Total Packages Analyzed: {len(results)}")
    lines.append("")

    # Summary Table
    lines.append("## Top 10 Packages Needing Attention")
    lines.append("| Package | Score | Issues |")
    lines.append("|---------|-------|--------|")
    for res in sorted_results[:10]:
        issue_summary = ", ".join(res["issues"][:2])
        if len(res["issues"]) > 2:
            issue_summary += ", ..."
        lines.append(f"| `{res['name']}` | **{res['score']}** | {issue_summary} |")

    lines.append("")

    # Detailed Breakdown
    lines.append("## Detailed Analysis")
    for res in sorted_results:
        lines.append(f"### {res['name']} (Score: {res['score']})")
        lines.append(f"- **Path**: `{res['path']}`")
        license_icon = '✅' if res['license_found'] else '❌'
        lines.append(
            f"- **package.xml**: Description len: {res['pkg_desc_len']}, License: {license_icon}"
        )
        lines.append(
            f"- **README**: {'✅' if res['readme_exists'] else '❌'} ({res['readme_lines']} lines)"
        )
        if res["readme_exists"]:
            headers = ", ".join(res["headers_found"]) if res["headers_found"] else "None"
            lines.append(f"    - **Sections Found**: {headers}")

        if res["issues"]:
            lines.append("- **Issues**:")
            for issue in res["issues"]:
                lines.append(f"    - {issue}")
        lines.append("")

    return "\n".join(lines)


def main():
    print(f"Scanning workspaces in {WORKSPACES_ROOT}...")
    packages = find_packages(WORKSPACES_ROOT)
    print(f"Found {len(packages)} packages.")

    results = []
    for pkg in packages:
        results.append(analyze_package(pkg))

    report = generate_report(results)

    # Ensure directory exists
    os.makedirs(os.path.dirname(REPORT_FILE), exist_ok=True)

    with open(REPORT_FILE, "w") as f:
        f.write(report)

    print(f"Report generated at: {REPORT_FILE}")


if __name__ == "__main__":
    main()
