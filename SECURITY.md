# Security Policy

## Supported Versions

This workspace is configured for ROS 2 Jazzy. We maintain security updates for:

| Version | Supported          |
| ------- | ------------------ |
| Jazzy   | :white_check_mark: |
| Humble  | :x:                |
| Iron    | :x:                |

## Reporting a Vulnerability

If you discover a security vulnerability in this workspace or any of its components, please report it by:

1. **DO NOT** open a public issue
2. Contact the maintainers directly via GitHub's private vulnerability reporting feature
3. Include:
   - Description of the vulnerability
   - Steps to reproduce
   - Potential impact
   - Suggested fix (if any)

## Security Best Practices

When working with this workspace:

- **SSH Keys**: Use SSH keys for repository access instead of passwords
- **Dependencies**: Regularly update dependencies using `rosdep update`
- **Build Artifacts**: Build artifacts in `layers/*/build/` and `layers/*/install/` are gitignored
- **Secrets**: Never commit credentials, API keys, or sensitive configuration
- **Code Review**: All changes should go through pull request review

## Known Security Considerations

- The `build_report_generator.py` script uses `eval()` on colcon log output. This is acceptable since logs are generated locally, but be aware when processing logs from untrusted sources.
- Scripts require sudo for system package installation during bootstrap. Review `.agent/scripts/bootstrap.sh` before running.
