#!/usr/bin/env bash
# Compatibility entry; every session uses the same Route B supervisor.
exec bash "$(dirname "${BASH_SOURCE[0]}")/run_session.sh" "$@"
