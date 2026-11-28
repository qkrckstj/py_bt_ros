#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
mkdir -p download
tar -czf download/dashboard_navigation.tar.gz -C transfer/execution_prepared real_executor.py bounded_real_map.py real_navigation.launch.py run_dashboard_navigation.sh command_wire.py limits.py map_wire.py nav_adapter.py ownership.py readiness.py scan_evidence.py swept_map_guard.py real_observer.py
tar -czf download/dashboard_mapping.tar.gz -C transfer/chanseo_update_download dashboard_mapping.launch.py run_dashboard_mapping.sh
cp transfer/chanseo_update_download/fix_dashboard_lidar.sh download/
