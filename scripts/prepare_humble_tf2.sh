#!/usr/bin/env bash
# Extract a fixed official tf2 package for this project; never install system packages.
set -euo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
package=ros-humble-tf2
version="$(apt-cache policy "$package" | awk '/Candidate:/ {print $2}')"
if [[ -z "$version" || "$version" == '(none)' ]] ||
   ! dpkg --compare-versions "$version" ge 0.25.24 ||
   ! dpkg --compare-versions "$version" lt 0.26; then
    echo 'A fixed Humble tf2 0.25.24+ package is not available in the apt index.' >&2
    exit 1
fi
destination="$repo_root/output/compat/tf2"
mkdir -p "$destination/download"
cd "$destination/download"
apt-get download "$package=$version"
archives=(ros-humble-tf2_*.deb)
archive=''
for candidate in "${archives[@]}"; do
    if [[ "$(dpkg-deb -f "$candidate" Version)" == "$version" ]]; then
        archive="$candidate"
        break
    fi
done
[[ -n "$archive" ]]
dpkg-deb -x "$archive" "$destination"
echo "Project-local tf2 $version prepared: $destination/opt/ros/humble/lib"
