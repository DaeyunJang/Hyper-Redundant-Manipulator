#!/usr/bin/env bash
# Install two signed official ROS debs into this workspace, without sudo or APT changes.
# Existing installations are never overwritten. Use --verify-only to check one.
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
install_dir="$repo_root/.ros_image_proc"
download_dir="$install_dir/downloads"
prefix="$install_dir/root/opt/ros/humble"
repository='http://packages.ros.org/ros2/ubuntu'
fingerprint='C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654'

verify_runtime() {
    python3 - "$install_dir" <<'PY'
import hashlib
import json
from pathlib import Path
import sys

directory = Path(sys.argv[1])
manifest = json.loads((directory / 'manifest.json').read_text())
for name, expected in manifest['runtime_files_sha256'].items():
    path = directory / 'root/opt/ros/humble' / name
    if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
        raise SystemExit(f'Runtime checksum mismatch: {path}')
for package in manifest['packages']:
    path = directory / 'downloads' / Path(package['filename']).name
    if hashlib.sha256(path.read_bytes()).hexdigest() != package['sha256']:
        raise SystemExit(f'Package checksum mismatch: {path}')
print('Manifest runtime/deb checksums verified.')
PY
    LD_LIBRARY_PATH="$prefix/lib:/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}" \
        ldd -r "$prefix/lib/librectify.so" > "$install_dir/runtime_ldd.txt" 2>&1
    if grep -E 'not found|undefined symbol' "$install_dir/runtime_ldd.txt"; then
        printf '%s\n' 'Unresolved runtime dependencies; do not use this prefix.' >&2
        return 1
    fi
    printf 'Verified local prefix: %s\n' "$prefix"
}

if [[ ${1:-} == '--verify-only' && $# == 1 ]]; then
    verify_runtime
    exit 0
elif [[ $# != 0 ]]; then
    printf 'Usage: bash %s [--verify-only]\n' "$0" >&2
    exit 2
fi
if [[ -e "$install_dir/root" || -L "$install_dir/root" ]]; then
    printf '%s\n' 'Local root already exists; refusing overwrite. Use --verify-only.' >&2
    exit 2
fi
[[ $(dpkg --print-architecture) == amd64 ]] || {
    printf '%s\n' 'This installer requires Ubuntu Jammy amd64 with ROS Humble.' >&2; exit 2;
}
for command in curl gpg gpgv dpkg-deb python3 ldd; do command -v "$command" >/dev/null; done
mkdir -p "$download_dir" "$install_dir/gnupg"
chmod 700 "$install_dir/gnupg"
touch "$install_dir/COLCON_IGNORE"

fetch() {
    curl --fail --location --retry 2 --max-time 60 --proto '=http,https' \
        --proto-redir '=http,https' "$1" --output "$2"
}
# The key is fetched via authenticated HTTPS and pinned to the existing ROS key.
# HTTP repository payloads are authenticated by InRelease signature + SHA256 chain.
fetch 'https://raw.githubusercontent.com/ros/rosdistro/master/ros.key' "$download_dir/ros.key"
actual_fingerprint=$(gpg --homedir "$install_dir/gnupg" --show-keys --with-colons \
    "$download_dir/ros.key" | awk -F: '$1 == "pub" {primary=1} \
        $1 == "fpr" && primary {print $10; primary=0}')
[[ "$actual_fingerprint" == "$fingerprint" ]] || {
    printf '%s\n' 'Official ROS signing key fingerprint changed; refusing install.' >&2; exit 1;
}
fetch "$repository/dists/jammy/InRelease" "$download_dir/InRelease"
gpgv --homedir "$install_dir/gnupg" --keyring "$download_dir/ros.key" \
    "$download_dir/InRelease"
fetch "$repository/dists/jammy/main/binary-amd64/Packages.gz" "$download_dir/Packages.gz"

# Parse the verified release and package records without executing remote content.
python3 - "$install_dir" "$repository" "$fingerprint" <<'PY'
from datetime import datetime, timezone
from email.utils import parsedate_to_datetime
import gzip
import hashlib
import json
from pathlib import Path
import re
import sys

directory, repository, fingerprint = Path(sys.argv[1]), sys.argv[2], sys.argv[3]
downloads = directory / 'downloads'
release = (downloads / 'InRelease').read_text()
date = re.search(r'^Date: (.+)$', release, re.M).group(1)
age = (datetime.now(timezone.utc) - parsedate_to_datetime(date)).total_seconds()
if not -86400 <= age <= 14 * 86400:
    raise SystemExit('Signed repository index is stale or from the future; refusing install.')
sha_section = release.split('\nSHA256:\n', 1)[1].split('-----BEGIN PGP SIGNATURE-----', 1)[0]
entries = [line.split() for line in sha_section.splitlines() if line.strip()]
expected, size, _ = next(row for row in entries if row[2] == 'main/binary-amd64/Packages.gz')
compressed = (downloads / 'Packages.gz').read_bytes()
if len(compressed) != int(size) or hashlib.sha256(compressed).hexdigest() != expected:
    raise SystemExit('Packages.gz size/SHA256 mismatch against signed InRelease.')
names = {'ros-humble-image-proc', 'ros-humble-tracetools-image-pipeline'}
packages = []
for paragraph in gzip.decompress(compressed).decode().split('\n\n'):
    fields = dict(line.split(': ', 1) for line in paragraph.splitlines()
                  if line and not line.startswith(' ') and ': ' in line)
    if fields.get('Package') not in names:
        continue
    name, filename = fields['Package'], fields['Filename']
    if (fields['Architecture'] != 'amd64'
            or not re.fullmatch(r'pool/main/r/' + name + '/' + name
                                + r'_[A-Za-z0-9.+:~_-]+_amd64\.deb', filename)
            or not re.fullmatch('[0-9a-f]{64}', fields['SHA256'])):
        raise SystemExit('Unexpected package architecture, path, or checksum.')
    packages.append(dict(name=name, version=fields['Version'], filename=filename,
                         sha256=fields['SHA256'], size=int(fields['Size'])))
if len(packages) != 2 or {p['name'] for p in packages} != names:
    raise SystemExit('Both required package records must occur exactly once.')
manifest = dict(schema_version=1, repository=repository, release_date=date,
                signing_key_fingerprint=fingerprint,
                inrelease_sha256=hashlib.sha256((downloads / 'InRelease').read_bytes()).hexdigest(),
                packages_index_sha256=expected, packages=packages,
                installation='dpkg-deb extraction; no maintainer scripts or system APT changes')
(directory / 'manifest.pending.json').write_text(json.dumps(manifest, indent=2) + '\n')
(downloads / 'selection.tsv').write_text(''.join(
    '\t'.join(str(p[k]) for k in ('name', 'version', 'filename', 'sha256', 'size')) + '\n'
    for p in packages))
PY

staging=$(mktemp -d "$install_dir/staging.XXXXXX")
mkdir "$staging/root"
while IFS=$'\t' read -r name version filename checksum size; do
    package="$download_dir/${filename##*/}"
    fetch "$repository/$filename" "$package"
    [[ $(stat -c %s "$package") == "$size" ]]
    printf '%s  %s\n' "$checksum" "$package" | sha256sum --check
    [[ $(dpkg-deb -f "$package" Package) == "$name" ]]
    [[ $(dpkg-deb -f "$package" Version) == "$version" ]]
    [[ $(dpkg-deb -f "$package" Architecture) == amd64 ]]
    dpkg-deb -x "$package" "$staging/root"
done < "$download_dir/selection.tsv"

python3 - "$install_dir" "$staging/root/opt/ros/humble" <<'PY'
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

directory, prefix = map(Path, sys.argv[1:])
manifest = json.loads((directory / 'manifest.pending.json').read_text())
manifest['runtime_files_sha256'] = {
    str(path.relative_to(prefix)): hashlib.sha256(path.read_bytes()).hexdigest()
    for path in prefix.rglob('*') if path.is_file()}
manifest['installed_at_utc'] = datetime.now(timezone.utc).isoformat()
(directory / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
PY
# Destination is fixed and must be absent: never replace an existing installation.
[[ ! -e "$install_dir/root" && ! -L "$install_dir/root" ]]
mv -- "$staging/root" "$install_dir/root"
verify_runtime
