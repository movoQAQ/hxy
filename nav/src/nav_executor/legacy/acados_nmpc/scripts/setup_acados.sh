#!/usr/bin/env bash
# Build the runtime that matches generated/manifest.json, without system changes.
set -euo pipefail
if [[ $(uname -s) != Linux || $(uname -m) != x86_64 ]]; then
  echo "This dependency recipe targets Linux x86_64; supply a platform-specific Acados installation on other targets." >&2
  exit 1
fi
package_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
workspace_dir=$(cd -- "$package_dir/../.." && pwd)
cache_dir="$workspace_dir/.cache"
source_dir="$cache_dir/acados-source"
install_dir="$cache_dir/acados"
revision=8ff7c2a17e235d6cad6671538e923ccb7428dc3e
mkdir -p "$cache_dir"
if [[ ! -d "$source_dir/.git" ]]; then
  git clone https://github.com/acados/acados.git "$source_dir"
fi
# A dirty dependency checkout must not be overwritten by this script.
if [[ -n "$(git -C "$source_dir" status --porcelain --untracked-files=no)" ]]; then
  echo 'Acados source has local changes; use a clean dependency checkout.' >&2
  exit 1
fi
git -C "$source_dir" checkout --detach "$revision"
git -C "$source_dir" submodule update --init external/blasfeo external/hpipm
cmake -S "$source_dir" -B "$cache_dir/acados-build" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$install_dir" -DCMAKE_INSTALL_RPATH='$ORIGIN' \
  -DACADOS_WITH_OPENMP=OFF -DBLASFEO_TARGET="${NMPC_BLASFEO_TARGET:-X64_AUTOMATIC}" -DHPIPM_TARGET="${NMPC_HPIPM_TARGET:-AVX}"
cmake --build "$cache_dir/acados-build" -j "${BUILD_JOBS:-4}"
cmake --install "$cache_dir/acados-build"
git -C "$source_dir" rev-parse HEAD > "$install_dir/lib/git_commit_hash"
# Code-generation environment is only required when regenerating the shipped C sources.
python3 -m venv "$cache_dir/nmpc-venv"
"$cache_dir/nmpc-venv/bin/pip" install 'casadi==3.7.2' PyYAML -e "$source_dir/interfaces/acados_template"
mkdir -p "$source_dir/bin"
renderer="$source_dir/bin/t_renderer"
curl -L --fail https://github.com/acados/tera_renderer/releases/download/v0.2.1/t_renderer-v0.2.1-linux-amd64 -o "$renderer"
printf '%s  %s\n' '64a0a0f8d85be0b92234231fd1a9ff50c9d8f13d6208063707e56c9354a976d3' "$renderer" | sha256sum -c -
chmod +x "$renderer"
printf 'Runtime installed. Regenerate with ACADOS_SOURCE_DIR=%q %q %q\n' \
  "$source_dir" "$cache_dir/nmpc-venv/bin/python" "$package_dir/scripts/generate_omni_solver.py"
