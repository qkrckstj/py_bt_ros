#!/usr/bin/env python3
"""Prefetch R2025a factory assets using HTTP/1.1, bypassing Qt HTTP/2 failures.

Uses Webots' documented-in-source SHA1(URL) cache layout. Assets stay in the
user's Webots cache, with their original contents/licenses; none are vendored.
Run with Webots closed, before opening limo_factory_hall.wbt for the first time.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import re
import time
from urllib.parse import urljoin, urlparse
from urllib.request import Request, urlopen


BASE = 'https://raw.githubusercontent.com/cyberbotics/webots/R2025a/'
EXTENSIONS = {'.proto', '.jpg', '.jpeg', '.png', '.hdr', '.obj', '.dae', '.stl', '.mtl', '.webp'}
REFERENCES = re.compile(
    r'(?:https://raw\.githubusercontent\.com/cyberbotics/webots/R2025a/|'
    r'webots://projects/|(?:\.\./)*(?:textures|meshes)/)[A-Za-z0-9_./-]+')


def read_url(url):
    for attempt in range(3):
        try:
            request = Request(url, headers={'User-Agent': 'limo-webots-asset-cache/1.0'})
            with urlopen(request, timeout=30) as response:
                return response.read()
        except Exception:
            if attempt == 2:
                raise
            time.sleep(attempt + 1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', type=Path, default=Path(__file__).resolve().parents[1]
                        / 'worlds/limo_factory_hall.wbt')
    parser.add_argument('--cache', type=Path, default=Path(os.environ.get(
        'XDG_CACHE_HOME', str(Path.home() / '.cache'))) / 'Cyberbotics/Webots/assets')
    args = parser.parse_args()
    args.cache.mkdir(parents=True, exist_ok=True)
    tree_paths = None
    seen = set()
    failures = []
    downloaded = 0

    def resolve(reference, origin):
        if reference.startswith('webots://'):
            return BASE + reference.removeprefix('webots://')
        return urljoin(origin, reference)

    def dependencies(text, origin):
        nonlocal tree_paths
        result = set()
        # Ignore license/documentation links, which are not asset requests.
        text = '\n'.join(line for line in text.splitlines() if not line.lstrip().startswith('#'))
        # Human models split their geometry into sibling PROTO files.
        for reference in re.findall(r'EXTERNPROTO\s+"([^"]+)"', text):
            if '/projects/humans/pedestrian/protos/' not in origin or '/' in reference:
                continue
            url = resolve(reference, origin)
            if url.startswith(BASE):
                result.add(url)
        for match in REFERENCES.finditer(text):
            url = resolve(match.group(), origin)
            if not url.startswith(BASE):
                continue
            suffix = Path(urlparse(url).path).suffix.lower()
            if suffix in EXTENSIONS:
                result.add(url)
            elif suffix == '':
                # Templates assemble texture paths from a prefix plus a field value.
                prefix = url.removeprefix(BASE)
                if prefix.rstrip('/') == 'projects/default/worlds/textures/cubic':
                    # Hall uses the factory HDR environment, with skybox disabled.
                    result.update(BASE + prefix.rstrip('/') + '/factory_' + face + '.hdr'
                                  for face in ('back', 'bottom', 'front', 'left', 'right', 'top'))
                    continue
                if tree_paths is None:
                    print('Reading R2025a asset index...', flush=True)
                    tree = json.loads(read_url(
                        'https://api.github.com/repos/cyberbotics/webots/git/trees/R2025a?recursive=1'))
                    if tree.get('truncated'):
                        raise RuntimeError('GitHub asset index is truncated')
                    tree_paths = [item['path'] for item in tree['tree']
                                  if item['type'] == 'blob'
                                  and Path(item['path']).suffix.lower() in EXTENSIONS]
                result.update(BASE + path for path in tree_paths if path.startswith(prefix))
        return result

    def fetch(url):
        target = args.cache / hashlib.sha1(url.encode()).hexdigest()
        if target.is_file() and target.stat().st_size:
            return url, target.read_bytes(), False, None
        try:
            content = read_url(url)
            temporary = target.with_suffix('.part')
            temporary.write_bytes(content)
            temporary.replace(target)
            return url, content, True, None
        except Exception as error:
            return url, b'', False, str(error)

    pending = dependencies(args.world.read_text(), BASE + 'projects/samples/environments/factory/worlds/hall.wbt')
    with ThreadPoolExecutor(max_workers=6) as pool:
        while pending:
            batch = sorted(pending - seen)
            if not batch:
                break
            seen.update(batch)
            pending = set()
            for url, content, fresh, error in pool.map(fetch, batch):
                if error:
                    failures.append((url, error))
                    print('FAILED:', url, error, flush=True)
                    continue
                downloaded += int(fresh)
                if url.endswith('.proto'):
                    pending.update(dependencies(content.decode('utf-8'), url))
            print(f'Checked {len(seen)} assets; downloaded {downloaded}; '
                  f'next dependencies {len(pending - seen)}', flush=True)
    if failures:
        raise SystemExit(f'{len(failures)} asset downloads failed; rerun before opening Webots.')
    print(f'Ready: {len(seen)} assets in {args.cache}', flush=True)


if __name__ == '__main__':
    main()
