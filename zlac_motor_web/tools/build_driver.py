#!/usr/bin/env python3
"""Build the untouched external package; keep every build product in the web folder."""
import argparse
import hashlib
import json
import re
import subprocess
from pathlib import Path


def fingerprint(source):
    paths = [source/'CMakeLists.txt',source/'package.xml',*sorted((source/'src').glob('*.cpp')),
             *sorted((source/'include').rglob('*.hpp'))]
    return {str(p.relative_to(source)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}


def read_config(source):
    header=(source/'include/zlac_driver/zlac_driver.hpp').read_text()
    values={}
    for name in ['wheel_base','wheel_radius','rpm_max','max_linear_speed','dir_left','dir_right']:
        match=re.search(r'\bdouble\s+'+name+r'\s*=\s*([+\-\d.eE]+)\s*;',header)
        if not match: raise ValueError('Cannot read original default: '+name)
        values[name]=float(match[1])
    timeouts=[]
    for name in ['zlac_driver.cpp','zlac_encoder.cpp']:
        match=re.search(r'VTIME\]\s*=\s*(\d+)',(source/'src'/name).read_text())
        if not match: raise ValueError('Cannot inspect serial timeout')
        timeouts.append(int(match[1])*100)
    values['io_timeout_ms']=max(timeouts)
    return values


def verify_manifest(path):
    manifest=json.loads(Path(path).read_text())
    if fingerprint(Path(manifest['source'])) != manifest['source_hashes']:
        raise ValueError('Driver source changed; rebuild with tools/build_driver.py')
    executable=Path(manifest['executable'])
    if hashlib.sha256(executable.read_bytes()).hexdigest() != manifest['executable_sha256']:
        raise ValueError('Driver binary changed; rebuild with tools/build_driver.py')
    if read_config(Path(manifest['source'])) != manifest['config']:
        raise ValueError('Driver configuration manifest mismatch')
    return manifest


def main():
    root=Path(__file__).resolve().parents[1]
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,default=root.parent/'zlac_driver_encoder')
    args=parser.parse_args();source=args.source.resolve();runtime=root/'.runtime'
    original=fingerprint(source);config=read_config(source)
    runtime.mkdir(exist_ok=True)
    subprocess.run(['colcon','--log-base',str(runtime/'log'),'build','--base-paths',str(source),
                    '--packages-select','zlac_driver','--build-base',str(runtime/'build'),
                    '--install-base',str(runtime/'install')],check=True,cwd=root)
    if fingerprint(source)!=original:
        raise RuntimeError('Source changed during build. Rebuild before use.')
    executable=runtime/'install/zlac_driver/lib/zlac_driver/zlac_node'
    manifest=dict(source=str(source),source_hashes=original,config=config,executable=str(executable),
                  executable_sha256=hashlib.sha256(executable.read_bytes()).hexdigest())
    path=runtime/'driver_manifest.json';path.write_text(json.dumps(manifest,indent=2)+'\n')
    print('Built unchanged driver. Manifest: '+str(path))


if __name__=='__main__':main()
