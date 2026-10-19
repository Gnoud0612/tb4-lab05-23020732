#!/usr/bin/env python3
"""Package reviewable source, maps, model assets and weights; keep reports/video separate."""
from pathlib import Path
import zipfile

root=Path(__file__).resolve().parents[1]
scripts=['env.bash','local_underlay.bash','build.sh','bootstrap_local.py','run.sh','stop_sim.sh',
         'low_resource.sh','apply_sim_profile.sh','undock.sh','prepare.sh','initialize_pose.py',
         'spawn_objects.py','perception.sh','mission.sh','setup_yolo.sh','validate_results.py','package_lab05.py']
files=[root/p for p in ['README.md','.gitignore','COLCON_IGNORE','LICENSE','THIRD_PARTY.md','THIRD_PARTY_LAB05.md','requirements-yolo.txt']]
files += [root/'scripts'/p for p in scripts]
for dirname in ['src','maps','models','weights']:
    files += [p for p in (root/dirname).rglob('*') if p.is_file()]
out=root/'submission/tb4_lab05_23020732.zip';out.parent.mkdir(exist_ok=True)
tmp=out.with_suffix('.zip.tmp')
with zipfile.ZipFile(tmp,'w',zipfile.ZIP_DEFLATED) as z:
    for p in sorted(set(files)):
        if any(v in {'__pycache__','.pytest_cache','.git'} for v in p.parts) or p.suffix=='.pyc':continue
        if not p.is_file() or p.is_symlink():raise RuntimeError('Missing or symlink source: '+str(p))
        z.write(p,str(Path('tb4_lab05')/p.relative_to(root)))
with zipfile.ZipFile(tmp) as z:
    if z.testzip() is not None:raise RuntimeError('ZIP integrity failure')
    print('Files:',len(z.namelist()))
tmp.replace(out);print(out)
