import os, subprocess, time, sys, json
from pathlib import Path
FOLDER = os.path.join(os.path.dirname(os.path.realpath(__file__)),'..','parameter_files')

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Must pass in a calbration file to {}!'.format(Path(__file__).name))
    calfile = sys.argv[1]
    cal = json.loads(Path(calfile).read_text())
    ns = cal.get('namespace','')
    if ns == ' ':
        ns = ''
    parms = cal.get('ros2_parameters','').split(',')
    for node in cal.get('nodes',[]).split(','):
        cmd = ['ros2','run','boring_nodes',node.strip(),'--ros-args']
        if len(ns) > 0: cmd += ['-r','__ns:=/{}'.format(ns)]
        cmd += ['-p','calibration_file:='+calfile]
        for p in parms:
            if len(p.strip()) > 0: cmd += ['--params-file',os.path.join(FOLDER,p.strip())]
        # print(cmd,flush=True)
        subprocess.Popen(cmd)
    time.sleep(100000000)
