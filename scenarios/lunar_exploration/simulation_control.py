"""Runtime-only Webots pause/resume and state snapshot for failed trials.

Attach to the external lunar_runtime_control Supervisor robot. Does not alter
the team's source world. A paused experiment is not a successful mission.
"""
import json
import time
import sys
from pathlib import Path

from controller import Supervisor


supervisor = Supervisor()
command_file = Path('/tmp/lunar_simulation_command')
command_file.write_text(sys.argv[1] if len(sys.argv)>1 else 'pause')
last = None
while True:
    command = command_file.read_text().strip()
    if command != last:
        if command == 'pause':
            supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
            data = {'sim_time':supervisor.getTime(),'experiment_complete':False,
                    'robots':{f'robot{i}':supervisor.getFromDef(f'LIMO_{i}').getPosition() for i in range(1,5)}}
            Path('/tmp/lunar_physics_snapshot.json').write_text(json.dumps(data))
            print('Experiment paused; state preserved',json.dumps(data),flush=True)
        elif command == 'resume':
            supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_REAL_TIME)
            print('Experiment resumed',flush=True)
        elif command == 'exit':
            break
        last = command
    if last == 'resume':
        if supervisor.step(int(supervisor.getBasicTimeStep())) == -1:
            break
    else:
        time.sleep(.1)
