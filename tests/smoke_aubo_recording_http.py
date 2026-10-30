"""Real local HTTP routes with the fake native SDK; explicitly synthetic, no robot."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.request
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from teleop.aubo.quest_follow import main
from teleop.aubo.recording import RawRecorder


runtime=Path('/data/VR_teleoperation_runtime')
binary=next((runtime/'build-follow/fake-python').glob('_aubo_follow*.so'))
spec=importlib.util.spec_from_file_location('_aubo_follow',binary)
native=importlib.util.module_from_spec(spec)
spec.loader.exec_module(native)


def wait_for(predicate):
    deadline=time.monotonic()+4.
    while True:
        try:
            if predicate():return
        except (urllib.error.URLError, KeyError):
            pass
        if time.monotonic()>deadline:raise AssertionError('Synthetic HTTP state timed out')
        time.sleep(.01)


def test():
    base='http://127.0.0.1:18453'
    def get_status():
        with urllib.request.urlopen(base+'/aubo/status',timeout=.5) as response:
            return json.load(response)
    def command(value):
        with urllib.request.urlopen(urllib.request.Request(base+'/aubo/operator/'+value,method='POST'),timeout=.5) as response:
            assert response.status==202
            assert json.load(response)['queued']
    def synthetic_recorder(*args, **kwargs):
        kwargs['metadata'].update(synthetic=True,hardware_connected=False)
        return RawRecorder(*args,**kwargs)
    with tempfile.TemporaryDirectory(prefix='synthetic-recording-http-',dir=runtime) as directory:
        path=Path(directory)
        failures=[]
        def run():
            try:
                main(['--execute-six-axis','--command-mode','follow','--follow-hz','25',
                      '--serial-state-query','--waypoint-feedback','--host','offline','--port','1','--user','test',
                      '--usb','--headless','--relay-port','18453','--duration','15',
                      '--record',str(path/'raw'),'--task','ISOLATED SYNTHETIC HTTP TEST; no robot'])
            except BaseException as exc:
                failures.append(exc)
        with patch('teleop.aubo.quest_follow.ROOT',path), \
             patch('teleop.aubo.quest_follow.load_follow_native',return_value=native), \
             patch('teleop.aubo.quest_follow.RawRecorder',side_effect=synthetic_recorder), \
             patch('teleop.aubo.quest_follow.getpass.getpass',return_value='synthetic'), \
             patch.object(native,'RobotStateObserver',side_effect=AssertionError('Unexpected SDK B')):
            thread=threading.Thread(target=run,daemon=True)
            thread.start()
            try:
                def ready():
                    if failures:
                        raise failures[0]
                    return get_status()['collection']['state']=='ready'
                wait_for(ready)
                assert get_status()['hardware']['input_timeout_s']==1.
                assert json.loads((path/'raw/manifest.json').read_text())['metadata']['input_timeout_s']==1.
                with urllib.request.urlopen(base+'/aubo',timeout=.5) as response:
                    html=response.read().decode()
                assert 'query-mode' in html and 'collection' in html and 'toggle_episode' in html
                assert '视野下缘会显示录制状态卡' in html
                assert 'restart_loop' in html and 'exit_loop' in html and 'exit_record' in html
                assert '左 Y 按住直接回原位' in html and '无需右 Grip' in html
                assert '左摇杆左推＝作废当前段并重启循环' in html
                assert '右摇杆按下可添加事件标记' in html
                assert 'left X=start' not in html
                assert '输入超时会暂停' in html and '重新按下并成功接合后续录同一段' in html
                with urllib.request.urlopen(base+'/static/recording_hud.js',timeout=.5) as response:
                    hud_script=response.read().decode()
                assert 'VRTeleopRecordingHud' in hud_script
                ready_status=get_status()['collection']
                assert ready_status['active_episode_elapsed_s'] is None
                assert ready_status['saving_episodes']==[]
                command('restart_episode')
                wait_for(lambda:get_status()['recording'])
                active_status=get_status()['collection']
                assert active_status['active_episode']==0
                assert active_status['active_episode_elapsed_s'] is not None
                command('software_reset')
                wait_for(lambda:get_status()['operator_message'].startswith('软件复位完成'))
                assert get_status()['collection']['active_episode']==0
                command('mark');command('pause')
                wait_for(lambda:get_status()['operator_paused'])
                command('pause')
                wait_for(lambda:not get_status()['operator_paused'])
                command('restart_episode')
                wait_for(lambda:get_status()['collection']['active_episode']==1)
                restarted_status=get_status()['collection']
                assert restarted_status['saved_episodes']==1
                assert restarted_status['last_result']==dict(episode_id=0,disposition='saved',flushed=True)
                assert json.loads((path/'raw/episodes/episode_000000.json').read_text())['complete']
                command('toggle_episode')
                wait_for(lambda:get_status()['collection']['saved_episodes']==2)
                assert not get_status()['recording']
                assert get_status()['hardware']['submitted']==0
                command('finish_session')
            finally:
                thread.join(5.)
            assert not thread.is_alive(),'Synthetic server did not close'
            if failures:raise failures[0]
            status=json.loads((path/'raw/session_status.json').read_text())
            assert status['complete'] and not status['dropped_records']
            print(json.dumps(dict(synthetic=True,hardware_connected=False,complete=True,
                                  saved_episodes=2,sdk_connections=1,robot_commands=0),indent=2))


if __name__=='__main__':test()
