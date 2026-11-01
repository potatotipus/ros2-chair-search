"""Focused offline checks; no ROS installation or robot connections required."""
import ast
import json
import math
import time
from pathlib import Path
from types import SimpleNamespace as NS


class FakeNode:
    def __init__(self, name):
        self.params = {}
        self.logs = []
        self.now_ns = 10_000_000_000

    @property
    def handle(self):
        return object()

    @handle.setter
    def handle(self, value):
        raise AttributeError('handle cannot be modified after node creation')

    def declare_parameter(self, name, value):
        self.params[name] = value

    def get_parameter(self, name):
        return NS(value=self.params[name])

    def get_clock(self):
        return NS(now=lambda: NS(nanoseconds=self.now_ns))

    def get_logger(self):
        return NS(info=self.logs.append, warning=self.logs.append, error=self.logs.append)

    def create_subscription(self, *args):
        return object()

    def create_timer(self, *args):
        return object()


path = Path(__file__).resolve().parents[1] / 'search_chair.py'
tree = ast.parse(path.read_text(encoding='utf-8'))
class_ast = next(n for n in tree.body if isinstance(n, ast.ClassDef))
scope = dict(Node=FakeNode, ActionClient=lambda *args: NS(server_is_ready=lambda: False),
             NavigateToPose=object(), Spin=NS(Goal=lambda: NS(time_allowance=NS())), String=object(),
             json=json, math=math, time=time,
             GoalStatus=NS(STATUS_CANCELED=5, STATUS_SUCCEEDED=4))
exec(compile(ast.Module(body=[class_ast], type_ignores=[]), str(path), 'exec'), scope)
ChairSearch = scope['ChairSearch']
node = ChairSearch()
assert node._search_goal_handle is None and not node.done
print('PASS: construction with read-only Node.handle')


def frame(sec, found=True):
    return NS(data=json.dumps({'stamp': {'sec': sec, 'nanosec': 0},
                              'detections': [{'label': 'chair', 'confidence': 0.8}] if found else []}))


node.on_detection(frame(8))
node.on_detection(frame(8))
assert node.votes == 1
node.on_detection(frame(9))
assert not node.done
node.on_detection(frame(10))
assert node.done and node.stop_reason.startswith('FOUND')
print('PASS: distinct fresh frames required; already-visible chair finishes')

node = ChairSearch()
cancel_calls = []


def cancel():
    cancel_calls.append(True)
    return NS(add_done_callback=lambda callback: callback(
        NS(result=lambda: NS(goals_canceling=[object()]))))


node._search_goal_handle = NS(cancel_goal_async=cancel)
node.request_stop('FOUND')
assert len(cancel_calls) == 1 and not node.done
node.completed(NS(result=lambda: NS(status=5, result=NS())))
assert node.done and node._search_goal_handle is None
print('PASS: active goal canceled; result received before completion')

node = ChairSearch()
node.now_ns = 0
node.tick()
assert node.started is None and not node.done
node.now_ns = 800_000_000_000
node.tick()
assert node.started == 800 and not node.done
node.now_ns = 804_000_000_000
node.tick()
assert not node.done
node.now_ns = 831_000_000_000
node.tick()
assert node.done and 'AI frames=0' in node.logs[-1]
assert 'navigate_to_pose ready=False' in node.logs[-1]
print('PASS: first simulation clock starts timeout; diagnostics identify missing inputs')

node = ChairSearch()
node.state = 'scanning'
node._scan_progress = 1.2
node._search_goal_handle = NS(cancel_goal_async=cancel)
before = len(cancel_calls)
node.on_detection(frame(10))
assert node._verify_requested and len(cancel_calls) == before + 1
node.completed(NS(result=lambda: NS(status=5, result=NS())))
assert node.state == 'verifying' and not node.done
assert abs(node._scan_remaining - (2 * math.pi - 1.2)) < 1e-6
for sec, found in [(11, True), (12, False), (13, True), (14, False), (15, True)]:
    node.now_ns = sec * 1_000_000_000
    node.on_detection(frame(sec, found))
assert node.done and node.stop_reason.startswith('FOUND')
print('PASS: first chair hit cancels rotation; 3 of 5 stationary frames confirms')

node = ChairSearch()
node.state = 'verifying'
node._scan_remaining = 4.0
sent = []
node.send = lambda client, goal, state: sent.append((goal.target_yaw, state))
for sec in range(10, 15):
    node.now_ns = sec * 1_000_000_000
    node.on_detection(frame(sec, found=False))
assert not node.done and sent == [(4.0, 'scanning')]
print('PASS: unconfirmed candidate resumes only the remaining turn')
