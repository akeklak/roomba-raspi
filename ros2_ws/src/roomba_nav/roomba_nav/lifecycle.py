"""Brings lifecycle nodes (slam_toolbox, map_server, AMCL and Nav2's servers) up to active.

Launch files can do this with events, and Nav2 has its own lifecycle manager, but both give up for
good (or wait forever) when one service reply goes missing, which happens while a busy computer
(the Pi, or the sim starting) is still discovering everything. This asks each node for its state
and moves it on one step at a time, checking again after each step, so a lost reply only costs a
retry.
"""

import time
from typing import Callable, Dict, List, Optional, Tuple

from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState

# Nav2's servers (navigation.launch.py starts them), in the order they are brought up.
NAV2_NODES = ['controller_server', 'planner_server', 'behavior_server', 'velocity_smoother', 'bt_navigator']

# What to do from each state to get closer to active.
NEXT = {
    State.PRIMARY_STATE_UNCONFIGURED: Transition.TRANSITION_CONFIGURE,
    State.PRIMARY_STATE_INACTIVE: Transition.TRANSITION_ACTIVATE,
}


class Lifecycle:

    def __init__(self, node, call: Callable, callback_group=None) -> None:
        """`call(client, request, timeout)` makes a service call and returns the response, or None
        if there was no answer in time."""
        self.node = node
        self.call = call
        self.group = callback_group
        self.clients: Dict[str, Tuple] = {}

    def _clients(self, name: str):
        if name not in self.clients:
            self.clients[name] = (
                self.node.create_client(GetState, f'/{name}/get_state', callback_group=self.group),
                self.node.create_client(ChangeState, f'/{name}/change_state', callback_group=self.group),
            )
        return self.clients[name]

    def state(self, name: str) -> Optional[int]:
        """The node's lifecycle state id, or None if it doesn't answer."""
        response = self.call(self._clients(name)[0], GetState.Request(), 3.0)
        return None if response is None else response.current_state.id

    def bring_up(self, names: List[str], timeout: float,
                 keep_going: Callable[[], bool] = lambda: True) -> Optional[str]:
        """Activates the nodes in order. Returns None once all are active, else the name of the
        first one that isn't (it didn't answer, failed, or `keep_going` said to stop)."""
        end = time.monotonic() + timeout
        for name in names:
            while True:
                if time.monotonic() > end or not keep_going():
                    return name
                state = self.state(name)
                if state == State.PRIMARY_STATE_ACTIVE:
                    break
                if state == State.PRIMARY_STATE_FINALIZED:
                    return name
                transition = NEXT.get(state)
                if transition is None:  # no answer yet, or in the middle of a transition
                    time.sleep(0.5)
                    continue
                request = ChangeState.Request()
                request.transition.id = transition
                # Configuring can take a while (slam_toolbox loading a saved map). Whatever the
                # answer, the state is read back on the next pass.
                self.call(self._clients(name)[1], request, 60.0)
        return None
