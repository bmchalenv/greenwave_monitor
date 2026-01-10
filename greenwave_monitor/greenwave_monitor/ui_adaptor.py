#!/usr/bin/env python3

# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

"""
Greenwave monitor diagnostics helpers for UI frontends.

This module contains small data containers plus a monitor class intended to be embedded in
UI processes. The `greenwave_monitor_node` publishes `diagnostic_msgs/DiagnosticArray`
messages on `/diagnostics`. `GreenwaveUiAdaptor` subscribes to that topic and maintains a
thread-safe, easy-to-consume view (`UiDiagnosticData`) per monitored topic, including the
timestamp of the last update for each topic.

In addition to passively subscribing, `GreenwaveUiAdaptor` exposes:
- Parameter-based topic monitoring: start/stop monitoring a topic via the
  `greenwave_diagnostics.<topic>.enabled` parameter (`toggle_topic_monitoring`).
- Parameter-based frequency configuration: set/clear the expected publish rate and tolerance
  for a topic (`set_expected_frequency`) via ROS parameters. Expected rates are also cached
  locally in `expected_frequencies` as `(expected_hz, tolerance_percent)` so UIs can display
  the configured values alongside live diagnostics.
"""

from dataclasses import dataclass
import math
import threading
import time
from typing import Dict

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
import rclpy
from rclpy.node import Node

# Parameter name constants
TOPIC_PARAM_PREFIX = 'greenwave_diagnostics.'
FREQ_SUFFIX = '.expected_frequency'
TOL_SUFFIX = '.tolerance'
ENABLED_SUFFIX = '.enabled'
DEFAULT_TOLERANCE_PERCENT = 5.0


def build_full_node_name(node_name: str, node_namespace: str, is_client: bool = False) -> str:
    """Build full ROS node name from name and namespace."""
    join_list = []
    # Strip leading '/' from namespace to avoid double slashes when joining
    if node_namespace and node_namespace != '/':
        join_list.append(node_namespace.lstrip('/'))
    if node_name:
        join_list.append(node_name)
    joined = '/'.join(join_list)
    if not is_client:
        return f'/{joined}'
    return joined


def make_freq_param(topic: str) -> str:
    """Build frequency parameter name for a topic."""
    return f'{TOPIC_PARAM_PREFIX}{topic}{FREQ_SUFFIX}'


def make_tol_param(topic: str) -> str:
    """Build tolerance parameter name for a topic."""
    return f'{TOPIC_PARAM_PREFIX}{topic}{TOL_SUFFIX}'


def _make_param(name: str, value) -> Parameter:
    """Create a Parameter message from a name and Python value (or ParameterValue)."""
    param = Parameter()
    param.name = name
    if isinstance(value, ParameterValue):
        param.value = value
    else:
        param.value = ParameterValue()
        if isinstance(value, str):
            param.value.type = ParameterType.PARAMETER_STRING
            param.value.string_value = value
        elif isinstance(value, bool):
            param.value.type = ParameterType.PARAMETER_BOOL
            param.value.bool_value = value
        elif isinstance(value, int):
            param.value.type = ParameterType.PARAMETER_INTEGER
            param.value.integer_value = value
        else:
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = float(value)
    return param


def set_ros_parameters(node: Node, target_node: str, params: dict,
                       timeout_sec: float = 3.0) -> tuple[bool, list[str]]:
    """
    Set one or more parameters on a target node.

    :param node: The ROS node to use for service calls.
    :param target_node: Full name of target node (e.g., '/my_node' or '/ns/my_node').
    :param params: Dict mapping parameter names to values.
    :param timeout_sec: Service call timeout.
    :returns: Tuple of (all_successful, list of failure reasons).
    """
    if '/' not in target_node:
        target_node = f'/{target_node}'
    client = node.create_client(SetParameters, f'{target_node}/set_parameters')
    if not client.wait_for_service(timeout_sec=min(timeout_sec, 5.0)):
        node.destroy_client(client)
        return False, ['Service not available']

    request = SetParameters.Request()
    request.parameters = [_make_param(name, value) for name, value in params.items()]

    try:
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
        node.destroy_client(client)

        if future.result() is None:
            return False, ['Service call timed out']

        results = future.result().results
        failures = [r.reason for r in results if not r.successful and r.reason]
        return all(r.successful for r in results), failures
    except Exception as e:
        node.destroy_client(client)
        return False, [str(e)]


def get_ros_parameters(node: Node, target_node: str, param_names: list,
                       timeout_sec: float = 5.0) -> dict:
    """
    Get parameters from a target node.

    :param node: The ROS node to use for service calls.
    :param target_node: Full name of target node (e.g., '/my_node' or '/ns/my_node').
    :param param_names: List of parameter names to retrieve.
    :param timeout_sec: Service call timeout.
    :returns: Dict mapping parameter names to their values (float or None if not found/invalid).
    """
    client = node.create_client(GetParameters, f'{target_node}/get_parameters')
    if not client.wait_for_service(timeout_sec=min(timeout_sec, 5.0)):
        node.destroy_client(client)
        return {name: None for name in param_names}

    request = GetParameters.Request()
    request.names = param_names

    try:
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
        node.destroy_client(client)

        if future.result() is None or not future.result().values:
            return {name: None for name in param_names}

        result = {}
        for name, value in zip(param_names, future.result().values):
            result[name] = param_value_to_python(value)
        return result
    except Exception:
        node.destroy_client(client)
        return {name: None for name in param_names}


def param_value_to_python(value: ParameterValue):
    """Convert a ParameterValue to its Python equivalent."""
    if value.type == ParameterType.PARAMETER_BOOL:
        return value.bool_value
    elif value.type == ParameterType.PARAMETER_DOUBLE:
        return value.double_value
    elif value.type == ParameterType.PARAMETER_INTEGER:
        return value.integer_value
    elif value.type == ParameterType.PARAMETER_STRING:
        return value.string_value
    return None


_STATUS_LEVEL_MAP = {
    DiagnosticStatus.OK: 'OK',
    DiagnosticStatus.WARN: 'WARN',
    DiagnosticStatus.ERROR: 'ERROR',
    DiagnosticStatus.STALE: 'STALE',
}


@dataclass
class UiDiagnosticData:
    """
    UI-ready snapshot of diagnostics for a monitored topic.

    Fields are stored as strings for straightforward rendering in UI components.
    `status` is one of: 'OK' | 'WARN' | 'ERROR' | 'STALE' | 'UNKNOWN' (or '-' if unset).
    `last_update` stores the epoch timestamp when diagnostics were last refreshed.
    """

    node_name: str = '-'
    pub_rate: str = '-'
    msg_rate: str = '-'
    latency: str = '-'
    expected_frequency: str = '-'
    tolerance: str = '-'
    status: str = '-'
    last_update: float = 0.0

    @classmethod
    def from_status(cls, status: DiagnosticStatus) -> 'UiDiagnosticData':
        """Create UiDiagnosticData from DiagnosticStatus."""
        data = cls()
        data.status = _STATUS_LEVEL_MAP.get(status.level, 'UNKNOWN')

        for kv in status.values:
            if kv.key == 'frame_rate_node':
                data.pub_rate = kv.value
            elif kv.key == 'frame_rate_msg':
                data.msg_rate = kv.value
            elif kv.key == 'current_delay_from_realtime_ms':
                data.latency = kv.value
            elif kv.key == 'expected_frequency':
                data.expected_frequency = kv.value
            elif kv.key == 'tolerance':
                data.tolerance = kv.value
        return data


class GreenwaveUiAdaptor:
    """
    Subscribe to `/diagnostics` and manage topic monitoring for UI consumption.

    Designed for UI frontends, this class keeps per-topic `UiDiagnosticData` up to date,
    provides a toggle for monitoring via the `greenwave_diagnostics.<topic>.enabled` parameter,
    and exposes helpers to set/clear expected frequencies via ROS parameters.

    """

    def __init__(self, node: Node, monitor_node_name: str = 'greenwave_monitor'):
        """Initialize the UI adaptor for subscribing to diagnostics and managing topics."""
        self.node = node
        # Just use the bare node name - ROS will expand with the caller's namespace
        self.monitor_node_name = monitor_node_name
        self.data_lock = threading.Lock()
        self.ui_diagnostics: Dict[str, UiDiagnosticData] = {}
        # { topic_name : (expected_hz, tolerance) }
        self.expected_frequencies: Dict[str, tuple[float, float]] = {}
        # Cache for topic to target node mapping
        self._topic_node_cache: Dict[str, str] = {}

        self._setup_ros_components()

    def _setup_ros_components(self):
        """Initialize ROS2 subscriptions, clients, and timers."""
        self.subscription = self.node.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self._on_diagnostics,
            100
        )


    def _parse_diagnostic_name(self, diagnostic_name: str) -> tuple[str, str]:
        """
        Parse node name and topic from diagnostic status name.

        Format: "<node> <topic>" (e.g., "my_node /camera/image")
        Returns: (node_name, topic_name)
        """
        idx = diagnostic_name.find(' /')
        if idx >= 0:
            return (diagnostic_name[:idx], diagnostic_name[idx + 1:])
        return ('', diagnostic_name)

    def _on_diagnostics(self, msg: DiagnosticArray):
        """Process incoming diagnostic messages."""
        with self.data_lock:
            for status in msg.status:
                ui_data = UiDiagnosticData.from_status(status)
                ui_data.last_update = time.time()
                node_name, topic_name = self._parse_diagnostic_name(status.name)
                ui_data.node_name = node_name
                self.ui_diagnostics[topic_name] = ui_data
                # Update topic to node cache from diagnostics
                if node_name:
                    self._topic_node_cache[topic_name] = node_name
                # Update expected_frequencies from diagnostics message
                try:
                    freq = float(ui_data.expected_frequency)
                    tol = float(ui_data.tolerance)
                    if freq > 0 and not math.isnan(freq):
                        self.expected_frequencies[topic_name] = (freq, tol)
                    else:
                        self.expected_frequencies.pop(topic_name, None)
                except (ValueError, TypeError):
                    self.expected_frequencies.pop(topic_name, None)

    def _get_target_node_for_topic(self, topic_name: str) -> str:
        """
        Get the target node for setting parameters on a topic.

        Checks if the topic's publisher node has the enabled parameter.
        Returns the publisher node name if it does, otherwise returns the default monitor node.
        Results are cached.
        """
        with self.data_lock:
            if topic_name in self._topic_node_cache:
                return self._topic_node_cache[topic_name]

        # Get publishers for this topic
        publishers = self.node.get_publishers_info_by_topic(topic_name)
        if not publishers:
            with self.data_lock:
                self._topic_node_cache[topic_name] = self.monitor_node_name
            return self.monitor_node_name

        # Get the first publisher's node name
        pub_info = publishers[0]
        pub_node_name = f'/{pub_info.node_namespace.lstrip("/")}/{pub_info.node_name}'.replace('//', '/')
        if pub_node_name.startswith('//'):
            pub_node_name = pub_node_name[1:]

        # Check if this node has the enabled parameter for this topic
        enabled_param = f'{TOPIC_PARAM_PREFIX}{topic_name}{ENABLED_SUFFIX}'
        params = get_ros_parameters(self.node, pub_node_name, [enabled_param], timeout_sec=1.0)

        if params.get(enabled_param) is not None:
            with self.data_lock:
                self._topic_node_cache[topic_name] = pub_node_name
            return pub_node_name

        with self.data_lock:
            self._topic_node_cache[topic_name] = self.monitor_node_name
        return self.monitor_node_name

    def toggle_topic_monitoring(self, topic_name: str):
        """Toggle monitoring for a topic via enabled parameter."""
        with self.data_lock:
            is_monitoring = topic_name in self.ui_diagnostics
        new_enabled = not is_monitoring
        action = 'start' if new_enabled else 'stop'

        target_node = self._get_target_node_for_topic(topic_name)
        enabled_param = f'{TOPIC_PARAM_PREFIX}{topic_name}{ENABLED_SUFFIX}'
        success, failures = set_ros_parameters(
            self.node, target_node, {enabled_param: new_enabled})

        if not success:
            self.node.get_logger().error(
                f'Failed to {action} monitoring: {"; ".join(failures)}')
            return

        with self.data_lock:
            if not new_enabled and topic_name in self.ui_diagnostics:
                self.ui_diagnostics.pop(topic_name, None)

    def set_expected_frequency(self,
                               topic_name: str,
                               expected_hz: float = 0.0,
                               tolerance_percent: float = 0.0,
                               clear: bool = False
                               ) -> tuple[bool, str]:
        """Set or clear the expected frequency for a topic via parameters."""
        action = 'clear' if clear else 'set'

        target_node = self._get_target_node_for_topic(topic_name)
        params = {
            make_freq_param(topic_name): float('nan') if clear else expected_hz,
            make_tol_param(topic_name): DEFAULT_TOLERANCE_PERCENT if clear else tolerance_percent,
        }

        success, failures = set_ros_parameters(self.node, target_node, params)

        if not success:
            return False, f'Failed to {action} expected frequency: {"; ".join(failures)}'

        return True, f'{"Cleared" if clear else "Set"} expected frequency for {topic_name}'

    def get_topic_diagnostics(self, topic_name: str) -> UiDiagnosticData:
        """Get diagnostic data for a topic. Returns default values if topic not found."""
        with self.data_lock:
            return self.ui_diagnostics.get(topic_name, UiDiagnosticData())

    def get_expected_frequency(self, topic_name: str) -> tuple[float, float]:
        """Get monitoring settings for a topic. Returns (0.0, 0.0) if not set."""
        with self.data_lock:
            return self.expected_frequencies.get(topic_name, (0.0, 0.0))

    def get_expected_frequency_str(self, topic_name: str) -> str:
        """Get expected frequency as formatted string with tolerance (e.g., '30.0Hz ±5%')."""
        freq, tol = self.get_expected_frequency(topic_name)
        if freq <= 0.0 or math.isnan(freq):
            return '-'
        if tol > 0.0:
            return f'{freq:.1f}Hz±{tol:.0f}%'
        return f'{freq:.1f}Hz'
