# ros_bridge.py
import threading
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node as RclNode
from rclpy.parameter import Parameter


class ROSBridge:
    """
    단일 ROS 노드와 executor를 관리하는 싱글톤.
    모든 BT 노드들이 bridge.node를 통해 ROS와 통신한다.
    """

    _instance = None

    def __init__(self, node_name='space_bt_bridge', namespace='', use_sim_time=False, threaded=True):
        if ROSBridge._instance is not None:
            raise RuntimeError("ROSBridge is a singleton. Use ROSBridge.get().")

        # rclpy 초기화
        rclpy.init(args=None)

        # node 생성
        self.node = RclNode(node_name=node_name, namespace=namespace,
                            parameter_overrides=[Parameter('use_sim_time', value=use_sim_time)])

        # executor 생성
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

        # spin을 백그라운드에서 돌리는 스레드
        self.threaded = threaded
        self._spin_thread = None
        if threaded:
            self._spin_thread = threading.Thread(target=self._spin, daemon=True)
            self._spin_thread.start()

        ROSBridge._instance = self

    def _spin(self):
        try:
            self.executor.spin()
        except Exception as e:
            print(f"[ROSBridge] Spin thread stopped: {e}")

    @classmethod
    def get(cls, node_name='space_bt_bridge', namespace='', **kwargs):
        """
        싱글톤 인스턴스를 반환. 최초 호출 시에만 생성된다.
        """
        if cls._instance is None:
            cls._instance = ROSBridge(node_name=node_name, namespace=namespace, **kwargs)
        return cls._instance

    def spin_once(self):
        if not self.threaded:
            # ROS callbacks and BT state share one thread; bound work per tick.
            for _ in range(40):
                self.executor.spin_once(timeout_sec=0.0)

    def shutdown(self):
        """Stop executor thread and shutdown rclpy cleanly."""
        try:
            self.executor.shutdown()
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        ROSBridge._instance = None  # allow re-init later

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.shutdown()
