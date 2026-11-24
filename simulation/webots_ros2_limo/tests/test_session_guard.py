import tempfile
import unittest

from webots_ros2_limo.session_guard import acquire_factory_session


class SessionGuardTests(unittest.TestCase):
    def test_same_domain_is_blocked_until_owner_closes(self):
        with tempfile.TemporaryDirectory() as directory:
            with acquire_factory_session(42, directory):
                with self.assertRaisesRegex(RuntimeError, 'already running'):
                    acquire_factory_session(42, directory)
                with acquire_factory_session(43, directory):
                    pass
            with acquire_factory_session(42, directory):
                pass
