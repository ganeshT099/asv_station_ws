"""Static contracts for the standalone path-tracking demonstration."""

from pathlib import Path
import unittest


PACKAGE_ROOT = Path(__file__).parents[1]


class PathTrackingContractTest(unittest.TestCase):
    """Keep the independent package connected end-to-end."""

    def test_setup_installs_all_user_visible_assets(self):
        text = (PACKAGE_ROOT / 'setup.py').read_text()
        for value in (
            "'config/*.yaml'",
            "'launch/*.launch.py'",
            "'rviz/*.rviz'",
            "'dashboard/*.html'",
            "'dashboard/*.js'",
            "'dashboard/*.css'",
        ):
            self.assertIn(value, text)

    def test_launch_starts_the_independent_stack(self):
        text = (PACKAGE_ROOT / 'launch' /
                'path_tracking_demo.launch.py').read_text()
        for value in (
            'path_generator',
            'path_tracking_controller',
            'path_tracking_visualization',
            'path_tracking_gazebo',
            'path_tracking_dashboard',
            'path_tracking.rviz',
            'research_catamaran.urdf',
            'MarkerManager',
        ):
            self.assertIn(value, text)

    def test_gazebo_marker_bridge_is_a_separate_fortress_package(self):
        bridge_root = PACKAGE_ROOT.parent / 'asv_path_tracking_gazebo'
        cmake = (bridge_root / 'CMakeLists.txt').read_text()
        source = (bridge_root / 'src' /
                  'gazebo_path_marker_bridge.cpp').read_text()
        self.assertIn('ignition-msgs8', cmake)
        self.assertIn('ignition-transport11', cmake)
        self.assertIn('ADD_MODIFY', source)
        self.assertIn('/marker', source)

    def test_topics_are_explicit_and_separate(self):
        generator = (PACKAGE_ROOT / 'asv_path_tracking' /
                     'path_generator.py').read_text()
        controller = (PACKAGE_ROOT / 'asv_path_tracking' /
                      'path_tracking_controller.py').read_text()
        visualization = (PACKAGE_ROOT / 'asv_path_tracking' /
                         'path_tracking_visualization.py').read_text()
        for text in (generator, controller, visualization):
            self.assertIn('/asv_path_tracking/', text)
        self.assertIn('/asv_path_tracking/reference_path', generator)
        self.assertIn('/asv_path_tracking/actual_path', visualization)
        self.assertIn('/asv_path_tracking/status', controller)
        self.assertIn('/asv_path_tracking/reset', controller)

    def test_rviz_preloads_reference_and_actual_displays(self):
        text = (PACKAGE_ROOT / 'rviz' / 'path_tracking.rviz').read_text()
        for value in (
            '/asv_path_tracking/reference_path',
            '/asv_path_tracking/actual_path',
            '/asv_path_tracking/reference_markers',
            '/asv_path_tracking/markers',
            'Fixed Frame: world',
        ):
            self.assertIn(value, text)

    def test_dashboard_has_required_actions(self):
        text = (PACKAGE_ROOT / 'dashboard' / 'app.js').read_text()
        for value in ('/api/run', '/api/reset', '/api/parameters', 'reference'):
            self.assertIn(value, text)


if __name__ == '__main__':
    unittest.main()
