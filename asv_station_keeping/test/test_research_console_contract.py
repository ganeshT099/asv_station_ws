"""Static contracts for the live research demonstration console."""

from pathlib import Path
import unittest


PACKAGE_ROOT = Path(__file__).parents[1]


class ResearchConsoleContractTest(unittest.TestCase):
    """Keep the one-command console and its user-visible assets connected."""

    def test_visual_launch_starts_all_three_views(self):
        text = (PACKAGE_ROOT / 'launch' /
                'visual_station_keeping.launch.py').read_text()
        self.assertIn('research_station_keeping_visualization', text)
        self.assertIn('research_station_keeping_dashboard', text)
        self.assertIn("package='rviz2'", text)
        self.assertIn('dynamic_station_keeping.launch.py', text)

    def test_dashboard_assets_are_installed_and_referenced(self):
        setup_text = (PACKAGE_ROOT / 'setup.py').read_text()
        dashboard_text = (
            PACKAGE_ROOT / 'asv_station_keeping' /
            'station_keeping_dashboard.py').read_text()
        self.assertIn("share/' + package_name + '/dashboard'", setup_text)
        self.assertIn("'dashboard/*.html'", setup_text)
        self.assertIn("'dashboard/*.js'", setup_text)
        self.assertIn("'dashboard/*.css'", setup_text)
        self.assertIn("'/api/state'", dashboard_text)
        self.assertIn("'/api/parameters'", dashboard_text)
        self.assertIn("'/api/actions/start_demo'", dashboard_text)

    def test_live_parameter_and_reset_paths_exist(self):
        controller_text = (
            PACKAGE_ROOT / 'asv_station_keeping' /
            'station_keeping_pid.py').read_text()
        dashboard_text = (
            PACKAGE_ROOT / 'asv_station_keeping' /
            'station_keeping_dashboard.py').read_text()
        self.assertIn('add_on_set_parameters_callback', controller_text)
        self.assertIn('reset_integrators_callback', controller_text)
        self.assertIn('/research_catamaran/station_keeping/reset_integrators',
                      dashboard_text)
        self.assertIn('SetParameters', dashboard_text)
        demo_text = (
            PACKAGE_ROOT / 'asv_station_keeping' /
            'station_keeping_demo.py').read_text()
        self.assertIn('phase_duration', demo_text)
        self.assertIn('/research_catamaran/station_keeping/start_demo',
                      demo_text)

    def test_visualization_has_an_explicit_path_trace(self):
        model_text = (
            PACKAGE_ROOT / 'asv_station_keeping' /
            'visualization_model.py').read_text()
        visualization_text = (
            PACKAGE_ROOT / 'asv_station_keeping' /
            'station_keeping_visualization.py').read_text()
        self.assertIn("'actual_path': 32", model_text)
        self.assertIn('_actual_path_marker', visualization_text)
        self.assertIn('path_visual_offset', visualization_text)

    def test_controller_slews_setpoint_changes_for_demo_stability(self):
        controller_text = (
            PACKAGE_ROOT / 'asv_station_keeping' /
            'station_keeping_pid.py').read_text()
        yaml_text = (PACKAGE_ROOT / 'config' /
                     'research_controller.yaml').read_text()
        self.assertIn('move_angle_toward', controller_text)
        self.assertIn('target_slew_rate', controller_text)
        self.assertIn('target_yaw_slew_rate', yaml_text)

    def test_research_report_assets_exist(self):
        self.assertTrue((PACKAGE_ROOT / 'diagnostics' /
                         'LIVE_DEMONSTRATION.md').is_file())
        self.assertTrue((PACKAGE_ROOT / 'dashboard' / 'index.html').is_file())
        self.assertTrue((PACKAGE_ROOT / 'rviz' /
                         'station_keeping.rviz').is_file())

    def test_evaluator_supports_archivable_reports(self):
        evaluator = (PACKAGE_ROOT / 'asv_station_keeping' /
                     'evaluate_run.py').read_text()
        self.assertIn('--json-output', evaluator)
        self.assertIn('--html-output', evaluator)
        self.assertIn('control_effort_Ns', evaluator)


if __name__ == '__main__':
    unittest.main()
