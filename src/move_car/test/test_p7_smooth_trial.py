"""Offline P7 trial preparation checks. No ROS node or vehicle is started."""
import argparse
import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import shlex
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import p7_smooth_trial as tool


class P7PrepareTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        self.production=self.root/'production'
        formal=self.production/'src/move_car/launch/movecar_improved.launch'
        formal.parent.mkdir(parents=True)
        formal.write_text('<launch><node pkg="move_car" type="control_pub" name="publish_speed_turn">'
            '<param name="wheel_speed_scale" value="0.001"/></node></launch>')
        self.formal=formal
        self.research=self.root/'research'
        for name in ('control_pub','p5_nmpc_worker_node'):
            p=self.research/'devel/lib/move_car'/name
            p.parent.mkdir(parents=True,exist_ok=True)
            p.write_text('synthetic binary fixture '+name)
        self.notes=self.root/'mapping.txt'
        self.notes.write_text('SYNTHETIC TOOL FIXTURE, NOT REAL MAPPING EVIDENCE')
        self.folder=self.root/'new_trial'
        def prepare(root,session,route,speed,production,run_id):
            self.folder.mkdir(exist_ok=False)
            e=dict(run_id=run_id,session_id=session,production_workspace=str(production))
            (self.folder/'software_freeze.json').write_text(json.dumps(dict(nominal_config_hash='base_fixture',files={})))
            (self.folder/'experiment.json').write_text(json.dumps(e))
            return self.folder,e
        runtime=dict(research_runtime=dict(config_hash='a'*64,
            profiles=dict(smooth=dict(values=dict(speed_cap=.4))),
            channel_policies=dict(yaw=dict(enabled=False)),scene_transition=dict(enabled=False)))
        for patch in (mock.patch.object(tool,'WORKSPACE',self.research),
                      mock.patch.object(tool,'prepare',side_effect=prepare),
                      mock.patch.object(tool,'prepare_runtime',return_value=runtime)):
            patch.start();self.addCleanup(patch.stop)

    def args(self,**changes):
        value=dict(operator='synthetic_fixture',imu_frame='base_link',mapping_evidence=self.notes,
            confirm_forward_wheel_positive=True,confirm_left_steering_positive=True,
            confirm_left_yaw_positive=True,speed_cap=.10,algorithm='flat_nmpc_8',
            session_id='synthetic_test_session',route='synthetic_route',output_root=self.root/'runs',
            production_workspace=self.production)
        value.update(changes)
        return argparse.Namespace(**value)

    def test_prepare_does_not_start_or_arm_and_preserves_formal(self):
        before=self.formal.read_bytes()
        with mock.patch.object(tool.subprocess,'Popen') as popen:
            folder=tool.make_trial(self.args())
        popen.assert_not_called()
        self.assertEqual(before,self.formal.read_bytes())
        _,trial=tool.read_trial(folder)
        self.assertFalse(trial['automatic_arming'])
        self.assertFalse(trial['physical_calibration_certified'])
        self.assertFalse(trial['validated_deployment'])
        self.assertFalse(trial['flat_conditioned_geometry'])
        self.assertFalse(trial['flat_baseline_seed'])
        self.assertFalse(trial['flat_delay_aligned_reconstruction'])
        self.assertEqual(trial['algorithm'],'flat_nmpc_8')
        self.assertIn('operator_supervised',trial['execution_context'])
        freeze=json.loads((folder/'software_freeze.json').read_text())
        self.assertEqual(freeze['nominal_config_hash'],trial['nominal_config_hash'])
        self.assertEqual(freeze['base_layer_config_hash'],'base_fixture')

    def test_mapping_acknowledgements_required(self):
        for flag in ('confirm_forward_wheel_positive','confirm_left_steering_positive','confirm_left_yaw_positive'):
            with self.subTest(flag=flag),self.assertRaises(ValueError):
                tool.make_trial(self.args(**{flag:False}))
        self.assertFalse(self.folder.exists())

    def test_shadow_only_keeps_s8_and_uses_requested_new_algorithm(self):
        import yaml
        config=self.root/'isolated_candidate';config.mkdir()
        (config/'ISOLATED_ONLY.json').write_text('{}')
        with mock.patch.object(tool.subprocess,'Popen') as launch:
            folder=tool.make_trial(self.args(shadow_only=True,algorithm='flat_nmpc_slqr',config_root=config))
        launch.assert_not_called()
        _,trial=tool.read_trial(folder)
        self.assertEqual(trial['algorithm'],'nmpc_shadow')
        self.assertEqual(trial['computation_algorithm'],'flat_nmpc_slqr')
        self.assertFalse(trial['nmpc_vehicle_authority'])
        controller=yaml.safe_load((folder/'controller.yaml').read_text())
        self.assertEqual(controller['p6_algorithm_mode'],'nmpc_shadow')
        self.assertEqual(controller['p6_shadow_algorithm'],'flat_nmpc_slqr')
        self.assertEqual(controller['p5_mode'],'legacy_slqr')
        self.assertFalse(controller['p5_allow_test_nmpc'])

    def test_shadow_only_rejects_unspecified_computation_mode(self):
        with self.assertRaisesRegex(ValueError,'explicit computation algorithm'):
            tool.make_trial(self.args(shadow_only=True,algorithm='nmpc_shadow'))
        self.assertFalse(self.folder.exists())

    def test_causal_flat_strategy_frozen_into_shadow_worker(self):
        import yaml
        config=self.root/'causal_candidate';config.mkdir()
        strategy=dict(reference_revalidation=True,flat_feasibility_recovery=False,
                      flat_causal_reference_seed=True,flat_response_knot_monitor=True)
        (self.research/'devel/lib/move_car/control_pub').write_text(
            'SYNTHETIC revalidation_original_context_mismatch')
        (self.research/'devel/lib/move_car/p5_nmpc_worker_node').write_text(
            'SYNTHETIC p7_flat_causal_reference_seed p7_flat_response_knot_monitor')
        with mock.patch.object(tool,'candidate_strategy',return_value=strategy):
            folder=tool.make_trial(self.args(shadow_only=True,algorithm='flat_nmpc_slqr',config_root=config))
        _,trial=tool.read_trial(folder)
        self.assertFalse(trial['nmpc_vehicle_authority'])
        self.assertTrue(trial['flat_causal_reference_seed'])
        self.assertTrue(trial['flat_response_knot_monitor'])
        worker=yaml.safe_load((folder/'worker.yaml').read_text())
        self.assertTrue(worker['p7_flat_causal_reference_seed'])
        self.assertTrue(worker['p7_flat_response_knot_monitor'])

    def test_causal_flat_strategy_cannot_prepare_vehicle_authority(self):
        strategy=dict(reference_revalidation=True,flat_feasibility_recovery=False,
                      flat_causal_reference_seed=True,flat_response_knot_monitor=True)
        with mock.patch.object(tool,'candidate_strategy',return_value=strategy):
            with self.assertRaisesRegex(ValueError,'shadow-only collection'):
                tool.make_trial(self.args(algorithm='flat_nmpc_slqr'))
        self.assertFalse(self.folder.exists())

    def test_input_sqp_strategy_frozen_into_shadow_worker(self):
        import yaml
        config=self.root/'input_sqp_candidate';config.mkdir()
        strategy=dict(reference_revalidation=True,flat_feasibility_recovery=False,
                      flat_input_feasibility_sqp=True)
        (self.research/'devel/lib/move_car/control_pub').write_text(
            'SYNTHETIC revalidation_original_context_mismatch')
        worker=self.research/'devel/lib/move_car/p5_nmpc_worker_node'
        with mock.patch.object(tool,'candidate_strategy',return_value=strategy):
            with self.assertRaisesRegex(ValueError,'lacks p7_flat_input_feasibility_sqp'):
                tool.make_trial(self.args(shadow_only=True,algorithm='flat_nmpc_slqr',config_root=config))
            self.assertFalse(self.folder.exists())
            worker.write_text('SYNTHETIC p7_flat_input_feasibility_sqp')
            folder=tool.make_trial(self.args(shadow_only=True,algorithm='flat_nmpc_slqr',config_root=config))
        _,trial=tool.read_trial(folder)
        self.assertFalse(trial['nmpc_vehicle_authority'])
        self.assertTrue(trial['flat_input_feasibility_sqp'])
        for name in ('controller.yaml','worker.yaml'):
            params=yaml.safe_load((folder/name).read_text())
            self.assertTrue(params['p7_flat_input_feasibility_sqp'])
        self.assertEqual(yaml.safe_load((folder/'controller.yaml').read_text())['p6_algorithm_mode'],'nmpc_shadow')

    def test_input_sqp_strategy_cannot_prepare_vehicle_authority_or_skip_revalidation(self):
        for revalidate in (False,True):
            strategy=dict(reference_revalidation=revalidate,flat_feasibility_recovery=False,
                          flat_input_feasibility_sqp=True)
            with self.subTest(revalidate=revalidate),mock.patch.object(tool,'candidate_strategy',return_value=strategy):
                with self.assertRaisesRegex(ValueError,'reference revalidation and shadow-only collection'):
                    tool.make_trial(self.args(algorithm='flat_nmpc_slqr'))
            self.assertFalse(self.folder.exists())

    def test_input_sqp_optional_schema_requires_boolean(self):
        import yaml
        config=self.root/'schema_sqp_candidate';config.mkdir()
        strategy=dict(schema_version=1,kind='p7_algorithm_strategy',status='research_candidate',
            terrain='smooth',algorithm='flat_nmpc_slqr',source_run_id='p7real_synthetic_fixture',
            source_events_sha256='a'*64,reference_revalidation=True,flat_feasibility_recovery=False,
            flat_input_feasibility_sqp=True,safety_limits_unchanged=True,automatic_promotion=False,
            evidence=['SYNTHETIC TOOL FIXTURE'])
        path=config/'p7_strategy.yaml';path.write_text(yaml.safe_dump(strategy))
        self.assertTrue(tool.candidate_strategy(config,'flat_nmpc_slqr')['flat_input_feasibility_sqp'])
        strategy['flat_input_feasibility_sqp']='true';path.write_text(yaml.safe_dump(strategy))
        with self.assertRaisesRegex(ValueError,'boolean'):
            tool.candidate_strategy(config,'flat_nmpc_slqr')

    def test_response_base_frozen_into_shadow_worker_and_requires_binary(self):
        import yaml
        config=self.root/'response_base_candidate';config.mkdir()
        strategy=dict(reference_revalidation=True,flat_feasibility_recovery=False,
                      flat_response_base=True)
        (self.research/'devel/lib/move_car/control_pub').write_text(
            'SYNTHETIC revalidation_original_context_mismatch')
        worker=self.research/'devel/lib/move_car/p5_nmpc_worker_node'
        with mock.patch.object(tool,'candidate_strategy',return_value=strategy):
            with self.assertRaisesRegex(ValueError,'lacks p7_flat_response_base'):
                tool.make_trial(self.args(shadow_only=True,algorithm='flat_nmpc_slqr',config_root=config))
            self.assertFalse(self.folder.exists())
            worker.write_text('SYNTHETIC p7_flat_response_base')
            folder=tool.make_trial(self.args(shadow_only=True,algorithm='flat_nmpc_slqr',config_root=config))
        _,trial=tool.read_trial(folder)
        self.assertFalse(trial['nmpc_vehicle_authority'])
        self.assertTrue(trial['flat_response_base'])
        for name in ('controller.yaml','worker.yaml'):
            self.assertTrue(yaml.safe_load((folder/name).read_text())['p7_flat_response_base'])

    def test_budgeted_poll_requires_base_matching_binary_and_freezes_flag(self):
        import yaml
        config=self.root/'budgeted_poll_candidate';config.mkdir()
        strategy=dict(reference_revalidation=True,flat_feasibility_recovery=False,
                      flat_response_base=True,flat_budgeted_poll=True)
        (self.research/'devel/lib/move_car/control_pub').write_text('SYNTHETIC revalidation_original_context_mismatch')
        worker=self.research/'devel/lib/move_car/p5_nmpc_worker_node'
        with mock.patch.object(tool,'candidate_strategy',return_value=strategy):
            worker.write_text('SYNTHETIC p7_flat_response_base')
            with self.assertRaisesRegex(ValueError,'lacks p7_flat_budgeted_poll'):
                tool.make_trial(self.args(shadow_only=True,algorithm='flat_nmpc_slqr',config_root=config))
            self.assertFalse(self.folder.exists())
            worker.write_text('SYNTHETIC p7_flat_response_base p7_flat_budgeted_poll')
            folder=tool.make_trial(self.args(shadow_only=True,algorithm='flat_nmpc_slqr',config_root=config))
        _,trial=tool.read_trial(folder)
        self.assertTrue(trial['flat_budgeted_poll'])
        for name in ('controller.yaml','worker.yaml'):
            self.assertTrue(yaml.safe_load((folder/name).read_text())['p7_flat_budgeted_poll'])
        self.assertFalse(trial['nmpc_vehicle_authority'])

    def test_refinement_options_require_matching_worker_and_are_frozen(self):
        import yaml
        config=self.root/'refinement_candidate';config.mkdir()
        strategy=dict(reference_revalidation=True,flat_feasibility_recovery=False,flat_response_base=True,
                      flat_response_near_support=True,nominal_observation_hold=True)
        (self.research/'devel/lib/move_car/control_pub').write_text('SYNTHETIC revalidation_original_context_mismatch')
        worker=self.research/'devel/lib/move_car/p5_nmpc_worker_node'
        with mock.patch.object(tool,'candidate_strategy',return_value=strategy):
            worker.write_text('SYNTHETIC p7_flat_response_base')
            with self.assertRaisesRegex(ValueError,'lacks p7_flat_response_near_support'):
                tool.make_trial(self.args(shadow_only=True,algorithm='flat_nmpc_slqr',config_root=config))
            worker.write_text('SYNTHETIC p7_flat_response_base p7_flat_response_near_support p7_nominal_observation_hold')
            folder=tool.make_trial(self.args(shadow_only=True,algorithm='flat_nmpc_slqr',config_root=config))
        _,trial=tool.read_trial(folder)
        self.assertTrue(trial['flat_response_near_support'])
        self.assertTrue(trial['nominal_observation_hold'])
        self.assertFalse(trial['nmpc_vehicle_authority'])
        for name in ('controller.yaml','worker.yaml'):
            params=yaml.safe_load((folder/name).read_text())
            self.assertTrue(params['p7_flat_response_near_support'])
            self.assertTrue(params['p7_nominal_observation_hold'])

    def candidate_trial_fixture(self):
        """Synthetic startup contract only, never vehicle approval/evidence."""
        config=self.root/'explicit_candidate_trial';config.mkdir()
        strategy=dict(reference_revalidation=True,flat_feasibility_recovery=False,
            flat_near_knot_restoration=False,flat_response_base=True,
            flat_response_near_support=True,nominal_observation_hold=True)
        manifest=dict(schema_version=1,kind='operator_supervised_unvalidated_candidate',
            terrain='smooth',algorithm='flat_nmpc_slqr',automatic_promotion=False,
            safety_limits_unchanged=True,validated_deployment=False,
            evidence=['SYNTHETIC STARTUP FIXTURE; NOT PHYSICAL VALIDATION'])
        (config/'CANDIDATE_TRIAL.json').write_text(json.dumps(manifest))
        (self.research/'devel/lib/move_car/control_pub').write_text('SYNTHETIC revalidation_original_context_mismatch')
        self.worker_trial_capabilities=('p7_candidate_control_trial p7_flat_response_base '
            'p7_flat_response_near_support p7_nominal_observation_hold p7_flat_near_knot_restoration')
        (self.research/'devel/lib/move_car/p5_nmpc_worker_node').write_text(self.worker_trial_capabilities)
        return config,strategy

    def test_explicit_candidate_trial_freezes_actual_mode_manifest_and_no_automatic_start(self):
        import yaml
        config,strategy=self.candidate_trial_fixture()
        with mock.patch.object(tool,'candidate_strategy',return_value=strategy),mock.patch.object(tool.subprocess,'Popen') as launch:
            folder=tool.make_trial(self.args(config_root=config,algorithm='flat_nmpc_slqr',candidate_control_trial=True))
        launch.assert_not_called()
        _,trial=tool.read_trial(folder)
        self.assertTrue(trial['candidate_control_trial'])
        self.assertFalse(trial['shadow_only'])
        self.assertEqual(trial['algorithm'],'flat_nmpc_slqr')
        self.assertEqual(trial['computation_algorithm'],'flat_nmpc_slqr')
        self.assertEqual(trial['execution_context'],'operator_supervised_unvalidated_candidate')
        self.assertEqual(trial['nmpc_vehicle_authority'],'requires_live_operator_lease_and_all_candidate_checks')
        self.assertFalse(trial['automatic_arming']);self.assertFalse(trial['validated_deployment'])
        for name in ('controller.yaml','worker.yaml'):
            params=yaml.safe_load((folder/name).read_text())
            self.assertTrue(params['p7_candidate_control_trial'])
            self.assertTrue(params['p7_reference_revalidation'])
            self.assertFalse(params['p7_flat_budgeted_poll'])
        self.assertEqual(yaml.safe_load((folder/'controller.yaml').read_text())['p6_algorithm_mode'],'flat_nmpc_slqr')
        freeze=json.loads((folder/'software_freeze.json').read_text())
        frozen=trial['candidate_trial_manifest']
        self.assertEqual(frozen['sha256'],tool.sha(config/'CANDIDATE_TRIAL.json'))
        self.assertEqual(freeze['candidate_trial_manifest'],frozen)
        self.assertEqual(freeze['files'][frozen['path']],frozen['sha256'])
        # Merely editing metadata must not hide an enabled frozen worker flag.
        trial['candidate_control_trial']=False;(folder/'trial.json').write_text(json.dumps(trial))
        with self.assertRaisesRegex(ValueError,'frozen flag mismatch'):tool.read_trial(folder)

    def test_candidate_trial_is_explicit_mutually_exclusive_and_requires_exact_mode(self):
        for changes,reason in ((dict(shadow_only=True),'mutually exclusive'),
            (dict(algorithm='nmpc_12_fixed'),'requires flat_nmpc_slqr')):
            args=dict(candidate_control_trial=True,algorithm='flat_nmpc_slqr');args.update(changes)
            with self.subTest(changes=changes),self.assertRaisesRegex(ValueError,reason):
                tool.make_trial(self.args(**args))
        self.assertFalse(self.folder.exists())

    def test_candidate_trial_requires_manifest_and_new_worker_capability(self):
        config,strategy=self.candidate_trial_fixture();manifest=config/'CANDIDATE_TRIAL.json'
        text=manifest.read_text();manifest.unlink()
        args=self.args(config_root=config,algorithm='flat_nmpc_slqr',candidate_control_trial=True)
        with mock.patch.object(tool,'candidate_strategy',return_value=strategy):
            with self.assertRaisesRegex(ValueError,'requires CANDIDATE_TRIAL'):tool.make_trial(args)
            manifest.write_text(text)
            worker=self.research/'devel/lib/move_car/p5_nmpc_worker_node'
            worker.write_text(self.worker_trial_capabilities.replace('p7_candidate_control_trial',''))
            with self.assertRaisesRegex(ValueError,'lacks p7_candidate_control_trial'):tool.make_trial(args)
        self.assertFalse(self.folder.exists())

    def test_response_warm_trial_requires_matching_worker_and_freezes_identity(self):
        import yaml
        config,strategy=self.candidate_trial_fixture()
        strategy['flat_response_warm_start']=True
        args=self.args(config_root=config,algorithm='flat_nmpc_slqr',candidate_control_trial=True)
        worker=self.research/'devel/lib/move_car/p5_nmpc_worker_node'
        with mock.patch.object(tool,'candidate_strategy',return_value=strategy):
            with self.assertRaisesRegex(ValueError,'lacks p7_flat_response_warm_start'):
                tool.make_trial(args)
            worker.write_text(self.worker_trial_capabilities+' p7_flat_response_warm_start')
            folder=tool.make_trial(args)
        _,trial=tool.read_trial(folder)
        self.assertTrue(trial['flat_response_warm_start'])
        for name in ('controller.yaml','worker.yaml'):
            self.assertTrue(yaml.safe_load((folder/name).read_text())['p7_flat_response_warm_start'])
        trial['flat_response_warm_start']=False
        (folder/'trial.json').write_text(json.dumps(trial))
        with self.assertRaisesRegex(ValueError,'warm-start frozen flag mismatch'):
            tool.read_trial(folder)

    def test_candidate_trial_does_not_relax_shadow_only_strategies_or_missing_base(self):
        config,strategy=self.candidate_trial_fixture()
        args=self.args(config_root=config,algorithm='flat_nmpc_slqr',candidate_control_trial=True)
        cases=[('flat_budgeted_poll',True,'shadow-only collection'),
            ('flat_causal_reference_seed',True,'shadow-only collection'),
            ('flat_response_knot_monitor',True,'shadow-only collection'),
            ('flat_input_feasibility_sqp',True,'shadow-only collection'),
            ('flat_response_base',False,'requires response base'),
            ('nominal_observation_hold',False,'requires response base'),
            ('flat_response_near_support',False,'requires response base'),
            ('reference_revalidation',False,'requires response base')]
        for key,value,reason in cases:
            with self.subTest(key=key),mock.patch.object(tool,'candidate_strategy',return_value=dict(strategy,**{key:value})):
                with self.assertRaisesRegex(ValueError,reason):tool.make_trial(args)
        self.assertFalse(self.folder.exists())

    def test_candidate_trial_cannot_reclassify_isolated_config_or_modify_frozen_manifest(self):
        config,strategy=self.candidate_trial_fixture();isolated=config/'ISOLATED_ONLY.json'
        args=self.args(config_root=config,algorithm='flat_nmpc_slqr',candidate_control_trial=True)
        with mock.patch.object(tool,'candidate_strategy',return_value=strategy):
            isolated.write_text('{}')
            with self.assertRaisesRegex(ValueError,'isolated-research-only'):tool.make_trial(args)
            isolated.unlink();folder=tool.make_trial(args)
        path=config/'CANDIDATE_TRIAL.json';manifest=json.loads(path.read_text())
        manifest['evidence'].append('changed after prepare');path.write_text(json.dumps(manifest))
        with self.assertRaisesRegex(ValueError,'manifest changed'):tool.read_trial(folder)

    def test_response_base_cannot_prepare_authority_or_skip_revalidation(self):
        for revalidate,shadow in ((False,False),(True,False),(False,True)):
            strategy=dict(reference_revalidation=revalidate,flat_feasibility_recovery=False,
                          flat_response_base=True)
            with self.subTest(revalidate=revalidate,shadow=shadow),mock.patch.object(tool,'candidate_strategy',return_value=strategy):
                with self.assertRaisesRegex(ValueError,'reference revalidation and shadow-only collection'):
                    tool.make_trial(self.args(algorithm='flat_nmpc_slqr',shadow_only=shadow))
            self.assertFalse(self.folder.exists())

    def test_response_base_optional_schema_requires_boolean(self):
        import yaml
        config=self.root/'schema_response_base_candidate';config.mkdir()
        strategy=dict(schema_version=1,kind='p7_algorithm_strategy',status='research_candidate',
            terrain='smooth',algorithm='flat_nmpc_slqr',source_run_id='p7real_synthetic_fixture',
            source_events_sha256='a'*64,reference_revalidation=True,flat_feasibility_recovery=False,
            flat_response_base=True,safety_limits_unchanged=True,automatic_promotion=False,
            evidence=['SYNTHETIC TOOL FIXTURE'])
        path=config/'p7_strategy.yaml';path.write_text(yaml.safe_dump(strategy))
        self.assertTrue(tool.candidate_strategy(config,'flat_nmpc_slqr')['flat_response_base'])
        strategy['flat_response_base']='true';path.write_text(yaml.safe_dump(strategy))
        with self.assertRaisesRegex(ValueError,'boolean'):
            tool.candidate_strategy(config,'flat_nmpc_slqr')

    def test_requested_speed_domain_and_evidence(self):
        for speed in (0.,-.1,.4001,float('nan'),float('inf')):
            with self.subTest(speed=speed),self.assertRaises(ValueError):
                tool.make_trial(self.args(speed_cap=speed))
        self.assertFalse(self.folder.exists())

    def profile(self):
        evidence=self.root/'evidence.txt';evidence.write_text('SYNTHETIC TOOL FIXTURE NOT VEHICLE EVIDENCE')
        p=dict(schema_version=1,kind='historical_engineering_mapping',imu_frame='gyro_link',
            wheel_speed_scale=.001,feedback_steer_sign=1.,imu_yaw_sign=1.,cmd_steer_scale=1.,cmd_steer_sign=1.,
            engineering_mapping_supported=True,physical_calibration_certified=False,
            declared_ground_speed_independent=False,maximum_observed_command_speed_mps=.32,
            source_files={str(evidence):tool.sha(evidence)})
        path=self.root/'profile.json';path.write_text(json.dumps(p));return path

    def test_history_profile_fills_engineering_mapping_at_point_four(self):
        a=self.args(mapping_profile=self.profile(),imu_frame=None,mapping_evidence=None,
            confirm_forward_wheel_positive=False,confirm_left_steering_positive=False,
            confirm_left_yaw_positive=False,speed_cap=.4)
        folder=tool.make_trial(a);_,trial=tool.read_trial(folder)
        self.assertEqual(a.imu_frame,'gyro_link');self.assertEqual(trial['speed_cap'],.4)
        self.assertFalse(trial['physical_calibration_certified'])
        import yaml
        parameters=yaml.safe_load((folder/'controller.yaml').read_text())
        for k in ('max_speed','cruise_speed','continuous_cruise_speed','continuous_catchup_max_speed','continuous_curve_max_speed'):
            self.assertEqual(parameters[k],.4)
        self.assertEqual(parameters['degraded_speed_limit'],.10)

    def test_history_source_modification_rejected(self):
        path=self.profile();(self.root/'evidence.txt').write_text('changed')
        with self.assertRaises(ValueError):tool.make_trial(self.args(mapping_profile=path,imu_frame=None,speed_cap=.4))

    def test_high_speed_without_history_profile_rejected(self):
        with self.assertRaises(ValueError):tool.make_trial(self.args(speed_cap=.4))

    def test_changed_config_or_binary_rejected(self):
        folder=tool.make_trial(self.args())
        config=folder/'controller.yaml'
        config.write_text(config.read_text()+'# changed\n')
        with self.assertRaises(ValueError):tool.read_trial(folder)

    def test_reference_revalidation_requires_matching_candidate_binary(self):
        a=self.args(revalidate_reference=True)
        with self.assertRaisesRegex(ValueError,'lacks reference revalidation'):tool.make_trial(a)
        self.assertFalse(self.folder.exists())
        binary=self.research/'devel/lib/move_car/control_pub'
        binary.write_text('SYNTHETIC TEST FIXTURE revalidation_original_context_mismatch')
        folder=tool.make_trial(a);_,trial=tool.read_trial(folder)
        self.assertTrue(trial['reference_revalidation'])
        import yaml
        self.assertTrue(yaml.safe_load((folder/'controller.yaml').read_text())['p7_reference_revalidation'])
        self.assertTrue(yaml.safe_load((folder/'worker.yaml').read_text())['p7_reference_revalidation'])

    def test_flat_recovery_cannot_silently_use_old_worker_or_skip_revalidation(self):
        a=self.args(flat_feasibility_recovery=True)
        with self.assertRaisesRegex(ValueError,'requires reference revalidation'):tool.make_trial(a)
        a.revalidate_reference=True
        (self.research/'devel/lib/move_car/control_pub').write_text('revalidation_original_context_mismatch')
        with self.assertRaisesRegex(ValueError,'worker binary lacks'):tool.make_trial(a)
        self.assertFalse(self.folder.exists())
        (self.research/'devel/lib/move_car/p5_nmpc_worker_node').write_text('flat8_feasible_bounded_restoration')
        folder=tool.make_trial(a);_,trial=tool.read_trial(folder)
        self.assertTrue(trial['flat_feasibility_recovery'])
        import yaml
        self.assertTrue(yaml.safe_load((folder/'worker.yaml').read_text())['p7_flat_feasibility_recovery'])

    def test_candidate_strategy_is_strict_and_bound_to_selected_algorithm(self):
        import yaml
        config=self.root/'candidate';config.mkdir()
        strategy=dict(schema_version=1,kind='p7_algorithm_strategy',status='research_candidate',
            terrain='smooth',algorithm='flat_nmpc_slqr',source_run_id='p7real_synthetic_fixture',
            source_events_sha256='a'*64,reference_revalidation=True,flat_feasibility_recovery=True,
            safety_limits_unchanged=True,automatic_promotion=False,evidence=['SYNTHETIC TEST FIXTURE'])
        p=config/'p7_strategy.yaml';p.write_text(yaml.safe_dump(strategy))
        self.assertEqual(tool.candidate_strategy(config,'flat_nmpc_slqr'),strategy)
        with self.assertRaisesRegex(ValueError,'mismatch'):tool.candidate_strategy(config,'nmpc_12_fixed')
        for key,value in [('automatic_promotion',True),('reference_revalidation',False),('source_events_sha256','g'*64)]:
            altered=dict(strategy,**{key:value});p.write_text(yaml.safe_dump(altered))
            with self.subTest(key=key),self.assertRaises(ValueError):tool.candidate_strategy(config,'flat_nmpc_slqr')
        p.write_text(yaml.safe_dump(dict(strategy,unknown_option=True)))
        with self.assertRaisesRegex(ValueError,'schema'):tool.candidate_strategy(config,'flat_nmpc_slqr')

    def test_straight_yaw_weight_requires_straight_gate_and_valid_weight(self):
        import yaml
        config,strategy=self.candidate_trial_fixture()
        strategy.update(schema_version=1,kind='p7_algorithm_strategy',status='research_candidate',
            terrain='smooth',algorithm='flat_nmpc_slqr',source_run_id='p7real_synthetic_fixture',
            source_events_sha256='a'*64,safety_limits_unchanged=True,automatic_promotion=False,
            evidence=['SYNTHETIC TEST FIXTURE'])
        path=config/'p7_strategy.yaml'
        for enabled,weight in ((False,2.),(True,-1.),(True,4.1),(True,float('nan'))):
            with self.subTest(enabled=enabled,weight=weight):
                path.write_text(yaml.safe_dump(dict(strategy,straight_tracking_reference=enabled,
                                                     straight_yaw_damping_weight=weight)))
                with self.assertRaisesRegex(ValueError,'straight yaw damping'):
                    tool.candidate_strategy(config,'flat_nmpc_slqr')
        path.write_text(yaml.safe_dump(dict(strategy,straight_tracking_reference=True,
                                             straight_yaw_damping_weight=2.)))
        self.assertEqual(tool.candidate_strategy(config,'flat_nmpc_slqr')['straight_yaw_damping_weight'],2.)

    def test_prepare_prints_explicit_shell_assignment_without_updating_parent(self):
        output=io.StringIO()
        with mock.patch.dict(os.environ,{'TRIAL':'previous_trial'}), \
                contextlib.redirect_stdout(output), \
                mock.patch.object(tool.subprocess,'Popen') as launch:
            folder=tool.make_trial(self.args())
            self.assertEqual(os.environ['TRIAL'],'previous_trial')
        launch.assert_not_called()
        lines=output.getvalue().splitlines()
        self.assertEqual(lines[0],'TRIAL_DIRECTORY='+str(folder))
        self.assertIn('TRIAL='+shlex.quote(str(folder)),lines)
        self.assertIn('cannot change TRIAL',output.getvalue())

    def test_accuracy_strategy_requires_compatible_worker_and_freezes_real_flags(self):
        import yaml
        config=self.root/'accuracy_candidate';config.mkdir()
        strategy=dict(schema_version=1,kind='p7_algorithm_strategy',status='research_candidate',
            terrain='smooth',algorithm='flat_nmpc_slqr',source_run_id='p7real_synthetic_fixture',
            source_events_sha256='a'*64,reference_revalidation=True,flat_feasibility_recovery=True,
            flat_accuracy_refinement=True,bounded_slqr_feedback=True,
            safety_limits_unchanged=True,automatic_promotion=False,evidence=['SYNTHETIC TOOL FIXTURE'])
        (config/'p7_strategy.yaml').write_text(yaml.safe_dump(strategy))
        (self.research/'devel/lib/move_car/control_pub').write_text('revalidation_original_context_mismatch')
        worker=self.research/'devel/lib/move_car/p5_nmpc_worker_node'
        worker.write_text('flat8_feasible_bounded_restoration')
        a=self.args(config_root=config,algorithm='flat_nmpc_slqr')
        with self.assertRaisesRegex(ValueError,'lacks flat accuracy'):tool.make_trial(a)
        worker.write_text('flat8_feasible_bounded_restoration p7_flat_accuracy_refinement feedback_bounded_rollout_feasible')
        folder=tool.make_trial(a);_,trial=tool.read_trial(folder)
        self.assertTrue(trial['flat_accuracy_refinement'])
        self.assertTrue(trial['bounded_slqr_feedback'])
        for filename in ['controller.yaml','worker.yaml']:
            params=yaml.safe_load((folder/filename).read_text())
            self.assertTrue(params['p7_flat_accuracy_refinement'])
            self.assertTrue(params['p7_bounded_slqr_feedback'])
        strategy['bounded_slqr_feedback']='true'
        (config/'p7_strategy.yaml').write_text(yaml.safe_dump(strategy))
        with self.assertRaisesRegex(ValueError,'boolean'):tool.candidate_strategy(config,'flat_nmpc_slqr')

    def test_duplicate_start_preserves_record_and_never_reads_ros_or_launches(self):
        folder=tool.make_trial(self.args())
        marker=folder/'started.json';marker.write_text('{"previous_start":true}\n')
        original=marker.read_bytes()
        with mock.patch.object(tool,'read_master') as graph, \
                mock.patch.object(tool.subprocess,'Popen') as launch, \
                mock.patch.object(tool.subprocess,'check_output') as lookup:
            with self.assertRaises(ValueError) as error:
                tool.start(argparse.Namespace(trial=folder,master_uri='http://unused:11311'))
        self.assertIn(str(folder),str(error.exception))
        self.assertIn('does not update your shell',str(error.exception))
        graph.assert_not_called();launch.assert_not_called();lookup.assert_not_called()
        self.assertEqual(marker.read_bytes(),original)

    def test_isolated_candidate_cannot_prepare_real_trial(self):
        config=self.root/'isolated_only';config.mkdir()
        (config/'ISOLATED_ONLY.json').write_text('{"vehicle_authority":false}')
        with self.assertRaisesRegex(ValueError,'isolated-research-only'):
            tool.make_trial(self.args(config_root=config))
        self.assertFalse(self.folder.exists())

    def test_flat_seed_strategy_checks_capability_and_freezes_flags(self):
        import yaml
        config=self.root/'conditioned_candidate';config.mkdir()
        strategy=dict(schema_version=1,kind='p7_algorithm_strategy',status='research_candidate',
            terrain='smooth',algorithm='flat_nmpc_slqr',source_run_id='p7real_synthetic_fixture',
            source_events_sha256='a'*64,reference_revalidation=True,flat_feasibility_recovery=False,
            flat_near_knot_restoration=False,flat_conditioned_geometry=True,flat_baseline_seed=True,flat_delay_aligned_reconstruction=True,
            safety_limits_unchanged=True,automatic_promotion=False,evidence=['SYNTHETIC TOOL FIXTURE'])
        path=config/'p7_strategy.yaml';path.write_text(yaml.safe_dump(strategy))
        (self.research/'devel/lib/move_car/control_pub').write_text('revalidation_original_context_mismatch')
        worker=self.research/'devel/lib/move_car/p5_nmpc_worker_node'
        a=self.args(config_root=config,algorithm='flat_nmpc_slqr')
        with self.assertRaisesRegex(ValueError,'lacks p7_flat_conditioned_geometry'):tool.make_trial(a)
        worker.write_text('p7_flat_conditioned_geometry')
        with self.assertRaisesRegex(ValueError,'lacks p7_flat_baseline_seed'):tool.make_trial(a)
        self.assertFalse(self.folder.exists())
        worker.write_text('p7_flat_conditioned_geometry p7_flat_baseline_seed')
        with self.assertRaisesRegex(ValueError,'lacks p7_flat_delay_aligned_reconstruction'):tool.make_trial(a)
        worker.write_text('p7_flat_conditioned_geometry p7_flat_baseline_seed p7_flat_delay_aligned_reconstruction p7_flat_near_knot_restoration')
        folder=tool.make_trial(a);_,trial=tool.read_trial(folder)
        self.assertFalse(trial['flat_near_knot_restoration'])
        self.assertFalse(yaml.safe_load((folder/'worker.yaml').read_text())['p7_flat_near_knot_restoration'])
        for key in ('flat_conditioned_geometry','flat_baseline_seed','flat_delay_aligned_reconstruction'):
            self.assertTrue(trial[key])
            self.assertTrue(trial['algorithm_strategy'][key])
            for filename in ('controller.yaml','worker.yaml'):
                self.assertTrue(yaml.safe_load((folder/filename).read_text())['p7_'+key])
            strategy[key]='true';path.write_text(yaml.safe_dump(strategy))
            with self.assertRaisesRegex(ValueError,'boolean'):tool.candidate_strategy(config,'flat_nmpc_slqr')
            strategy[key]=True

    def test_old_trial_without_new_strategy_fields_remains_readable(self):
        folder=tool.make_trial(self.args())
        path=folder/'trial.json';trial=json.loads(path.read_text())
        del trial['flat_conditioned_geometry'];del trial['flat_baseline_seed'];del trial['flat_delay_aligned_reconstruction']
        path.write_text(json.dumps(trial))
        _,read=tool.read_trial(folder)
        self.assertNotIn('flat_conditioned_geometry',read)
        self.assertNotIn('flat_baseline_seed',read)

    def test_cli_value_error_is_clear_without_traceback(self):
        message='trial already started: /old/trial; use the newly prepared TRIAL'
        errors=io.StringIO()
        with mock.patch.object(tool,'start',side_effect=ValueError(message)), \
                contextlib.redirect_stderr(errors),self.assertRaises(SystemExit) as result:
            tool.main(['start','--trial','/old/trial'])
        self.assertEqual(result.exception.code,2)
        self.assertIn(message,errors.getvalue())
        self.assertNotIn('Traceback',errors.getvalue())


if __name__=='__main__':unittest.main()
