"""Field launcher dry-run/argument boundaries; never uses sudo or robot SDK."""
import os
from pathlib import Path
import subprocess
import shutil
import tempfile
import unittest

SCRIPT=Path(__file__).resolve().parents[1]/'scripts/aubo_home_field.sh'


class FieldLauncherTests(unittest.TestCase):
    def run_script(self, *args):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            # Any accidental entry into the execution branch fails visibly.
            for name in ('sudo','tcpdump','python3'):
                path=root/name
                path.write_text('#!/bin/sh\nprintf "UNEXPECTED_EXTERNAL_CALL\\n" >&2\nexit 99\n')
                path.chmod(0o755)
            runtime=root/'must-not-exist'
            env=dict(os.environ,PATH=directory+':'+os.environ['PATH'],
                AUBO_RUNTIME_DIR=str(runtime),TELEOP_RUNTIME_ROOT=str(runtime))
            proc=subprocess.run(['/bin/bash',str(SCRIPT),*args],env=env,
                text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
            self.assertFalse(runtime.exists())
            self.assertNotIn('UNEXPECTED_EXTERNAL_CALL',proc.stdout)
            return proc

    def test_default_is_side_effect_free_plan(self):
        proc=self.run_script()
        self.assertEqual(proc.returncode,0)
        self.assertIn('Plan only',proc.stdout)

    def test_placement_is_explicit_and_can_be_previewed_without_loading_file(self):
        proc=self.run_script('--placement-reference','/not-created/basket.json')
        self.assertEqual(proc.returncode,0)
        self.assertIn('/not-created/basket.json',proc.stdout)
        self.assertEqual(self.run_script('--placement-reference').returncode,2)

    def test_invalid_duration_never_enters_execution(self):
        for value in ('0','29','1801','nope','-1','1.5','999999999999999999999999'):
            with self.subTest(value=value):
                self.assertEqual(self.run_script('--execute','--duration',value).returncode,2)

    def test_help_and_unknown_options_do_not_enter_execution(self):
        self.assertEqual(self.run_script('--help').returncode,0)
        self.assertEqual(self.run_script('--bogus').returncode,2)
        self.assertEqual(self.run_script('--duration').returncode,2)

    def test_valid_duration_can_be_previewed_without_execution(self):
        for value in ('30','120','300','301','1800','030'):
            self.assertEqual(self.run_script('--duration',value).returncode,0)

    def test_relay_port_is_validated_and_previewed(self):
        proc=self.run_script('--relay-port','8444')
        self.assertEqual(proc.returncode,0)
        self.assertIn('http://localhost:8444/',proc.stdout)
        for value in ('0','65536','nope'):
            with self.subTest(value=value):
                self.assertEqual(self.run_script('--relay-port',value).returncode,2)

    def test_missing_dependency_reports_its_name_before_sudo_or_sdk(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);bin_dir=root/'bin';bin_dir.mkdir()
            runtime=root/'runtime';runtime.mkdir()
            for name in ('dirname','realpath'):
                (bin_dir/name).symlink_to(shutil.which(name))
            # A deliberately closed PATH contains no grep (and no rg). These
            # commands may be found but must never be invoked by --check.
            for name in ('sudo','timeout','tcpdump','python3'):
                path=bin_dir/name
                path.write_text('#!/bin/sh\nprintf "UNEXPECTED_EXTERNAL_CALL\\n" >&2\nexit 99\n')
                path.chmod(0o755)
            env=dict(os.environ,PATH=str(bin_dir),AUBO_RUNTIME_DIR=str(runtime),
                TELEOP_RUNTIME_ROOT=str(runtime),AUBO_ALLOWED_ROOTS=directory)
            proc=subprocess.run(['/bin/bash',str(SCRIPT),'--check'],env=env,text=True,
                stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
            self.assertEqual(proc.returncode,2)
            self.assertIn('Missing required command: grep',proc.stdout)
            self.assertNotIn('UNEXPECTED_EXTERNAL_CALL',proc.stdout)
            self.assertEqual(list(runtime.iterdir()),[])

    def test_check_and_execute_are_mutually_exclusive(self):
        proc=self.run_script('--check','--execute')
        self.assertEqual(proc.returncode,2)
        self.assertIn('Choose either',proc.stdout)

    def test_quickack_requires_explicit_execute_and_can_be_previewed(self):
        proc=self.run_script('--quickack')
        self.assertEqual(proc.returncode,0)
        self.assertIn('QUICKACK experiment selected: 1',proc.stdout)
        self.assertIn('Plan only',proc.stdout)

    def test_unvalidated_quickack_cannot_start_or_pass_preflight(self):
        for mode in ('--check', '--execute'):
            with self.subTest(mode=mode):
                proc=self.run_script(mode,'--quickack')
                self.assertEqual(proc.returncode,2)
                self.assertIn('RTLD_DEEPBIND',proc.stdout)
                self.assertIn('matched zero receives',proc.stdout)

    def test_sdk_native_ack_mode_is_previewable_and_validated(self):
        proc=self.run_script('--ack-mode','quickack')
        self.assertEqual(proc.returncode,0)
        self.assertIn('TCP option experiment: quickack',proc.stdout)
        self.assertEqual(self.run_script('--ack-mode','bogus').returncode,2)


if __name__=='__main__':unittest.main(verbosity=2)
