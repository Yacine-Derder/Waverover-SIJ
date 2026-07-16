import fcntl
import os
from pathlib import Path
import subprocess

import pytest


SCRIPT = Path(__file__).parents[1] / 'update_and_build.sh'


def run(command, cwd=None, check=True, env=None):
    return subprocess.run(
        command,
        cwd=cwd,
        check=check,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )


class MockDeployment:
    def __init__(self, tmp_path):
        self.root = tmp_path
        self.remote = tmp_path / 'remote.git'
        self.author = tmp_path / 'author'
        self.workspace = tmp_path / 'workspace'
        self.repo = self.workspace / 'src'
        run(['git', 'init', '--bare', '--initial-branch=main', self.remote])
        run(['git', 'clone', self.remote, self.author])
        run(['git', 'config', 'user.name', 'Updater Test'], self.author)
        run(['git', 'config', 'user.email', 'test@example.invalid'], self.author)
        (self.author / '.gitignore').write_text(
            '/waverover/config/robot_identity.yaml\n', encoding='utf-8'
        )
        (self.author / 'version.txt').write_text('one\n', encoding='utf-8')
        run(['git', 'add', '.'], self.author)
        run(['git', 'commit', '-m', 'initial'], self.author)
        run(['git', 'push', 'origin', 'main'], self.author)
        self.workspace.mkdir()
        run(['git', 'clone', self.remote, self.repo])
        run(['git', 'config', 'user.name', 'Updater Test'], self.repo)
        run(['git', 'config', 'user.email', 'test@example.invalid'], self.repo)
        self.identity = self.repo / 'waverover/config/robot_identity.yaml'
        self.identity.parent.mkdir(parents=True)
        self.identity.write_text('robot_name: "test_131"\n', encoding='utf-8')
        self.install = self.workspace / 'install/setup.bash'
        self.install.parent.mkdir()
        self.install.write_text('# mock overlay\n', encoding='utf-8')
        self.ros_setup = tmp_path / 'ros_setup.bash'
        self.ros_setup.write_text(':\n', encoding='utf-8')
        self.build_log = tmp_path / 'build.log'
        self.build = tmp_path / 'build.sh'
        self.build.write_text(
            '#!/usr/bin/env bash\n'
            'echo build >> "$MOCK_BUILD_LOG"\n'
            'count=$(wc -l < "$MOCK_BUILD_LOG")\n'
            'if [[ ${MOCK_FAIL_MODE:-never} == always ]]; then exit 1; fi\n'
            'if [[ ${MOCK_FAIL_MODE:-never} == first && $count -eq 1 ]]; '
            'then exit 1; fi\n'
            'mkdir -p install; : > install/setup.bash\n',
            encoding='utf-8',
        )
        self.build.chmod(0o755)

    def env(self, **updates):
        value = os.environ.copy()
        value.update({
            'REPO_DIR': str(self.repo),
            'WORKSPACE_DIR': str(self.workspace),
            'IDENTITY_FILE': str(self.identity),
            'REMOTE': 'origin',
            'BRANCH': 'main',
            'EXPECTED_REMOTE_URL': str(self.remote),
            'NETWORK_TIMEOUT_SEC': '2',
            'LOCK_FILE': str(self.root / 'update.lock'),
            'ROS_SETUP_FILE': str(self.ros_setup),
            'BUILD_COMMAND': str(self.build),
            'MOCK_BUILD_LOG': str(self.build_log),
        })
        value.update(updates)
        return value

    def update_remote(self, text='two\n'):
        (self.author / 'version.txt').write_text(text, encoding='utf-8')
        run(['git', 'add', 'version.txt'], self.author)
        run(['git', 'commit', '-m', text.strip()], self.author)
        run(['git', 'push', 'origin', 'main'], self.author)

    def invoke(self, check=False, **environment):
        return run(
            [str(SCRIPT)],
            check=check,
            env=self.env(**environment),
        )


@pytest.fixture
def deployment(tmp_path):
    return MockDeployment(tmp_path)


def build_count(deployment):
    if not deployment.build_log.exists():
        return 0
    return len(deployment.build_log.read_text(encoding='utf-8').splitlines())


def test_no_update_skips_existing_install(deployment):
    result = deployment.invoke()
    assert result.returncode == 0
    assert 'build skipped' in result.stdout
    assert build_count(deployment) == 0


def test_missing_install_triggers_build(deployment):
    deployment.install.unlink()
    assert deployment.invoke().returncode == 0
    assert build_count(deployment) == 1


def test_ros_setup_allows_optional_unset_variable(deployment):
    setup_log = deployment.root / 'ros_setup.log'
    deployment.ros_setup.write_text(
        ': "$OPTIONAL_ROS_SETUP_VARIABLE"\n'
        'printf "sourced\\n" > "$MOCK_ROS_SETUP_LOG"\n'
        'export MOCK_ROS_SETUP_COMPLETE=1\n',
        encoding='utf-8',
    )
    deployment.install.unlink()

    result = deployment.invoke(
        MOCK_ROS_SETUP_LOG=str(setup_log),
        BUILD_COMMAND=(
            f'[[ $MOCK_ROS_SETUP_COMPLETE == 1 ]] && {deployment.build}'
        ),
    )

    assert result.returncode == 0
    assert setup_log.read_text(encoding='utf-8') == 'sourced\n'
    assert build_count(deployment) == 1


def test_failed_ros_setup_returns_clear_error(deployment):
    deployment.ros_setup.write_text('return 1\n', encoding='utf-8')
    deployment.install.unlink()

    result = deployment.invoke()

    assert result.returncode != 0
    assert f'failed to source ROS setup: {deployment.ros_setup}' in result.stdout
    assert build_count(deployment) == 0


def test_offline_fetch_is_non_blocking(deployment):
    run(['git', 'remote', 'set-url', 'origin', str(deployment.root / 'gone')],
        deployment.repo)
    result = deployment.invoke(
        EXPECTED_REMOTE_URL=str(deployment.root / 'gone')
    )
    assert result.returncode == 0
    assert 'fetch unavailable' in result.stdout


@pytest.mark.parametrize('untracked', [False, True])
def test_dirty_or_untracked_tree_is_refused(deployment, untracked):
    path = deployment.repo / ('untracked.txt' if untracked else 'version.txt')
    path.write_text('dirty\n', encoding='utf-8')
    result = deployment.invoke()
    assert result.returncode != 0
    assert 'working tree is dirty' in result.stdout


def test_missing_identity_is_refused(deployment):
    deployment.identity.unlink()
    assert deployment.invoke().returncode != 0


def test_tracked_identity_is_refused(deployment):
    run(['git', 'add', '-f', 'waverover/config/robot_identity.yaml'],
        deployment.repo)
    run(['git', 'commit', '-m', 'unsafe identity'], deployment.repo)
    assert 'tracked by Git' in deployment.invoke().stdout


def test_nonignored_identity_is_refused(deployment):
    run(['git', 'check-ignore', '-v', 'waverover/config/robot_identity.yaml'],
        deployment.repo)
    (deployment.repo / '.git/info/exclude').write_text(
        '', encoding='utf-8'
    )
    (deployment.repo / '.gitignore').write_text('', encoding='utf-8')
    run(['git', 'add', '.gitignore'], deployment.repo)
    run(['git', 'commit', '-m', 'remove ignore'], deployment.repo)
    assert 'not ignored' in deployment.invoke().stdout


def test_fast_forward_builds_and_deploys(deployment):
    deployment.update_remote()
    result = deployment.invoke()
    assert result.returncode == 0
    assert 'deployed commit' in result.stdout
    assert (deployment.repo / 'version.txt').read_text() == 'two\n'
    assert build_count(deployment) == 1


def test_diverged_branch_is_refused(deployment):
    (deployment.repo / 'local.txt').write_text('local\n', encoding='utf-8')
    run(['git', 'add', 'local.txt'], deployment.repo)
    run(['git', 'commit', '-m', 'local'], deployment.repo)
    deployment.update_remote()
    result = deployment.invoke()
    assert result.returncode != 0
    assert 'diverged' in result.stdout


def test_failed_new_build_rolls_back_and_rebuilds(deployment):
    old = run(['git', 'rev-parse', 'HEAD'], deployment.repo).stdout.strip()
    deployment.update_remote()
    result = deployment.invoke(MOCK_FAIL_MODE='first')
    assert result.returncode == 0
    assert 'rollback build succeeded' in result.stdout
    assert run(['git', 'rev-parse', 'HEAD'], deployment.repo).stdout.strip() == old
    assert build_count(deployment) == 2


def test_failed_new_and_rollback_build_returns_error(deployment):
    old = run(['git', 'rev-parse', 'HEAD'], deployment.repo).stdout.strip()
    deployment.update_remote()
    result = deployment.invoke(MOCK_FAIL_MODE='always')
    assert result.returncode != 0
    assert 'both failed' in result.stdout
    assert run(['git', 'rev-parse', 'HEAD'], deployment.repo).stdout.strip() == old


def test_lock_contention_is_refused(deployment):
    lock = open(deployment.root / 'update.lock', 'w', encoding='utf-8')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        result = deployment.invoke()
        assert result.returncode != 0
        assert 'holds the lock' in result.stdout
    finally:
        lock.close()
