#!/usr/bin/env bash
# Run interactively on the robot-connected ThinkPad, outside Docker.
# User authorization: install the Ubuntu Pro real-time kernel for FLASH/FR3.
set -euo pipefail

flash_rt_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
flash_rt_package=linux-realtime-hwe-24.04

if [[ "$(hostname)" != dase-hw101-ThinkPad-T16-Gen-4 ]]; then
    echo '请在连接 FR3 的 ThinkPad 宿主机运行此脚本。' >&2
    exit 1
fi
if (( EUID == 0 )); then
    echo '请用普通用户执行 bash scripts/setup_flash_fr3_realtime.sh，按提示使用 sudo。' >&2
    exit 1
fi

/usr/bin/python3 - <<'PY'
import pathlib
import platform
import shutil

release = dict(line.split('=', 1) for line in pathlib.Path('/etc/os-release').read_text().splitlines()
               if '=' in line)
if release.get('ID', '').strip('"') != 'ubuntu' or release.get('VERSION_ID', '').strip('"') != '24.04':
    raise SystemExit('此安装步骤只适用于 Ubuntu 24.04。')
if platform.machine() != 'x86_64':
    raise SystemExit('此安装步骤只适用于这台 x86_64 ThinkPad。')
if not pathlib.Path('/usr/bin/pro').exists():
    raise SystemExit('没有找到已检查过的 Ubuntu Pro 客户端。')
print('当前内核：', platform.release())
print('根分区剩余空间：%.2f GiB' % (shutil.disk_usage('/').free / 2**30))
PY

echo '请在本机输入 sudo 密码；账户登录按终端给出的浏览器链接完成。'
sudo -v

flash_rt_is_attached() {
    /usr/bin/python3 - <<'PY'
import json
import subprocess

result = subprocess.run(['/usr/bin/pro', 'status', '--format', 'json'],
                        capture_output=True, text=True, timeout=30)
try:
    attached = json.loads(result.stdout).get('attached') is True
except (ValueError, AttributeError):
    raise SystemExit('无法读取 Ubuntu Pro 绑定状态，请先运行 pro status 检查。')
raise SystemExit(0 if attached else 1)
PY
}

if ! flash_rt_is_attached; then
    # Avoid enabling unrelated services while attaching the account.
    sudo /usr/bin/pro attach --no-auto-enable
fi
if ! flash_rt_is_attached; then
    echo 'Ubuntu Pro 尚未绑定成功，未开始安装内核。' >&2
    exit 1
fi

echo '启用 Ubuntu Pro 实时内核软件源，准备安装 Ubuntu 24.04 HWE 实时内核。'
sudo /usr/bin/pro enable realtime-kernel --access-only --assume-yes
sudo apt-get update

if ! apt-cache show "$flash_rt_package" >/dev/null 2>&1; then
    echo '软件源中没有找到 linux-realtime-hwe-24.04，停止安装，请返回上述输出。' >&2
    exit 1
fi

echo '先检查软件包依赖；拒绝通过移除现有软件包来完成安装。'
sudo apt-get --simulate --no-remove install "$flash_rt_package"
sudo apt-get --yes --no-remove install "$flash_rt_package"

/usr/bin/python3 - "$flash_rt_workspace" <<'PY'
import datetime
import json
import pathlib
import platform
import sys
from zoneinfo import ZoneInfo

boot = pathlib.Path('/boot')
kernels = []
for config in sorted(boot.glob('config-*')):
    if 'CONFIG_PREEMPT_RT=y' not in config.read_text().splitlines():
        continue
    release = config.name.removeprefix('config-')
    if (boot / ('vmlinuz-' + release)).exists() and (boot / ('initrd.img-' + release)).exists():
        kernels.append(release)
if not kernels:
    raise SystemExit('软件包安装后未发现完整的 PREEMPT_RT 启动文件，请返回安装输出。')

now = datetime.datetime.now(ZoneInfo('Asia/Hong_Kong'))
output = pathlib.Path(sys.argv[1]) / 'artifacts/flash_fr3/runtime_checks'
output.mkdir(parents=True, exist_ok=True)
report = {
    'status': 'REALTIME_KERNEL_INSTALLED',
    'checked_at': now.isoformat(),
    'host': platform.node(),
    'running_kernel': platform.release(),
    'installed_realtime_kernels_with_boot_files': kernels,
    'package': 'linux-realtime-hwe-24.04',
    'robot_motion_commanded': False,
}
path = output / ('thinkpad_realtime_install_' + now.strftime('%Y%m%d_%H%M%S') + '.json')
path.write_text(json.dumps(report, indent=2) + '\n')
print('已安装的实时内核：', ', '.join(kernels))
print('当前运行内核：', platform.release())
print('安装记录：', path)
print('FLASH_REALTIME_INSTALL=COMPLETE_REBOOT_PENDING')
print('下一步需重启并在 GRUB 中选择实时内核；请先返回以上安装结果。')
PY
