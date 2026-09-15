"""Create a standalone ZIP with only the single-arm dependency set (no sim source)."""
import argparse
from pathlib import Path
import zipfile

PACKAGES = ('fr3_control_panel', 'fr3_real_bringup', 'fairino_hardware_v3_9_7',
            'fairino_msgs', 'ros2_hkv_gripper')


def package(root, output):
    files = []
    for name in PACKAGES:
        for path in sorted((root/'src'/name).rglob('*')):
            if (path.is_file() and not any(p in ('.git', '__pycache__', '.pytest_cache', '.vscode') for p in path.parts)
                    and path.suffix not in ('.pyc', '.bak', '.exe')):
                files.append((path, path.relative_to(root).as_posix()))
    for name in ('SINGLE_ARM.md', 'THIRD_PARTY.md', 'scripts/build_single_arm.sh',
                 'scripts/preview_single_arm.py'):
        files.append((root/name, name))
    files.append((root/'SINGLE_ARM.md', 'README.md'))
    output.parent.mkdir(parents=True, exist_ok=True)
    # Exclusive creation prevents silently replacing a previously delivered archive.
    with zipfile.ZipFile(output, 'x', compression=zipfile.ZIP_DEFLATED) as archive:
        for path, relative in files:
            archive.write(path, 'fr3-single-arm/'+relative)
        archive.writestr('fr3-single-arm/pytest.ini',
            '[pytest]\naddopts = --import-mode=importlib\n'
            'pythonpath = src/fr3_control_panel src/fr3_real_bringup\n'
            'testpaths = src/fr3_control_panel/test src/fr3_real_bringup/test\n')
    return len(files)+1


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    count = package(Path(__file__).resolve().parents[1], args.output.resolve())
    print(f'{args.output.resolve()} ({count} files; no simulation/task packages)')
