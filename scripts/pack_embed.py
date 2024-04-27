import subprocess
from pathlib import Path
import shutil, os
from urllib.request import urlretrieve
import zipfile
import platform

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]

Project_PATH = (ROOT/'..').resolve()
Version = "v0.1"


def download_progress_hook(count, blockSize, totalSize):
    s = count * blockSize
    print(f"{100*s/totalSize:.1f}%,\t {s}/{totalSize}")

def download_file(url, filename):
    print(f"downloading {filename} from {url}")
    d = urlretrieve(url, filename, reporthook=download_progress_hook)
    print("end download")


if __name__ == "__main__":

    build_path = Project_PATH.parent/(Project_PATH.stem+f"-{platform.system().lower()}-{platform.machine().lower()}-{Version}")
    if build_path.exists():
        shutil.rmtree(build_path, ignore_errors=True)

    program_path = build_path/"program"

    shutil.copytree(str(Project_PATH), str(program_path), ignore=shutil.ignore_patterns(".git", "*.pyc", "__pycache__"))

    embed_pack = build_path/Path("python-3.8.10-embed-amd64.zip")
    env_folder = build_path/Path(embed_pack.stem)
    print(env_folder.parts[-1])
    if not embed_pack.exists():
        download_file("https://www.python.org/ftp/python/3.8.10/python-3.8.10-embed-amd64.zip", str(embed_pack))

    with zipfile.ZipFile(str(embed_pack), 'r') as zip_ref:
        zip_ref.extractall(str(env_folder))
    embed_pack.unlink()
    
    download_file("https://bootstrap.pypa.io/get-pip.py", str(env_folder/"get-pip.py"))
    with open(env_folder/"python38._pth", "r+") as f:
        _pth = f.read()
        _pth = _pth.replace("#import site", "import site")
        f.seek(0)
        f.write(_pth)

    subprocess.check_call([str(env_folder.absolute()/"python"), env_folder/"get-pip.py"])
    subprocess.check_call([str(env_folder.absolute()/"Scripts"/"pip.exe"), "install", "-r", str(Project_PATH/"requirements.txt")])

    with open(build_path/"run.bat", "w") as f:
        f.write(f"set PATH=%~dp0\{env_folder.parts[-1]};%~dp0\{env_folder.parts[-1]}\Scripts;%PATH%\n")
        f.write(f"cmd.exe /K \"%~dp0\{env_folder.parts[-1]}\python\" {program_path.relative_to(build_path)}/xrobot.py\n")


