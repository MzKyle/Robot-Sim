# Debian Packaging

`packaging/build_deb.sh` 会把当前工作空间构建为 `robot-sim` deb 包。

## 本地构建

```bash
source /opt/ros/humble/setup.bash
export GZ_VERSION=harmonic
bash packaging/build_deb.sh
```

输出：

```text
dist/robot-sim_<version>-<revision>_<arch>.deb
```

自定义版本：

```bash
PACKAGE_VERSION=0.2.0 PACKAGE_REVISION=1 bash packaging/build_deb.sh
```

## 安装

```bash
sudo apt install ./dist/robot-sim_0.1.0-1_amd64.deb
robot-sim-check
robot-sim run-case --case industrial_fixture_to_pallet
robot-sim sim_profile:=panda sim_mode:=light
robot-sim sim_profile:=fanuc_m20id12l sim_mode:=full
```

## GitHub Release

推送 `vMAJOR.MINOR.PATCH` tag 会触发 `.github/workflows/release.yml`：

```bash
git tag v0.1.0
git push origin v0.1.0
```

workflow 会运行本脚本，并把 `dist/*.deb` 上传到对应 Release。
