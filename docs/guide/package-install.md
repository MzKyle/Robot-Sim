# Deb 打包与 Release

## 本地构建 deb

```bash
source /opt/ros/humble/setup.bash
export GZ_VERSION=harmonic
bash packaging/build_deb.sh
```

输出：

```text
dist/robot-sim_<version>-<revision>_<arch>.deb
```

安装：

```bash
sudo apt install ./dist/robot-sim_0.1.0-1_amd64.deb
robot-sim-check
robot-sim run-case --case industrial_fixture_to_pallet
robot-sim migrate-config --input old.yaml --output new.yaml
robot-sim scaffold-robot --package my_robot_sim --robot-name my_robot --output /tmp --joint-names joint_1 joint_2 joint_3 joint_4 joint_5 joint_6
robot-sim sim_profile:=panda sim_mode:=light
```

自定义版本：

```bash
PACKAGE_VERSION=0.2.0 PACKAGE_REVISION=1 bash packaging/build_deb.sh
```

## GitHub Release

推送 tag 会触发 release workflow：

```bash
git tag v0.1.0
git push origin v0.1.0
```

workflow 会构建 deb，并上传到对应 GitHub Release。
