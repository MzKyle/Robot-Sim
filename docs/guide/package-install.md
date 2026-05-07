# 安装与打包

## 打包命令

```bash
cd /home/kyle/sany/weld_data_collect_ws
bash packaging/build_deb.sh
```

## 产物位置

```text
dist/weld-data-collect_0.1.0-1_amd64.deb
```

## 安装方式

```bash
sudo apt install ./dist/weld-data-collect_0.1.0-1_amd64.deb
weld-data-collect-check
```

## 安装后常用命令

```bash
weld-data-collect
weld-data-collect-ui
```

## 打包注意事项

- 默认配置会安装到 `/etc/weld_data_collect/nodemanage.yaml`。
- 默认数据目录为 `/var/lib/weld_data_collect/data`。
- 生产包建议在干净的 Ubuntu 22.04 + ROS 2 Humble 环境中重新构建。
- 如果主机带有自定义 OpenCV，打包时可能会链接到 `/usr/local/lib/libopencv*.so`，生产发布前建议统一构建环境。
