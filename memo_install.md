# 自動運転AIチャレンジ2024シミュレーション大会の環境構築メモ（GPU環境）

https://automotiveaichallenge.github.io/aichallenge-documentation-2024/setup/requirements.html

# PC環境
OS: Ubuntu 22.04 LTS
CPU: 13th Gen Intel(R) Core(TM) i7-13700F (16コア)
GPU: GeForce RTX 3060 12GB
Memory: 32GB
Storage: 1.0TB

過去大会の環境がインストールされているPCに今回大会の環境をインストールしていきます。

https://qiita.com/kiwsdiv/items/33b80778f0ddceb6ec44
https://qiita.com/kiwsdiv/items/474c851c1ee0b83b9e29


# ワークスペースのクローン
## 依存パッケージのインストール
Alt+Ctrl+Tでターミナルを立ち上げてから、以下に従ってコマンドをCtrl+Shift+Pで貼り付け、Enterで実行します。 まずは必要なライブラリをインストールします。

```bash
$ sudo apt update
```
大会用リポジトリのクローン
ワークスペース用のリポジトリをクローンします。ここではホームディレクトリを指定していますが、お好きなディレクトリに配置していただいて構いません。

```bash
$ cd ~
$ git clone https://github.com/AutomotiveAIChallenge/aichallenge-2024.git
```

# 仮想環境のインストール
## Dockerのインストール
前回大会までにインストール済み。
## rockerのインストール
前回大会までにインストール済み。
## Autoware環境のDockerイメージ取得

AIチャレンジで使用するautoware環境のDockerイメージを取得します。
Dockerイメージは10GB程度のサイズがあり、ダウンロードには時間が掛かるため有線LANでのダウンロードを推奨します。

作業前の状況を確認しておきます。
```bash
$ docker images
REPOSITORY                                                                       TAG                           IMAGE ID       CREATED         SIZE
<none>                                                                           <none>                        55af70099bca   6 months ago    7.43GB
aichallenge-train                                                                latest                        b057863e4e57   6 months ago    7.43GB
<none>                                                                           <none>                        64b4f44a303b   6 months ago    7.38GB
<none>                                                                           <none>                        a5f29f38cf2c   7 months ago    7.38GB
ghcr.io/automotiveaichallenge/aichallenge2023-racing/autoware-universe-no-cuda   latest                        b7fdf9678bc2   7 months ago    7.32GB
<none>                                                                           <none>                        2f805c4516be   10 months ago   15.4GB
aichallenge-eval                                                                 latest                        0487a9798c94   10 months ago   15.4GB
<none>                                                                           <none>                        f3fdbc8d286a   10 months ago   15.4GB
<none>                                                                           <none>                        5a77f44122e0   12 months ago   14.9GB
<none>                                                                           <none>                        abcae1bfea3c   12 months ago   1.07GB
golang                                                                           1.19                          6bf7ba70b6b4   12 months ago   1.06GB
nvidia/cuda                                                                      11.6.2-base-ubuntu20.04       af978b91c939   13 months ago   154MB
ghcr.io/automotiveaichallenge/aichallenge2023-sim/autoware-universe-cuda         v1                            f5f05f758f55   13 months ago   14.9GB
hello-world                                                                      latest                        9c7a54a9a43c   14 months ago   13.3kB
nvidia/opengl                                                                    1.0-glvnd-devel-ubuntu18.04   9d806b36b807   2 years ago     413MB
```

作業
```bash
$ docker images
REPOSITORY                                                                       TAG                           IMAGE ID       CREATED         SIZE
aichallenge-train                                                                latest                        b057863e4e57   6 months ago    7.43GB
ghcr.io/automotiveaichallenge/aichallenge2023-racing/autoware-universe-no-cuda   latest                        b7fdf9678bc2   7 months ago    7.32GB
aichallenge-eval                                                                 latest                        0487a9798c94   10 months ago   15.4GB
nvidia/cuda                                                                      11.6.2-base-ubuntu20.04       af978b91c939   13 months ago   154MB
ghcr.io/automotiveaichallenge/aichallenge2023-sim/autoware-universe-cuda         v1                            f5f05f758f55   13 months ago   14.9GB
nvidia/opengl                                                                    1.0-glvnd-devel-ubuntu18.04   9d806b36b807   2 years ago     413MB
```

Dockerイメージのダウンロードをしていきます。
```bash
$ docker pull ghcr.io/automotiveaichallenge/autoware-universe:humble-latest
humble-latest: Pulling from automotiveaichallenge/autoware-universe
3c645031de29: Pull complete 
051ef2a5d92c: Pull complete 
614ba3116b49: Pull complete 
03eac597c470: Pull complete 
fc76258c029c: Pull complete 
4f4fb700ef54: Pull complete 
c228afb9affc: Pull complete 
fbdc2c48cb4c: Pull complete 
b4660e304d25: Pull complete 
7adeea067886: Pull complete 
e77ce1d3d431: Pull complete 
0e361536324c: Pull complete 
92399c91acc1: Pull complete 
5015e87e6ff4: Pull complete 
a5bea2f0db06: Pull complete 
Digest: sha256:bea365a548dee5d35ee4d2726a5534fb68d661666668bd25e5e80dcb2d4c02bd
Status: Downloaded newer image for ghcr.io/automotiveaichallenge/autoware-universe:humble-latest
ghcr.io/automotiveaichallenge/autoware-universe:humble-latest
```

```bash
$ docker images
REPOSITORY                                                                       TAG                           IMAGE ID       CREATED         SIZE
ghcr.io/automotiveaichallenge/autoware-universe                                  humble-latest                 30c59f3fb415   2 months ago    8.84GB
<none>                                                                           <none>                        55af70099bca   6 months ago    7.43GB
aichallenge-train                                                                latest                        b057863e4e57   6 months ago    7.43GB
<none>                                                                           <none>                        64b4f44a303b   6 months ago    7.38GB
<none>                                                                           <none>                        a5f29f38cf2c   7 months ago    7.38GB
ghcr.io/automotiveaichallenge/aichallenge2023-racing/autoware-universe-no-cuda   latest                        b7fdf9678bc2   7 months ago    7.32GB
<none>                                                                           <none>                        2f805c4516be   10 months ago   15.4GB
aichallenge-eval                                                                 latest                        0487a9798c94   10 months ago   15.4GB
<none>                                                                           <none>                        f3fdbc8d286a   10 months ago   15.4GB
<none>                                                                           <none>                        5a77f44122e0   12 months ago   14.9GB
<none>                                                                           <none>                        abcae1bfea3c   12 months ago   1.07GB
golang                                                                           1.19                          6bf7ba70b6b4   12 months ago   1.06GB
nvidia/cuda                                                                      11.6.2-base-ubuntu20.04       af978b91c939   13 months ago   154MB
ghcr.io/automotiveaichallenge/aichallenge2023-sim/autoware-universe-cuda         v1                            f5f05f758f55   13 months ago   14.9GB
hello-world                                                                      latest                        9c7a54a9a43c   14 months ago   13.3kB
nvidia/opengl                                                                    1.0-glvnd-devel-ubuntu18.04   9d806b36b807   2 years ago     413MB
```

以下の表示が出ているので正しくインストールできている模様。
```
TAG                           IMAGE ID       CREATED         SIZE
ghcr.io/automotiveaichallenge/autoware-universe                                  humble-latest                 30c59f3fb415   2 months ago    8.84GB
```

# AWSIMのダウンロード （描画ありGPU版）

NVIDIAドライバ、NVIDIA Container Toolkit、Valkunは過去にインストール済みなので、AWSIMのダウンロードから作業しました。

## NVIDIAドライバのインストール
すでにインストール済み。インストール時のメモは以下。
https://qiita.com/kiwsdiv/items/3e8dac099cb27c7c4b01

```bash
$ nvidia-smi
Mon Jul 15 12:35:59 2024       
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 525.147.05   Driver Version: 525.147.05   CUDA Version: 12.0     |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|                               |                      |               MIG M. |
|===============================+======================+======================|
|   0  NVIDIA GeForce ...  Off  | 00000000:01:00.0  On |                  N/A |
|  0%   48C    P8    17W / 170W |    176MiB / 12288MiB |     26%      Default |
|                               |                      |                  N/A |
+-------------------------------+----------------------+----------------------+
                                                                               
+-----------------------------------------------------------------------------+
| Processes:                                                                  |
|  GPU   GI   CI        PID   Type   Process name                  GPU Memory |
|        ID   ID                                                   Usage      |
|=============================================================================|
|    0   N/A  N/A      4756      G   /usr/lib/xorg/Xorg                108MiB |
|    0   N/A  N/A      4899      G   /usr/bin/gnome-shell               26MiB |
|    0   N/A  N/A    167369      G   ...RendererForSitePerProcess       38MiB |
+-----------------------------------------------------------------------------+
```

## NVIDIA Container Toolkit
こちらも過去にインストール済み。
インストール時のメモ。

https://qiita.com/kiwsdiv/items/33b80778f0ddceb6ec44#nvidia-container-toolkit%E3%81%AE%E3%82%A4%E3%83%B3%E3%82%B9%E3%83%88%E3%83%BC%E3%83%AB

```bash
$ sudo docker run --rm --runtime=nvidia --gpus all nvidia/cuda:11.6.2-base-ubuntu20.04 nvidia-smi
Mon Jul 15 03:37:56 2024       
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 525.147.05   Driver Version: 525.147.05   CUDA Version: 12.0     |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|                               |                      |               MIG M. |
|===============================+======================+======================|
|   0  NVIDIA GeForce ...  Off  | 00000000:01:00.0  On |                  N/A |
|  0%   48C    P8    15W / 170W |    168MiB / 12288MiB |      4%      Default |
|                               |                      |                  N/A |
+-------------------------------+----------------------+----------------------+
                                                                               
+-----------------------------------------------------------------------------+
| Processes:                                                                  |
|  GPU   GI   CI        PID   Type   Process name                  GPU Memory |
|        ID   ID                                                   Usage      |
|=============================================================================|
+-----------------------------------------------------------------------------+
```

## AWSIMのダウンロード
[Google Drive](https://drive.google.com/drive/folders/1ftIoamNGAet90sXeG48lKa89dkpVy45y) から最新の AWSIM_GPU_**.zip をダウンロードし、aichallenge-2024/aichallenge/simulator に展開します。

## AWSIMの起動確認

```
$ ./docker_build.sh dev
[+] Building 12.6s (11/11) FINISHED                                                 docker:default
~~~
```

```
$ ./docker_run.sh dev gpu
```

コンテナを起動したターミナル(コンテナ内)で以下を実行します。
```
$ cd /aichallenge
$ ./build_autoware.bash
~~~
---
Finished <<< goal_pose_setter [13.7s]
Finished <<< simple_pure_pursuit [15.3s]                       
                      
Summary: 8 packages finished [15.5s]
  1 package had stderr output: goal_pose_setter

```

Autowareのビルド後、2つのファイルを変更します。AISIMのパスをAISIM_GPUに変更、AWSIM_GPUには程展開したディレクトリを指定します。

- run_simulator.bash

```
#!/bin/bash
# AWSIM_DIRECTORY=/aichallenge/simulator/AWSIM
AWSIM_DIRECTORY=/aichallenge/simulator/AWSIM_GPU

# shellcheck disable=SC1091
source /aichallenge/workspace/install/setup.bash
sudo ip link set multicast on lo
$AWSIM_DIRECTORY/AWSIM.x86_64
```
- run_simulator.bash
```
#!/bin/bash
# AWSIM_DIRECTORY=/aichallenge/simulator/AWSIM
AWSIM_DIRECTORY=/aichallenge/simulator/AWSIM_GPU
```

とりあえず1週走りました。


