# セットアップ

対象はUbuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic。実行確認環境はWindows 11 + WSL 2 + NVIDIA GPU。

## 1. ROSと必要なパッケージ

ROS 2が未導入の場合は、先に[ROS 2 Jazzy公式手順](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)で配布元を登録する。

```bash
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-jazzy-ros-gz ros-dev-tools \
  ros-jazzy-turtlebot4-simulator ros-jazzy-irobot-create-nodes \
  ros-jazzy-turtlebot4-navigation ros-jazzy-rqt-image-view \
  ros-jazzy-cv-bridge python3-venv python3-yaml
source /opt/ros/jazzy/setup.bash
```

## 2. センサーのOgre 2設定

TurtleBot 4のセンサー設定によって、全方向のレーザーが0.164mになる既知の問題がある。
修正内容は[開発元のPR #249](https://github.com/iRobotEducation/create3_sim/pull/249)を参照。

```bash
CREATE3_XACRO="$(ros2 pkg prefix --share irobot_create_description)/urdf/create3.urdf.xacro"
grep -n 'render_engine' "$CREATE3_XACRO"
```

`<render_engine>ogre</render_engine>` がある場合のみ、バックアップを作成して変更する。

```bash
sudo cp -n "$CREATE3_XACRO" "${CREATE3_XACRO}.before-ogre2"
sudo sed -i 's#<render_engine>ogre</render_engine>#<render_engine>ogre2</render_engine>#g' "$CREATE3_XACRO"
```

すでにogre2なら変更不要。パッケージの更新で設定が変わった場合は再確認する。

## 3. AI環境

このリポジトリのフォルダーで実行する。ROSのPythonパッケージを利用するため、system-site-packagesを指定する。

```bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
python -m pip install -r requirements.txt
deactivate
```

モデル `yolo11s.pt` は初回起動時にダウンロードする。
Pythonライブラリの厳密なバージョン固定は行っていない。
既存の `~/robot_ai/.venv` がある場合は、demo.shがそれを利用することもできる。

## 4. 起動

```bash
bash demo.sh prepare
```

以後はREADMEの起動順に従う。各端末はリポジトリのフォルダーで開く。
demo.sh内でROS環境と、WSLgのNVIDIA描画設定を適用する。

WSLgでは[Microsoft公式のGPU選択方法](https://github.com/microsoft/wslg/wiki/GPU-selection-in-WSLg)を使用している。
この起動スクリプトの描画設定はNVIDIA/WSLg向け。別のOSやGPUでは適宜調整する。

## 5. 診断・録画

```bash
bash demo.sh check
bash demo.sh status
```

- 位置推定とNav2がactive、navigate_to_poseのサーバーが1つであることを確認する。
- レーザーのmin/median/maxが全方向0.164mになっていないことを確認する。
- メモリが不足する場合は、撮影に不要なRViz等を閉じ、長時間起動したままにしない。
- カメラ原画像は `runtime/snapshots/latest_camera.png`、検出結果は `latest_detection.jpg` に5秒ごとに保存する。
- WSLgのクリップボード連携が原因と疑われるGUI終了があったため、診断画像には自動保存を使った。
- GazeboのVideo Recorderで3D画面を録画できる。カメラの検出枠も映す場合は、別途画面録画や編集を行う。
- Video Recorderの設定はシミュレーション時刻に合わせている。提出前に実際の動画の長さを確認する。

終了は各端末でCtrl+C。再試行はsim・navも終了し、sim → nav → undock → ai → camera → searchの順に起動する。
