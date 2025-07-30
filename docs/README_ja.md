[English](README.md) | [日本語](README_ja.md) | **README_ja**

# Rosbridge MCP Server

Model Context Protocol (MCP) サーバーで、rosbridgeを介してROSトピックにメッセージを公開するツールを提供します。これはMCPサーバーとROSの統合を示すPython実装です。

## 機能

- rosbridge WebSocketを介して任意のROSトピックにメッセージを公開
- 環境変数による設定可能なrosbridge接続
- 任意のROSメッセージタイプをサポート
- シンプルなツール: `publish_topic`

## 使用方法

ニーズに応じて、以下の例から選択してください：

**基本的な使用方法（localhost）：**

```json
{
  "mcpServers": {
    "rosbridge": {
      "command": "uvx",
      "args": ["rosbridge-mcp-server"]
    }
  }
}
```

**カスタムrosbridgeホスト：**

```json
{
  "mcpServers": {
    "rosbridge": {
      "command": "uvx",
      "args": ["rosbridge-mcp-server"],
      "env": {
        "ROSBRIDGE_HOST": "192.168.1.100",
        "ROSBRIDGE_PORT": "9090"
      }
    }
  }
}
```

**リモートROSシステム：**

```json
{
  "mcpServers": {
    "rosbridge": {
      "command": "uvx",
      "args": ["rosbridge-mcp-server"],
      "env": {
        "ROSBRIDGE_HOST": "ros-robot.local",
        "ROSBRIDGE_PORT": "9091"
      }
    }
  }
}
```

## 設定

サーバーは環境変数を使用して設定できます：

### `ROSBRIDGE_HOST`

rosbridgeサーバーのホスト（デフォルト: "localhost"）

例：
- `localhost`: ローカルのrosbridge
- `192.168.1.100`: リモートIPアドレス
- `ros-robot.local`: ホスト名

### `ROSBRIDGE_PORT`

rosbridgeサーバーのポート（デフォルト: "9090"）

標準的なrosbridge WebSocketポートは9090です。

## 利用可能なツール

### `publish_topic`

ROSトピックにメッセージを公開

パラメータ：

- `topic`（必須）：ROSトピック名（例: "/cmd_vel"）
- `message_type`（必須）：ROSメッセージタイプ（例: "geometry_msgs/Twist"）
- `message`（必須）：JSONオブジェクトとしてのメッセージデータ

使用例：

```json
{
  "name": "publish_topic",
  "arguments": {
    "topic": "/cmd_vel",
    "message_type": "geometry_msgs/Twist",
    "message": {
      "linear": {"x": 0.5, "y": 0.0, "z": 0.0},
      "angular": {"x": 0.0, "y": 0.0, "z": 0.1}
    }
  }
}
```

## 開発

1. **このリポジトリをクローン**

   ```bash
   git clone https://github.com/yourusername/rosbridge-mcp-server.git
   cd rosbridge-mcp-server
   ```

2. **uvを使用して依存関係をインストール**

   ```bash
   uv sync
   ```

3. **ROSシステムでrosbridgeを起動**

   ```bash
   roslaunch rosbridge_server rosbridge_websocket.launch
   ```

4. **サーバーを実行**

   ```bash
   uv run rosbridge-mcp-server
   ```

5. **MCP Inspectorでテスト（オプション）**

   ```bash
   npx @modelcontextprotocol/inspector uv run rosbridge-mcp-server
   ```

## PyPIへの公開

このプロジェクトはGitHub Actions経由で安全でトークンレスな公開のために、PyPIのTrusted Publishers機能を使用しています。

### 1. PyPI Trusted Publisherの設定

1. **PyPIにログイン**（必要に応じてアカウントを作成）

   - https://pypi.org/ にアクセス

2. **公開設定に移動**

   - アカウント設定に移動
   - "Publishing"をクリック、または https://pypi.org/manage/account/publishing/ にアクセス

3. **GitHubパブリッシャーを追加**
   - "Add a new publisher"をクリック
   - パブリッシャーとして"GitHub"を選択
   - 以下を入力：
     - **Owner**: `yourusername`（GitHubユーザー名/組織）
     - **Repository**: `rosbridge-mcp-server`
     - **Workflow name**: `pypi-publish.yml`
     - **Environment**: `pypi`（オプションだが推奨）
   - "Add"をクリック

### 2. GitHub環境の設定（推奨）

1. **リポジトリ設定に移動**

   - GitHubリポジトリに移動
   - "Settings" → "Environments"をクリック

2. **PyPI環境を作成**
   - "New environment"をクリック
   - 名前: `pypi`
   - 保護ルールを設定（オプション）：
     - 必要なレビュアーを追加
     - 特定のブランチ/タグに制限

### 3. GitHubパーソナルアクセストークンの設定（リリーススクリプト用）

リリーススクリプトはGitHubにプッシュする必要があるため、GitHubトークンが必要です：

1. **GitHubパーソナルアクセストークンを作成**

   - https://github.com/settings/tokens にアクセス
   - "Generate new token" → "Generate new token (classic)"をクリック
   - 有効期限を設定（推奨：90日またはカスタム）
   - スコープを選択：
     - ✅ `repo`（プライベートリポジトリの完全な制御）
   - "Generate token"をクリック
   - 生成されたトークンをコピー（`ghp_`で始まる）

2. **Gitにトークンを設定**

   ```bash
   # オプション1: GitHub CLIを使用（推奨）
   gh auth login

   # オプション2: gitにトークンを使用するよう設定
   git config --global credential.helper store
   # パスワードを求められたら、代わりにトークンを使用
   ```

### 4. 新しいバージョンをリリース

リリーススクリプトを使用して、自動的にバージョン管理、タグ付け、公開をトリガー：

```bash
# 初回設定
chmod +x scripts/release.sh

# パッチバージョンを増分（0.1.0 → 0.1.1）
./scripts/release.sh patch

# マイナーバージョンを増分（0.1.0 → 0.2.0）
./scripts/release.sh minor

# メジャーバージョンを増分（0.1.0 → 1.0.0）
./scripts/release.sh major

# 特定のバージョンを設定
./scripts/release.sh 1.2.3
```

### 5. 公開の確認

1. **GitHub Actionsを確認**

   - リポジトリの"Actions"タブに移動
   - "Publish to PyPI"ワークフローが正常に完了したことを確認

2. **PyPIパッケージを確認**
   - https://pypi.org/project/rosbridge-mcp-server/ にアクセス
   - または実行: `pip show rosbridge-mcp-server`

### リリースプロセスフロー

1. `release.sh`スクリプトがすべてのファイルのバージョンを更新
2. gitコミットとタグを作成
3. GitHubにプッシュ
4. 新しいタグでGitHub Actionsワークフローがトリガー
5. ワークフローがOIDCを使用してPyPIで認証（トークン不要！）
6. ワークフローがプロジェクトをビルドしてPyPIに公開
7. パッケージが`pip install`または`uvx`経由でグローバルに利用可能に

## コード品質

このプロジェクトはリンティングとフォーマットに`ruff`を使用しています：

```bash
# リンターを実行
uv run ruff check

# リンティングの問題を修正
uv run ruff check --fix

# コードをフォーマット
uv run ruff format
```

## プロジェクト構造

```
rosbridge-mcp-server/
├── src/
│   ├── __init__.py              # パッケージ初期化
│   ├── __main__.py              # メインエントリーポイント
│   └── server.py                # サーバー実装
├── pyproject.toml               # プロジェクト設定
├── uv.lock                      # 依存関係ロックファイル
├── .github/
│   └── workflows/
│       └── pypi-publish.yml     # Trusted Publishersを使用したPyPI公開ワークフロー
├── scripts/
│   └── release.sh               # リリース自動化スクリプト
├── docs/
│   ├── README.md                # 英語版ドキュメント
│   └── README_ja.md             # このファイル
└── .gitignore                   # Git無視ファイル
```

## ROSメッセージの例

### geometry_msgs/Twist（ロボットの速度）

```json
{
  "topic": "/cmd_vel",
  "message_type": "geometry_msgs/Twist",
  "message": {
    "linear": {"x": 1.0, "y": 0.0, "z": 0.0},
    "angular": {"x": 0.0, "y": 0.0, "z": 0.5}
  }
}
```

### std_msgs/String（シンプルなテキスト）

```json
{
  "topic": "/chatter",
  "message_type": "std_msgs/String",
  "message": {
    "data": "Hello, ROS!"
  }
}
```

### sensor_msgs/JointState（ジョイント位置）

```json
{
  "topic": "/joint_states",
  "message_type": "sensor_msgs/JointState",
  "message": {
    "name": ["joint1", "joint2", "joint3"],
    "position": [0.0, 1.57, -1.57],
    "velocity": [0.0, 0.0, 0.0],
    "effort": [0.0, 0.0, 0.0]
  }
}
```

## トラブルシューティング

### 接続失敗

"Failed to connect to rosbridge"が表示される場合：

1. **rosbridgeが実行されていることを確認**
   ```bash
   rosnode list | grep rosbridge
   ```

2. **rosbridgeポートを確認**
   ```bash
   rostopic list  # 利用可能なトピックが表示されるはず
   ```

3. **rosbridge接続をテスト**
   ```bash
   # rosbridgeクライアントをインストール
   pip install roslibpy
   
   # 接続をテスト
   python -c "import roslibpy; client = roslibpy.Ros(host='localhost', port=9090); client.run(); print('Connected!' if client.is_connected else 'Failed')"
   ```

4. **ファイアウォール設定を確認**
   - ポート9090（またはカスタムポート）が開いていることを確認
   - WebSocket接続をブロックするファイアウォールがないことを確認

### メッセージ公開の問題

1. **トピックが存在することを確認**
   ```bash
   rostopic list | grep your_topic_name
   ```

2. **メッセージタイプを確認**
   ```bash
   rostopic info /your_topic_name
   ```

3. **トピックをモニタリング**
   ```bash
   rostopic echo /your_topic_name
   ```

## ライセンス

MIT