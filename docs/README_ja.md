[English](README.md) | [日本語](README_ja.md) | **README_ja**

# Rosbridge MCP Server

rosbridge WebSocket 接続を介して ROS（Robot Operating System）と包括的に連携するための Model Context Protocol（MCP）サーバーです。この Python 実装により、AI アシスタントが標準化されたインターフェースを通じて ROS システムを監視・制御できます。

## 機能

- **トピック操作**: トピック一覧、トピック情報取得、メッセージ公開
- **サービス操作**: サービス一覧と ROS サービス呼び出し
- **アクション操作**: アクションサーバー一覧、ゴール送信、アクションキャンセル

## 使用方法

ニーズに応じて、以下の例から選択してください：

**基本的な使用方法（localhost）：**

```json
{
  "mcpServers": {
    "rosbridge": {
      "command": "uvx",
      "args": ["takanarishimbo-rosbridge-mcp-server"]
    }
  }
}
```

**カスタム rosbridge ホスト：**

```json
{
  "mcpServers": {
    "rosbridge": {
      "command": "uvx",
      "args": ["takanarishimbo-rosbridge-mcp-server"],
      "env": {
        "ROSBRIDGE_HOST": "192.168.1.100",
        "ROSBRIDGE_PORT": "9090"
      }
    }
  }
}
```

**リモート ROS システム：**

```json
{
  "mcpServers": {
    "rosbridge": {
      "command": "uvx",
      "args": ["takanarishimbo-rosbridge-mcp-server"],
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

rosbridge サーバーのホスト（デフォルト: "localhost"）

例：

- `localhost`: ローカルの rosbridge
- `192.168.1.100`: リモート IP アドレス
- `ros-robot.local`: ホスト名

### `ROSBRIDGE_PORT`

rosbridge サーバーのポート（デフォルト: "9090"）

標準的な rosbridge WebSocket ポートは 9090 です。

## 利用可能なツール

### トピック操作

#### `list_topics`

利用可能なすべての ROS トピックとその型を一覧表示します。

パラメータは不要です。

#### `get_topic_info`

パブリッシャーやサブスクライバーを含む、特定のトピックの詳細情報を取得します。

パラメータ：

- `topic`（必須）：ROS トピック名（例: "/cmd_vel"）

#### `publish_topic`

ROS トピックにメッセージを公開します。

パラメータ：

- `topic`（必須）：ROS トピック名（例: "/cmd_vel"）
- `message_type`（必須）：ROS メッセージタイプ（例: "geometry_msgs/Twist"）
- `message`（必須）：JSON オブジェクトとしてのメッセージデータ

例：

```json
{
  "name": "publish_topic",
  "arguments": {
    "topic": "/cmd_vel",
    "message_type": "geometry_msgs/Twist",
    "message": {
      "linear": { "x": 0.5, "y": 0.0, "z": 0.0 },
      "angular": { "x": 0.0, "y": 0.0, "z": 0.1 }
    }
  }
}
```

### サービス操作

#### `list_services`

利用可能なすべての ROS サービスを一覧表示します。

パラメータは不要です。

#### `publish_service`

ROS サービスを呼び出します。

パラメータ：

- `service`（必須）：ROS サービス名（例: "/add_two_ints"）
- `service_type`（必須）：ROS サービスタイプ（例: "rospy_tutorials/AddTwoInts"）
- `request`（必須）：JSON オブジェクトとしてのサービスリクエストデータ
- `timeout`（オプション）：タイムアウト秒数（デフォルト: 10）

例：

```json
{
  "name": "publish_service",
  "arguments": {
    "service": "/add_two_ints",
    "service_type": "rospy_tutorials/AddTwoInts",
    "request": {
      "a": 10,
      "b": 20
    }
  }
}
```

### アクション操作

#### `list_actions`

利用可能なすべての ROS アクションサーバーを一覧表示します。

パラメータは不要です。

#### `publish_action`

ROS アクションサーバーにゴールを送信します。

パラメータ：

- `action_name`（必須）：ROS アクションサーバー名（例: "/move_base"）
- `action_type`（必須）：ROS アクションタイプ（例: "move_base_msgs/MoveBaseAction"）
- `goal`（必須）：JSON オブジェクトとしてのゴールデータ
- `timeout`（オプション）：結果待機のタイムアウト秒数（デフォルト: 30）

例：

```json
{
  "name": "publish_action",
  "arguments": {
    "action_name": "/move_base",
    "action_type": "move_base_msgs/MoveBaseAction",
    "goal": {
      "target_pose": {
        "header": {
          "frame_id": "map"
        },
        "pose": {
          "position": { "x": 1.0, "y": 2.0, "z": 0.0 },
          "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
        }
      }
    }
  }
}
```

#### `cancel_action`

実行中の ROS アクションゴールをキャンセルします。

パラメータ：

- `action_name`（必須）：ROS アクションサーバー名（例: "/move_base"）
- `goal_id`（オプション）：キャンセルする特定のゴール ID（指定しない場合は全てキャンセル）

## 開発

1. **このリポジトリをクローン**

   ```bash
   git clone https://github.com/TakanariShimbo/rosbridge-mcp-server.git
   cd rosbridge-mcp-server
   ```

2. **uv を使用して依存関係をインストール**

   ```bash
   uv sync
   ```

3. **ROS システムで rosbridge を起動**

   ```bash
   roslaunch rosbridge_server rosbridge_websocket.launch
   ```

4. **サーバーを実行**

   ```bash
   uv run takanarishimbo-rosbridge-mcp-server
   ```

5. **MCP Inspector でテスト（オプション）**

   ```bash
   npx @modelcontextprotocol/inspector uv run takanarishimbo-rosbridge-mcp-server
   ```

## PyPI への公開

このプロジェクトは GitHub Actions 経由で安全でトークンレスな公開のために、PyPI の Trusted Publishers 機能を使用しています。

### 1. PyPI Trusted Publisher の設定

1. **PyPI にログイン**（必要に応じてアカウントを作成）

   - https://pypi.org/ にアクセス

2. **公開設定に移動**

   - アカウント設定に移動
   - "Publishing"をクリック、または https://pypi.org/manage/account/publishing/ にアクセス

3. **GitHub パブリッシャーを追加**
   - "Add a new publisher"をクリック
   - パブリッシャーとして"GitHub"を選択
   - 以下を入力：
     - **Owner**: `TakanariShimbo`（GitHub ユーザー名/組織）
     - **Repository**: `rosbridge-mcp-server`
     - **Workflow name**: `pypi-publish.yml`
     - **Environment**: `pypi`（オプションだが推奨）
   - "Add"をクリック

### 2. GitHub 環境の設定（推奨）

1. **リポジトリ設定に移動**

   - GitHub リポジトリに移動
   - "Settings" → "Environments"をクリック

2. **PyPI 環境を作成**
   - "New environment"をクリック
   - 名前: `pypi`
   - 保護ルールを設定（オプション）：
     - 必要なレビュアーを追加
     - 特定のブランチ/タグに制限

### 3. GitHub パーソナルアクセストークンの設定（リリーススクリプト用）

リリーススクリプトは GitHub にプッシュする必要があるため、GitHub トークンが必要です：

1. **GitHub パーソナルアクセストークンを作成**

   - https://github.com/settings/tokens にアクセス
   - "Generate new token" → "Generate new token (classic)"をクリック
   - 有効期限を設定（推奨：90 日またはカスタム）
   - スコープを選択：
     - ✅ `repo`（プライベートリポジトリの完全な制御）
   - "Generate token"をクリック
   - 生成されたトークンをコピー（`ghp_`で始まる）

2. **Git にトークンを設定**

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

1. **GitHub Actions を確認**

   - リポジトリの"Actions"タブに移動
   - "Publish to PyPI"ワークフローが正常に完了したことを確認

2. **PyPI パッケージを確認**
   - https://pypi.org/project/rosbridge-mcp-server/ にアクセス
   - または実行: `pip show rosbridge-mcp-server`

### リリースプロセスフロー

1. `release.sh`スクリプトがすべてのファイルのバージョンを更新
2. git コミットとタグを作成
3. GitHub にプッシュ
4. 新しいタグで GitHub Actions ワークフローがトリガー
5. ワークフローが OIDC を使用して PyPI で認証（トークン不要！）
6. ワークフローがプロジェクトをビルドして PyPI に公開
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
│   ├── server.py                # サーバー実装
│   └── tools/                   # ツール実装
│       ├── __init__.py          # ツールモジュール初期化
│       ├── list_topics.py       # トピック一覧ツール
│       ├── list_actions.py      # アクション一覧ツール
│       ├── list_services.py     # サービス一覧ツール
│       ├── get_topic_info.py    # トピック情報取得ツール
│       ├── publish_topic.py     # トピック公開ツール
│       ├── publish_action.py    # アクション公開ツール
│       ├── publish_service.py   # サービス公開ツール
│       └── cancel_action.py     # アクションキャンセルツール
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

## ライセンス

MIT
