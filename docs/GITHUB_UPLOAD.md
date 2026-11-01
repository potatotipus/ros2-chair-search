# GitHubへの登録方法

Ubuntuの端末で実行する。GitHubの個人アカウントを使用する例。
以下は新規リポジトリを作成してpushする手順。

## 1. GitHubへログイン

```bash
sudo apt update
sudo apt install -y git gh
gh auth login --hostname github.com --git-protocol https --web
gh auth setup-git
```

表示されたワンタイムコードを、ブラウザーの認証画面で入力する。
ブラウザーが開かなければ、Windowsのブラウザーで https://github.com/login/device を開く。

## 2. コードをGitで管理する

提出用ファイルを展開したフォルダーへ移動する。
例では `~/robot_ai/ros2-chair-search` に展開済みとする。

```bash
cd ~/robot_ai/ros2-chair-search
git init -b main
git config user.name "$(gh api user --jq .login)"
```

次のメールアドレスは実際に使用するコミット用メールへ置き換える。
GitHubの Settings → Emails にあるnoreplyアドレスも利用できる。

```bash
git config user.email "ここをGitHubのコミット用メールアドレスに置き換える"
git add .
git status --short
git commit -m "Add ROS 2 AI chair search demo"
```

## 3. 新しい公開リポジトリを作成してpush

```bash
gh repo create ros2-chair-search --public --source=. --remote=origin --push
gh repo view --json url --jq .url
```

最後に表示されたURLが、提出するリポジトリのURL。
このコマンドはGitHub上に公開リポジトリを作り、ローカルのコミットをpushする。
同名のリポジトリがすでにある場合は、リポジトリ名を別のものに変更する。
課題で公開範囲が指定されている場合は、その指定に従う。

ブラウザーで表示を確認する場合：

```bash
gh repo view --web
```

README.mdとPythonコードが表示されることを確認してから、URLを提出する。

## 4. 説明文を変更した後の更新

```bash
git add README.md docs/
git commit -m "Update project explanation"
git push
```

## 参考

- [GitHub公式：ローカルのコードをGitHubへ追加する](https://docs.github.com/ja/migrations/importing-source-code/using-the-command-line-to-import-source-code/adding-locally-hosted-code-to-github)
- [GitHub CLI公式：gh repo create](https://cli.github.com/manual/gh_repo_create)
- [GitHub公式：コミット用メールアドレス](https://docs.github.com/ja/account-and-profile/how-tos/email-preferences/setting-your-commit-email-address)
