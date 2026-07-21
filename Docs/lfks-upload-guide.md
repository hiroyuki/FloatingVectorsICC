# LFKS ファイルアップロード

> **オリジンとトークンは対で管理する。** ピン留めした `Assets/StreamingAssets/lfks/upload.ps1`
> は本番 `https://ntticc.lfks.app` を既定の API オリジンにしている（コミット `c2b69a4` の
> 本番移行）。このガイドに載っているトークンは**ステージング
> `lfks-staging.circuit-lab.workers.dev` 用**で、本番に出すと期限に関係なく
> `HTTP 404 invalid or expired token` で落ちる。
> オリジンを切り替えるには `ExperienceConfig.lfksApiUrl` を設定する（空 = スクリプト既定の本番）。
> 本番で動かすには**本番オリジン用のトークン**をサーバーチームから受け取ること。

## 《みちのからだ》専用ディレクトリとトークン

- ディレクトリ: <https://lfks-staging.circuit-lab.workers.dev/browse?token=tkd9Ik8hGZpVmKuLqvUEQhF0SULp8eD8VwgIMQS9>
- トークン: `tkd9Ik8hGZpVmKuLqvUEQhF0SULp8eD8VwgIMQS9`（有効期限: 2026年10月30日23時59分）

## アップロードスクリプト

- Windows: https://lfks-staging.circuit-lab.workers.dev/upload.ps1
- macOS / Linux: https://lfks-staging.circuit-lab.workers.dev/upload.sh

### オプション

|                    | 説明                                                                                     | macOS / Linux (`upload.sh`)                        | Windows (`upload.ps1`)                            |
| ------------------ | -------------------------------------------------------------------------------------- | -------------------------------------------------- | ------------------------------------------------- |
| トークン（必須）    | 作品用ディレクトリへのアクセストークン                                                 | `--token tkd9Ik8hGZpVmKuLqvUEQhF0SULp8eD8VwgIMQS9` | `-Token tkd9Ik8hGZpVmKuLqvUEQhF0SULp8eD8VwgIMQS9` |
| ファイルパス（必須） | アップロードするファイルのパス                                                          | `--file <path/to/file>`                            | `-File <path\to\file>`                            |
| サブディレクトリの指定 | サブディレクトリを指定してアップロード。未指定の場合は作品用ディレクトリのルートに保存（自動作成）。 | `--dir sub`                                         | `-Directory sub`                                  |
| ファイル名の指定    | サーバー上で保存するファイル名を、元のローカルファイル名から変更。未指定の場合は元のファイル名のまま保存。既存のファイル名を指定すると上書き。 | `--name renamed.bin`                               | `-Name renamed.bin`                               |
| JSON出力           | アップロード進捗の出力を抑制し、最終結果のJSONのみを出力。                              | `--json`                                           | `-Json`                                           |
| アップロード先サーバーの指定 | 通常は不要（取得元のサーバーを自動使用）。別のサーバーへ向ける場合に指定。          | `--url https://<worker-domain>`                    | `-Url https://<worker-domain>`                    |

### 出力（JSON）

| key          | value  | 説明                                              |
| ------------ | ------ | ------------------------------------------------- |
| id           | string | LFKS で内部的に使用されるファイルID                |
| name         | string | 保存されたファイル名（`--name`/`-Name` 指定時はその名前、未指定なら元のファイル名） |
| relativePath | string | 作品用ディレクトリを起点とした相対パス             |
| size         | number | ファイルサイズ（バイト）                           |
| downloadUrl  | string | ファイルIDに基づく固定のダウンロードURL            |
| fs           | string | 保存先パス・ファイル名に基づくURL                  |

### 実行例（スクリプト）

```
# macOS / Linux
$ curl -fsSL https://lfks-staging.circuit-lab.workers.dev/upload.sh | sh -s -- --token tkd9Ik8hGZpVmKuLqvUEQhF0SULp8eD8VwgIMQS9 --file /Users/user/file4.bin --dir test --name upload-test.bin
uploading file4.bin as test/upload-test.bin (83886080 bytes, 2 part(s))
part 1/2 ...
part 2/2 ...
done: upload-test.bin uploaded (file id: l8jzM9QtBpJDUE3f)
download: https://lfks-staging.circuit-lab.workers.dev/files/l8jzM9QtBpJDUE3f.bin
fs: https://lfks-staging.circuit-lab.workers.dev/fs/kids2026/unknown-but-yours/test/upload-test.bin
{"id":"l8jzM9QtBpJDUE3f","name":"upload-test.bin","relativePath":"test/upload-test.bin","size":83886080,"downloadUrl":"https://lfks-staging.circuit-lab.workers.dev/files/l8jzM9QtBpJDUE3f.bin","fs":"https://lfks-staging.circuit-lab.workers.dev/fs/kids2026/unknown-but-yours/test/upload-test.bin"}


$ curl -fsSL https://lfks-staging.circuit-lab.workers.dev/upload.sh -o upload.sh
$ sh upload.sh --token tkd9Ik8hGZpVmKuLqvUEQhF0SULp8eD8VwgIMQS9 --file /Users/user/test-file1.bin --json
{"id":"h1VDGHAVyDB4em3P","name":"test-file1.bin","relativePath":"test-file1.bin","size":524288000,"downloadUrl":"https://lfks-staging.circuit-lab.workers.dev/files/h1VDGHAVyDB4em3P.bin","fs":"https://lfks-staging.circuit-lab.workers.dev/fs/kids2026/unknown-but-yours/test-file1.bin"}

# Windows PowerShell
> & ([scriptblock]::Create((irm https://lfks-staging.circuit-lab.workers.dev/upload.ps1))) -Token tkd9Ik8hGZpVmKuLqvUEQhF0SULp8eD8VwgIMQS9 -File C:\Users\user\test-file2.usdz -Name upload-test.usdz -Directory sub-a/sub-b -Json
{"id":"f3GCzQdTsQfljMEW","name":"upload-test.usdz","relativePath":"sub-a/sub-b/upload-test.usdz","size":11019966,"downloadUrl":"https://lfks-staging.circuit-lab.workers.dev/files/f3GCzQdTsQfljMEW.usdz","fs":"https://lfks-staging.circuit-lab.workers.dev/fs/kids2026/unknown-but-yours/sub-a/sub-b/upload-test.usdz"}
```

### 実行例（Windows版 Python から呼び出し）

```python
import json
import subprocess
import urllib.request

WORKER_URL = "https://lfks-staging.circuit-lab.workers.dev"
TOKEN = "tkd9Ik8hGZpVmKuLqvUEQhF0SULp8eD8VwgIMQS9"
FILE_PATH = r"C:\path\to\file"

# 1. upload.ps1 をダウンロード
SCRIPT_PATH = "upload.ps1"
urllib.request.urlretrieve(f"{WORKER_URL}/upload.ps1", SCRIPT_PATH)

# 2. PowerShell から実行（-Json で最終JSON行だけを標準出力に流す）
# powershell.exe の -File は「次の1トークンをスクリプトパスとして扱い、
# それ以降は全部スクリプトへの引数として渡す」ので、
# ps1側の -File パラメータ名と衝突しても問題なく動く。
result = subprocess.run(
    [
        "powershell.exe",
        "-NoProfile",
        "-ExecutionPolicy", "Bypass",
        "-File", SCRIPT_PATH,
        "-Token", TOKEN,
        "-File", FILE_PATH,
        # "-Directory", "sub",       # 必要なら
        # "-Name", "renamed.bin",    # 必要なら
        "-Json",
    ],
    capture_output=True,
    text=True,
)

if result.returncode != 0:
    raise RuntimeError(f"upload failed (exit {result.returncode}): {result.stderr}")

data = json.loads(result.stdout)
print("uploaded:", data["id"], "->", data["downloadUrl"])
```

## アップロード済みファイルの確認

専用ページから確認/削除可能。

https://lfks-staging.circuit-lab.workers.dev/browse?token=tkd9Ik8hGZpVmKuLqvUEQhF0SULp8eD8VwgIMQS9
