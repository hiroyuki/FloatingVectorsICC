# CUDA 12 + cuDNN 9 インストール指示書(Phase 1 / 人手)

対象: 開発機(RTX 4070 / ドライバ 591.86)。本番機にも同じ手順を適用する。
ピン止め根拠は `eval/PLAN_live_gpu.md` Phase 0(ORT 1.25.1 → CUDA 12.x + cuDNN 9.x)。

> ドライバ更新は**不要**。既存の CUDA 11.6 は**残したまま**サイドバイサイドで入れる。

## 1. CUDA Toolkit 12.6

- ダウンロード(NVIDIA 公式のみ): https://developer.nvidia.com/cuda-12-6-3-download-archive
  - 選択: Windows / x86_64 / 11 / exe (local)
- インストーラで **カスタム(詳細)** を選び、以下だけにチェック:
  - CUDA > Runtime(必須)
  - CUDA > Development(必須 — cudart 等のライブラリ)
  - それ以外(Nsight 系、Driver components、GeForce Experience 系)は**全部外す**
    (ドライバ 591.86 を上書きさせない)
- インストーラが `C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.6\bin` を
  PATH に追加する(エディタ実行はこれで足りる)

## 2. cuDNN 9.x for CUDA 12

- ダウンロード(NVIDIA 公式のみ、要 NVIDIA アカウント):
  https://developer.nvidia.com/cudnn-downloads
  - 選択: Windows / x86_64 / **Tarball(zip)** / CUDA 12 向けの最新 9.x
- **SHA-256 照合**: ダウンロードページ記載の checksum と突き合わせる
  ```powershell
  Get-FileHash .\cudnn-windows-x86_64-9.*.zip -Algorithm SHA256
  ```
  一致した値を下の「チェックサム記録」に書き込む
- 展開して `bin` の DLL 群(`cudnn64_9.dll` ほか)を PATH の通った場所へ。推奨:
  `C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.6\bin` にコピー
  (CUDA の PATH に相乗りでき、環境変数を増やさない)

## 3. TensorRT 10.x(今は入れない)

Phase 3 で CUDA EP が 30Hz 未達だった場合のみ。版はその時点で ORT 1.25 の対応を確定してから。

## 4. 確認コマンド

```powershell
nvidia-smi                  # ドライバが生きていること
where.exe cudart64_12.dll   # CUDA 12 ランタイムが PATH にあること
where.exe cudnn64_9.dll     # cuDNN 9 が PATH にあること
```

3 つとも通ったら Phase 2(Unity 統合)に進める。

## チェックサム記録

| ファイル | SHA-256 | 取得日 |
|---|---|---|
| cudnn-windows-x86_64-9.24.0.43_cuda12-archive.zip | `b190b5d2c2a1634606ca8f843cde84fdc762ba92ddb8b73bda9fb894c673b767`(公式 redistrib_9.24.0.json と一致確認済み) | 2026-07-17 |

## 埋め込みパッケージの native DLL 復元(fresh clone / 本番機セットアップ時)

埋め込みパッケージのバイナリ(dll / aar / dylib / so / iOS~)は git 管理外
(全体 657MB、providers_cuda だけで 294MB)。C# レイヤと .meta はコミット済み。
Windows x64 の復元手順(本プロジェクトの動作にはこれだけで十分):

1. https://www.nuget.org/api/v2/package/Microsoft.ML.OnnxRuntime.Gpu.Windows/**1.26.0** を取得
   (zip として展開可能)。
   > 版は embed パッケージの managed 層(asus4 0.4.8 = ORT 1.26.0)と必ず一致させる。
   > 1.25.1 と 1.26.0 では境界フレームの推論結果が変わり、録画済み v11s と
   > 骨格が一致しなくなる(eval/results/offline_ab_*.md で実証済み)
2. `runtimes/win-x64/native/` から以下 4 つを
   `Packages/com.github.asus4.onnxruntime/Plugins/Windows/x64/` にコピー:
   - `onnxruntime.dll`(**GPU ビルド本体** — asus4 配布の CPU 版とは別物)
   - `onnxruntime_providers_shared.dll`
   - `onnxruntime_providers_cuda.dll`
   - `onnxruntime_providers_tensorrt.dll`
3. `.meta` は git 管理下にあるのでそのままで OK

他プラットフォームのバイナリが必要になった場合は asus4 の npm tarball
(`com.github.asus4.onnxruntime@0.4.5`)から該当 `Plugins/` を展開して戻す。

> 背景: asus4 core パッケージの onnxruntime.dll は CPU 専用ビルドで、
> win-x64-gpu パッケージには本体 dll が入っていない(eval/PLAN_live_gpu.md
> 「Phase 2 実施記録」参照)。

## 本番機での注意(Phase 4)

- GPU 型番を先に確認。4070 未満なら Phase 3 のベンチを本番機でやり直す
- スタンドアロンビルドは PATH に依存させず、CUDA/cuDNN DLL を Assets/Plugins 配置にする
  (手順は Phase 2 で文書化予定)
