# CUDA 13 + cuDNN 9 インストール指示書(Phase 1 / 人手)

対象: 開発機(RTX 4070)と本番機(RTX 5080)の両方。**cu13 統一**(2026-07-19 決定)。

> 旧 `INSTALL_cuda12.md` からの変更理由: RTX 5080(Blackwell / sm_120)では
> ORT 公式 cu12 ビルドが sm_120 カーネル未同梱で `cudaErrorInvalidPtx` になる報告あり
> (microsoft/onnxruntime#26177、未解決)。CUDA 13 系 ORT は Blackwell ネイティブ対応で、
> 4070(Ada / sm_89)も CUDA 13 のサポート範囲内(切り捨ては Volta 以前)。
> 両マシン同一バイナリにすることでビルドフレーバー差の骨格ズレ
> (`eval/PLAN_live_gpu.md` Phase 3 追補 2)も構造的に防ぐ。

> ドライバ更新は**不要**(CUDA 13 は r580 以降: 開発機 591.86 ✓ / 本番機 610.62 ✓)。
> 既存の CUDA 11.6 / 12.x は**残したまま**サイドバイサイドで入れる。

## 1. CUDA Toolkit 13.x

- ダウンロード(NVIDIA 公式のみ): https://developer.nvidia.com/cuda-downloads
  - 選択: Windows / x86_64 / 11 / exe (local)
- インストーラで **カスタム(詳細)** を選び、以下だけにチェック:
  - CUDA > Runtime(必須)
  - CUDA > Development(必須 — cudart 等のライブラリ)
  - それ以外(Nsight 系、Driver components 系)は**全部外す**(既存ドライバを上書きさせない)
- インストーラが `C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v13.x\bin` を
  PATH に追加する(エディタ実行はこれで足りる)

## 2. cuDNN 9.x for CUDA 13

- ダウンロード(NVIDIA 公式のみ): https://developer.nvidia.com/cudnn-downloads
  - 選択: Windows / x86_64 / **Tarball(zip)** / **CUDA 13** 向けの最新 9.x
    (cu12 向け zip と間違えないこと — ファイル名末尾 `_cuda13-archive`)
- **SHA-256 照合**: ダウンロードページ記載の checksum と突き合わせる
  ```powershell
  Get-FileHash .\cudnn-windows-x86_64-9.*_cuda13-archive.zip -Algorithm SHA256
  ```
  一致した値を下の「チェックサム記録」に書き込む
- 展開して `bin` の DLL 群(`cudnn64_9.dll` ほか)を PATH の通った場所へ。推奨:
  CUDA 13 の `bin` にコピー(CUDA の PATH に相乗りでき、環境変数を増やさない)

## 3. TensorRT(今は入れない)

CUDA EP が 30Hz 未達だった場合のみ(4070 では CUDA EP で到達済みのため、
本番機 5080 で未達になった場合に検討)。

## 4. 確認コマンド

ORT cu13 1.26.0 の providers_cuda.dll が実際に要求する DLL(バイナリ実測):
`cublas64_13` / `cublasLt64_13` / `cufft64_12` / `cudnn64_9`

```powershell
nvidia-smi                   # ドライバが生きていること
where.exe cublas64_13.dll    # CUDA 13 ランタイムが PATH にあること
where.exe cufft64_12.dll     # 同上(cuFFT は CUDA 13 でも soname 12)
where.exe cudnn64_9.dll      # cuDNN 9 (cu13) が PATH にあること
```

全部通ったらエディタで CUDA EP スモーク(`ActiveProvider == Cuda` 確認)に進める。

## チェックサム記録

| ファイル | SHA-256 | 取得日 |
|---|---|---|
| cudnn-windows-x86_64-9.24.0.43_cuda12-archive.zip(旧 cu12 構成、参考) | `b190b5d2c2a1634606ca8f843cde84fdc762ba92ddb8b73bda9fb894c673b767` | 2026-07-17 |
| cudnn-windows-x86_64-9.24.0.43_cuda13-archive.zip | `88f72bd1ce384197cedbc68496c6052d7ff0bd9fd0b3c74470402cf737507e06`(公式 redistrib_9.24.0.json と一致確認済み) | 2026-07-19 |
| onnxruntime-win-x64-gpu_cuda13-1.26.0.zip(GitHub 公式リリース) | `4fa096030ee766b2e590d71fb6676bbd00595c92ab87acf497fe075e98834d8b` | 2026-07-19 |

## 埋め込みパッケージの native DLL 復元(fresh clone / 本番機セットアップ時)

埋め込みパッケージのバイナリ(dll / aar / dylib / so / iOS~)は git 管理外。
C# レイヤと .meta はコミット済み。Windows x64 の復元手順:

1. https://github.com/microsoft/onnxruntime/releases/download/v1.26.0/onnxruntime-win-x64-gpu_cuda13-1.26.0.zip
   を取得し、上表の SHA-256 と照合
   > **cu13 ビルドを使う**(NuGet の Microsoft.ML.OnnxRuntime.Gpu.Windows 1.26.0 は
   > cu12 ビルドなので使わない)。版は embed パッケージの managed 層
   > (asus4 0.4.8 = ORT 1.26.0)と必ず一致させる
2. `lib/` から以下 4 つを
   `Packages/com.github.asus4.onnxruntime/Plugins/Windows/x64/` にコピー:
   - `onnxruntime.dll`(**GPU ビルド本体** — asus4 配布の CPU 版とは別物)
   - `onnxruntime_providers_shared.dll`
   - `onnxruntime_providers_cuda.dll`
   - `onnxruntime_providers_tensorrt.dll`
3. `.meta` は git 管理下。バイナリ欠落状態で Unity を開くと .meta が削除されるので、
   その場合は `git restore Packages/com.github.asus4.onnxruntime/Plugins/` で戻す

## cu12 → cu13 移行時の必須検証

ビルドフレーバーが変わると腕の境界フレームで骨格が変わりうる(Phase 3 追補 2 で実証)。

1. オフライン A/B 1 テイク: `FusedBodiesExport` → `LiveV11sVerify.CompareTakes` で
   現行 v11s リファレンスとの差を計測
2. ズレていたら v11s リファレンスを cu13 バックエンドで再生成
   (`FusedBatchConvert`、29 take ≈ 2h 自動)。**再生成はライブ本番と同じ 5080 機で行う**
   (同一バイナリでも GPU 世代でカーネル選択が変わりうるため)

## 本番機での注意(Phase 4)

- スタンドアロンビルドは PATH に依存させず、CUDA/cuDNN DLL を Assets/Plugins 配置にする
  (手順は Phase 2 で文書化予定)
