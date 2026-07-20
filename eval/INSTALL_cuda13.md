# CUDA 13 + cuDNN 9 (cu13) — ORT cu13 フレーバー移行記録

対象: 展示 2 台(開発機 RTX 4070 / 本番機 RTX 5080)。**2 台とも同一構成**にする。
移行理由: cu12 版 ORT の provider DLL は sm_120(Blackwell)カーネル非搭載で
RTX 5080 では CUDA EP が立たない(`cudaErrorNoKernelImageForDevice`)。
cu13 版は sm_75〜sm_120a を搭載し両世代をカバーする。

> ドライバ更新不要(591.86 で CUDA 13.1 対応)。CUDA Toolkit のインストールも
> **不要** — 必要な DLL を NVIDIA 公式 redist から取得して plugin フォルダに
> 直接配置する方式(toolkit / PATH に依存しない。ビルド配布もこれで解決)。

## 配置先

`Packages/com.github.asus4.onnxruntime/Plugins/Windows/x64/`
(DLL は .gitignore 済み — **各マシンで手動配置が必要**。.meta のみ git 管理)

## 1. ONNX Runtime 1.26.0 cu13 フレーバー

- 取得(GitHub 公式リリースのみ):
  https://github.com/microsoft/onnxruntime/releases/download/v1.26.0/onnxruntime-win-x64-gpu_cuda13-1.26.0.zip
- SHA-256: `4FA096030EE766B2E590D71FB6676BBD00595C92AB87ACF497FE075E98834D8B`
- `lib/` から 4 点を配置: `onnxruntime.dll`, `onnxruntime_providers_cuda.dll`,
  `onnxruntime_providers_shared.dll`, `onnxruntime_providers_tensorrt.dll`
- managed 側(asus4 0.4.8 の C# バインディング)は変更不要(同じ ORT 1.26.0)

## 2. CUDA 13.3.1 コンポーネント(NVIDIA 公式 redist)

`https://developer.download.nvidia.com/compute/cuda/redist/` から。
版とハッシュの根拠は `redistrib_13.3.1.json`(公式マニフェスト):

| コンポーネント | 版 | 配置 DLL | zip SHA-256 |
|---|---|---|---|
| cuda_cudart | 13.3.29 | cudart64_13.dll | `1feb7dd266813ffe8dbc24e115183a5ac35a4795c8d34aca0df85ab616b64d9c` |
| libcublas | 13.6.0.2 | cublas64_13.dll, cublasLt64_13.dll | `62e9fa30560c8f0a28e0cdcf9d6fc1fed347bcfab8847239b9ae1fdc1d86408a` |
| libcufft | 12.3.0.29 | cufft64_12.dll | `83df908ae67e2b3a86201de8463562ab49dd9ee8b3b5efc3fdc2e681b14b5dd9` |
| cuda_nvrtc | 13.3.33 | nvrtc64_130_0.dll, nvrtc-builtins64_133.dll | `8519f678588610bf380ccaac130729aa1a624c407183e7ad9c319c19ecc63d2f` |

(zip 内は `bin\x64\` 配下にある)

## 3. cuDNN 9.24.0.43 for CUDA 13

- 取得: `https://developer.download.nvidia.com/compute/cudnn/redist/cudnn/windows-x86_64/cudnn-windows-x86_64-9.24.0.43_cuda13-archive.zip`
- SHA-256(公式 `redistrib_9.24.0.json` 記載): `88f72bd1ce384197cedbc68496c6052d7ff0bd9fd0b3c74470402cf737507e06`
- `bin\` の DLL 10 点(cudnn64_9.dll + cudnn_*64_9.dll)を全部配置(計 ~542MB)
- ⚠ **必ず `_cuda13` アーカイブを使うこと**。cu12 用の同版 cuDNN は
  ファイル名もファイルバージョン(9.24.0.43)も同一で中身だけ違う

## 4. 同名 DLL 取り違えガード(コード)

このマシンのように PATH 上に CUDA 12.6 の bin(cu12 版 cuDNN 同居)が残っている場合、
bare-name の DLL 解決が PATH 側を拾い**ビルドフレーバーが黙って混ざる**恐れがある。
対策として `Assets/Scripts/Eval/Rtmpose/CudaDllPreload.cs` が ORT 初期化前に
plugin フォルダの全 CUDA/cuDNN DLL を絶対パスで先行ロードして名前解決を固定する
(`OrtRtmposeBackend` ctor から自動で呼ばれる。ログ: `[rtmpose] CUDA preload ...`)。

## 5. 検証結果(開発機 4070、2026-07-19)

- スモーク: `ActiveProvider=Cuda`(要求どおり)、pose 3ms/回(warm)、
  ロード済みモジュール全てが plugin フォルダの cu13 版であることを実パスで確認
- 数値 A/B: take 15-50-24 全 1816 フレームを FusedBodiesExport で再出力し
  `LiveV11sVerify.CompareTakes` で v11s アーカイブと比較
  → **cu12 時代のレポート(offline_ab_20260717_140621.md)と全指標が完全一致**
  (offline_ab_20260719_171902.md)。cu13 移行による数値変化はゼロ、
  v11s リファレンス再生成は不要

## 6. 本番機(5080)チェックリスト

- [ ] 同じ DLL セットを同じ場所に配置(このファイルのハッシュと照合)
- [ ] `ActiveProvider=Cuda` ログ確認 + `[rtmpose] CUDA preload` の loaded/failed 数確認
- [ ] FreshFusedHz ベンチ(Phase 3 と同条件)
