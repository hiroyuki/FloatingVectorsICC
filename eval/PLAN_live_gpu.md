# ライブ融合 30Hz 化プラン — CUDA/TensorRT EP 導入

目的: LiveFusedBodySource の融合レートを DirectML の 6〜20Hz から **安定 30Hz** に上げ、
ライブの骨格品質を録画 v11s と同等にする(ユーザー判定 2026-07-17:「DirectML ではだめ」)。

## 現状インベントリ(2026-07-17 実測)

| 項目 | 値 |
|---|---|
| GPU | **RTX 4070 12GB**(能力は十分) |
| NVIDIA ドライバ | 591.86 — CUDA 12.x ランタイム対応済み。**更新不要** |
| CUDA Toolkit | 11.6 のみ(12.x が必要) |
| cuDNN | 8(9 が必要) |
| ONNX Runtime | asus4 Unity パッケージ 0.4.8(core / .unity)。DirectML で稼働中 |
| GPU パッケージ | `com.github.asus4.onnxruntime.win-x64-gpu` npm に存在(最新 0.4.5) |

目標値の目安: 4 カメラ × 30Hz = 33ms 毎に 4 推論 → **1 カメラあたり track ≤ 6〜8ms**
(YOLOX detect は detect-once-then-track で償却)。RTX 4070 + CUDA/TensorRT なら妥当圏。

## Phase 0 — バージョン確定 【完了 2026-07-17】

npm レジストリ・GitHub リリースノート・PyPI ピン・パッケージ tarball の実測で確定:

| 項目 | 確定値 | 根拠 |
|---|---|---|
| asus4 パッケージ版 | **全部 0.4.5**(core/unity は 0.4.8→0.4.5 に**ダウングレード**) | win-x64-gpu の npm 最新が 0.4.5。0.4.6+ の gpu 版は未公開 |
| 同梱 ORT 本体 | **1.25.1** | asus4 v0.4.5 リリースノート(PR #75) |
| CUDA Toolkit | **12.x(12.6 を既定)** | ORT 1.25.0 で CUDA 11.x サポート打ち切り。PyPI ピン `nvidia-cuda-runtime-cu12~=12.0` |
| cuDNN | **9.x for CUDA 12** | PyPI ピン `nvidia-cudnn-cu12~=9.0` |
| TensorRT | **Phase 3 で未達時のみ**。版は導入時に ORT 1.25 の対応を再確認(公式表は ORT 1.22/TRT 10.9 止まり) | 公式ドキュメント未更新のため导入時に確定 |
| gpu 0.4.5 tarball 中身 | `Plugins/Windows/x64/` に `onnxruntime_providers_cuda.dll` / `onnxruntime_providers_shared.dll` / `onnxruntime_providers_tensorrt.dll`(計 ~180MB) | tarball 実測 |
| vendored C# API | `AppendExecutionProvider_CUDA(int / OrtCUDAProviderOptions)` と `AppendExecutionProvider_Tensorrt(int / OrtTensorRTProviderOptions)` が存在 | `Library/PackageCache/com.github.asus4.onnxruntime@*/Runtime/SessionOptions.shared.cs:249,263,304,318` |

現行コード監査結果: `OrtRtmposeBackend` は ctor の `useDirectML` bool で
`AppendExecutionProvider_DML(0)` 固定、失敗時は **warning ログのみで CPU に暗黙フォールバック**
(`OrtRtmposeBackend.cs:51-52`)。SessionOptions は YOLOX/RTMPose の 2 セッションで 1 個を共有。
プロバイダ enum + フォールバック連鎖はこの ctor に挿す。

**成果物**: `eval/INSTALL_cuda12.md`(ダウンロードリンク・チェックサム記録欄付きインストール指示書)

## Phase 1 — インストール(人手、~1 時間 + ダウンロード時間)

> ドライバ更新は不要。CUDA 11.6 は残したままサイドバイサイドで入れる(既存ツールに影響なし)

1. **CUDA Toolkit 12.6**(NVIDIA 公式サイトからのみ取得)
   - カスタムインストールでランタイム/開発ライブラリのみで可(Nsight 等は不要)
   - インストーラが `bin` を PATH に追加する(エディタ実行はこれで足りる)
2. **cuDNN 9.x for CUDA 12**(NVIDIA 公式からのみ。zip 展開 → bin の DLL 群を PATH に通すか、
   後述の Unity プラグイン配置へコピー)
   - **zip の SHA-256 を取得元ページの値と照合し、`eval/INSTALL_cuda12.md` に記録**
     (YamlDotNet の CHECKSUMS.txt と同じ運用)
3. (Phase 3 で必要になった場合のみ)**TensorRT 10.x** zip — 版は導入時に ORT 1.25 対応を確定
4. 確認: `nvidia-smi` ✓ / `where cudart64_12.dll` / `where cudnn64_9.dll`

## Phase 2 — Unity 統合(AI、~1〜2 時間)

1. manifest 変更: `com.github.asus4.onnxruntime.win-x64-gpu@0.4.5` を追加し、
   core / unity を **0.4.5 に揃える**(混在版は起動時クラッシュの最頻出原因)
2. `OrtRtmposeBackend` にプロバイダ選択を実装:
   - `enum Provider { Cpu, DirectML, Cuda, TensorRt }`。**フォールバック連鎖は
     「要求プロバイダから下位方向のみ」**:
     - 要求 TensorRt → TensorRT→CUDA→DirectML→CPU
     - 要求 Cuda → CUDA→DirectML→CPU(TensorRT は要求時以外試さない)
     - 要求 DirectML → DirectML→CPU(現行挙動と同じ)
     - CPU は正当な終端状態として enum に含める
   - **ActiveProvider の確定は「YOLOX と RTMPose の両 InferenceSession が
     そのプロバイダで生成に成功した後」**。AppendExecutionProvider の成功だけでは
     不十分(依存 DLL 欠落はセッション生成時に露見する)。片方だけ成功して
     もう片方が失敗したら、**成功した方も dispose してから**次のプロバイダで
     両方作り直す(EP 混在セッションを作らない)
   - ctor 完了時に `[rtmpose] EP=<actual> (requested=<requested>)` を必ずログ、
     **要求 EP と実効 EP が違う場合は Debug.LogError**(黙った CPU フォールバックで
     ベンチが無効になる事故の防止 — 現行 DirectML 経路の warning-only を格上げ)
   - LiveFusedBodySource / FusedBatchConvert の両方から指定可能に(Inspector 露出)
3. DLL 解決: エディタは CUDA インストーラの PATH でよいが、**スタンドアロンビルド用に
   CUDA/cuDNN DLL を Assets/Plugins 配置にする手順**も書き残す(本番はビルドで動く想定のため)
4. コンパイル確認 + スモーク:
   - Backend.Ready == true かつ **ActiveProvider == Cuda** で 1 フレーム推論成功
   - **Playmode enter/exit を 3 回繰り返して**エディタがクラッシュ・ハングしないこと
     (CUDA EP はネイティブ挙動が変わるので、LiveFusedBodySource の
     worker abandon 経路〔5 秒ハング時のセッション意図的リーク〕が CUDA でも安全か確認)

## Phase 2 実施記録(2026-07-17)

実装中に判明した事実と、それに伴う構成変更:

1. **ORT 1.25 はレガシー C エクスポート
   (`OrtSessionOptionsAppendExecutionProvider_CUDA/_DML/_Tensorrt`)を廃止**。
   vendored C# の int 版オーバーロードは全滅する。DML は generic
   `AppendExecutionProvider("DML")`、CUDA/TensorRT は V2 options API
   (OrtApi 関数テーブル経由)を使う
2. **asus4 core パッケージの Windows x64 onnxruntime.dll は Microsoft の
   CPU 専用 NuGet 由来**(`download-onnx-libs.sh` 実測)。CUDA ブリッジも DML も
   コンパイルされていない。win-x64-gpu パッケージは provider DLL だけで
   本体 dll を含まないため、**そのままでは CUDA は動かない(パッケージングの罠)**
3. **⚠ これまでの「DirectML 6-20Hz」は実際には CPU 実行だった**。0.4.8 も同じ
   CPU-only 構成で、旧コードは DML append 失敗を warning だけ出して CPU に落ちていた
   (まさに今回のプランで LogError に格上げした事故そのもの)
4. 対処: core パッケージを **embed**(`Packages/com.github.asus4.onnxruntime/`)し、
   `Plugins/Windows/x64/onnxruntime.dll` を **Microsoft.ML.OnnxRuntime.Gpu.Windows
   1.25.1 NuGet の GPU ビルドに差し替え**、provider DLL(cuda/tensorrt/shared)も
   同じフォルダに同居。ORT は本体 dll と同じディレクトリから provider を探すので、
   エディタ/ビルド両方で自己解決する。win-x64-gpu パッケージは manifest から削除
   (重複 plugin 回避)
5. 注意: この構成では **DirectML は使えない**(GPU ビルドに DML は入っていない)。
   フォールバック連鎖は実質 Cuda→CPU

## Phase 3 — ベンチマークと判定(AI、~30 分)

1. **live-sim ベンチ**(エディタをフォーカスした状態で統一): オリジナル 15-50-24 を再生し
   LiveFusedBodySource ON で計測。記録項目:
   - **FreshFusedHz(判定指標)= held frame を除いた新規融合レート**。
     現行 `FusedHz` は Heartbeat の temporal-hold フレーム
     (`FusedRtmposeAdapter.Heartbeat` → `EmitHeldFrame`)込みのカデンツなので、
     **推論が遅くても 30Hz に見える**。fresh / held を別カウントし、
     held 込みの FusedHz は参考値に格下げする
   - **1 推論あたりの ms: mean と p95**(track / detect 別)— ウォームアップ
     (最初の ~100 推論)は集計から除外
   - **内訳カウンタ**: detect 呼び出し数 / track(pose)呼び出し数 /
     fresh 融合 emit 数 / held emit 数 / no-detect スキップ数 / キュー drop 数。
     実装ポイントは `RtmPoseAdapter.SubmitFrame`(detect/track 分岐)と
     `FusedRtmposeAdapter` の emit 経路(fresh=`PushLagAndEmit`, held=`EmitHeldFrame`)。
     レイテンシ数値だけでは融合レートが動いた理由が説明できないため
   - ActiveProvider == Cuda であることをログで確認してから計測開始
2. 判定ゲート:
   - **CUDA EP で FreshFusedHz ≥ 30(ベンチ区間中 held emit ≈ 0)→ ここで完了**
     (TensorRT は複雑さに見合わないので入れない)
   - 30 未満 → TensorRT EP を有効化して再測。TensorRT は初回にエンジンビルド
     (モデル×入力形状ごとに数分)が走るため、**エンジンキャッシュをディスクに保存**する
     設定(`trt_engine_cache_enable` + `trt_engine_cache_path`)を必ず入れる
     (起動毎ビルドは運用不可)
3. 30Hz 到達後にユーザーの目視判定: 30Hz では median-5(66ms)等のフィルタが設計どおりに
   戻るので、録画 v11s との見た目差を最終確認

## Phase 3 実施記録(2026-07-17)

live-sim(15-50-24-rtmfused 再生、4 カメラ 26.5fps)での段階的結果:

| 構成 | FreshFusedHz | ボトルネック |
|---|---|---|
| CUDA EP 化直後(逐次) | 14.8 | CPU 前処理(BuildInput 7.6ms/frame)|
| + BuildInput 行並列化 | 22.5 | pose GPU 呼び出し 6.3ms × 4 逐次 |
| + バースト並列化(カメラ別 adapter + 並列 Run) | **27.5(録画レート上限)** | 解消。held ≈ 0.5/s、ドロップなし |

- 実装: `FusedRtmposeAdapter` をカメラ別 `RtmPoseAdapter` に分割し `SubmitBurst` で
  並列推論(融合状態は worker スレッドのみが触る、逐次 semantics 保存)。
  backend は Pose 並列安全化(ローカル入力配列)+ Detect は `_detectLock` で直列化
- 並列時の pose 単発は ~10ms に伸びる(GPU 競合)が重なって走るので
  バースト wall-time ≈ 12ms → **実カメラ 30fps(33ms 周期)に余裕**
- YOLOX letterbox も行並列化済み(detect 39→ 実測 40ms 前後、頻度低)
- **30Hz ゲートの最終判定は実カメラ(30fps)接続時**。live-sim は録画が 26.5fps 上限
- 計測カウンタ: FreshFusedHz / HeldEmitHz(LiveFusedBodySource)、
  per-stage ms リング(RtmPoseAdapter.LiftMs/DetectMs/PreMs/PoseMs)

## Phase 4 — 本番機への展開(実機日)

- 本番リグの PC にも Phase 1 と同じインストール(GPU 型番を先に確認 — 4070 未満なら
  Phase 3 のベンチを本番機でやり直して判定)
- スタンドアロンビルドでの動作スモーク(Plugins 配置 DLL で CUDA EP が立つこと)
- LiveFusedBodySource の実カメラ試験と同日にまとめて:
  trigger2ImageDelayUs スタガ / GetSyncConfig 確認 / カラー露光アップ / 三脚移動

## リスクと対策

| リスク | 対策 |
|---|---|
| ORT×CUDA×cuDNN のバージョン不整合(最頻出) | Phase 0 で完全にピン止め済み(ORT 1.25.1 / CUDA 12.x / cuDNN 9)。混ぜない |
| GPU パッケージ(0.4.5)とコア(0.4.8)の版ずれ | 全パッケージを 0.4.5 に統一(コアのダウングレード) |
| **黙った CPU フォールバックでベンチ無効** | ActiveProvider 公開 + 要求と実効の不一致は LogError。ベンチは ActiveProvider 確認後に開始 |
| **held frame で FusedHz が水増しされベンチ合格に見える** | 判定指標を FreshFusedHz(held 除外)に変更。fresh/held を別カウント |
| TensorRT 初回エンジンビルドの分単位の待ち | エンジンキャッシュ必須。キャッシュディレクトリをプロジェクト外に固定 |
| エディタは PATH で動くがビルドで DLL 欠落 | Plugins 配置手順を Phase 2 で文書化、ビルドスモークを Phase 4 に含める |
| ネイティブ DLL のサプライチェーン | CUDA/cuDNN/TensorRT は NVIDIA 公式のみから取得、cuDNN/TRT zip は SHA-256 を記録。asus4 gpu パッケージは npm attestation 付き(tarball に .attestation.p7m を確認済み) |
| CUDA EP でのハング/クラッシュ挙動変化 | Phase 2 スモークに playmode enter/exit ×3 を含める。worker abandon 経路は既存設計を流用 |
| それでも 30Hz 未達 | rtmpose-s / yolox-tiny への差し替え(同じ v11s 比較手順で精度を定量評価) |

## やらないこと

- DirectML のさらなるチューニング(天井が見えた)
- 低レート前提のフィルタ再設計(30Hz 到達が正道。ただし medianLagFilter の
  Inspector 露出だけは小さいのでやる — 現状はリフレクションでしか切れない)
- モデルファイル探索の hash 検証(`FirstOnnx()` の緩い探索は既知だが、eval 段階では
  ログに実パスを出す現状運用で足りる。本番ビルド時にファイル名ピン止めを検討)
- 複数 GPU / device_id 選択(本番機も単一 GPU 前提)
