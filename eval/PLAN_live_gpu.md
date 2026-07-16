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
| GPU パッケージ | `com.github.asus4.onnxruntime.win-x64-gpu` npm に存在(確認時点の最新 0.4.5) |

目標値の目安: 4 カメラ × 30Hz = 33ms 毎に 4 推論 → **1 カメラあたり track ≤ 6〜8ms**
(YOLOX detect は detect-once-then-track で償却)。RTX 4070 + CUDA/TensorRT なら妥当圏。

## Phase 0 — バージョン確定(AI、~30 分、インストール前に完了させる)

1. asus4 パッケージのバージョン整合を確定:
   - `win-x64-gpu` の全バージョンと、対応する ORT 本体バージョンを CHANGELOG で確認
   - core/unity/gpu を**同一バージョンに揃える**(例: 全部 0.4.5、または gpu に 0.4.8 が
     あればそのまま)。ORT 本体バージョンが下の CUDA/cuDNN ピンを決める
2. ORT 公式の要件表で正確にピン止め(想定: ORT 1.20 系 → **CUDA 12.x + cuDNN 9.x**、
   TensorRT EP は **TensorRT 10.4/10.5**)
3. OrtRtmposeBackend の現行コード監査: DirectML 固定箇所、SessionOptions の作り、
   プロバイダ切替の挿入点を特定

**成果物**: 「これをダウンロードしてください」リンク付きの正確なインストール指示書

## Phase 1 — インストール(人手、~1 時間 + ダウンロード時間)

> ドライバ更新は不要。CUDA 11.6 は残したままサイドバイサイドで入れる(既存ツールに影響なし)

1. **CUDA Toolkit 12.x**(Phase 0 でピン止めした版。既定: 12.6)
   - カスタムインストールでランタイム/開発ライブラリのみで可(Nsight 等は不要)
2. **cuDNN 9.x for CUDA 12**(zip 展開 → bin の DLL 群を PATH に通すか、後述の
   Unity プラグイン配置へコピー)
3. (Phase 3 で必要になった場合のみ)**TensorRT 10.x** zip
4. 確認: `nvidia-smi` ✓ / `where cudart64_12.dll` / `where cudnn64_9.dll`

## Phase 2 — Unity 統合(AI、~1〜2 時間)

1. manifest に `com.github.asus4.onnxruntime.win-x64-gpu` を追加(バージョンは Phase 0 の決定)
2. `OrtRtmposeBackend` にプロバイダ選択を実装:
   - `enum Provider { DirectML, Cuda, TensorRt }` + **フォールバック連鎖**
     (TensorRT→CUDA→DirectML の順に AppendExecutionProvider を試し、失敗はログして次へ)
   - LiveFusedBodySource / FusedBatchConvert の両方から指定可能に
3. DLL 解決: エディタは PATH でよいが、**スタンドアロンビルド用に CUDA/cuDNN DLL を
   Assets/Plugins 配置にする手順**も書き残す(本番はビルドで動く想定のため)
4. コンパイル確認 + スモーク(Backend.Ready が CUDA EP で true、1 フレーム推論成功)

## Phase 3 — ベンチマークと判定(AI、~30 分)

1. **live-sim ベンチ**(エディタをフォーカスした状態で統一): オリジナル 15-50-24 を再生し
   LiveFusedBodySource ON、FusedHz と 1 推論あたりの ms を計測
2. 判定ゲート:
   - **CUDA EP で FusedHz ≥ 30 → ここで完了**(TensorRT は複雑さに見合わないので入れない)
   - 30 未満 → TensorRT EP を有効化して再測。TensorRT は初回にエンジンビルド
     (モデル×入力形状ごとに数分)が走るため、**エンジンキャッシュをディスクに保存**する
     設定を必ず入れる(起動毎ビルドは運用不可)
3. 30Hz 到達後にユーザーの目視判定: 30Hz では median-5(66ms)等のフィルタが設計どおりに
   戻るので、録画 v11s との見た目差を最終確認

## Phase 4 — 本番機への展開(実機日)

- 本番リグの PC にも Phase 1 と同じインストール(GPU 型番を先に確認 — 4070 未満なら
  Phase 3 のベンチを本番機でやり直して判定)
- LiveFusedBodySource の実カメラ試験と同日にまとめて:
  trigger2ImageDelayUs スタガ / GetSyncConfig 確認 / カラー露光アップ / 三脚移動

## リスクと対策

| リスク | 対策 |
|---|---|
| ORT×CUDA×cuDNN のバージョン不整合(最頻出) | Phase 0 で完全にピン止めしてからダウンロード。混ぜない |
| GPU パッケージ(0.4.5)とコア(0.4.8)の版ずれ | 全パッケージを同一版に揃える(コアのダウングレード許容) |
| TensorRT 初回エンジンビルドの分単位の待ち | エンジンキャッシュ必須。キャッシュディレクトリをプロジェクト外に固定 |
| エディタは PATH で動くがビルドで DLL 欠落 | Plugins 配置手順を Phase 2 で文書化、ビルドスモークを Phase 4 に含める |
| それでも 30Hz 未達 | rtmpose-s / yolox-tiny への差し替え(同じ v11s 比較手順で精度を定量評価) |

## やらないこと

- DirectML のさらなるチューニング(天井が見えた)
- 低レート前提のフィルタ再設計(30Hz 到達が正道。ただし medianLagFilter の
  Inspector 露出だけは小さいのでやる — 現状はリフレクションでしか切れない)
