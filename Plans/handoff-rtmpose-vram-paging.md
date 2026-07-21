# 引き継ぎ: ライブ融合が VRAM を埋めてページングする

作成 2026-07-21。**この件だけを単体で調べるための資料**です。
元の「骨格が1本も出ない」件（`handoff-rtmpose-no-detection.md`）は**解決済み** — 原因は
capture volume の座標系で、`RigCaptureVolume` で rig から導出するようにして直っています。
本書はその過程で表に出た**別問題**です。

## 症状

`LiveFusedBodySource` を有効にして**人が立つと**、ORT の CUDA セッション2本
（yolox-m + rtmpose-m）が VRAM を約 7.4 GB 占有し、空きが数百 MB まで落ちる。
そこから WDDM がページングを始め、描画も推論も同時に失速する。

- エディタ全体が 27fps → 5〜12fps
- YOLOX detect が 40ms → 270〜600ms、RTMPose pose が 4.6ms → 55〜200ms
- 目標は FusedHz 30、実測 1.8〜9.5

**人が居ない間は起きない**（pose が走らないため）。検出が始まった瞬間に埋まる。

## 実測（2026-07-21、5080機、ライブ4カメラ、クリーンな Play セッション）

すべて `nvidia-smi --query-gpu=memory.used,memory.free`。

| 条件 | VRAM 使用 | 空き | FPS | FusedHz |
|---|---|---|---|---|
| LFBS OFF・人あり | 8.0 GB | 7.9 GB | 26.2 | — |
| LFBS ON・アリーナ修正前 | 15.3 GB | 0.7 GB | 1.7 | 0 |
| LFBS ON・アリーナ修正後 | 15.4 GB | 0.4 GB | **12.4** | 9.5 |
| 同上 + SerializePose | 15.4 GB | 0.6 GB | 5.7 | 1.8 |
| Play 停止時 | 2.5 GB | 13.5 GB | — | — |

**「LFBS OFF・人あり = 8.0GB」が基準線**。TSDF や点群ではなく ORT の分が 7.4GB。

単体ベンチ（Play せず、GPU アイドル、1セッション、逐次実行）では
**detect 40.4ms / pose 4.6ms、VRAM は +811MB のみ**。つまり単体では健全で、
ライブの4カメラ構成でだけ 7.4GB まで膨らむ。

## 済んでいる対処（コミット済み・維持すること）

`OrtRtmposeBackend` の CUDA provider options に2つ追加。1.7fps → 12.4fps の改善はこれ。

```
arena_extend_strategy        = kSameAsRequested   （既定 kNextPowerOfTwo が倍々に予約する）
cudnn_conv_use_max_workspace = 0
```

どちらも数値出力を変えない。`use_tf32=0` / `cudnn_conv_algo_search=HEURISTIC` は
オフライン A/B で精度のために固定した値なので**触らないこと**。

## 切り分け済み（＝もう疑わなくていい）

| 項目 | 結果 |
|---|---|
| TSDF / 点群が犯人か | **違う**。人が立った状態で LFBS だけ切ると 8.0GB / 26fps |
| k4abt worker 4本の残留 | **違う**。停止しても FPS 変わらず（ただし別途「無駄」ではある、下記） |
| `SerializePose`（Pose 同時実行がピークを4倍にしている説） | **違う**。VRAM 不変、スループットは悪化。既定 false に戻した |
| `gpu_mem_limit` によるハードキャップ | **使ってはいけない**。2GB で試すと足りない時に停まらず**例外を投げる**。4カメラ同時で全 detect が死ぬ（`BFCArena::AllocateRawInternal Available memory of 2971904 is smaller than requested bytes of 4915200`） |
| モデル / EP / cu13 | 健全。`ActiveProvider=Cuda`、単体 40ms/4.6ms |

## 計測時の重大な注意

**`LiveFusedBodySource` を on/off トグルすると、それ以降の VRAM 値は信用できない。**
ORT の CUDA アリーナはプロセス単位で、セッションを Dispose してもドライバに返らない。
トグルのたびに高水位マークだけが積み上がる（Play 停止＝ドメインリロードで初めて解放）。

比較計測は**毎回 Play を入れ直して1条件だけ測る**こと。今回これで一度、汚染された数値を
掴んで無駄足を踏んだ。

## 【解決】原因 = ORT CUDA EP の per-thread context（2026-07-21）

**確定・修正済み。** 以下は経緯と数字。

ORT の CUDA EP は **Run を呼んだスレッドごとに context を作って保持し、解放しない**
（cuBLAS/cuDNN ハンドル + 作業領域）。この実装は推論を .NET スレッドプールから
呼んでいたため、時間とともに「ORT に触ったことのあるスレッド」が増え、
device arena が無制限に伸びていた。

### オフライン再現（Play なし・カメラなし・TSDF なし、9秒で再現）

同一テイク・同一セッション・360 バースト（4カメラ）:

| バースト | スレッドプール経路 | 単一スレッド対照 | 修正後（専用スレッド） |
|---|---|---|---|
| 100 | 8348 MiB | 3198 MiB | 4153 MiB |
| 200 | 9220 MiB | 3198 MiB | 4153 MiB |
| 300 | 10692 MiB | 3198 MiB | 4153 MiB |
| 360 | **11450 MiB** | **3198 MiB(平坦)** | **4153 MiB(平坦)** |
| 所要 | 9s / emitted 340 | 15s / emitted 357 | 10s / emitted 348 |

- セッション構築だけなら **+310MB**。膨らむのは推論を回している間だけ
- `Dispose` で完全に返る（ドライバのリークではない）
- 単一スレッド対照が平坦 = モデルでも同時実行数でも TSDF でもなく
  **スレッドの種類数**が原因、と確定

### 修正内容

スレッドプールをやめ、**長命の専用スレッドに固定**（並列性は維持）:

- `FusedRtmposeAdapter.SubmitBurst` の `Parallel.For` → カメラごとの常駐
  `BurstWorker` スレッド（`RunGroupOnDedicatedThreads`）。バリア意味論は同じ
- `RtmPoseAdapter.KickAsyncDetect` の `Task.Run` → アダプタごとの常駐 detect
  スレッド（`DetectLoop`）
- 退役時にスレッドを止める: `FusedRtmposeAdapter.Dispose()` が burst worker と
  inner の `StopDetectThread()` を呼ぶ（**共有バックエンドは落とさない**）。
  `LiveFusedBodySource.RebuildAdapter`（playback ループ）と OnDisable の
  クリーン終了経路が、旧アダプタを Dispose するようになった

ORT に触るスレッドは pose 4 + detect 4 = **8本に固定**。

### 注意

`ThreadPool.SetMaxThreads(4,4)` でプールを絞る案は**使えない** —
ProcessorCount を下回る値は設定を拒否される（`false` が返り、無言で無効化される）。

### 残る問題

VRAM は解決したが、**フルシーンでの FPS はまだ実測していない**。ページングが
無くなって初めて「計算上の天井」が測れる。下の「30Hz は達成済みは誤り」節も参照。

## 旧・本命仮説（上で確定したもの。記録として残す）

ORT の CUDA EP は **Run を呼んだスレッドごとに context を作って保持する**（cuBLAS/cuDNN
ハンドルと作業領域を持ち、スレッド単位でキャッシュされる）。

この実装は推論を **.NET スレッドプール**から呼んでいる:

- `RtmPoseAdapter.KickAsyncDetect` → `Task.Run(...)`（RtmPoseAdapter.cs:106 付近）
- `FusedRtmposeAdapter.SubmitBurst` → `Parallel.For(...)`（FusedRtmposeAdapter.cs:363 付近）

プールのスレッドは動的に増減するので、時間とともに「ORT に触ったことのある異なる
スレッド」が増え続け、context が積み上がる、という筋。観測と全部整合する:

| 観測 | この仮説での説明 |
|---|---|
| 単体ベンチは 811MB | メインスレッド1本しか Run を呼ばない |
| ライブだけ 7.4GB | プール由来の多数のスレッドが Run を呼ぶ |
| 時間とともに増える(10.9 → 14.3 → 15.4GB) | 新しいスレッドが触るたびに増える |
| `SerializePose` が効かない | 直列化してもスレッドはプール由来のまま |
| アリーナ設定は効くが根治しない | context 1個あたりが小さくなるだけ |

**対処の方向**: ORT を呼ぶスレッドを固定する。カメラごとに長命の専用スレッド4本
（またはカメラ単位のシリアルキュー）にして `Task.Run` / `Parallel.For` をやめる。
融合ワーカーは既に専用スレッドなので構造的には素直に入る。

**検証手順（Play もカメラも不要）**: 録画テイクに対してオフラインで4カメラ融合を
回しながら `nvidia-smi` を見る。VRAM が GB 単位に伸びれば再現成立で、TSDF も描画も
無関係と確定する。再現したらスレッドを固定して同じ計測をもう一度。

## まだ調べていない筋

1. **7.4GB の内訳そのもの** — 単体 811MB との差が 9倍。ORT のスレッド/ストリーム単位の
   アロケータか、4つの `RtmPoseAdapter` が同一バックエンドを共有する構成に起因するのか。
   ORT の `arena_extend_strategy` 以外のメモリ系オプション、あるいは
   `SessionOptions.EnableMemoryPattern` / `AddSessionConfigEntry` 系を洗う
2. **IOBinding で入出力を事前確保したデバイスバッファに固定する** — 毎回
   `OrtValue.CreateTensorValueFromMemory` で作り直しているのをやめる
3. **セッションを2本持つ必要があるか** — yolox と rtmpose で別 `InferenceSession` だが
   `SessionOptions` は共有。別々にして片方だけ絞れるか
4. **fp16 モデル** — 活性値が半分になる。精度への影響は既存のオフライン A/B 基盤
   （`LiveV11sVerify.CompareTakes`）で測れる
5. **カメラ4台すべてで pose を回す必要があるか** — 融合は2台の合意があれば成立する
   （`FusedRtmposeAdapter` の cluster は `bestCount >= 2` で strong）。
   負荷に応じて pose を回すカメラを間引く案

## 「30Hz は達成済み」は誤り — 目標値の再設定が要る

調べた結果、**実カメラ + フルシーンで 30Hz が出たことは一度もない**。

- 実測の最高値は **27.5 FreshFusedHz**（`eval/PLAN_live_gpu.md:135-153`）。条件は
  **4070**・**録画再生（26.5fps 上限）**・**TSDF も体験フローも無い隔離ベンチ**
- 同文書 151 行に「**30Hz ゲートの最終判定は実カメラ(30fps)接続時**」とあり、
  その判定は未実施（`eval/INSTALL_cuda13.md:70` のチェックボックスが未チェック）
- 5080 での FreshFusedHz ベンチは**存在しない**（スモークテストと精度 A/B のみ）
- `Plans/visitor-sequence-plan.md:42` と `Docs/onsite-checklist.md:14` の
  「30Hz 実測済」は、コミット `ae8d2b0` が "User-clarified architecture" として
  書き足したもので、ベンチ結果もログも紐づいていない。**この2箇所は訂正すべき**
- 注意: 素の `FusedHz` は Heartbeat の held フレームを含むので、遅い EP でも
  30Hz に見える（`FusedRtmposeAdapter.cs:252-253` が明記）。**必ず `FreshFusedHz`
  を見ること**。過去の「30Hz 実測」もこれを見誤った可能性がある

したがって VRAM を直しても 30Hz に届く保証は無い。**TSDF だけで約 25ms/フレーム**
（`Plans/realtime-performance-tuning.md:4`）で 33ms 予算のほとんどを推論前に消費する。
VRAM 問題とは独立した構造的コストなので、目標フレームレートは実測ベースで
引き直す必要がある。

## 4070 セットについて

**未計測。** 持ち越せるのは「ORT 分 = 7.4GB」だけで、基準線の 8.0GB は
計測機（5080機）固有 — Chrome / Slack / Discord / VS Code / Edge が GPU に乗っている分を含む。
4070 は VRAM 12GB で 4GB 少ないので、より厳しい可能性が高い。

展示機で2分で分かる:
1. Play 前に `nvidia-smi --query-gpu=memory.used,memory.free --format=csv`
2. LFBS 有効・人が立った状態でもう一度
3. 空きが 1GB を切っていなければ問題なし

cu13 は両セット導入済み（2026-07-21 ユーザー確認）。EP フォールバックの心配は無い。

## ついでに見つけた別件（未修正）

`SkeletonMerger.useExternalBodies` が true になっても、**すでに spawn 済みの
k4abt worker プロセスは停止しない**。給餌と出力は止まる（`DispatchRawFrame` /
`OnWorkerSkeletons` の早期 return）が、プロセスは生き続けて k4abt の DNN 分の
VRAM を握ったままになる。FPS への影響は測定上見えなかったが、VRAM が逼迫している
本件の文脈では純粋な無駄。`SkeletonMerger` の 683 / 711 行付近に既にある
`workerHost.StopWorker(serial)` ループを、フラグが立った時にも回せばよい。

## 環境

- 5080 セット（RTX 5080、16GB）。カメラ serial `CL8F25300CA / C6 / F0 / HJ`
- Unity 6.4 (6000.4.9f1)、DX11
- `SensorManager.playbackOnly = true` なので、ライブ検証は Play 後に
  `SensorManager.StartLive()` を呼ぶ
- 単体ベンチ: `BodyTracking.Eval.Rtmpose.RtmposeVerify.Run(root, host, serial, frame, ...)`
