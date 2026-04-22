# Femto Bolt Point Cloud Visualizer

## プロジェクト概要
Orbbec Femto Bolt（RGB-Dカメラ）から取得した深度+カラーデータを
Unity上でリアルタイムにポイントクラウドとして描画する。
将来的にはオプティカルフロー+深度からリボン状の3Dオブジェクトを
生成するインタラクティブインスタレーションに発展させる。

## 技術スタック
- Unity 6 LTS (URP - Universal 3D)
- OrbbecSDK v2（C/C++ネイティブSDK、直接P/Invoke経由で利用）
- K4A Wrapperは使わない。v2 APIを直接叩く
- 言語: C#
- 対象プラットフォーム: Windows x64

## ファイル構成
- OrbbecSDK v2ヘッダ: C:\dev\OrbbecSDK_v2\include\libobsensor\h\
- OrbbecSDK v2 Cサンプル: C:\dev\OrbbecSDK_v2\examples\c\
- DLL配置済み: Assets\Plugins\OrbbecSDK\
- ラッパーC#: Assets\Scripts\Orbbec\
- 描画スクリプト: Assets\Scripts\PointCloud\

## 実装方針

###基本姿勢
AIは迂回や別アプローチを勝手に行わず、最初の計画が失敗したら次の計画の確認を取る
AIはツールであり決定権は常にユーザーにある
AIは問題が発生した際に、憶測で対応をするのでなく、原因究明をおこない、確実に原因が判明してから対応策を検討する。
AIは新たなセッションになったら必ずCLAUDE-MEMの最新記憶を確認してから続きを進める
AIはタスクのまとまりごとにgit worktreeをつかってブランチを作成してそこで作業する
AIはソースコードの編集を開始する前に必ず適切なワークツリーにいるかどうか確認する
AIはソースの変更を行う前に正しい作業ブランチにいるかどうかを確認する
AIはソースの変更を行う前に他のセッションの作業中のものがないか確認し、混ざらないように気をつける
AIはcodexレビュー（/codex-review）で APPROVED を受けたら、そのままコミット＋mainへのマージまで自動で実行してよい。レビュー未実施のまま勝手にコミットするのは禁止。
AIはコミットする際に、毎フレーム大量に出力するようなログは削除してコミットする
AIはコミットする際に、ファイル構成やDB定義等に変更があればCLAUDE.mdも同時に更新する
AIはコードを編集する際、冗長な実装を避け、既存モジュールの再利用・共通化を常に検討する
AIはユーザーからの質問には端的に答える。長文・整理表・複数選択肢の提示は禁止。1〜3文で核心だけ返答し、不明点はユーザーから追加質問してもらう

### P/Invokeラッパー（Assets\Scripts\Orbbec\）
- OrbbecNative.cs: 全P/Invoke宣言をまとめる
  - DllImportの対象は "OrbbecSDK"（拡張子なし）
  - CallingConvention は Cdecl
- OrbbecContext.cs: ob_context のIDisposableラッパー
- OrbbecDevice.cs: ob_device のIDisposableラッパー
- OrbbecPipeline.cs: ob_pipeline のIDisposableラッパー
- OrbbecFrame.cs: ob_frame のIDisposableラッパー
- OrbbecFilter.cs: ob_filter のIDisposableラッパー（ポイントクラウド変換用）
- OrbbecException.cs: エラーハンドリング用例外クラス

### 描画（Assets\Scripts\PointCloud\）
- PointCloudRenderer.cs: MonoBehaviour
  - Start: デバイスオープン、パイプライン開始
  - Update: フレーム取得、ポイントクラウドフィルタで3D点群変換、Mesh更新
  - OnDestroy: リソース解放
- PointCloudBoundingBox.cs: MonoBehaviour
  - Transform（位置・回転・localScale）で OBB を定義し、KeepInside/KeepOutside で点群をフィルタ
  - showVisualization で Scene/Game 両方にワイヤーキューブを表示（Gizmo + runtime mesh）
- PointCloudDecimater.cs: MonoBehaviour
  - reductionPercent (0-100%) で毎フレーム各点を独立にランダムドロップ（Bernoulliサンプリング）

## コーディング規約
- ネイティブリソースは必ずIDisposableパターン + ファイナライザで管理
- P/Invoke宣言は推測で書かず、必ず以下のヘッダを参照して書く:
  - C:\dev\OrbbecSDK_v2\include\libobsensor\h\ObTypes.h
  - C:\dev\OrbbecSDK_v2\include\libobsensor\h\Context.h
  - C:\dev\OrbbecSDK_v2\include\libobsensor\h\Device.h
  - C:\dev\OrbbecSDK_v2\include\libobsensor\h\Pipeline.h
  - C:\dev\OrbbecSDK_v2\include\libobsensor\h\Frame.h
  - C:\dev\OrbbecSDK_v2\include\libobsensor\h\Filter.h
  - C:\dev\OrbbecSDK_v2\include\libobsensor\h\StreamProfile.h
- C/C++サンプルの参考実装:
  - C:\dev\OrbbecSDK_v2\examples\c\point_cloud\point_cloud.c
  - 上記サンプルと同等の処理をUnity C#で再現する
- 関数名や型名を「思い出し」で書かない。ヘッダで確定してから書く
- OrbbecSDK v1のAPIとv2は名前が違う関数がある。v2のヘッダのみを参照
- string marshalingはUTF-8（UnmanagedType.LPUTF8Str）
- 座標系: OrbbecSDKはmm単位。Unity側でm単位（1/1000スケール）に変換
- 深度+カラーの両方のストリームを有効にする
- D2C（Depth to Color）アライメントを有効にして、カラーが点群に正しくマッピングされるようにする

## 禁止事項
- OrbbecSDK v1のAPIを使わない（v2専用）
- Azure Kinect SDK / K4A APIに依存しない（v2直接経路）
- ヘッダを読まずにAPIシグネチャを推測しない