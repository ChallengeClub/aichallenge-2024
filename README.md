# aichallenge-2024

本リポジトリでは、2024年度に実施される自動運転AIチャレンジでご利用いただく開発環境を提供します。参加者の皆様には、Autoware Universe をベースとした自動運転ソフトウェアを開発し、予選大会では End to End シミュレーション空間を走行するレーシングカートにインテグレートしていただきます。開発した自動運転ソフトウェアで、安全に走行しながらタイムアタックに勝利することが目標です。また、決勝大会では本物のレーシングカートへのインテグレーションを行っていただきます。

This repository provides a development environment use in the Automotive AI Challenge which will be held in 2024. For the preliminaries, participants will develop autonomous driving software based on Autoware Universe and integrate it into a racing kart that drives in the End to End simulation space. The goal is to win in time attack while driving safely with the developed autonomous driving software. Also, for the finals, qualifiers will integrate it into a real racing kart.

## ドキュメント / Documentation

下記ページにて、本大会に関する情報 (ルールの詳細や環境構築方法) を提供する予定です。ご確認の上、奮って大会へご参加ください。

Toward the competition, we will update the following pages to provide information such as rules and how to set up your dev environment. Please follow them. We are looking forward your participation!

- [日本語ページ](https://automotiveaichallenge.github.io/aichallenge-documentation-2024/)
- [English Page](https://automotiveaichallenge.github.io/aichallenge-documentation-2024/en/)

## コントローラ操作メモ

コントローラ：
https://www.elecom.co.jp/products/JC-U4113SBK.html

コントローラアサインはこちらを参照：
https://qiita.com/bushio/items/8b2c86162e77c7bf2ffe

## 手順

Dockerビルド
```
$ cd ~/aichallenge-2024
$ ./docker_build.sh dev
```

Dcokerコンテナ起動
```
$ ./docker_run.sh dev gpu
```

Dcokerコンテナ内で以下実行. シミュレータが起動し、コントローラ操作できる。
```
$ cd /aichallenge/
$ ./build_autoware.bash
```

```
$ ./run_controller.bash
```