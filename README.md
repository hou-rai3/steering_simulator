2025年1月にステアタイヤシミュレーターを作成。マウスのポインタを目標値として、ロボットがそこに移動するようになっている。              
PID制御をすることで、遠ければ遠いほど移動が速く、近ければ近いほど遅く近づくことで、機械的にも優しくスムーズな移動をするようになっている。画面にはデバッグ用のステータスが表示されており、数値の遷移が視覚的に確認できる。
ウィンドウ上にあるマウスの位置をのx軸とy軸をターゲットとして取得し、現在の位置との差から目標点に移動するために必要な速度と角度変位が求められる。pygameで毎フレームごとにタイヤの角度と速度を更新することで、このシュミレーターを実現している。              
