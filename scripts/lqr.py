import matplotlib.pyplot as plt
import numpy as np
import scipy.linalg as la
from control.matlab import *
import numpy as np

simTime = 3.0
dt = 0.001

# 倒立振子のパラメータ
m = 0.6  # 車体の質量[kg]
I = 0.002267  # 車体の慣性モーメント[kgm^2]
mw = 0.037  # 車輪の質量[kg]
Iw = 0.0000221 # 車輪の慣性モーメント[kgm^2]
r = 0.058 / 2  # 車輪の半径[m]
h = 0.035  # 車軸から車体重心までの距離[m]
g = 9.8  # 重力加速度[m/s^2]

# 係数
a = g*h*m / (I + Iw + m*h**2 + 2*h*m*r + m*r**2 + mw*r**2)
b = -(Iw + h*m*r + m*r**2 + mw*r**2) / (I + Iw + m*h**2 + 2*h*m*r + m*r**2 + mw*r**2)
print("a:", a)
print("b:", b)

# 可制御性のチェック
def check_ctrb(A, B):
  Uc = ctrb(A, B) # 可制御性行列の計算
  Nu = np.linalg.matrix_rank(Uc)  # Ucのランクを計算
  (N, N) = np.matrix(A).shape     # 正方行列Aのサイズ(N*N)
  # 可制御性の判別
  if Nu == N: return 0            # 可制御
  else: return -1                 # 可制御でない

  # 可観測性のチェック
def check_obsv(A, C):
  Uo = obsv(C, A) # 可制御性行列の計算
  No = np.linalg.matrix_rank(Uo)  # Ucのランクを計算
  (N, N) = np.matrix(A).shape     # 正方行列Aのサイズ(N*N)
  # 可制御性の判別
  if No == N: return 0            # 可観測
  else: return -1                 # 可観測でない

def sim(A, B, C, K):
    sys = ss(A - B*K, B, C, 0)
    y, t = initial(sys, T=np.arange(0, simTime, dt), X0=[0.1, 0.0, 0.0])
    plt.axhline(0, linestyle="--")
    plt.plot(t, y)
    plt.show()
    
def main():
  A = np.array([[0.0, 1.0, 0.0],
               [a, 0.0, 0.0],
               [0.0, 0.0, 0.0]])
  B = np.array([[0.0],
                  [b],
                  [1.0]])
  C = np.array([[1.0, 0.0, 0.0]])

  Q = np.array([[100.0, 0.0, 0.0],
                  [0.0, 40.0, 0.0],
                  [0.0, 0.0, 5.0]])
  R = np.array([[0.03]])
  # システムが可制御・可観測でなければ終了
  if check_ctrb(A, B) == -1 :
    print("システムが可制御でないので終了")
    return 0
  if check_obsv(np.sqrt(Q), A) == -1 :
    print("システムが可観測でないので終了")
    return 0
  # 最適レギュレータの設計
  K, P, e = lqr(A, B, Q, R)
  # 結果表示
  print("リカッチ方程式の解:\n",P)
  print("状態フィードバックゲイン:\n",K)
  print("閉ループ系の固有値:\n",e)
  print("{", format(-K[0,0],'g'), ",", format(-K[0,1],'g'), ",", format(-K[0,2],'g'), "}")
  # シミュレーション
  sim(A, B, C, K)

if __name__ == "__main__":
  main()
